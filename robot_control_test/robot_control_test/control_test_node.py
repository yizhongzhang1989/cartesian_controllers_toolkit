#!/usr/bin/env python3
"""Robot control test bench -- a thin HTTP/ROS client with a 3D canvas.

This node serves a small web dashboard that lets you *verify a robot works
correctly with each of its ros2_control controllers*. It does NOT import any
controller internals; everything is done over the standard ROS graph:

  * builds a **3D view** of the robot from the live URDF (``/robot_description``)
    and per-link world transforms read from **TF** (``base_frame -> link``) --
    no Pinocchio / no server-side FK (mirrors the aux_frame_manager dashboard,
    so this package stays inside cartesian_controllers_toolkit with no
    cross-submodule dependency on ikt_core);
  * discovers controllers on ``/controller_manager`` (``list_controllers``) and
    classifies each one (joint-trajectory / forward-position / cartesian
    motion / compliance / force);
  * activates / deactivates a controller (``switch_controller``), deactivating
    any active controller that claims the same command interfaces so the
    mutual exclusion ros2_control requires is handled automatically;
  * drives the engaged controller with safe, speed-limited test commands:
      - **JointTrajectoryController**: a timed ``trajectory_msgs/JointTrajectory``
        published to ``/<controller>/joint_trajectory``;
      - **forward-position controller** (Duco ``ForwardCommandController`` /
        UR ``JointGroupPositionController``): a velocity-ramped
        ``std_msgs/Float64MultiArray`` streamed to ``/<controller>/commands``;
      - **cartesian_motion / cartesian_compliance**: a ``geometry_msgs/
        PoseStamped`` target jogged on ``/<controller>/target_frame`` (seeded
        from the current TCP pose via TF);
      - **cartesian_force / cartesian_compliance**: a ``geometry_msgs/
        WrenchStamped`` setpoint on ``/<controller>/target_wrench``.

ThreadingHTTPServer in a daemon thread; rclpy on a MultiThreadedExecutor with a
ReentrantCallbackGroup so synchronous service calls issued from the HTTP handler
thread don't deadlock the ROS spin. Default port 8200 (8080/8100/8120/8140/8160/
8180 are used by the other toolkit dashboards).
"""

from __future__ import annotations

import json
import math
import mimetypes
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional
from urllib.parse import parse_qs, quote, unquote, urlparse

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import tf2_ros
    _HAVE_TF = True
except Exception:  # pragma: no cover
    _HAVE_TF = False

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None  # type: ignore

try:
    from controller_manager_msgs.srv import ListControllers, SwitchController
    _HAS_CM = True
except Exception:  # pragma: no cover
    _HAS_CM = False

_STATIC_DIR = Path(__file__).resolve().parent / "static"

# Joint kinds we can drive directly (joint-space command).
_JOINT_KINDS = ("joint_trajectory", "forward_position")
# Kinds that take a Cartesian target_frame pose.
_POSE_KINDS = ("cartesian_motion", "cartesian_compliance")
# Kinds that take a target_wrench.
_WRENCH_KINDS = ("cartesian_force", "cartesian_compliance")


def _latched_qos() -> QoSProfile:
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


# ---------------------------------------------------------------------------
# URDF parsing helpers (stdlib only)
# ---------------------------------------------------------------------------
def parse_link_names(urdf_xml: str) -> List[str]:
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return []
    return [ln.get("name") for ln in root.findall("link") if ln.get("name")]


def parse_visuals(urdf_xml: str) -> List[dict]:
    """Return ``[{link, filename, xyz, rpy, scale}]`` for mesh visuals."""
    out: List[dict] = []
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return out
    for link in root.findall("link"):
        lname = link.get("name")
        if not lname:
            continue
        for vis in link.findall("visual"):
            geom = vis.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is None or not mesh.get("filename"):
                continue
            origin = vis.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                if origin.get("xyz"):
                    xyz = [float(x) for x in origin.get("xyz").split()]
                if origin.get("rpy"):
                    rpy = [float(x) for x in origin.get("rpy").split()]
            scale = [1.0, 1.0, 1.0]
            if mesh.get("scale"):
                scale = [float(x) for x in mesh.get("scale").split()]
            out.append({"link": lname, "filename": mesh.get("filename"),
                        "xyz": xyz, "rpy": rpy, "scale": scale})
    return out


def parse_joint_tree(urdf_xml: str) -> List[dict]:
    """Return ``[{parent, child, type}]`` for the 3D skeleton lines."""
    out: List[dict] = []
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return out
    for j in root.findall("joint"):
        p, c = j.find("parent"), j.find("child")
        if p is None or c is None:
            continue
        pl, cl = p.get("link"), c.get("link")
        if pl and cl:
            out.append({"parent": pl, "child": cl,
                        "type": j.get("type", "fixed")})
    return out


def parse_movable_joints(urdf_xml: str) -> List[dict]:
    """Return ``[{name, type, lower, upper, child}]`` for actuated joints.

    Continuous joints report ``lower``/``upper`` as ``None`` (no finite limit);
    the dashboard falls back to a +/-pi slider range for those.
    """
    out: List[dict] = []
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return out
    for j in root.findall("joint"):
        jtype = j.get("type", "fixed")
        if jtype not in ("revolute", "prismatic", "continuous"):
            continue
        name = j.get("name")
        if not name:
            continue
        lower = upper = None
        limit = j.find("limit")
        if limit is not None and jtype != "continuous":
            if limit.get("lower") is not None:
                lower = float(limit.get("lower"))
            if limit.get("upper") is not None:
                upper = float(limit.get("upper"))
        child = j.find("child")
        out.append({"name": name, "type": jtype, "lower": lower,
                    "upper": upper,
                    "child": child.get("link") if child is not None else ""})
    return out


# ---------------------------------------------------------------------------
# Small math helpers (no numpy: keep the package dependency-light)
# ---------------------------------------------------------------------------
def _quat_to_R(x, y, z, w) -> List[List[float]]:
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _R_to_rpy(R) -> List[float]:
    sy = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
    if sy > 1e-6:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0
    return [roll, pitch, yaw]


def _norm_wxyz(q) -> List[float]:
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def _qmul(a, b) -> List[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw]


def _axis_quat(axis_idx: int, angle: float) -> List[float]:
    h = angle / 2.0
    s, c = math.sin(h), math.cos(h)
    v = [0.0, 0.0, 0.0]
    v[axis_idx] = s
    return [c, v[0], v[1], v[2]]


def classify_controller(ctype: str) -> str:
    """Map a ros2_control controller plugin type to a test-bench kind."""
    t = (ctype or "").lower()
    if "cartesian_compliance" in t:
        return "cartesian_compliance"
    if "cartesian_force" in t:
        return "cartesian_force"
    if "cartesian_motion" in t:
        return "cartesian_motion"
    if t.endswith("jointtrajectorycontroller"):
        return "joint_trajectory"
    if "forward_command_controller" in t or "jointgrouppositioncontroller" in t:
        return "forward_position"
    return "other"


# ===========================================================================
class RobotControlTest(Node):
    def __init__(self) -> None:
        super().__init__("robot_control_test")
        self.declare_parameter("dashboard_port", 8200)
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_frame", "")
        self.declare_parameter("max_joint_speed", 0.5)
        self.declare_parameter("send_rate", 100.0)

        gp = self.get_parameter
        self._port = int(gp("dashboard_port").value)
        self._cm_ns = str(gp("controller_manager").value).rstrip("/")
        self._desc_topic = str(gp("robot_description_topic").value)
        self._js_topic = str(gp("joint_states_topic").value)
        self._base_frame = str(gp("base_frame").value)
        self._tip_frame = str(gp("tip_frame").value)
        self._max_joint_speed = max(1e-3, float(gp("max_joint_speed").value))
        self._send_rate = max(10.0, float(gp("send_rate").value))
        self._host = "0.0.0.0"

        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # robot model / live state
        self._urdf = ""
        self._links: List[str] = []
        self._visuals: List[dict] = []
        self._joint_tree: List[dict] = []
        self._movable: List[dict] = []
        self._joint_pos: Dict[str, float] = {}
        self._js_stamp = 0.0
        self._pkg_dirs: Dict[str, Optional[str]] = {}

        # controllers
        self._controllers: List[dict] = []
        self._cm_ok = False

        # engaged-controller command state
        self._joint_ctrl: Optional[str] = None
        self._joint_kind = ""
        self._joint_names: List[str] = []
        self._target_pos: Dict[str, float] = {}
        self._stream_pos: Dict[str, float] = {}
        self._streaming = False
        self._cart_ctrl: Optional[str] = None
        self._cart_target: Optional[dict] = None
        self._wrench_ctrl: Optional[str] = None
        self._wrench = {"force": [0.0, 0.0, 0.0], "torque": [0.0, 0.0, 0.0]}
        self._last_msg = ""

        # cached publishers (per controller name)
        self._cmd_pubs: Dict[str, object] = {}
        self._pose_pubs: Dict[str, object] = {}
        self._wrench_pubs: Dict[str, object] = {}

        self.create_subscription(String, self._desc_topic, self._on_urdf,
                                 _latched_qos(), callback_group=self._cbg)
        self.create_subscription(JointState, self._js_topic, self._on_js,
                                 qos_profile_sensor_data, callback_group=self._cbg)

        if _HAVE_TF:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None

        self._cli_list = self._cli_switch = None
        if _HAS_CM:
            self._cli_list = self.create_client(
                ListControllers, f"{self._cm_ns}/list_controllers",
                callback_group=self._cbg)
            self._cli_switch = self.create_client(
                SwitchController, f"{self._cm_ns}/switch_controller",
                callback_group=self._cbg)

        self.create_timer(1.5, self._refresh_controllers, callback_group=self._cbg)
        self.create_timer(1.0 / self._send_rate, self._fpc_tick,
                          callback_group=self._cbg)
        self.create_timer(0.05, self._cart_tick, callback_group=self._cbg)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._start_http()
        self.get_logger().info(
            "robot_control_test on http://%s:%d  (controller_manager=%s, "
            "base_frame=%s) -- activate a controller and jog it to verify the "
            "robot moves." % (self._host, self._port, self._cm_ns,
                              self._base_frame))

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_urdf(self, msg: String) -> None:
        if not msg.data:
            return
        with self._lock:
            if msg.data == self._urdf and self._links:
                return
            self._urdf = msg.data
            self._links = parse_link_names(msg.data)
            self._visuals = parse_visuals(msg.data)
            self._joint_tree = parse_joint_tree(msg.data)
            self._movable = parse_movable_joints(msg.data)
        self.get_logger().info(
            "URDF: %d links, %d mesh visuals, %d movable joints."
            % (len(self._links), len(self._visuals), len(self._movable)))

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            self._js_stamp = time.monotonic()
            for i, n in enumerate(msg.name):
                if i < len(msg.position):
                    self._joint_pos[n] = float(msg.position[i])

    # ------------------------------------------------------------------ #
    # Controller discovery
    # ------------------------------------------------------------------ #
    def _refresh_controllers(self) -> None:
        if self._cli_list is None or not self._cli_list.service_is_ready():
            return
        fut = self._cli_list.call_async(ListControllers.Request())
        fut.add_done_callback(self._on_controllers)

    def _on_controllers(self, fut) -> None:
        try:
            resp = fut.result()
        except Exception:  # noqa: BLE001
            return
        out: List[dict] = []
        for c in resp.controller:
            ctype = c.type or ""
            cmd_ifaces = list(getattr(c, "required_command_interfaces", []) or [])
            joints: List[str] = []
            for ci in cmd_ifaces:
                jn, _, _iface = ci.rpartition("/")
                if jn and jn not in joints:
                    joints.append(jn)
            out.append({"name": c.name, "type": ctype, "state": c.state,
                        "kind": classify_controller(ctype), "joints": joints,
                        "cmd_ifaces": cmd_ifaces})
        out.sort(key=lambda c: c["name"])
        with self._lock:
            self._controllers = out
            self._cm_ok = True

    def _controller_info(self, name: str) -> Optional[dict]:
        with self._lock:
            for c in self._controllers:
                if c["name"] == name:
                    return dict(c)
        return None

    # ------------------------------------------------------------------ #
    # controller_manager switching
    # ------------------------------------------------------------------ #
    def _switch(self, activate: List[str], deactivate: List[str],
                timeout: float = 5.0) -> bool:
        if not _HAS_CM or self._cli_switch is None:
            return False
        if not (activate or deactivate):
            return True
        if not self._cli_switch.wait_for_service(timeout_sec=timeout):
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        try:
            req.activate_asap = True
        except Exception:  # noqa: BLE001
            pass
        fut = self._cli_switch.call_async(req)
        resp = self._wait(fut, timeout)
        return bool(resp and resp.ok)

    def _wait(self, fut, timeout: float):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if fut.done():
                try:
                    return fut.result()
                except Exception:  # noqa: BLE001
                    return None
            time.sleep(0.01)
        return None

    def _activate_exclusive(self, info: dict) -> bool:
        """Activate ``info`` and deactivate every active controller whose
        command interfaces overlap it (ros2_control forbids two controllers
        claiming the same interface)."""
        name = info["name"]
        want = set(info.get("cmd_ifaces", []))
        deact: List[str] = []
        with self._lock:
            ctrls = list(self._controllers)
        for c in ctrls:
            if c["name"] == name or c["state"] != "active":
                continue
            if want and set(c.get("cmd_ifaces", [])) & want:
                deact.append(c["name"])
        activate = [] if info["state"] == "active" else [name]
        return self._switch(activate, deact)

    # ------------------------------------------------------------------ #
    # Command publishers (cached per controller)
    # ------------------------------------------------------------------ #
    def _cmd_pub(self, name: str):
        with self._lock:
            pub = self._cmd_pubs.get(name)
        if pub is None:
            pub = self.create_publisher(
                Float64MultiArray, f"/{name}/commands", 10)
            with self._lock:
                self._cmd_pubs[name] = pub
        return pub

    def _jtc_pub(self, name: str):
        with self._lock:
            pub = self._cmd_pubs.get("jtc:" + name)
        if pub is None:
            pub = self.create_publisher(
                JointTrajectory, f"/{name}/joint_trajectory", 10)
            with self._lock:
                self._cmd_pubs["jtc:" + name] = pub
        return pub

    def _pose_pub(self, name: str):
        with self._lock:
            pub = self._pose_pubs.get(name)
        if pub is None:
            pub = self.create_publisher(
                PoseStamped, f"/{name}/target_frame", 10)
            with self._lock:
                self._pose_pubs[name] = pub
        return pub

    def _wrench_pub(self, name: str):
        with self._lock:
            pub = self._wrench_pubs.get(name)
        if pub is None:
            pub = self.create_publisher(
                WrenchStamped, f"/{name}/target_wrench", 10)
            with self._lock:
                self._wrench_pubs[name] = pub
        return pub

    # ------------------------------------------------------------------ #
    # Joint limits / clamping
    # ------------------------------------------------------------------ #
    def _limits(self, joint: str):
        for m in self._movable:
            if m["name"] == joint:
                return m["lower"], m["upper"]
        return None, None

    def _clamp(self, joint: str, value: float) -> float:
        lo, hi = self._limits(joint)
        if lo is not None and value < lo:
            return lo
        if hi is not None and value > hi:
            return hi
        return value

    # ------------------------------------------------------------------ #
    # Engage / disengage
    # ------------------------------------------------------------------ #
    def engage(self, name: str) -> dict:
        info = self._controller_info(name)
        if info is None:
            return {"ok": False, "message": "controller '%s' not found" % name}
        kind = info["kind"]
        if kind == "other":
            return {"ok": False,
                    "message": "controller '%s' (%s) is not a kind this bench "
                               "can drive" % (name, info["type"])}
        # seed Cartesian pose from the live TCP BEFORE switching (TF is live)
        seed = self._tcp_pose() if kind in (_POSE_KINDS + _WRENCH_KINDS) else None
        if not self._activate_exclusive(info):
            return {"ok": False,
                    "message": "switch_controller failed for '%s' (is the "
                               "controller_manager up?)" % name}
        with self._lock:
            # engaging anything new releases the previous joint stream
            self._streaming = False
            if kind in _JOINT_KINDS:
                self._joint_ctrl = name
                self._joint_kind = kind
                self._joint_names = list(info["joints"])
                self._target_pos = {j: self._joint_pos.get(j, 0.0)
                                    for j in self._joint_names}
                self._stream_pos = dict(self._target_pos)
                self._streaming = (kind == "forward_position")
                self._cart_ctrl = None
                self._wrench_ctrl = None
            else:
                self._joint_ctrl = None
                self._joint_kind = ""
                self._joint_names = []
                self._cart_ctrl = name if kind in _POSE_KINDS else None
                self._wrench_ctrl = name if kind in _WRENCH_KINDS else None
                if kind in _POSE_KINDS:
                    self._cart_target = seed
                if kind in _WRENCH_KINDS:
                    self._wrench = {"force": [0.0, 0.0, 0.0],
                                    "torque": [0.0, 0.0, 0.0]}
            self._last_msg = "engaged %s (%s)" % (name, kind)
        return {"ok": True, "name": name, "kind": kind,
                "message": "engaged %s (%s)" % (name, kind)}

    def disengage(self, name: str) -> dict:
        info = self._controller_info(name)
        self._switch([], [name])
        with self._lock:
            if self._joint_ctrl == name:
                self._joint_ctrl = None
                self._joint_kind = ""
                self._streaming = False
            if self._cart_ctrl == name:
                self._cart_ctrl = None
                self._cart_target = None
            if self._wrench_ctrl == name:
                self._wrench_ctrl = None
            self._last_msg = "disengaged %s" % name
        kind = info["kind"] if info else ""
        return {"ok": True, "name": name, "kind": kind,
                "message": "disengaged %s" % name}

    # ------------------------------------------------------------------ #
    # Joint commands
    # ------------------------------------------------------------------ #
    def _send_jtc(self) -> None:
        """Publish a single timed point to the engaged JTC."""
        with self._lock:
            name = self._joint_ctrl
            joints = list(self._joint_names)
            target = dict(self._target_pos)
            meas = {j: self._joint_pos.get(j, 0.0) for j in joints}
        if not name or not joints:
            return
        dist = max((abs(target.get(j, meas[j]) - meas[j]) for j in joints),
                   default=0.0)
        dt = max(0.5, dist / self._max_joint_speed)
        traj = JointTrajectory()
        traj.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(target.get(j, meas[j])) for j in joints]
        pt.time_from_start = Duration(sec=int(dt),
                                      nanosec=int((dt - int(dt)) * 1e9))
        traj.points = [pt]
        self._jtc_pub(name).publish(traj)

    def _dispatch_joint(self) -> None:
        """Apply the current joint targets to the engaged controller."""
        with self._lock:
            kind = self._joint_kind
        if kind == "joint_trajectory":
            self._send_jtc()
        # forward_position is streamed by _fpc_tick from self._target_pos

    def set_joint_targets(self, positions: Dict[str, float]) -> dict:
        with self._lock:
            if not self._joint_ctrl:
                return {"ok": False, "message": "no joint controller engaged"}
            for j, v in positions.items():
                if j in self._target_pos:
                    self._target_pos[j] = self._clamp(j, float(v))
        self._dispatch_joint()
        return {"ok": True}

    def joint_jog(self, joint: str, delta: float) -> dict:
        with self._lock:
            if not self._joint_ctrl:
                return {"ok": False, "message": "no joint controller engaged"}
            if joint not in self._target_pos:
                return {"ok": False, "message": "joint '%s' not controlled"
                        % joint}
            self._target_pos[joint] = self._clamp(
                joint, self._target_pos[joint] + float(delta))
        self._dispatch_joint()
        return {"ok": True}

    def joint_sync(self) -> dict:
        """Set the targets to the measured joint positions (hold here)."""
        with self._lock:
            if not self._joint_ctrl:
                return {"ok": False, "message": "no joint controller engaged"}
            self._target_pos = {j: self._joint_pos.get(j, 0.0)
                                for j in self._joint_names}
            self._stream_pos = dict(self._target_pos)
        self._dispatch_joint()
        return {"ok": True, "message": "synced targets to current pose"}

    def joint_stop(self) -> dict:
        """Halt motion: hold at the current measured pose."""
        with self._lock:
            if not self._joint_ctrl:
                return {"ok": False, "message": "no joint controller engaged"}
            self._target_pos = {j: self._joint_pos.get(j, 0.0)
                                for j in self._joint_names}
            self._stream_pos = dict(self._target_pos)
        self._dispatch_joint()
        return {"ok": True, "message": "stopped (holding current pose)"}

    def _fpc_tick(self) -> None:
        with self._lock:
            if self._joint_kind != "forward_position" or not self._streaming:
                return
            name = self._joint_ctrl
            joints = list(self._joint_names)
            target = dict(self._target_pos)
            stream = dict(self._stream_pos)
        if not name or not joints:
            return
        step = self._max_joint_speed / self._send_rate
        data: List[float] = []
        new_stream: Dict[str, float] = {}
        for j in joints:
            cur = stream.get(j, self._joint_pos.get(j, 0.0))
            tgt = target.get(j, cur)
            d = tgt - cur
            if d > step:
                cur += step
            elif d < -step:
                cur -= step
            else:
                cur = tgt
            new_stream[j] = cur
            data.append(cur)
        msg = Float64MultiArray()
        msg.data = data
        self._cmd_pub(name).publish(msg)
        with self._lock:
            self._stream_pos = new_stream

    # ------------------------------------------------------------------ #
    # Cartesian pose commands
    # ------------------------------------------------------------------ #
    def _tcp_pose(self) -> Optional[dict]:
        tip = self._resolve_tip()
        if not tip or self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, tip, rclpy.time.Time())
        except Exception:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return {"xyz": [t.x, t.y, t.z],
                "quat": _norm_wxyz([r.w, r.x, r.y, r.z]),
                "frame_id": self._base_frame, "tip": tip}

    def _resolve_tip(self) -> str:
        if self._tip_frame:
            return self._tip_frame
        with self._lock:
            jt = list(self._joint_tree)
            links = list(self._links)
        parents = {j["parent"] for j in jt}
        leaves = [j["child"] for j in jt if j["child"] not in parents]
        return leaves[-1] if leaves else (links[-1] if links else "")

    def cart_jog(self, axis: str, delta: float) -> dict:
        with self._lock:
            name = self._cart_ctrl
            tgt = dict(self._cart_target) if self._cart_target else None
        if not name:
            return {"ok": False, "message": "no Cartesian controller engaged"}
        if tgt is None:
            tgt = self._tcp_pose()
            if tgt is None:
                return {"ok": False,
                        "message": "could not seed target from TCP (TF "
                                   "unavailable)"}
        idx = {"x": 0, "y": 1, "z": 2}.get(axis)
        if idx is not None:
            tgt["xyz"][idx] = float(tgt["xyz"][idx]) + float(delta)
        else:
            ridx = {"rx": 0, "ry": 1, "rz": 2}.get(axis)
            if ridx is None:
                return {"ok": False, "message": "bad axis '%s'" % axis}
            tgt["quat"] = _norm_wxyz(
                _qmul(_axis_quat(ridx, float(delta)), tgt["quat"]))
        with self._lock:
            self._cart_target = tgt
        self._publish_pose(name, tgt)
        return {"ok": True}

    def cart_reset(self) -> dict:
        with self._lock:
            name = self._cart_ctrl
        if not name:
            return {"ok": False, "message": "no Cartesian controller engaged"}
        seed = self._tcp_pose()
        if seed is None:
            return {"ok": False, "message": "could not read TCP pose (TF)"}
        with self._lock:
            self._cart_target = seed
        self._publish_pose(name, seed)
        return {"ok": True, "message": "reset target to current TCP"}

    def _publish_pose(self, name: str, tgt: dict) -> None:
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = tgt.get("frame_id") or self._base_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            (float(tgt["xyz"][0]), float(tgt["xyz"][1]), float(tgt["xyz"][2]))
        q = tgt["quat"]
        m.pose.orientation.w = float(q[0])
        m.pose.orientation.x = float(q[1])
        m.pose.orientation.y = float(q[2])
        m.pose.orientation.z = float(q[3])
        self._pose_pub(name).publish(m)

    # ------------------------------------------------------------------ #
    # Wrench commands
    # ------------------------------------------------------------------ #
    def set_wrench(self, force: List[float], torque: List[float]) -> dict:
        with self._lock:
            name = self._wrench_ctrl
        if not name:
            return {"ok": False, "message": "no force controller engaged"}
        f = [float(v) for v in (force or [0, 0, 0])][:3]
        t = [float(v) for v in (torque or [0, 0, 0])][:3]
        while len(f) < 3:
            f.append(0.0)
        while len(t) < 3:
            t.append(0.0)
        with self._lock:
            self._wrench = {"force": f, "torque": t}
        self._publish_wrench(name, f, t)
        return {"ok": True}

    def wrench_zero(self) -> dict:
        return self.set_wrench([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def _publish_wrench(self, name: str, f: List[float], t: List[float]) -> None:
        m = WrenchStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._base_frame
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = f
        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = t
        self._wrench_pub(name).publish(m)

    def _cart_tick(self) -> None:
        """Keepalive: republish the live Cartesian target / wrench so a freshly
        activated controller reliably latches onto them."""
        with self._lock:
            cart = self._cart_ctrl
            tgt = dict(self._cart_target) if self._cart_target else None
            wname = self._wrench_ctrl
            wr = dict(self._wrench)
        if cart and tgt:
            self._publish_pose(cart, tgt)
        if wname:
            self._publish_wrench(wname, wr["force"], wr["torque"])

    # ------------------------------------------------------------------ #
    # 3D / TF FK helpers
    # ------------------------------------------------------------------ #
    def _link_tf(self) -> Dict[str, list]:
        out: Dict[str, list] = {}
        if self._tf_buffer is None:
            return out
        with self._lock:
            links = list(self._links)
            base = self._base_frame
        for link in links:
            try:
                tf = self._tf_buffer.lookup_transform(
                    base, link, rclpy.time.Time())
            except Exception:
                continue
            t, r = tf.transform.translation, tf.transform.rotation
            R = _quat_to_R(r.x, r.y, r.z, r.w)
            out[link] = [
                [R[0][0], R[0][1], R[0][2], t.x],
                [R[1][0], R[1][1], R[1][2], t.y],
                [R[2][0], R[2][1], R[2][2], t.z],
                [0.0, 0.0, 0.0, 1.0],
            ]
        return out

    def _mesh_url(self, filename: str) -> str:
        if filename.startswith("package://"):
            pkg, _, rel = filename[len("package://"):].partition("/")
            return f"/mesh?pkg={quote(pkg)}&path={quote(rel)}"
        if filename.startswith("file://"):
            return f"/mesh?path={quote(filename[len('file://'):])}"
        return f"/mesh?path={quote(filename)}"

    def _package_dir(self, pkg: str) -> Optional[str]:
        if pkg in self._pkg_dirs:
            return self._pkg_dirs[pkg]
        resolved = None
        if get_package_share_directory is not None:
            try:
                resolved = get_package_share_directory(pkg)
            except Exception:
                resolved = None
        self._pkg_dirs[pkg] = resolved
        return resolved

    def read_mesh(self, pkg: str, rel: str) -> Optional[bytes]:
        rel = unquote(rel or "")
        if pkg:
            base = self._package_dir(unquote(pkg))
            if base is None:
                return None
            path = Path(base) / rel
        else:
            path = Path(rel)
        try:
            return path.resolve().read_bytes()
        except Exception:
            return None

    # ------------------------------------------------------------------ #
    # Snapshot for the dashboard
    # ------------------------------------------------------------------ #
    def snapshot(self) -> dict:
        with self._lock:
            links = list(self._links)
            visuals = list(self._visuals)
            joint_tree = list(self._joint_tree)
            movable = list(self._movable)
            joint_pos = dict(self._joint_pos)
            controllers = list(self._controllers)
            cm_ok = self._cm_ok
            js_age = (time.monotonic() - self._js_stamp
                      if self._js_stamp else None)
            engaged = {
                "joint": self._joint_ctrl, "joint_kind": self._joint_kind,
                "cartesian": self._cart_ctrl, "wrench": self._wrench_ctrl,
                "streaming": self._streaming,
            }
            joint_names = list(self._joint_names)
            targets = dict(self._target_pos)
            cart_target = dict(self._cart_target) if self._cart_target else None
            wrench = dict(self._wrench)
            last_msg = self._last_msg
        link_tf = self._link_tf()
        tip = self._resolve_tip()
        tcp = None
        if tip in link_tf:
            m = link_tf[tip]
            R = [[m[0][0], m[0][1], m[0][2]],
                 [m[1][0], m[1][1], m[1][2]],
                 [m[2][0], m[2][1], m[2][2]]]
            tcp = {"xyz": [m[0][3], m[1][3], m[2][3]], "rpy": _R_to_rpy(R),
                   "tip": tip}
        # The 3D viewer renders STL only, so mesh visuals in another format
        # (e.g. UR ships COLLADA .dae) are NOT displayable here. Base
        # has_meshes on the RENDERABLE (STL) visuals so the dashboard shows the
        # skeleton and auto-disables the mesh toggle, and flag the case where
        # the URDF DOES declare meshes but none are renderable.
        renderable = [v for v in visuals
                      if str(v["filename"]).lower().endswith(".stl")]
        mesh_unsupported = bool(visuals) and not renderable
        return {
            "have_model": bool(links),
            "base_frame": self._base_frame,
            "tip_frame": tip,
            "controller_manager": self._cm_ns,
            "cm_ok": cm_ok,
            "links": links,
            "has_meshes": bool(renderable),
            "mesh_unsupported": mesh_unsupported,
            "visuals": [
                {"link": v["link"], "url": self._mesh_url(v["filename"]),
                 "xyz": v["xyz"], "rpy": v["rpy"], "scale": v["scale"]}
                for v in renderable],
            "joint_tree": joint_tree,
            "link_tf": link_tf,
            "movable_joints": movable,
            "joint_values": joint_pos,
            "js_age": round(js_age, 2) if js_age is not None else None,
            "controllers": controllers,
            "engaged": engaged,
            "joint_names": joint_names,
            "joint_targets": targets,
            "cart_target": ({"xyz": cart_target["xyz"],
                             "rpy": _R_to_rpy(_quat_to_R(
                                 cart_target["quat"][1], cart_target["quat"][2],
                                 cart_target["quat"][3], cart_target["quat"][0]))}
                            if cart_target else None),
            "wrench": wrench,
            "tcp": tcp,
            "message": last_msg,
        }

    # ------------------------------------------------------------------ #
    # HTTP server
    # ------------------------------------------------------------------ #
    def _start_http(self) -> None:
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # quiet
                return

            def _send(self, code, body, ctype="application/json"):
                if isinstance(body, str):
                    body = body.encode()
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except Exception:
                    pass

            def _read_json(self) -> dict:
                n = int(self.headers.get("Content-Length", 0) or 0)
                raw = self.rfile.read(n) if n else b"{}"
                try:
                    return json.loads(raw or b"{}")
                except Exception:
                    return {}

            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    return self._serve_static("index.html")
                if path == "/api/state":
                    return self._send(200, json.dumps(dash.snapshot()))
                if path == "/mesh":
                    qs = {k: v[0] for k, v in
                          parse_qs(urlparse(self.path).query).items()}
                    data = dash.read_mesh(qs.get("pkg", ""), qs.get("path", ""))
                    if data is None:
                        return self._send(404, "not found", "text/plain")
                    self.send_response(200)
                    self.send_header("Content-Type", "model/stl")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "public, max-age=3600")
                    self.end_headers()
                    try:
                        self.wfile.write(data)
                    except Exception:
                        pass
                    return
                if path.startswith("/vendor/") or path.endswith(
                        (".css", ".js", ".html")):
                    return self._serve_static(path.lstrip("/"))
                return self._send(404, '{"error":"not found"}')

            def do_POST(self):
                path = urlparse(self.path).path
                b = self._read_json()
                routes = {
                    "/api/engage": lambda: dash.engage(b.get("name", "")),
                    "/api/disengage": lambda: dash.disengage(b.get("name", "")),
                    "/api/joint/set":
                        lambda: dash.set_joint_targets(b.get("positions", {})),
                    "/api/joint/jog":
                        lambda: dash.joint_jog(b.get("joint", ""),
                                               b.get("delta", 0.0)),
                    "/api/joint/sync": dash.joint_sync,
                    "/api/joint/stop": dash.joint_stop,
                    "/api/cart/jog":
                        lambda: dash.cart_jog(b.get("axis", ""),
                                              b.get("delta", 0.0)),
                    "/api/cart/reset": dash.cart_reset,
                    "/api/wrench/set":
                        lambda: dash.set_wrench(b.get("force", []),
                                                b.get("torque", [])),
                    "/api/wrench/zero": dash.wrench_zero,
                }
                fn = routes.get(path)
                if fn is None:
                    return self._send(404, '{"error":"not found"}')
                try:
                    out = fn()
                except Exception as exc:  # noqa: BLE001
                    out = {"ok": False, "message": "error: %s" % exc}
                return self._send(200, json.dumps(out))

            def _serve_static(self, relpath):
                fp = (_STATIC_DIR / relpath).resolve()
                if not str(fp).startswith(str(_STATIC_DIR.resolve())) \
                        or not fp.is_file():
                    return self._send(404, "not found", "text/plain")
                ctype = mimetypes.guess_type(str(fp))[0] \
                    or "application/octet-stream"
                if fp.suffix == ".js":
                    ctype = "text/javascript"
                return self._send(200, fp.read_bytes(), ctype)

        self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def destroy_node(self):
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotControlTest()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
