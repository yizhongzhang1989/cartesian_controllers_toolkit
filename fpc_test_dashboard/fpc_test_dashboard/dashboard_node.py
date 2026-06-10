"""FPC test dashboard -- ROS node + HTTP server + automatic test runner.

A self-contained web platform to characterise a robot's forward-position-control
(FPC) passthrough before layering host-side admittance / Cartesian force control
on top of it.

What it does
------------
1. Reads the robot's movable joints from ``/robot_description`` (the URDF).
2. Discovers ``ForwardCommandController`` instances on ``/controller_manager``
   and the ordered joints each one commands.
3. The operator picks a controller + a subset of its joints in the web UI.
4. For each selected joint it runs an automatic battery (see test_logic):
   SMOOTH (continuous reference), STAIR (low-rate teleop-style stepped command),
   SMOOTHED (stair + a host low-pass) -- exactly the conditions an admittance
   controller will drive the FPC through -- while recording the actual joint
   motion from ``/joint_states`` (and the wrist wrench, if a topic is given,
   as the independent ground truth for real vibration).
5. Plots the live traces and reports per-segment metrics (tracking RMSE, lag,
   vibration) so you can see whether the FPC is stable / low-latency enough,
   and how much a command smoother helps.

Safety
------
* Every commanded joint is hard-clamped to ``baseline ± limit`` (default ~8 deg).
* Only the one joint under test moves; all others hold their start position.
* Smoothstep ramps in and out of every segment; on stop / error / disconnect the
  arm is ramped back to baseline and the original controllers are restored.
* The run refuses to start if ``/joint_states`` is stale or the chosen FPC's
  joints are unknown.

The HTTP server runs in a daemon thread; rclpy runs in a MultiThreadedExecutor so
the runner thread's service calls (switch_controller) are processed concurrently.

Parameters
----------
  host               string  default "0.0.0.0"
  port               int     default 8140
  joint_states_topic string  default "/joint_states"
  wrench_topic       string  default ""  (empty => no force ground-truth)
  controller_manager string  default "/controller_manager"
  send_rate          double  default 200.0   (command publish + analysis rate)
  default_limit_deg  double  default 8.0     (per-joint safety envelope)
  report_dir         string  default "~/.ros/fpc_test_dashboard/runs"
                             (each run is saved here as a self-contained HTML
                              report + raw data, browsable from the web UI)
"""
from __future__ import annotations

import json
import math
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import urlparse

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

try:
    from controller_manager_msgs.srv import ListControllers, SwitchController
    _HAS_CM = True
except Exception:  # noqa: BLE001
    _HAS_CM = False

from . import test_logic as L
from . import report as R

_STATIC_DIR = Path(__file__).resolve().parent / "static"
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


def _smoothstep(a: float) -> float:
    a = 0.0 if a < 0 else (1.0 if a > 1 else a)
    return a * a * (3.0 - 2.0 * a)


# ===========================================================================
# ROS node
# ===========================================================================
class FpcTestDashboard(Node):
    def __init__(self):
        super().__init__("fpc_test_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8140)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("wrench_topic", "")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("send_rate", 200.0)
        self.declare_parameter("default_limit_deg", 8.0)
        self.declare_parameter("report_dir", "~/.ros/fpc_test_dashboard/runs")

        gp = self.get_parameter
        self._host = gp("host").value
        self._port = int(gp("port").value)
        self._js_topic = gp("joint_states_topic").value
        self._wrench_topic = gp("wrench_topic").value
        self._cm_ns = gp("controller_manager").value.rstrip("/")
        self._send_rate = float(gp("send_rate").value)
        self._default_limit_deg = float(gp("default_limit_deg").value)
        self._report_dir = str(Path(gp("report_dir").value).expanduser())

        self._lock = threading.Lock()
        self._urdf_joints: List[L.JointInfo] = []
        self._joint_pos: Dict[str, float] = {}
        self._js_mono: float = 0.0
        self._wrench: Optional[Tuple[float, float, float]] = None
        self._wrench_mono: float = 0.0
        self._controllers: List[Dict[str, Any]] = []   # discovered FPCs

        # run state (shared with HTTP + runner)
        self._run_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._status = "idle"           # idle | running | done | error | stopped
        self._status_msg = ""
        self._progress = {"joint": "", "segment": "", "i": 0, "n": 0}
        self._live = []                 # recent (t, cmd_deg, act_deg, fhf) for plotting
        self._results: List[Dict[str, Any]] = []
        self._run_meta: Dict[str, Any] = {}
        self._last_report: Optional[Dict[str, Any]] = None  # newest saved run

        self._cbg = ReentrantCallbackGroup()

        urdf_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, "/robot_description", self._on_urdf, urdf_qos)
        self.create_subscription(JointState, self._js_topic, self._on_js, 50,
                                 callback_group=self._cbg)
        if self._wrench_topic:
            wq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=20)
            self.create_subscription(WrenchStamped, self._wrench_topic,
                                     self._on_wrench, wq, callback_group=self._cbg)

        self._cli_list = self._cli_switch = None
        if _HAS_CM:
            self._cli_list = self.create_client(
                ListControllers, f"{self._cm_ns}/list_controllers",
                callback_group=self._cbg)
            self._cli_switch = self.create_client(
                SwitchController, f"{self._cm_ns}/switch_controller",
                callback_group=self._cbg)

        self._cmd_pubs: Dict[str, Any] = {}     # controller -> publisher
        # periodically refresh the controller catalogue
        self.create_timer(2.0, self._refresh_controllers, callback_group=self._cbg)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._start_http()
        self.get_logger().info(
            f"fpc_test_dashboard on http://{self._host}:{self._port}  "
            f"(js={self._js_topic}, wrench={self._wrench_topic or 'none'}, "
            f"cm={self._cm_ns})")

    # ---- subscriptions ----------------------------------------------------
    def _on_urdf(self, msg: String) -> None:
        try:
            joints = L.parse_urdf_joints(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"URDF parse failed: {exc}")
            return
        with self._lock:
            self._urdf_joints = joints
        self.get_logger().info(f"URDF: {len(joints)} movable joints")

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            self._js_mono = time.monotonic()
            for i, n in enumerate(msg.name):
                if i < len(msg.position):
                    self._joint_pos[n] = float(msg.position[i])

    def _on_wrench(self, msg: WrenchStamped) -> None:
        with self._lock:
            self._wrench = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
            self._wrench_mono = time.monotonic()

    # ---- controller discovery --------------------------------------------
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
        found: List[Dict[str, Any]] = []
        for c in resp.controller:
            ctype = c.type or ""
            if "forward_command_controller" not in ctype.lower():
                continue
            # ordered joints from required_command_interfaces ("joint/position")
            joints, pos_only = [], True
            for ci in list(getattr(c, "required_command_interfaces", []) or []):
                jn, _, iface = ci.rpartition("/")
                if iface != "position":
                    pos_only = False
                if jn and jn not in joints:
                    joints.append(jn)
            if joints and pos_only:
                found.append({"name": c.name, "type": ctype,
                              "state": c.state, "joints": joints})
        with self._lock:
            self._controllers = found

    def _list_controllers_sync(self, timeout=2.0):
        if self._cli_list is None or not self._cli_list.wait_for_service(timeout_sec=timeout):
            return None
        fut = self._cli_list.call_async(ListControllers.Request())
        return self._wait(fut, timeout)

    def _switch(self, activate: List[str], deactivate: List[str], timeout=4.0) -> bool:
        if not _HAS_CM or self._cli_switch is None:
            return False
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
        """Block the calling (runner/HTTP) thread until ``fut`` completes.

        The executor spins in the main thread, so we just poll the future.
        """
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if fut.done():
                try:
                    return fut.result()
                except Exception:  # noqa: BLE001
                    return None
            time.sleep(0.01)
        return None

    def _pub_for(self, controller: str):
        with self._lock:
            pub = self._cmd_pubs.get(controller)
        if pub is None:
            pub = self.create_publisher(Float64MultiArray, f"/{controller}/commands", 10)
            with self._lock:
                self._cmd_pubs[controller] = pub
        return pub

    # ---- snapshots for the API -------------------------------------------
    def api_info(self) -> Dict[str, Any]:
        with self._lock:
            joints = [j.to_dict() for j in self._urdf_joints]
            controllers = list(self._controllers)
            wrench = bool(self._wrench_topic)
        return {
            "joints": joints,
            "controllers": controllers,
            "wrench_available": wrench,
            "send_rate": self._send_rate,
            "report_dir": self._report_dir,
            "defaults": {
                "amp_deg": 4.0, "freq_hz": 0.2, "seg_seconds": 9.0,
                "stair_step_deg": 1.5, "lp_hz": 10.0,
                "limit_deg": self._default_limit_deg,
                "tests": ["smooth", "stair", "smoothed"],
            },
            "status": self._status,
        }

    def api_state(self) -> Dict[str, Any]:
        with self._lock:
            js_age = time.monotonic() - self._js_mono if self._js_mono else None
            return {
                "status": self._status,
                "message": self._status_msg,
                "progress": dict(self._progress),
                "results": list(self._results),
                "meta": dict(self._run_meta),
                "js_age": js_age,
                "live": list(self._live[-1200:]),
                "last_report": dict(self._last_report) if self._last_report else None,
            }

    def api_runs(self) -> Dict[str, Any]:
        try:
            runs = R.list_runs(self._report_dir)
        except Exception as exc:  # noqa: BLE001
            return {"report_dir": self._report_dir, "runs": [], "error": str(exc)}
        return {"report_dir": self._report_dir, "runs": runs}

    # ---- run control ------------------------------------------------------
    def start_run(self, cfg: Dict[str, Any]) -> Tuple[bool, str]:
        with self._lock:
            if self._status == "running":
                return False, "a test is already running"
            ctrl = cfg.get("controller")
            controllers = {c["name"]: c for c in self._controllers}
            if ctrl not in controllers:
                return False, f"unknown controller '{ctrl}'"
            cinfo = controllers[ctrl]
            sel = [j for j in cfg.get("joints", []) if j in cinfo["joints"]]
            if not sel:
                return False, "no valid joints selected"
            amp = float(cfg.get("amp_deg", 4.0))
            limit = float(cfg.get("limit_deg", self._default_limit_deg))
            if amp > limit + 1e-6:
                return False, (
                    f"amplitude {amp:.1f}° exceeds the safety limit {limit:.1f}° — the "
                    f"command would be clamped (flat-topped). Raise the safety limit "
                    f"to ≥ {amp:.0f}° (make sure the workspace allows that travel) or "
                    f"reduce the amplitude.")
            if self._js_mono == 0.0 or (time.monotonic() - self._js_mono) > 0.5:
                return False, "/joint_states stale or missing"
            base = {j: self._joint_pos.get(j) for j in cinfo["joints"]}
            if any(v is None for v in base.values()):
                return False, "baseline positions unknown (joint_states incomplete)"
        self._stop_flag.clear()
        self._run_thread = threading.Thread(
            target=self._run_battery, args=(cinfo, sel, cfg, base), daemon=True)
        self._run_thread.start()
        return True, "started"

    def stop_run(self) -> None:
        self._stop_flag.set()

    # ---- the runner -------------------------------------------------------
    def _run_battery(self, cinfo, sel, cfg, base) -> None:
        import numpy as np
        ctrl = cinfo["name"]
        order = cinfo["joints"]                       # full command order
        jmap = {j: i for i, j in enumerate(order)}
        limit = float(cfg.get("limit_deg", self._default_limit_deg)) * DEG2RAD
        params = L.TestParams(
            amp=float(cfg.get("amp_deg", 4.0)) * DEG2RAD,
            freq=float(cfg.get("freq_hz", 0.2)),
            seg_seconds=float(cfg.get("seg_seconds", 9.0)),
            send_rate=self._send_rate,
            stair_step=float(cfg.get("stair_step_deg", 1.5)) * DEG2RAD,
            lp_hz=float(cfg.get("lp_hz", 10.0)),
            include=tuple(cfg.get("tests", ["smooth", "stair", "smoothed"])))
        # prismatic joints use metres; convert amp/limit back (treat deg as the
        # raw unit the UI sent -- for prismatic the UI sends mm, see static JS).
        is_pris = {j.name: j.is_prismatic for j in self._urdf_joints}
        dt = 1.0 / self._send_rate
        pub = self._pub_for(ctrl)
        base_vec = [base[j] for j in order]

        deactivated: List[str] = []
        with self._lock:
            self._status = "running"
            self._status_msg = "preparing"
            self._results = []
            self._live = []
            self._last_report = None
            self._run_meta = {"controller": ctrl, "joints": sel,
                              "amp_deg": cfg.get("amp_deg", 4.0),
                              "freq_hz": cfg.get("freq_hz", 0.2),
                              "seg_seconds": cfg.get("seg_seconds", 9.0),
                              "stair_step_deg": cfg.get("stair_step_deg", 1.5),
                              "lp_hz": cfg.get("lp_hz", 10.0),
                              "limit_deg": cfg.get("limit_deg", self._default_limit_deg),
                              "tests": list(params.include),
                              "wrench": bool(self._wrench_topic),
                              "started": time.time()}
        records: List[Dict[str, Any]] = []   # full per-segment traces for the report

        def clamp_vec(v):
            return [min(base_vec[i] + limit, max(base_vec[i] - limit, v[i]))
                    for i in range(len(v))]

        def publish(vec):
            m = Float64MultiArray()
            m.data = [float(x) for x in clamp_vec(vec)]
            pub.publish(m)

        def actual(j):
            with self._lock:
                return self._joint_pos.get(j)

        def cur_wrench():
            with self._lock:
                if self._wrench and (time.monotonic() - self._wrench_mono) < 0.5:
                    return self._wrench
            return None

        def ramp_to(vec_target, dur=1.5):
            start = list(base_vec)
            cur = [actual(j) for j in order]
            start = [c if c is not None else b for c, b in zip(cur, base_vec)]
            n = max(1, int(dur * self._send_rate))
            for k in range(n):
                if self._stop_flag.is_set():
                    break
                a = _smoothstep((k + 1) / n)
                publish([start[i] + a * (vec_target[i] - start[i]) for i in range(len(order))])
                time.sleep(dt)

        try:
            # ensure the chosen FPC is active; deactivate conflicting controllers
            # on the same joints (best-effort) and remember them to restore.
            resp = self._list_controllers_sync()
            if resp is not None:
                jset = set(order)
                for c in resp.controller:
                    if c.name == ctrl:
                        continue
                    if (c.state == "active" and
                            any(ci.rpartition("/")[0] in jset
                                for ci in list(getattr(c, "required_command_interfaces", []) or []))):
                        deactivated.append(c.name)
            self._switch(activate=[ctrl], deactivate=deactivated)
            time.sleep(0.3)
            # seed FPC with the baseline so it owns a setpoint at the current pose
            for _ in range(20):
                publish(list(base_vec)); time.sleep(dt)

            plan = L.build_plan(params)
            seg_total = len(sel) * len(plan)
            done = 0
            for j in sel:
                if self._stop_flag.is_set():
                    break
                jx = jmap[j]
                ramp_to(list(base_vec), 1.0)
                for seg in plan:
                    if self._stop_flag.is_set():
                        break
                    with self._lock:
                        self._progress = {"joint": j, "segment": seg.name,
                                          "i": done, "n": seg_total}
                        self._status_msg = f"{j}: {seg.name}"
                        # start each segment with a clean live trace -- otherwise
                        # every segment's per-segment time (0..duration) overlays
                        # on the same x-range and the plot turns into a mess.
                        self._live = []
                    rec_t, rec_cmd, rec_act, rec_f = [], [], [], []
                    lp = L.TwoPoleLowPass(seg.lp_hz, dt, x0=0.0)
                    n = int(seg.duration * self._send_rate)
                    t0 = time.monotonic()
                    for k in range(n):
                        if self._stop_flag.is_set():
                            break
                        off = L.waveform_offset(seg, k, dt, params.amp, params.freq,
                                                params.stair_step)
                        off = lp.step(off)
                        vec = list(base_vec); vec[jx] = base_vec[jx] + off
                        publish(vec)
                        tnow = time.monotonic() - t0
                        a = actual(j)
                        w = cur_wrench()
                        rec_t.append(tnow)
                        rec_cmd.append(off)
                        rec_act.append((a - base_vec[jx]) if a is not None else float("nan"))
                        rec_f.append(w if w is not None else (float("nan"),) * 3)
                        # throttle live plot to ~50 Hz
                        if k % max(1, int(self._send_rate / 50)) == 0:
                            scale = 1000.0 if is_pris.get(j) else RAD2DEG
                            fhf = (math.sqrt(sum(x * x for x in w)) if w else None)
                            with self._lock:
                                self._live.append([round(tnow, 4),
                                                   round(off * scale, 4),
                                                   round((rec_act[-1]) * scale, 4)
                                                   if rec_act[-1] == rec_act[-1] else None,
                                                   round(fhf, 3) if fhf is not None else None])
                                if len(self._live) > 6000:
                                    self._live = self._live[-4000:]
                        time.sleep(max(0.0, t0 + (k + 1) * dt - time.monotonic()))
                    # analyse (convert to deg/mm for readability)
                    scale = 1000.0 if is_pris.get(j) else RAD2DEG
                    t = np.array(rec_t)
                    cmd = np.array(rec_cmd) * scale
                    act = np.array(rec_act) * scale
                    force = np.array(rec_f) if self._wrench_topic else None
                    res = L.analyze_segment(t, cmd, act, params.freq,
                                            self._send_rate, force=force, kind=seg.kind)
                    res.update({"joint": j, "segment": seg.name,
                                "unit": "mm" if is_pris.get(j) else "deg"})
                    with self._lock:
                        self._results.append(res)
                    records.append({
                        "joint": j, "segment": seg.name, "kind": seg.kind,
                        "unit": "mm" if is_pris.get(j) else "deg",
                        "fs": self._send_rate,
                        "t": t, "cmd": cmd, "act": act,
                        "force": (force if force is not None else None),
                        "metrics": dict(res),
                    })
                    done += 1
                ramp_to(list(base_vec), 1.0)

            with self._lock:
                self._status = "stopped" if self._stop_flag.is_set() else "done"
                self._status_msg = ("stopped by operator" if self._stop_flag.is_set()
                                    else "completed")
                self._progress["i"] = self._progress["n"]
            # persist the run (plots + self-contained HTML) for later review
            if records:
                try:
                    info = R.save_run(self._report_dir, dict(self._run_meta), records)
                    with self._lock:
                        self._last_report = info
                        self._status_msg += f"  ·  report saved: {info['id']}"
                    self.get_logger().info(f"report saved -> {info['html']}")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"report save failed: {exc}")
                    with self._lock:
                        self._status_msg += f"  ·  report save failed: {exc}"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"run failed: {exc}")
            with self._lock:
                self._status = "error"
                self._status_msg = str(exc)
        finally:
            # ramp back + restore the controllers we deactivated
            try:
                ramp_to(list(base_vec), 1.0)
            except Exception:  # noqa: BLE001
                pass
            if deactivated:
                self._switch(activate=deactivated, deactivate=[ctrl])

    # ---- HTTP -------------------------------------------------------------
    def _start_http(self) -> None:
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_a):  # noqa: N802
                return

            def _json(self, status, payload):
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def _static(self, rel):
                target = (_STATIC_DIR / rel).resolve()
                try:
                    target.relative_to(_STATIC_DIR)
                except ValueError:
                    self._json(403, {"error": "forbidden"}); return
                if not target.is_file():
                    self._json(404, {"error": "not found"}); return
                mime = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
                body = target.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _send_file(self, target, mime, download=None):
                body = target.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                if download:
                    self.send_header("Content-Disposition",
                                     f'attachment; filename="{download}"')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802
                path = urlparse(self.path).path
                if path == "/" or path == "/index.html":
                    self._static("index.html")
                elif path.startswith("/static/"):
                    self._static(path[len("/static/"):])
                elif path == "/api/info":
                    self._json(200, dash.api_info())
                elif path == "/api/state":
                    self._json(200, dash.api_state())
                elif path == "/api/runs":
                    self._json(200, dash.api_runs())
                elif path.startswith("/api/runs/"):
                    parts = path[len("/api/runs/"):].split("/", 1)
                    if len(parts) != 2:
                        self._json(404, {"error": "not found"}); return
                    run_id, fname = parts
                    target = R.resolve_run_file(dash._report_dir, run_id, fname)
                    if target is None:
                        self._json(404, {"error": "not found"}); return
                    if fname == "report.html":
                        self._send_file(target, "text/html; charset=utf-8")
                    elif fname == "run.npz":
                        self._send_file(target, "application/octet-stream",
                                        download=f"{run_id}.npz")
                    else:
                        self._send_file(target, "application/json")
                else:
                    self._json(404, {"error": "not found"})

            def do_POST(self):  # noqa: N802
                path = urlparse(self.path).path
                length = int(self.headers.get("Content-Length", "0") or "0")
                raw = self.rfile.read(length) if length else b""
                try:
                    body = json.loads(raw.decode("utf-8")) if raw else {}
                except Exception:  # noqa: BLE001
                    self._json(400, {"error": "invalid JSON"}); return
                if path == "/api/run":
                    ok, msg = dash.start_run(body)
                    self._json(200 if ok else 409, {"ok": ok, "message": msg})
                elif path == "/api/stop":
                    dash.stop_run()
                    self._json(200, {"ok": True, "message": "stopping"})
                else:
                    self._json(404, {"error": "not found"})

        self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
        t = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = FpcTestDashboard()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_run()
        except Exception:  # noqa: BLE001
            pass
        if node._httpd is not None:
            node._httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
