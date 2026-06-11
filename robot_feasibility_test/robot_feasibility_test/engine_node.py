"""Robot feasibility test ENGINE -- headless ROS node that runs the test battery.

This is the *main task*: it owns the hardware interaction (controller discovery,
JTC<->FPC switching, the command stream), the safety envelope, the metrics
(``test_logic``) and the saved reports (``report``).  It has **no** web server
and runs fine headless (CI, scripted multi-robot characterisation, autonomous
sessions).

It exposes a small ROS API so the *optional* dashboard, a CLI, or any supervisor
can drive and observe it without sharing a process:

* parameter ``run_config`` (JSON string) -- the battery config for the next run.
* service ``~/run``  (std_srvs/Trigger) -- start a run using ``run_config``;
  returns ok/message synchronously (validation happens here).
* service ``~/stop`` (std_srvs/Trigger) -- stop the current run, ramp back.
* topic   ``~/status`` (std_msgs/String, JSON) -- published continuously: the
  discovered joints/controllers, defaults, run status/progress, the live trace
  tail, per-segment results and the last saved report.

The actual test logic (waveforms, metrics) lives in ``test_logic`` (pure Python)
and report rendering in ``report`` -- both shared with the dashboard and CLI.

Parameters
----------
  joint_states_topic string  default "/joint_states"
  wrench_topic       string  default ""   (empty => no force ground-truth)
  controller_manager string  default "/controller_manager"
  controller_name    string  default ""   (pin ONE controller; empty => discover)
  send_rate          double  default 200.0
  default_limit_deg  double  default 8.0
  report_dir         string  default "~/.ros/robot_feasibility_test/runs"
  status_rate        double  default 10.0  (Hz; ~/status publish rate)
  run_config         string  default ""    (JSON config read by ~/run)
"""
from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

try:
    from controller_manager_msgs.srv import ListControllers, SwitchController
    _HAS_CM = True
except Exception:  # noqa: BLE001
    _HAS_CM = False

from . import test_logic as L
from . import report as R

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# Controller plugin types whose ``<controller>/commands`` accepts a
# Float64MultiArray of joint positions -- the forward-position-control variants
# the engine can drive (Duco's ForwardCommandController, the UR driver's
# JointGroupPositionController). Matched case-insensitively as substrings; the
# position-only interface check in ``_on_controllers`` is always enforced too.
_FPC_TYPE_KEYS = ("forward_command_controller", "jointgrouppositioncontroller")


def _smoothstep(a: float) -> float:
    a = 0.0 if a < 0 else (1.0 if a > 1 else a)
    return a * a * (3.0 - 2.0 * a)


# ===========================================================================
class FeasibilityTestEngine(Node):
    def __init__(self):
        super().__init__("feasibility_test_engine")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("wrench_topic", "")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("controller_name", "")
        self.declare_parameter("send_rate", 200.0)
        self.declare_parameter("default_limit_deg", 8.0)
        self.declare_parameter("report_dir", "~/.ros/robot_feasibility_test/runs")
        self.declare_parameter("status_rate", 10.0)
        self.declare_parameter("run_config", "")

        gp = self.get_parameter
        self._js_topic = gp("joint_states_topic").value
        self._wrench_topic = gp("wrench_topic").value
        self._cm_ns = gp("controller_manager").value.rstrip("/")
        self._ctrl_name = str(gp("controller_name").value).strip()
        self._send_rate = float(gp("send_rate").value)
        self._default_limit_deg = float(gp("default_limit_deg").value)
        self._report_dir = str(Path(gp("report_dir").value).expanduser())
        self._status_rate = max(1.0, float(gp("status_rate").value))

        self._lock = threading.Lock()
        self._urdf_joints: List[L.JointInfo] = []
        self._joint_pos: Dict[str, float] = {}
        self._js_mono: float = 0.0
        self._wrench: Optional[Tuple[float, float, float]] = None
        self._wrench_mono: float = 0.0
        self._controllers: List[Dict[str, Any]] = []

        # run state
        self._run_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._status = "idle"           # idle | running | done | error | stopped
        self._status_msg = ""
        self._progress = {"joint": "", "segment": "", "i": 0, "n": 0}
        self._live: List[Any] = []
        self._results: List[Dict[str, Any]] = []
        self._run_meta: Dict[str, Any] = {}
        self._last_report: Optional[Dict[str, Any]] = None

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

        self._cmd_pubs: Dict[str, Any] = {}
        self.create_timer(2.0, self._refresh_controllers, callback_group=self._cbg)

        # ---- ROS API (run/stop services + status topic) -------------------
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_service(Trigger, "~/run", self._srv_run,
                            callback_group=self._cbg)
        self.create_service(Trigger, "~/stop", self._srv_stop,
                            callback_group=self._cbg)
        self.create_timer(1.0 / self._status_rate, self._publish_status,
                         callback_group=self._cbg)

        self.get_logger().info(
            f"feasibility_test_engine ready (headless): js={self._js_topic}, "
            f"wrench={self._wrench_topic or 'none'}, cm={self._cm_ns}"
            + (f", controller={self._ctrl_name}" if self._ctrl_name else "")
            + f"; run via the ~/run service (set the run_config param first) "
            f"or the dashboard. report_dir={self._report_dir}")

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
        want = self._ctrl_name
        found: List[Dict[str, Any]] = []
        named_match = None
        for c in resp.controller:
            if want and c.name != want:
                continue
            if want:
                named_match = c
            ctype = c.type or ""
            if not want and not any(k in ctype.lower() for k in _FPC_TYPE_KEYS):
                continue
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
        if want and not found:
            if named_match is None:
                self.get_logger().warn(
                    f"controller_name='{want}' is not loaded on {self._cm_ns}",
                    throttle_duration_sec=10.0)
            else:
                self.get_logger().warn(
                    f"controller_name='{want}' (type='{named_match.type}') is not "
                    f"drivable: a forward-position controller must command ONLY "
                    f"<joint>/position interfaces", throttle_duration_sec=10.0)
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

    # ---- status payload (published on ~/status as JSON) ------------------
    def _status_payload(self) -> Dict[str, Any]:
        with self._lock:
            js_age = time.monotonic() - self._js_mono if self._js_mono else None
            return {
                # info
                "joints": [j.to_dict() for j in self._urdf_joints],
                "controllers": list(self._controllers),
                "wrench_available": bool(self._wrench_topic),
                "send_rate": self._send_rate,
                "report_dir": self._report_dir,
                "defaults": {
                    "amp_deg": 4.0, "freq_hz": 0.2, "seg_seconds": 9.0,
                    "stair_step_deg": 1.5, "lp_hz": 10.0,
                    "limit_deg": self._default_limit_deg,
                    "resonance_deg": 2.0, "sweep_deg": 1.5,
                    "sweep_f0_hz": 0.3, "sweep_f1_hz": 6.0,
                    "tests": ["smooth", "stair", "smoothed"],
                },
                # state
                "status": self._status,
                "message": self._status_msg,
                "progress": dict(self._progress),
                "results": list(self._results),
                "meta": dict(self._run_meta),
                "js_age": js_age,
                "live": list(self._live[-800:]),
                "last_report": dict(self._last_report) if self._last_report else None,
                "engine_time": time.time(),
            }

    def _publish_status(self) -> None:
        try:
            self._status_pub.publish(
                String(data=json.dumps(R.json_safe(self._status_payload()))))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"status publish failed: {exc}",
                                   throttle_duration_sec=10.0)

    # ---- ROS service handlers --------------------------------------------
    def _srv_run(self, _req, resp):
        raw = str(self.get_parameter("run_config").value or "").strip()
        if not raw:
            resp.success = False
            resp.message = "run_config parameter is empty (set it to a JSON config first)"
            return resp
        try:
            cfg = json.loads(raw)
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = f"run_config is not valid JSON: {exc}"
            return resp
        ok, msg = self.start_run(cfg)
        resp.success = ok
        resp.message = msg
        return resp

    def _srv_stop(self, _req, resp):
        self.stop_run()
        resp.success = True
        resp.message = "stopping"
        return resp

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
        order = cinfo["joints"]
        jmap = {j: i for i, j in enumerate(order)}
        limit = float(cfg.get("limit_deg", self._default_limit_deg)) * DEG2RAD
        # resonance uses a small near-step (default 2 deg, clamped to the limit)
        res_amp = float(cfg.get("resonance_deg", 2.0)) * DEG2RAD
        res_amp = min(res_amp, limit) if limit > 0 else res_amp
        # sweep uses a small chirp amplitude (default 1.5 deg, clamped)
        swp_amp = float(cfg.get("sweep_deg", 1.5)) * DEG2RAD
        swp_amp = min(swp_amp, limit) if limit > 0 else swp_amp
        params = L.TestParams(
            amp=float(cfg.get("amp_deg", 4.0)) * DEG2RAD,
            freq=float(cfg.get("freq_hz", 0.2)),
            seg_seconds=float(cfg.get("seg_seconds", 9.0)),
            send_rate=self._send_rate,
            stair_step=float(cfg.get("stair_step_deg", 1.5)) * DEG2RAD,
            lp_hz=float(cfg.get("lp_hz", 10.0)),
            resonance_amp=res_amp,
            sweep_amp=swp_amp,
            sweep_f0=float(cfg.get("sweep_f0_hz", 0.3)),
            sweep_f1=float(cfg.get("sweep_f1_hz", 6.0)),
            include=tuple(cfg.get("tests", ["smooth", "stair", "smoothed"])))
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
        records: List[Dict[str, Any]] = []

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
                        # Downsample the live trace to ~25 Hz: it only feeds the
                        # dashboard plot, and a smaller buffer keeps the polled
                        # status payload light so a CPU-starved browser renderer
                        # (busy host) can keep up. The saved report keeps the full
                        # 200 Hz data.
                        if k % max(1, int(self._send_rate / 25)) == 0:
                            scale = 1000.0 if is_pris.get(j) else RAD2DEG
                            fhf = (math.sqrt(sum(x * x for x in w)) if w else None)
                            with self._lock:
                                self._live.append([round(tnow, 4),
                                                   round(off * scale, 4),
                                                   round((rec_act[-1]) * scale, 4)
                                                   if rec_act[-1] == rec_act[-1] else None,
                                                   round(fhf, 3) if fhf is not None else None])
                                if len(self._live) > 2000:
                                    self._live = self._live[-1500:]
                        time.sleep(max(0.0, t0 + (k + 1) * dt - time.monotonic()))
                    scale = 1000.0 if is_pris.get(j) else RAD2DEG
                    t = np.array(rec_t)
                    cmd = np.array(rec_cmd) * scale
                    act = np.array(rec_act) * scale
                    force = np.array(rec_f) if self._wrench_topic else None
                    res = L.analyze_segment(t, cmd, act, params.freq,
                                            self._send_rate, force=force, kind=seg.kind,
                                            meta_f0=seg.f0, meta_f1=seg.f1)
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

            # Persist the run (plots + self-contained HTML) BEFORE flipping to a
            # terminal status, so any client reacting to "done"/"stopped" already
            # sees ``last_report`` (no race between status and the saved report).
            terminal = "stopped" if self._stop_flag.is_set() else "done"
            base_msg = ("stopped by operator" if self._stop_flag.is_set()
                        else "completed")
            if records:
                with self._lock:
                    self._status = "saving"
                    self._status_msg = "saving report..."
                info = None
                try:
                    info = R.save_run(self._report_dir, dict(self._run_meta), records)
                    self.get_logger().info(f"report saved -> {info['html']}")
                    base_msg += f"  ·  report saved: {info['id']}"
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"report save failed: {exc}")
                    base_msg += f"  ·  report save failed: {exc}"
                with self._lock:
                    self._last_report = info
                    self._status = terminal
                    self._status_msg = base_msg
                    self._progress["i"] = self._progress["n"]
            else:
                with self._lock:
                    self._status = terminal
                    self._status_msg = base_msg
                    self._progress["i"] = self._progress["n"]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"run failed: {exc}")
            with self._lock:
                self._status = "error"
                self._status_msg = str(exc)
        finally:
            try:
                ramp_to(list(base_vec), 1.0)
            except Exception:  # noqa: BLE001
                pass
            if deactivated:
                self._switch(activate=deactivated, deactivate=[ctrl])


def main(args=None):
    rclpy.init(args=args)
    node = FeasibilityTestEngine()
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
