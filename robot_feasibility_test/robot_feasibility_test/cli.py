"""feasibility_test -- headless CLI front-end for the robot feasibility test engine.

Drives ``feasibility_test_engine`` over its ROS API with **no** web server: sets
the ``run_config`` parameter, calls the ``~/run`` service, then follows
``~/status`` until the battery finishes -- for CI, scripted multi-robot
characterisation and autonomous sessions.

Examples
--------
  # run the default battery on two joints of the right arm and wait for it
  ros2 run robot_feasibility_test feasibility_test \\
      --controller right_arm_forward_position_controller \\
      --joints right_arm_joint2,right_arm_joint4

  # custom params, then exit when done
  ros2 run robot_feasibility_test feasibility_test --controller <ctrl> --joints j2 \\
      --tests smooth,stair,smoothed --amp 4 --freq 0.2 --limit 8

  # just stop a run in progress
  ros2 run robot_feasibility_test feasibility_test --stop
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class _Client(Node):
    def __init__(self, engine: str):
        super().__init__("feasibility_test_cli")
        self._engine = engine.strip("/")
        self._status: Optional[Dict[str, Any]] = None
        self.create_subscription(String, f"/{self._engine}/status",
                                 self._on_status, 10)
        self.cli_run = self.create_client(Trigger, f"/{self._engine}/run")
        self.cli_stop = self.create_client(Trigger, f"/{self._engine}/stop")
        self.cli_set = self.create_client(
            SetParameters, f"/{self._engine}/set_parameters")

    def _on_status(self, msg: String) -> None:
        try:
            self._status = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            pass

    def status(self) -> Optional[Dict[str, Any]]:
        return self._status

    def _call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done():
                return fut.result()
        return None

    def set_run_config(self, cfg: Dict[str, Any]) -> bool:
        req = SetParameters.Request()
        pv = ParameterValue(type=ParameterType.PARAMETER_STRING,
                            string_value=json.dumps(cfg))
        req.parameters = [Parameter(name="run_config", value=pv)]
        r = self._call(self.cli_set, req)
        return bool(r and r.results and r.results[0].successful)

    def run(self):
        return self._call(self.cli_run, Trigger.Request())

    def stop(self):
        return self._call(self.cli_stop, Trigger.Request())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="feasibility_test", description=__doc__)
    ap.add_argument("--engine", default="feasibility_test_engine")
    ap.add_argument("--controller", default="")
    ap.add_argument("--joints", default="", help="comma-separated joint names")
    ap.add_argument("--tests", default="smooth,stair,smoothed",
                    help="comma list of: smooth,stair,smoothed,step,resonance,sweep")
    ap.add_argument("--amp", type=float, default=4.0)
    ap.add_argument("--freq", type=float, default=0.2)
    ap.add_argument("--seg", type=float, default=9.0)
    ap.add_argument("--stair-step", type=float, default=1.5)
    ap.add_argument("--lp", type=float, default=10.0)
    ap.add_argument("--limit", type=float, default=8.0)
    ap.add_argument("--resonance-deg", type=float, default=2.0,
                    help="amplitude of the small resonance tap (deg)")
    ap.add_argument("--sweep-deg", type=float, default=1.5,
                    help="amplitude of the frequency-sweep chirp (deg)")
    ap.add_argument("--sweep-band", default="0.3,6.0",
                    help="chirp band 'f0,f1' in Hz")
    ap.add_argument("--timeout", type=float, default=600.0,
                    help="max seconds to wait for completion")
    ap.add_argument("--stop", action="store_true", help="stop a run and exit")
    args = ap.parse_args(argv)

    rclpy.init()
    cli = _Client(args.engine)
    rc = 0
    try:
        if args.stop:
            r = cli.stop()
            print(r.message if r else "engine did not respond")
            return 0 if (r and r.success) else 1

        if not args.controller or not args.joints:
            print("error: --controller and --joints are required", file=sys.stderr)
            return 2

        cfg = {
            "controller": args.controller,
            "joints": [j for j in args.joints.split(",") if j],
            "tests": [t for t in args.tests.split(",") if t],
            "amp_deg": args.amp, "freq_hz": args.freq, "seg_seconds": args.seg,
            "stair_step_deg": args.stair_step, "lp_hz": args.lp,
            "limit_deg": args.limit, "resonance_deg": args.resonance_deg,
            "sweep_deg": args.sweep_deg,
        }
        try:
            f0s, f1s = (args.sweep_band.split(",") + ["", ""])[:2]
            cfg["sweep_f0_hz"] = float(f0s)
            cfg["sweep_f1_hz"] = float(f1s)
        except (ValueError, AttributeError):
            pass
        print(f"[feasibility_test] engine='{args.engine}' config={cfg}")
        if not cli.set_run_config(cfg):
            print("error: could not set run_config on the engine "
                  "(is it running?)", file=sys.stderr)
            return 1
        r = cli.run()
        if r is None:
            print("error: engine did not respond to ~/run", file=sys.stderr)
            return 1
        print(f"[feasibility_test] run: ok={r.success} message='{r.message}'")
        if not r.success:
            return 1

        # follow status until the run leaves "running"
        end = time.monotonic() + args.timeout
        last_prog = None
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(cli, timeout_sec=0.1)
            st = cli.status()
            if not st:
                continue
            prog = st.get("progress", {})
            tag = (prog.get("joint", ""), prog.get("segment", ""),
                   prog.get("i", 0), prog.get("n", 0))
            if tag != last_prog and tag[1]:
                print(f"[feasibility_test] {tag[0]} {tag[1]} ({tag[2]}/{tag[3]})")
                last_prog = tag
            status = st.get("status")
            if status in ("done", "stopped", "error"):
                print(f"[feasibility_test] {status}: {st.get('message', '')}")
                rep = st.get("last_report")
                if rep:
                    print(f"[feasibility_test] report: {rep.get('html', rep.get('id'))}")
                rc = 0 if status == "done" else 1
                break
        else:
            print("[feasibility_test] timed out waiting for completion", file=sys.stderr)
            rc = 1
    finally:
        cli.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
