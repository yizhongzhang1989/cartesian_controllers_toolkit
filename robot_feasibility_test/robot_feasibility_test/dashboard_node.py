"""Robot feasibility test DASHBOARD -- optional web UI; a thin client of the engine.

This process holds **no** test logic and **never** touches the robot directly.
It is a pure client of ``feasibility_test_engine``'s ROS API:

* subscribes the engine's ``~/status`` (std_msgs/String JSON) and relays it to
  the browser (``/api/info`` + ``/api/state``);
* on ``POST /api/run`` it sets the engine's ``run_config`` parameter and calls
  the engine ``~/run`` service; ``POST /api/stop`` calls ``~/stop``;
* serves the saved reports straight from ``report_dir`` on disk.

Because it is decoupled, the dashboard is **optional** (the engine runs the full
battery headless without it) and **independent** (start/stop/reload it, or run it
on another host, without interrupting a run). It degrades gracefully when the
engine is absent ("engine not connected") and reconnects when it reappears.

Parameters
----------
  host         string  default "0.0.0.0"
  port         int     default 8140
  engine_node  string  default "feasibility_test_engine"  (engine node to target)
  report_dir   string  default "~/.ros/robot_feasibility_test/runs"
"""
from __future__ import annotations

import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urlparse

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from . import report as R

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_STALE_S = 3.0   # engine status older than this => treated as offline


class FeasibilityTestDashboard(Node):
    def __init__(self):
        super().__init__("feasibility_test_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8140)
        self.declare_parameter("engine_node", "feasibility_test_engine")
        self.declare_parameter("report_dir", "~/.ros/robot_feasibility_test/runs")

        gp = self.get_parameter
        self._host = gp("host").value
        self._port = int(gp("port").value)
        self._engine = str(gp("engine_node").value).strip("/")
        self._report_dir = str(Path(gp("report_dir").value).expanduser())

        self._lock = threading.Lock()
        self._status: Optional[Dict[str, Any]] = None
        self._status_mono: float = 0.0

        self._cbg = ReentrantCallbackGroup()
        # The engine publishes ~/status as /<engine_node>/status.
        self.create_subscription(
            String, f"/{self._engine}/status", self._on_status, 10,
            callback_group=self._cbg)
        self._cli_run = self.create_client(
            Trigger, f"/{self._engine}/run", callback_group=self._cbg)
        self._cli_stop = self.create_client(
            Trigger, f"/{self._engine}/stop", callback_group=self._cbg)
        self._cli_setparam = self.create_client(
            SetParameters, f"/{self._engine}/set_parameters",
            callback_group=self._cbg)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._start_http()
        self.get_logger().info(
            f"feasibility_test_dashboard (thin client of '{self._engine}') on "
            f"http://{self._host}:{self._port}  report_dir={self._report_dir}")

    # ---- engine status ----------------------------------------------------
    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._status = data
            self._status_mono = time.monotonic()

    def _cached_status(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._status is None:
                return None
            if (time.monotonic() - self._status_mono) > _STALE_S:
                return None
            return dict(self._status)

    def _offline_payload(self) -> Dict[str, Any]:
        return {
            "joints": [], "controllers": [], "wrench_available": False,
            "report_dir": self._report_dir,
            "defaults": {"amp_deg": 4.0, "freq_hz": 0.2, "seg_seconds": 9.0,
                         "stair_step_deg": 1.5, "lp_hz": 10.0, "limit_deg": 8.0,
                         "tests": ["smooth", "stair", "smoothed"]},
            "status": "engine_offline",
            "message": f"engine '{self._engine}' not connected",
            "progress": {"joint": "", "segment": "", "i": 0, "n": 0},
            "results": [], "meta": {}, "js_age": None, "live": [],
            "last_report": None,
        }

    def api_status(self, full: bool = True) -> Dict[str, Any]:
        """Status payload served to clients.

        ``full=True`` (``/api/info``) returns everything including the static
        catalogue (joints, controllers, defaults). ``full=False`` (``/api/state``,
        polled several times a second) drops that static catalogue and trims the
        live trace, so the frequently-parsed payload stays small -- important when
        the browser renderer is competing for CPU on a busy host.
        """
        s = self._cached_status()
        if s is None:
            s = self._offline_payload()
        if full:
            return s
        # lightweight per-poll view: keep only what the live UI needs
        live = s.get("live") or []
        if len(live) > 300:
            live = live[-300:]
        return {
            "status": s.get("status"),
            "message": s.get("message"),
            "progress": s.get("progress", {}),
            "results": s.get("results", []),
            "js_age": s.get("js_age"),
            "wrench_available": s.get("wrench_available", False),
            "live": live,
            "last_report": s.get("last_report"),
        }

    def api_runs(self) -> Dict[str, Any]:
        try:
            runs = R.list_runs(self._report_dir)
        except Exception as exc:  # noqa: BLE001
            return {"report_dir": self._report_dir, "runs": [], "error": str(exc)}
        return {"report_dir": self._report_dir, "runs": runs}

    # ---- engine control (ROS) --------------------------------------------
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

    def start_run(self, cfg: Dict[str, Any]) -> Tuple[bool, str]:
        # 1) set the engine's run_config parameter to the JSON config.
        if not self._cli_setparam.wait_for_service(timeout_sec=2.0):
            return False, f"engine '{self._engine}' not reachable (set_parameters)"
        req = SetParameters.Request()
        pv = ParameterValue(type=ParameterType.PARAMETER_STRING,
                            string_value=json.dumps(cfg))
        req.parameters = [Parameter(name="run_config", value=pv)]
        resp = self._wait(self._cli_setparam.call_async(req), 3.0)
        if resp is None or not resp.results or not resp.results[0].successful:
            return False, "failed to set run_config on the engine"
        # 2) call the engine ~/run service (validates + starts; returns result).
        if not self._cli_run.wait_for_service(timeout_sec=2.0):
            return False, f"engine '{self._engine}' run service unavailable"
        r = self._wait(self._cli_run.call_async(Trigger.Request()), 5.0)
        if r is None:
            return False, "engine did not respond to ~/run"
        return bool(r.success), r.message

    def stop_run(self) -> Tuple[bool, str]:
        if not self._cli_stop.wait_for_service(timeout_sec=2.0):
            return False, "engine stop service unavailable"
        r = self._wait(self._cli_stop.call_async(Trigger.Request()), 3.0)
        if r is None:
            return False, "engine did not respond to ~/stop"
        return bool(r.success), r.message

    # ---- HTTP -------------------------------------------------------------
    def _start_http(self) -> None:
        dash = self

        class Handler(BaseHTTPRequestHandler):
            # Keep connections alive: the dashboard polls /api/state at 10 Hz
            # (plus /api/info and /api/runs), so without HTTP/1.1 keep-alive every
            # request would open a fresh TCP connection that is then closed,
            # piling up hundreds of TIME_WAIT sockets and exhausting the browser's
            # per-host connection pool -> fetches stall and the UI shows
            # "disconnected" even though the server is healthy. HTTP/1.1 + a
            # correct Content-Length on every response lets the browser reuse a
            # couple of persistent connections instead.
            protocol_version = "HTTP/1.1"
            timeout = 30  # close idle kept-alive connections so threads are freed

            def log_message(self, *_a):  # noqa: N802
                return

            def _json(self, status, payload):
                # Sanitise NaN/Infinity -> null so the response is strictly valid
                # JSON for the browser (json.dumps would otherwise emit bare NaN
                # tokens that fetch().json() rejects, breaking the whole page).
                body = json.dumps(R.json_safe(payload)).encode("utf-8")
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
                    self._json(200, dash.api_status(full=True))
                elif path == "/api/state":
                    self._json(200, dash.api_status(full=False))
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
                    ok, msg = dash.stop_run()
                    self._json(200, {"ok": ok, "message": msg})
                else:
                    self._json(404, {"error": "not found"})

        self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
        t = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = FeasibilityTestDashboard()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._httpd is not None:
            node._httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
