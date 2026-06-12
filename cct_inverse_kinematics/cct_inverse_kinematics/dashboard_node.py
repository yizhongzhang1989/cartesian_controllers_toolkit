"""Optional IK web dashboard: a thin HTTP/JSON client of ik_node's ROS API.

Mirrors the cartesian_controller_dashboard / robot_feasibility_test pattern: a
ThreadingHTTPServer in a daemon thread serves the static UI and a small JSON API
that relays to ik_node over its ROS topics (``~/solve_request`` /
``~/solve_response`` / ``~/status``). The solver runs fine without this; the UI
never touches hardware. Default port 8160.
"""

from __future__ import annotations

import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

_STATIC_DIR = Path(__file__).resolve().parent / "static"


class IKDashboard(Node):
    def __init__(self) -> None:
        super().__init__("ik_dashboard")
        self.declare_parameter("port", 8160)
        self.declare_parameter("ik_ns", "/ik_node")
        self._port = int(self.get_parameter("port").value)
        self._ns = self.get_parameter("ik_ns").value or "/ik_node"
        self._host = "0.0.0.0"

        self._cbg = ReentrantCallbackGroup()
        self._status: Optional[dict] = None
        self._last_response: Optional[dict] = None
        self._lock = threading.Lock()

        self.create_subscription(String, f"{self._ns}/status",
                                 self._on_status, 10,
                                 callback_group=self._cbg)
        self.create_subscription(String, f"{self._ns}/solve_response",
                                 self._on_response, 10,
                                 callback_group=self._cbg)
        self._req_pub = self.create_publisher(
            String, f"{self._ns}/solve_request", 10)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._start_http()
        self.get_logger().info(
            "IK dashboard on http://%s:%d  (ik_ns=%s) — UI only, advisory."
            % (self._host, self._port, self._ns))

    def _on_status(self, msg: String) -> None:
        try:
            with self._lock:
                self._status = json.loads(msg.data)
        except Exception:
            pass

    def _on_response(self, msg: String) -> None:
        try:
            with self._lock:
                self._last_response = json.loads(msg.data)
        except Exception:
            pass

    def publish_request(self, payload: dict) -> None:
        m = String()
        m.data = json.dumps(payload)
        self._req_pub.publish(m)

    def snapshot(self) -> dict:
        with self._lock:
            return {"status": self._status, "last_response": self._last_response}

    # ---- HTTP -------------------------------------------------------------
    def _start_http(self) -> None:
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # silence access log
                pass

            def _send(self, code, body, ctype="application/json"):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    return self._serve_static("index.html")
                if path == "/api/state":
                    body = json.dumps(dash.snapshot()).encode()
                    return self._send(200, body)
                if path.startswith("/static/") or path.endswith(
                        (".css", ".js", ".html")):
                    return self._serve_static(Path(path).name)
                return self._send(404, b'{"error":"not found"}')

            def do_POST(self):
                path = urlparse(self.path).path
                if path != "/api/solve":
                    return self._send(404, b'{"error":"not found"}')
                n = int(self.headers.get("Content-Length", 0))
                raw = self.rfile.read(n) if n else b"{}"
                try:
                    req = json.loads(raw or b"{}")
                except Exception as exc:  # noqa: BLE001
                    return self._send(400, json.dumps(
                        {"ok": False, "error": f"bad JSON: {exc}"}).encode())
                dash.publish_request(req)
                # give ik_node a moment to answer, then return the latest response
                deadline = time.time() + 5.0
                rid = req.get("id")
                while time.time() < deadline:
                    snap = dash.snapshot().get("last_response")
                    if snap is not None and (rid is None or snap.get("id") == rid):
                        return self._send(200, json.dumps(snap).encode())
                    time.sleep(0.05)
                return self._send(200, json.dumps(
                    {"ok": False, "error": "no response from ik_node (timeout)"}
                ).encode())

            def _serve_static(self, name):
                fp = _STATIC_DIR / name
                if not fp.is_file():
                    return self._send(404, b"not found", "text/plain")
                ctype = mimetypes.guess_type(str(fp))[0] or "text/plain"
                return self._send(200, fp.read_bytes(), ctype)

        self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def destroy_node(self):
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKDashboard()
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
