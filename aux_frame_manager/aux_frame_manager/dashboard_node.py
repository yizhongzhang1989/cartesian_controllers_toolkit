#!/usr/bin/env python3
"""Optional web dashboard for ``aux_frame_manager`` — a thin HTTP/ROS client.

Launched automatically when ``aux_frame_manager`` is started with a
``dashboard_port`` argument. It is a *client* of the running manager node (it
does not import the manager's internals):

  * subscribes the **canonical** URDF topic (the manager's output) and the
    **base** URDF topic (the manufacturer URDF) — the difference of their link
    sets is exactly the set of AUX (added) frames;
  * subscribes the manager's ``~/status``;
  * publishes the manager's ``~/set_aux_frames`` (JSON) to replace the whole
    added-frame list, and ``~/edit_frame`` (JSON) to edit ONE frame's offset —
    including a frame already baked into the launch URDF, so EVERY fixed frame
    is editable here, not only the ones the manager appended;
  * reads each link's world transform from **TF** (``base_frame -> link``) so
    the 3D canvas shows the robot at its live pose with no server-side FK
    (mirrors the cartesian_controller_dashboard approach; no Pinocchio).

The **3D canvas** (Three.js, mirroring the ikt_pose_commander viewer) renders
the URDF meshes and clearly distinguishes ORIGINAL links (neutral) from ADDED
aux frames (highlighted markers + triads + labels), so you can see at a glance
what the manager injected and where.

ThreadingHTTPServer in a daemon thread; rclpy on a MultiThreadedExecutor with a
ReentrantCallbackGroup so synchronous work from the HTTP thread never deadlocks
the ROS spin. Default port 8160 (8080/8100/8120/8140/8180 are used by the other
toolkit dashboards).
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

try:
    import tf2_ros
    _HAVE_TF = True
except Exception:  # pragma: no cover
    _HAVE_TF = False

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None  # type: ignore

_STATIC_DIR = Path(__file__).resolve().parent / "static"


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


def parse_aux_definitions(urdf_xml: str, aux_links: List[str]) -> List[dict]:
    """Recover ``{name, parent, xyz, rpy}`` for each aux link from the fixed
    joint whose child is that link."""
    defs: Dict[str, dict] = {}
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return []
    aux = set(aux_links)
    for j in root.findall("joint"):
        c = j.find("child")
        p = j.find("parent")
        if c is None or p is None:
            continue
        child = c.get("link")
        if child not in aux:
            continue
        origin = j.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(x) for x in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(x) for x in origin.get("rpy").split()]
        defs[child] = {"name": child, "parent": p.get("link"),
                       "xyz": xyz, "rpy": rpy}
    return [defs[k] for k in aux_links if k in defs]


def parse_fixed_frames(urdf_xml: str) -> List[dict]:
    """Recover ``{name, parent, xyz, rpy}`` for EVERY link held by a fixed joint.

    These are all the frames whose static offset can be edited -- the ones this
    manager appended AND the ones already baked into the launch URDF. Links on a
    movable joint are skipped (their pose comes from joint state)."""
    out: List[dict] = []
    try:
        root = ET.fromstring(urdf_xml)
    except Exception:
        return out
    for j in root.findall("joint"):
        if j.get("type") != "fixed":
            continue
        c = j.find("child")
        p = j.find("parent")
        if c is None or p is None or not c.get("link") or not p.get("link"):
            continue
        origin = j.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(x) for x in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(x) for x in origin.get("rpy").split()]
        out.append({"name": c.get("link"), "parent": p.get("link"),
                    "xyz": xyz, "rpy": rpy})
    return out


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


class AuxFrameDashboard(Node):
    def __init__(self) -> None:
        super().__init__("aux_frame_dashboard")
        self.declare_parameter("port", 8160)
        self.declare_parameter("manager_ns", "/aux_frame_manager")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("canonical_topic", "/cartesian/robot_description")
        self.declare_parameter("base_urdf_topic", "/robot_description")

        self._port = int(self.get_parameter("port").value)
        self._ns = str(self.get_parameter("manager_ns").value).rstrip("/")
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._canon_topic = str(self.get_parameter("canonical_topic").value)
        self._base_topic = str(self.get_parameter("base_urdf_topic").value)
        self._host = "0.0.0.0"

        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._canon_urdf = ""
        self._base_urdf = ""
        self._base_links: List[str] = []
        self._links: List[str] = []
        self._aux_links: List[str] = []
        self._visuals: List[dict] = []
        self._joint_tree: List[dict] = []
        self._aux_defs: List[dict] = []
        # Every editable (fixed-joint) frame, each tagged source=added|base.
        self._editable: List[dict] = []
        self._status: Optional[dict] = None
        self._status_aux: List[str] = []
        self._status_stamp = 0.0
        self._pkg_dirs: Dict[str, Optional[str]] = {}

        self.create_subscription(String, self._canon_topic, self._on_canon,
                                 _latched_qos(), callback_group=self._cbg)
        self.create_subscription(String, self._base_topic, self._on_base,
                                 _latched_qos(), callback_group=self._cbg)
        # status is latched by the manager (TRANSIENT_LOCAL) — match it so we
        # get the last value on (re)connect
        self.create_subscription(String, f"{self._ns}/status", self._on_status,
                                 _latched_qos(), callback_group=self._cbg)
        # publisher to drive the manager's live editor (the test interface)
        self._set_pub = self.create_publisher(
            String, f"{self._ns}/set_aux_frames", 10)
        # publisher to edit a single frame's offset (added OR pre-existing)
        self._edit_pub = self.create_publisher(
            String, f"{self._ns}/edit_frame", 10)

        if _HAVE_TF:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None

        self._httpd = None
        self._start_http()
        self.get_logger().info(
            "aux_frame_dashboard: http://localhost:%d  (manager=%s, base=%s, "
            "canonical=%s)" % (self._port, self._ns, self._base_topic,
                               self._canon_topic))

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_base(self, msg: String) -> None:
        if not msg.data:
            return
        with self._lock:
            self._base_urdf = msg.data
            self._base_links = parse_link_names(msg.data)
        self._recompute_aux()

    def _on_canon(self, msg: String) -> None:
        if not msg.data:
            return
        with self._lock:
            self._canon_urdf = msg.data
            self._links = parse_link_names(msg.data)
            self._visuals = parse_visuals(msg.data)
            self._joint_tree = parse_joint_tree(msg.data)
        self._recompute_aux()

    def _recompute_aux(self) -> None:
        with self._lock:
            canon = set(self._links)
            base = set(self._base_links)
            # Prefer the manager's authoritative aux-frame names (robust even
            # when the manager mirrors the canonical URDF back onto the base
            # topic, which would otherwise make the link-set diff empty).
            status_aux = [n for n in self._status_aux if n in canon]
            if status_aux:
                self._aux_links = status_aux
            else:
                # fallback: links present in canonical but not in the base URDF
                self._aux_links = [ln for ln in self._links if ln not in base]
            self._aux_defs = parse_aux_definitions(self._canon_urdf,
                                                   self._aux_links)
            # Every fixed frame is editable; tag each by origin so the UI can
            # show added vs pre-existing (baked into the launch URDF) frames.
            added = set(self._aux_links)
            self._editable = [
                {**fr, "source": "added" if fr["name"] in added else "base"}
                for fr in parse_fixed_frames(self._canon_urdf)]

    def _on_status(self, msg: String) -> None:
        try:
            s = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._status = s
            self._status_stamp = time.monotonic()
            # the manager reports the authoritative list of frame names it added
            aux = s.get("aux_frames")
            if isinstance(aux, list):
                self._status_aux = [str(a) for a in aux]
        self._recompute_aux()

    # ------------------------------------------------------------------ #
    # TF -> per-link world transforms (4x4 row-major nested lists)
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
            t = tf.transform.translation
            r = tf.transform.rotation
            R = _quat_to_R(r.x, r.y, r.z, r.w)
            out[link] = [
                [R[0][0], R[0][1], R[0][2], t.x],
                [R[1][0], R[1][1], R[1][2], t.y],
                [R[2][0], R[2][1], R[2][2], t.z],
                [0.0, 0.0, 0.0, 1.0],
            ]
        return out

    # ------------------------------------------------------------------ #
    # Mesh resolution / serving
    # ------------------------------------------------------------------ #
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
    # Snapshot for the 3D canvas + panels
    # ------------------------------------------------------------------ #
    def snapshot(self) -> dict:
        with self._lock:
            links = list(self._links)
            base_links = list(self._base_links)
            aux_links = list(self._aux_links)
            visuals = list(self._visuals)
            joint_tree = list(self._joint_tree)
            aux_defs = list(self._aux_defs)
            editable = list(self._editable)
            status = self._status
            age = (time.monotonic() - self._status_stamp
                   if self._status_stamp else None)
        link_tf = self._link_tf()
        return {
            "have_model": bool(links),
            "base_frame": self._base_frame,
            "manager_ns": self._ns,
            "canonical_topic": self._canon_topic,
            "links": links,
            "base_links": base_links,
            "aux_links": aux_links,
            "aux_frames": aux_defs,
            "editable_frames": editable,
            "has_meshes": bool(visuals),
            "visuals": [
                {"link": v["link"], "url": self._mesh_url(v["filename"]),
                 "xyz": v["xyz"], "rpy": v["rpy"], "scale": v["scale"]}
                for v in visuals],
            "joint_tree": joint_tree,
            "link_tf": link_tf,
            "status": status,
            "status_age": round(age, 2) if age is not None else None,
            "n_links": len(links),
            "n_aux": len(aux_links),
        }

    # ------------------------------------------------------------------ #
    # Test interface: live edit via the manager's ~/set_aux_frames
    # ------------------------------------------------------------------ #
    def set_aux_frames(self, frames: list) -> dict:
        """Publish a full frame list to the manager (JSON ~/set_aux_frames).

        The manager validates + rebuilds + republishes; the 3D canvas then
        reflects the change on the next poll.
        """
        if not isinstance(frames, list):
            return {"ok": False, "message": "frames must be a list"}
        # light validation so obvious mistakes get a clear message here
        clean = []
        for f in frames:
            if not isinstance(f, dict) or not f.get("name") or not f.get("parent"):
                return {"ok": False,
                        "message": "each frame needs 'name' and 'parent'"}
            clean.append({
                "name": str(f["name"]), "parent": str(f["parent"]),
                "xyz": [float(v) for v in (f.get("xyz") or [0, 0, 0])],
                "rpy": [float(v) for v in (f.get("rpy") or [0, 0, 0])],
            })
        m = String()
        m.data = json.dumps(clean)
        for _ in range(3):
            self._set_pub.publish(m)
            time.sleep(0.02)
        return {"ok": True,
                "message": "sent %d frame(s) to %s/set_aux_frames"
                           % (len(clean), self._ns),
                "frames": clean}

    def current_frames(self) -> list:
        with self._lock:
            return list(self._aux_defs)

    def edit_frame(self, frame: dict) -> dict:
        """Edit ONE frame's offset via the manager's ``~/edit_frame`` topic.

        Works for a frame this manager added AND for a frame already present in
        the launch URDF (a pre-existing fixed frame). ``parent`` is optional --
        supply it only to create a new frame or re-parent an added one; it is
        ignored when overriding a pre-existing frame's offset.
        """
        if not isinstance(frame, dict) or not frame.get("name"):
            return {"ok": False, "message": "edit needs a 'name'"}
        try:
            payload = {
                "name": str(frame["name"]),
                "xyz": [float(v) for v in (frame.get("xyz") or [0, 0, 0])],
                "rpy": [float(v) for v in (frame.get("rpy") or [0, 0, 0])],
            }
        except (TypeError, ValueError):
            return {"ok": False, "message": "xyz/rpy must be 3 numbers each"}
        if frame.get("parent"):
            payload["parent"] = str(frame["parent"])
        m = String()
        m.data = json.dumps(payload)
        for _ in range(3):
            self._edit_pub.publish(m)
            time.sleep(0.02)
        return {"ok": True,
                "message": "edited '%s' via %s/edit_frame"
                           % (payload["name"], self._ns),
                "frame": payload}

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

            def _read_json(self):
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
                if path == "/api/frames":
                    return self._send(200, json.dumps(
                        {"frames": dash.current_frames()}))
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
                if path == "/api/set_frames":
                    b = self._read_json()
                    return self._send(200, json.dumps(
                        dash.set_aux_frames(b.get("frames", []))))
                if path == "/api/edit_frame":
                    b = self._read_json()
                    return self._send(200, json.dumps(
                        dash.edit_frame(b.get("frame", {}))))
                return self._send(404, '{"error":"not found"}')

            def _serve_static(self, relpath):
                fp = (_STATIC_DIR / relpath).resolve()
                if not str(fp).startswith(str(_STATIC_DIR.resolve())) \
                        or not fp.is_file():
                    return self._send(404, "not found", "text/plain")
                ctype = mimetypes.guess_type(str(fp))[0] or "application/octet-stream"
                if fp.suffix == ".js":
                    ctype = "text/javascript"
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
    node = AuxFrameDashboard()
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
