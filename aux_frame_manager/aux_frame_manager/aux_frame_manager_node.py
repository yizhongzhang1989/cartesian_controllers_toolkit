#!/usr/bin/env python3
"""aux_frame_manager — single-writer owner of the canonical augmented URDF.

Consumes the manufacturer URDF from the basic bringup (``/robot_description``,
unchanged), appends the configured auxiliary frames (FT-sensor / compliance /
operation-origin links) from a **config file** and/or **direct arguments**, and
publishes the single canonical URDF on a **latched** topic that the FZI
Cartesian controllers read as their ONLY URDF source (consistency). Optionally
also pushes the canonical URDF to ``robot_state_publisher`` so TF/RViz share one
source of truth.

This node never commands the robot. It is the sole writer of the canonical
topic; running a second writer would defeat the consistency guarantee.

Live edits: publish a JSON frame list on ``~/set_aux_frames`` (std_msgs/String)
to replace the whole managed aux-frame list (offsets, add/remove). To edit just
ONE frame's offset -- including a frame already baked into the launch URDF --
publish ``{name, xyz, rpy}`` on ``~/edit_frame``; pre-existing fixed frames are
rewritten in place (offset override) so *every* fixed frame is editable, not
only the ones this node appended. The canonical URDF is rebuilt and re-published
(and re-pushed to RSP). Status is published on ``~/status``.
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

from aux_frame_manager.frame_source import (apply_overrides, build_partial_canonical,
                                            link_names, list_fixed_frames,
                                            merge_frames, parse_inline_frames,
                                            parse_spec_string, strip_aux_frames)


def _vec3(value) -> List[float]:
    """Coerce an optional 3-sequence to ``[x, y, z]`` floats (default zeros)."""
    if value is None:
        return [0.0, 0.0, 0.0]
    out = [float(v) for v in value]
    if len(out) != 3:
        raise ValueError(f"expected 3 numbers, got {value!r}")
    return out


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class AuxFrameManager(Node):
    def __init__(self) -> None:
        super().__init__("aux_frame_manager")

        # --- parameters --------------------------------------------------
        self.declare_parameter("base_urdf_topic", "/robot_description")
        self.declare_parameter("output_topic", "/cartesian/robot_description")
        self.declare_parameter("update_robot_state_publisher", True)
        self.declare_parameter("robot_state_publisher_name", "robot_state_publisher")
        # Frame sources (Req 1): a config-file section + an inline override.
        self.declare_parameter("config_file", "")          # "" -> auto-resolve
        # Direct-argument frames (Req 1): rcl-safe compact specs
        # 'name:parent[:x,y,z[:r,p,yw]]' separated by ';'. A bracketed YAML/JSON
        # string does NOT survive the rcl param parser, so this compact form is
        # used for the CLI/launch/params-file argument.
        self.declare_parameter("aux_frames", "")           # inline compact specs
        # Startup watchdog: if no canonical URDF is published within this many
        # seconds, log a loud, actionable error (wrong base topic, or aux frames
        # whose parent links are absent from THIS robot's URDF). 0 disables it.
        self.declare_parameter("startup_timeout", 10.0)

        self._base_topic = str(self.get_parameter("base_urdf_topic").value)
        self._out_topic = str(self.get_parameter("output_topic").value)
        self._push_rsp = bool(self.get_parameter("update_robot_state_publisher").value)
        self._rsp_name = str(self.get_parameter("robot_state_publisher_name").value)
        self._startup_timeout = float(self.get_parameter("startup_timeout").value)

        self._frames: List[Dict] = self._load_frames()
        # Offset overrides for frames that ALREADY exist in the incoming URDF
        # (e.g. aux frames baked into the launch URDF, or any manufacturer fixed
        # frame): name -> {xyz, rpy}. These are NOT stripped/re-augmented like
        # self._frames; their existing fixed joint's <origin> is rewritten in
        # place on every build, so any fixed frame is editable -- not only the
        # ones this manager appended. Re-applied each rebuild, so they persist
        # across base-URDF refreshes (an RSP echo or a bringup re-publish).
        self._overrides: Dict[str, Dict] = {}
        # The latest manufacturer base URDF, with every managed aux frame
        # stripped off (recovered from each incoming /robot_description). Live
        # edits rebuild from THIS, so a remove/rename can't strand a stale frame.
        self._base_urdf: str = ""
        # Every aux-frame name this manager has ever added -- stripped from each
        # incoming URDF (incl. an RSP echo of our own output) so removed/renamed
        # frames don't survive.
        self._managed_names = set(f["name"] for f in self._frames)
        self._canonical: str = ""
        # Per-frame build diagnostics for ~/status / the dashboard:
        # [{name, parent, xyz, rpy, valid, error}]. Invalid frames are reported
        # but NOT fatal -- the base + valid frames are still published.
        self._frame_diags: List[Dict] = []
        self._diag_sig: str = ""       # signature to skip redundant status pubs
        # Startup-diagnostics state (see _on_startup_timeout).
        self._got_base = False        # a non-empty base URDF has arrived
        self._published = False       # a canonical URDF has been published
        self._last_error = ""         # last build/parse failure, for the watchdog

        # --- pubs / subs / services -------------------------------------
        self._pub = self.create_publisher(String, self._out_topic, latched_qos())
        self._status_pub = self.create_publisher(String, "~/status", latched_qos())
        self.create_subscription(String, self._base_topic, self._on_base_urdf,
                                 latched_qos())
        self.create_subscription(String, "~/set_aux_frames", self._on_set_frames, 10)
        # Edit a SINGLE frame's offset (added OR pre-existing) by name.
        self.create_subscription(String, "~/edit_frame", self._on_edit_frame, 10)

        self._cli_rsp: Optional[rclpy.client.Client] = None
        if self._push_rsp:
            self._cli_rsp = self.create_client(
                SetParameters, f"/{self._rsp_name}/set_parameters")

        self.get_logger().info(
            "aux_frame_manager up: base='%s' -> canonical='%s' (%d aux frames: %s)"
            "%s. Sole writer of the canonical URDF."
            % (self._base_topic, self._out_topic, len(self._frames),
               ", ".join(f["name"] for f in self._frames) or "-",
               "; mirroring to /%s" % self._rsp_name if self._push_rsp else ""))
        self._publish_status("initialised; waiting for base URDF on %s"
                             % self._base_topic)

        # Surface silent failures (no base URDF, or aux frames that don't fit
        # THIS robot's URDF) as a loud error at launch, not an endless wait.
        self._startup_timer = None
        if self._startup_timeout > 0.0:
            self._startup_timer = self.create_timer(
                self._startup_timeout, self._on_startup_timeout)

    # ------------------------------------------------------------------ #
    # Frame loading (config file + inline argument)
    # ------------------------------------------------------------------ #
    def _load_frames(self) -> List[Dict]:
        file_frames: List[Dict] = []
        cfg_path = str(self.get_parameter("config_file").value or "")
        try:
            from cct_common.config_manager import get_config, read_aux_frames
            if not cfg_path:
                cfg = get_config()
                cfg_path = str(getattr(cfg, "config_path", "") or "")
            if cfg_path:
                file_frames = read_aux_frames(cfg_path, "aux_frame_manager")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("could not read aux_frames from config: %r" % exc)

        inline = str(self.get_parameter("aux_frames").value or "")
        try:
            arg_frames = parse_spec_string(inline)
        except ValueError as exc:
            self.get_logger().error("invalid inline aux_frames: %s" % exc)
            arg_frames = []

        try:
            merged = merge_frames(file_frames, arg_frames)
        except ValueError as exc:
            self.get_logger().error("invalid aux_frames: %s" % exc)
            merged = []
        if not merged:
            self.get_logger().warn(
                "no aux frames configured (file=%r, inline=%r) -- "
                "canonical URDF will equal the base." % (cfg_path, inline))
        return merged

    # ------------------------------------------------------------------ #
    # Base URDF in -> canonical URDF out (idempotent / loop-safe)
    # ------------------------------------------------------------------ #
    def _on_base_urdf(self, msg: String) -> None:
        if not msg.data:
            return
        self._got_base = True
        # Recover the true manufacturer base: strip every frame we manage (incl.
        # since-removed names) so an RSP echo of our own augmented output, or a
        # bringup that already baked some frames, collapses back to the base.
        names = set(self._managed_names) | {f["name"] for f in self._frames}
        try:
            self._base_urdf = strip_aux_frames(msg.data, names)
        except Exception as exc:  # noqa: BLE001
            self._last_error = "bad incoming URDF (%r)" % exc
            self.get_logger().error("could not parse incoming URDF: %r" % exc)
            self._publish_status("error: bad incoming URDF (%r)" % exc)
            return
        self._rebuild_and_publish()

    def _rebuild_and_publish(self) -> None:
        """(Re)build the canonical URDF from the stored base + current frames and
        publish if it changed. Shared by the base-URDF and live-edit paths.

        Tolerant of bad frames: an aux frame whose parent is absent from THIS
        robot's URDF (or a cycle / duplicate / collision) is NOT fatal. The base
        URDF plus every VALID frame is still published so the robot renders, and
        each invalid frame is reported per-frame on ~/status so the dashboard can
        flag it (with the reason) for the operator to fix."""
        if not self._base_urdf:
            return
        try:
            canonical, valid, invalid = build_partial_canonical(
                self._base_urdf, self._frames)
        except Exception as exc:  # noqa: BLE001 -- only a malformed BASE URDF
            self._last_error = str(exc)
            self.get_logger().error("could not build canonical URDF: %r" % exc)
            self._publish_status("error: could not build canonical URDF (%r)" % exc)
            return
        # Keep ALL configured frames (valid first, then invalid) so invalid ones
        # stay visible + editable; record per-frame diagnostics for ~/status.
        self._frames = [dict(f) for f in valid] + [
            {"name": f.get("name", ""), "parent": f.get("parent", ""),
             "xyz": f.get("xyz", [0.0, 0.0, 0.0]),
             "rpy": f.get("rpy", [0.0, 0.0, 0.0])} for f in invalid]
        self._managed_names |= {f["name"] for f in self._frames if f.get("name")}
        self._frame_diags = (
            [{"name": f["name"], "parent": f["parent"], "xyz": f["xyz"],
              "rpy": f["rpy"], "valid": True, "error": ""} for f in valid]
            + [{"name": f.get("name", ""), "parent": f.get("parent", ""),
                "xyz": f.get("xyz", [0.0, 0.0, 0.0]),
                "rpy": f.get("rpy", [0.0, 0.0, 0.0]),
                "valid": False, "error": f.get("error", "")} for f in invalid])
        self._last_error = invalid[0].get("error", "") if invalid else ""
        # Rewrite the offsets of any pre-existing fixed frames the operator has
        # edited (frames baked into the launch URDF that we do NOT augment).
        if self._overrides:
            canonical, _updated, missing = apply_overrides(canonical, self._overrides)
            if missing:
                self.get_logger().debug(
                    "override targets not present in URDF yet (skipped): %s" % missing)
        if invalid:
            self.get_logger().warn(
                "%d aux frame(s) INVALID for this robot, skipped (base + %d valid "
                "frame(s) still published so it renders): %s"
                % (len(invalid), len(valid),
                   "; ".join("'%s'<-'%s': %s" % (f.get("name", "?"),
                             f.get("parent", "?"), f.get("error", ""))
                             for f in invalid)))
        diag_sig = json.dumps(self._frame_diags, sort_keys=True)
        if canonical != self._canonical:
            self._canonical = canonical
            self._diag_sig = diag_sig
            self._publish_canonical()
        elif diag_sig != self._diag_sig:
            # URDF unchanged but the per-frame diagnostics changed (e.g. an
            # invalid frame was edited) -- refresh status so the dashboard updates.
            self._diag_sig = diag_sig
            self._publish_status(self._build_summary())

    def _publish_canonical(self) -> None:
        m = String()
        m.data = self._canonical
        self._pub.publish(m)
        self._published = True
        if self._cli_rsp is not None:
            self._push_to_rsp(self._canonical)
        n_valid = sum(1 for f in self._frame_diags if f["valid"])
        n_invalid = sum(1 for f in self._frame_diags if not f["valid"])
        self.get_logger().info(
            "published canonical URDF (%d chars, %d valid aux frame(s)%s) on %s"
            % (len(self._canonical), n_valid,
               ", %d INVALID skipped" % n_invalid if n_invalid else "",
               self._out_topic))
        self._publish_status(self._build_summary())

    def _build_summary(self) -> str:
        """One-line ~/status message reflecting the current per-frame diagnostics."""
        valid = [f for f in self._frame_diags if f["valid"]]
        invalid = [f for f in self._frame_diags if not f["valid"]]
        if not invalid:
            return ("ok: published canonical URDF with frames [%s]"
                    % (", ".join(f["name"] for f in valid) or "-"))
        return ("partial: published base + %d valid frame(s) [%s]; %d invalid "
                "frame(s) skipped: %s"
                % (len(valid), ", ".join(f["name"] for f in valid) or "-",
                   len(invalid),
                   "; ".join("%s<-%s (%s)" % (f.get("name") or "?",
                             f.get("parent") or "?", f.get("error", ""))
                             for f in invalid)))

    def _push_to_rsp(self, urdf: str) -> None:
        if not self._cli_rsp.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "robot_state_publisher set_parameters unavailable; TF will not "
                "reflect the canonical URDF until RSP is up")
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="robot_description",
            value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value=urdf))]
        fut = self._cli_rsp.call_async(req)
        fut.add_done_callback(self._on_rsp_result)

    def _on_rsp_result(self, fut) -> None:
        try:
            res = fut.result()
            ok = bool(res.results and res.results[0].successful)
            if not ok:
                reason = res.results[0].reason if res.results else "no result"
                self.get_logger().warn("RSP rejected robot_description: %s" % reason)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("RSP set_parameters failed: %r" % exc)

    # ------------------------------------------------------------------ #
    # Live edits
    # ------------------------------------------------------------------ #
    def _on_set_frames(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            new_frames = parse_inline_frames(data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("set_aux_frames ignored: %r" % exc)
            self._publish_status("error: bad set_aux_frames (%r)" % exc)
            return
        self._frames = new_frames
        self._managed_names |= {f["name"] for f in new_frames}
        self.get_logger().info("set_aux_frames: %d frames [%s]"
                               % (len(new_frames),
                                  ", ".join(f["name"] for f in new_frames)))
        if not self._base_urdf:
            self._publish_status("frames staged; waiting for base URDF")
            return
        # Rebuild from the stored manufacturer base (NOT from the last canonical),
        # so removing/renaming a frame correctly drops the old one.
        self._rebuild_and_publish()

    def _on_edit_frame(self, msg: String) -> None:
        """Edit ONE frame's offset by name -- works for a frame this manager
        added AND for a frame already present in the launch URDF.

        JSON ``{name, xyz, rpy, parent?}``. Routing by ``name``:
          1. a frame we already augment -> update its xyz/rpy (and parent if
             given) in the managed list and rebuild;
          2. a pre-existing fixed frame in the base URDF -> record an offset
             override (its existing joint <origin> is rewritten each build);
          3. otherwise, if a ``parent`` is given -> add it as a new managed frame.

        Unlike ``~/set_aux_frames`` (which REPLACES the whole managed list), this
        touches only the named frame, so editing a baked-in frame never disturbs
        the others.
        """
        try:
            d = json.loads(msg.data)
            if not isinstance(d, dict):
                raise ValueError("edit_frame expects a JSON object")
            name = str(d.get("name") or "").strip()
            if not name:
                raise ValueError("edit_frame needs a non-empty 'name'")
            xyz = _vec3(d.get("xyz"))
            rpy = _vec3(d.get("rpy"))
            parent = str(d.get("parent") or "").strip()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("edit_frame ignored: %r" % exc)
            self._publish_status("error: bad edit_frame (%r)" % exc)
            return

        # (1) a frame this manager augments -> edit it in place.
        for f in self._frames:
            if f["name"] == name:
                f["xyz"], f["rpy"] = xyz, rpy
                if parent:
                    f["parent"] = parent
                self._overrides.pop(name, None)
                self.get_logger().info("edit_frame: updated managed frame '%s'" % name)
                self._rebuild_and_publish()
                return

        # (2) a frame already in the launch URDF -> override its fixed-joint
        # origin in place (any pre-existing fixed frame is editable this way).
        base_fixed = ({f["name"] for f in list_fixed_frames(self._base_urdf)}
                      if self._base_urdf else set())
        if name in base_fixed:
            self._overrides[name] = {"xyz": xyz, "rpy": rpy}
            self.get_logger().info(
                "edit_frame: override pre-existing frame '%s' xyz=%s rpy=%s"
                % (name, xyz, rpy))
            self._rebuild_and_publish()
            return

        # (3) a brand-new frame needs a parent to hang off.
        if parent:
            self._frames.append(
                {"name": name, "parent": parent, "xyz": xyz, "rpy": rpy})
            self._managed_names.add(name)
            self.get_logger().info("edit_frame: added new frame '%s' on '%s'"
                                   % (name, parent))
            self._rebuild_and_publish()
            return

        why = ("edit_frame: '%s' is neither a managed frame nor a fixed frame in "
               "the base URDF, and no 'parent' was given to create it" % name)
        self.get_logger().error(why)
        self._publish_status("error: %s" % why)

    def _base_link_names(self) -> List[str]:
        """Link names in the stored base URDF (for diagnostics); [] if none/bad."""
        if not self._base_urdf:
            return []
        try:
            return link_names(self._base_urdf)
        except Exception:  # noqa: BLE001
            return []

    def _on_startup_timeout(self) -> None:
        """One-shot watchdog: if no canonical URDF is out yet, the robot/bringup
        is almost certainly mismatched. Log a loud, actionable error at launch
        (and reflect it in ~/status) instead of waiting silently forever."""
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        if self._published:
            return  # all good -- the canonical URDF is published
        if not self._got_base:
            why = ("no base URDF received on '%s' within %.0fs -- the canonical "
                   "URDF cannot be built. Is the robot bringup running and "
                   "publishing '%s'? (topic or namespace mismatch?)"
                   % (self._base_topic, self._startup_timeout, self._base_topic))
        elif self._last_error:
            why = ("received the base URDF but could NOT build the canonical "
                   "URDF: %s. Configured aux frames: [%s]; links available in "
                   "the URDF: [%s]. The aux frames reference links absent from "
                   "THIS robot's URDF (e.g. Duco's 'link_6' on a UR robot) -- "
                   "fix aux_frame_manager.aux_frames for this robot."
                   % (self._last_error,
                      ", ".join(f["name"] for f in self._frames) or "-",
                      ", ".join(self._base_link_names()) or "-"))
        else:
            why = ("canonical URDF still not published %.0fs after start; "
                   "base_topic='%s'." % (self._startup_timeout, self._base_topic))
        self.get_logger().error("STARTUP CHECK FAILED: %s" % why)
        self._publish_status("error: %s" % why)

    def _publish_status(self, message: str) -> None:
        valid_names = [f["name"] for f in self._frame_diags if f.get("valid")]
        payload = {
            "message": message,
            "output_topic": self._out_topic,
            "base_topic": self._base_topic,
            # valid (in-URDF) added frame names -- the dashboard keys aux-link
            # detection off this, so it must list only frames actually published.
            "aux_frames": valid_names,
            # full per-frame diagnostics so the dashboard can list + highlight
            # INVALID frames (with the reason) even though they are not in the URDF.
            "frames": self._frame_diags,
            "n_invalid": sum(1 for f in self._frame_diags if not f.get("valid")),
            "overrides": sorted(self._overrides.keys()),
            "have_canonical": bool(self._canonical),
            "mirror_to_rsp": self._push_rsp,
        }
        m = String()
        m.data = json.dumps(payload)
        self._status_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AuxFrameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
