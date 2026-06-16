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
to change offsets or add/remove frames; the canonical URDF is rebuilt and
re-published (and re-pushed to RSP). Status is published on ``~/status``.
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

from aux_frame_manager.frame_source import (build_canonical_urdf, merge_frames,
                                            parse_inline_frames, parse_spec_string,
                                            strip_aux_frames)


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
        self.declare_parameter("aux_frames_section", "")   # "" -> from config
        # Direct-argument frames (Req 1): rcl-safe compact specs
        # 'name:parent[:x,y,z[:r,p,yw]]' separated by ';'. A bracketed YAML/JSON
        # string does NOT survive the rcl param parser, so this compact form is
        # used for the CLI/launch/params-file argument.
        self.declare_parameter("aux_frames", "")           # inline compact specs

        self._base_topic = str(self.get_parameter("base_urdf_topic").value)
        self._out_topic = str(self.get_parameter("output_topic").value)
        self._push_rsp = bool(self.get_parameter("update_robot_state_publisher").value)
        self._rsp_name = str(self.get_parameter("robot_state_publisher_name").value)

        self._frames: List[Dict] = self._load_frames()
        # The latest manufacturer base URDF, with every managed aux frame
        # stripped off (recovered from each incoming /robot_description). Live
        # edits rebuild from THIS, so a remove/rename can't strand a stale frame.
        self._base_urdf: str = ""
        # Every aux-frame name this manager has ever added -- stripped from each
        # incoming URDF (incl. an RSP echo of our own output) so removed/renamed
        # frames don't survive.
        self._managed_names = set(f["name"] for f in self._frames)
        self._canonical: str = ""

        # --- pubs / subs / services -------------------------------------
        self._pub = self.create_publisher(String, self._out_topic, latched_qos())
        self._status_pub = self.create_publisher(String, "~/status", latched_qos())
        self.create_subscription(String, self._base_topic, self._on_base_urdf,
                                 latched_qos())
        self.create_subscription(String, "~/set_aux_frames", self._on_set_frames, 10)

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

    # ------------------------------------------------------------------ #
    # Frame loading (config file + inline argument)
    # ------------------------------------------------------------------ #
    def _load_frames(self) -> List[Dict]:
        file_frames: List[Dict] = []
        section = str(self.get_parameter("aux_frames_section").value or "")
        cfg_path = str(self.get_parameter("config_file").value or "")
        try:
            from cct_common.config_manager import get_config, read_aux_frames
            if not section:
                cfg = get_config()
                section = str(cfg.get("cartesian_control_manager.aux_frames_section",
                                      cfg.get("aux_frames_section", "")) or "")
            if not cfg_path:
                cfg = get_config()
                cfg_path = str(getattr(cfg, "config_path", "") or "")
            if cfg_path and section:
                file_frames = read_aux_frames(cfg_path, section)
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
                "no aux frames configured (section=%r, file=%r, inline=%r) -- "
                "canonical URDF will equal the base." % (section, cfg_path, inline))
        return merged

    # ------------------------------------------------------------------ #
    # Base URDF in -> canonical URDF out (idempotent / loop-safe)
    # ------------------------------------------------------------------ #
    def _on_base_urdf(self, msg: String) -> None:
        if not msg.data:
            return
        # Recover the true manufacturer base: strip every frame we manage (incl.
        # since-removed names) so an RSP echo of our own augmented output, or a
        # bringup that already baked some frames, collapses back to the base.
        names = set(self._managed_names) | {f["name"] for f in self._frames}
        try:
            self._base_urdf = strip_aux_frames(msg.data, names)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("could not parse incoming URDF: %r" % exc)
            self._publish_status("error: bad incoming URDF (%r)" % exc)
            return
        self._rebuild_and_publish()

    def _rebuild_and_publish(self) -> None:
        """(Re)build the canonical URDF from the stored base + current frames and
        publish if it changed. Shared by the base-URDF and live-edit paths."""
        if not self._base_urdf:
            return
        try:
            canonical, norm = build_canonical_urdf(self._base_urdf, self._frames)
        except ValueError as exc:
            self.get_logger().error(
                "REFUSED to publish canonical URDF: %s (frames invalid; keeping "
                "previous output)" % exc)
            self._publish_status("error: %s" % exc)
            return
        # Adopt the canonical (dependency-ordered) frame list so the stored
        # state matches what we publish regardless of the order a client sent
        # (a child may arrive before its parent; build_canonical_urdf sorts it).
        self._frames = norm
        self._managed_names |= {f["name"] for f in norm}
        if canonical == self._canonical:
            return  # no change (incl. RSP echo of our own output) -> loop-safe
        self._canonical = canonical
        self._publish_canonical()

    def _publish_canonical(self) -> None:
        m = String()
        m.data = self._canonical
        self._pub.publish(m)
        if self._cli_rsp is not None:
            self._push_to_rsp(self._canonical)
        self.get_logger().info(
            "published canonical URDF (%d chars, %d aux frames) on %s"
            % (len(self._canonical), len(self._frames), self._out_topic))
        self._publish_status("ok: published canonical URDF with frames [%s]"
                             % ", ".join(f["name"] for f in self._frames))

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

    def _publish_status(self, message: str) -> None:
        payload = {
            "message": message,
            "output_topic": self._out_topic,
            "base_topic": self._base_topic,
            "aux_frames": [f["name"] for f in self._frames],
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
