#!/usr/bin/env python3
"""aux_frame_guard — pre-activate check that the canonical URDF is FZI-ready.

Subscribes to the canonical ``robot_description`` topic and verifies that the
configured Cartesian-controller endpoint/reference frames exist AND lie on the
``robot_base_link`` -> ``end_effector_link`` kinematic chain (exactly FZI's
``getChain`` + ``robotChainContains`` configure-time constraint). On success it
latches ``<topic>_ready`` (std_msgs/Bool true); on failure it prints an
actionable error naming the missing frames so the operator can fix the
aux_frame_manager config/args or the launch ordering -- instead of staring at
FZI's cryptic "robot_description is empty" / chain-build failure.

    ros2 run aux_frame_manager aux_frame_guard --ros-args \
        -p robot_base_link:=base_link -p end_effector_link:=compliance_link \
        -p required_frames:='[ft_sensor_link, compliance_link]'
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String

from aux_frame_manager.frame_source import check_frames_in_chain


def latched_qos() -> QoSProfile:
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AuxFrameGuard(Node):
    def __init__(self) -> None:
        super().__init__("aux_frame_guard")
        self.declare_parameter("robot_description_topic", "/cartesian/robot_description")
        self.declare_parameter("robot_base_link", "base_link")
        self.declare_parameter("end_effector_link", "")
        self.declare_parameter("required_frames", [""])

        self._topic = str(self.get_parameter("robot_description_topic").value)
        self._base = str(self.get_parameter("robot_base_link").value)
        self._ee = str(self.get_parameter("end_effector_link").value)
        self._required = [str(f) for f in
                          (self.get_parameter("required_frames").value or []) if str(f)]
        self._ready = False

        self._ready_pub = self.create_publisher(
            Bool, self._topic.rstrip("/") + "_ready", latched_qos())
        self.create_subscription(String, self._topic, self._on_urdf, latched_qos())
        self.get_logger().info(
            "aux_frame_guard: checking %s -> %s for frames %s on '%s'"
            % (self._base, self._ee or "(unset)", self._required, self._topic))
        self._publish_ready(False)

    def _on_urdf(self, msg: String) -> None:
        if not msg.data or not self._ee:
            return
        required = list(self._required)
        if self._ee not in required:
            required.append(self._ee)
        ok, why = check_frames_in_chain(msg.data, self._base, self._ee, required)
        if ok and not self._ready:
            self._ready = True
            self.get_logger().info(
                "READY: canonical URDF contains %s on the %s->%s chain"
                % (required, self._base, self._ee))
            self._publish_ready(True)
        elif not ok:
            if self._ready:
                self._ready = False
                self._publish_ready(False)
            self.get_logger().error(
                "NOT READY: %s. Check aux_frame_manager config/args (are the "
                "frames declared?) and that it published before FZI activates."
                % why)

    def _publish_ready(self, value: bool) -> None:
        m = Bool()
        m.data = value
        self._ready_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AuxFrameGuard()
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
