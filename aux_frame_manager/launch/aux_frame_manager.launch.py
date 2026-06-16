"""Launch the aux_frame_manager: publish the canonical augmented URDF.

Assumes the basic robot bringup is already running (manufacturer URDF on
``/robot_description``). This node appends the configured aux frames and
publishes the canonical URDF on a latched topic for the FZI controllers.

    ros2 launch aux_frame_manager aux_frame_manager.launch.py
    ros2 launch aux_frame_manager aux_frame_manager.launch.py \
        aux_frames:='[{name: tcp, parent: link_6, xyz: [0,0,0.1]}]'

Pass ``dashboard_port`` to also start the optional web dashboard (3D view of
the canonical URDF with the added aux frames highlighted, plus a live editor):

    ros2 launch aux_frame_manager aux_frame_manager.launch.py dashboard_port:=8160
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("base_urdf_topic", default_value="/robot_description"),
        DeclareLaunchArgument("output_topic",
                              default_value="/cartesian/robot_description"),
        DeclareLaunchArgument("update_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("robot_state_publisher_name",
                              default_value="robot_state_publisher"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("aux_frames_section", default_value=""),
        DeclareLaunchArgument("aux_frames", default_value=""),
        DeclareLaunchArgument(
            "dashboard_port", default_value="",
            description="If set, also start the web dashboard on this port."),
        DeclareLaunchArgument(
            "base_frame", default_value="base_link",
            description="Root TF frame the dashboard expresses link poses in."),
    ]

    node = Node(
        package="aux_frame_manager",
        executable="aux_frame_manager",
        name="aux_frame_manager",
        output="screen",
        parameters=[{
            "base_urdf_topic": LaunchConfiguration("base_urdf_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "update_robot_state_publisher":
                LaunchConfiguration("update_robot_state_publisher"),
            "robot_state_publisher_name":
                LaunchConfiguration("robot_state_publisher_name"),
            "config_file": LaunchConfiguration("config_file"),
            "aux_frames_section": LaunchConfiguration("aux_frames_section"),
            "aux_frames": LaunchConfiguration("aux_frames"),
        }],
    )

    # Optional dashboard — only started when dashboard_port is non-empty.
    dashboard = Node(
        package="aux_frame_manager",
        executable="aux_frame_dashboard",
        name="aux_frame_dashboard",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("dashboard_port"),
                              "' != ''"])),
        parameters=[{
            "port": LaunchConfiguration("dashboard_port"),
            "manager_ns": "/aux_frame_manager",
            "base_frame": LaunchConfiguration("base_frame"),
            "canonical_topic": LaunchConfiguration("output_topic"),
            "base_urdf_topic": LaunchConfiguration("base_urdf_topic"),
        }],
    )

    return LaunchDescription(args + [node, dashboard])
