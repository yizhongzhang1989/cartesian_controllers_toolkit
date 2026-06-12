"""Launch the cct_pose_commander web dashboard (independent of the commander).

The dashboard is a thin HTTP/ROS client: it monitors a running commander's
``~/status`` and drives it via ``~/enable`` / ``~/disable`` / ``~/target_pose``.
The commander runs fine without it.

    ros2 launch cct_pose_commander dashboard.launch.py            # right arm, :8180
    ros2 launch cct_pose_commander dashboard.launch.py \
        commander_ns:=/cct_pose_commander_left base_frame:=base_link port:=8181
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("port", default_value="8180"),
        DeclareLaunchArgument("commander_ns",
                              default_value="/cct_pose_commander_right"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
    ]

    node = Node(
        package="cct_pose_commander",
        executable="dashboard_node",
        name="cct_pose_commander_dashboard",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "commander_ns": LaunchConfiguration("commander_ns"),
            "base_frame": LaunchConfiguration("base_frame"),
        }],
    )

    return LaunchDescription(args + [node])
