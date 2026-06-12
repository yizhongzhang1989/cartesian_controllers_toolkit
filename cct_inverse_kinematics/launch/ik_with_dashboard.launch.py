"""Launch the headless IK solver AND the web dashboard together.

Example::

    ros2 launch cct_inverse_kinematics ik_with_dashboard.launch.py port:=8160
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("cct_inverse_kinematics")
    defaults = os.path.join(pkg, "config", "ik_defaults.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=defaults),
        DeclareLaunchArgument("base_frame", default_value=""),
        DeclareLaunchArgument("port", default_value="8160"),
        Node(
            package="cct_inverse_kinematics",
            executable="ik_node",
            name="ik_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"base_frame": LaunchConfiguration("base_frame")},
            ],
        ),
        Node(
            package="cct_inverse_kinematics",
            executable="dashboard_node",
            name="ik_dashboard",
            output="screen",
            parameters=[{"port": LaunchConfiguration("port")}],
        ),
    ])
