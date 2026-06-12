"""Launch the optional IK web dashboard (connects to a running ik_node).

The dashboard is a pure UI client of ik_node's ROS API; the solver runs fine
without it. Default port 8160.

Example::

    ros2 launch cct_inverse_kinematics dashboard.launch.py port:=8160
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8160"),
        Node(
            package="cct_inverse_kinematics",
            executable="dashboard_node",
            name="ik_dashboard",
            output="screen",
            parameters=[{"port": LaunchConfiguration("port")}],
        ),
    ])
