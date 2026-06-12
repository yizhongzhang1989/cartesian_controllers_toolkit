"""Launch the headless IK solver node (no dashboard).

The solver reads ``/robot_description`` and ``/joint_states`` and exposes its
JSON solve API + ``~/solution`` / ``~/status`` outputs. It is advisory only and
never commands the robot.

Examples::

    ros2 launch cct_inverse_kinematics ik.launch.py
    ros2 launch cct_inverse_kinematics ik.launch.py base_frame:=base_link
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
        DeclareLaunchArgument(
            "robot_description_topic", default_value="/robot_description"),
        DeclareLaunchArgument(
            "joint_states_topic", default_value="/joint_states"),
        Node(
            package="cct_inverse_kinematics",
            executable="ik_node",
            name="ik_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "base_frame": LaunchConfiguration("base_frame"),
                    "robot_description_topic":
                        LaunchConfiguration("robot_description_topic"),
                    "joint_states_topic":
                        LaunchConfiguration("joint_states_topic"),
                },
            ],
        ),
    ])
