"""Convenience launch: the feasibility test ENGINE, plus an OPTIONAL dashboard.

The engine (the main task) always starts; the web dashboard is **opt-in** via
``dashboard:=true`` (default **false** -- the dashboard is genuinely optional).

Examples::

    ros2 launch robot_feasibility_test test.launch.py \\
        wrench_topic:=/right_arm_force_torque_sensor_broadcaster/wrench
    ros2 launch robot_feasibility_test test.launch.py dashboard:=true port:=8140
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# String engine args forwarded to engine.launch.py (empty => engine default).
_ENGINE_STR_ARGS = {
    "joint_states_topic": "/joint_states",
    "wrench_topic": "",
    "controller_manager": "/controller_manager",
    "controller_name": "",
    "report_dir": "~/.ros/robot_feasibility_test/runs",
}


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("robot_feasibility_test")
    engine_launch = os.path.join(share, "launch", "engine.launch.py")
    dash_launch = os.path.join(share, "launch", "dashboard.launch.py")

    decls = [DeclareLaunchArgument("dashboard", default_value="false",
                                   description="also start the optional web UI"),
             DeclareLaunchArgument("port", default_value="8140")]
    decls += [DeclareLaunchArgument(k, default_value=v)
              for k, v in _ENGINE_STR_ARGS.items()]

    engine_args = {k: LaunchConfiguration(k) for k in _ENGINE_STR_ARGS}

    return LaunchDescription([
        *decls,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(engine_launch),
            launch_arguments=engine_args.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dash_launch),
            condition=IfCondition(LaunchConfiguration("dashboard")),
            launch_arguments={"port": LaunchConfiguration("port")}.items()),
    ])
