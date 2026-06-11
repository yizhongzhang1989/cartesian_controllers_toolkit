"""Launch the feasibility test ENGINE (the main, headless task).

This is what you run to characterise a robot. It needs **no** dashboard: it runs
the automatic battery (driven via its ``~/run``/``~/stop`` services or the
``feasibility_test`` CLI) and saves self-contained HTML reports. Pass
``dashboard_port:=<port>`` to also bring up the optional web UI in the same
launch, or start ``dashboard.launch.py`` separately at any time.

Run AFTER the robot is up (so /robot_description and /controller_manager exist).
Defaults are read from ``config/robot_config.yaml`` under ``robot_feasibility_test:``
(via ``cct_common``) when present, with hard-coded fallbacks.

Examples::

    ros2 launch robot_feasibility_test engine.launch.py \\
        wrench_topic:=/right_arm_force_torque_sensor_broadcaster/wrench
    # then drive it headless:
    ros2 run robot_feasibility_test feasibility_test --controller <ctrl> --joints j2,j4

    # engine + dashboard in one shot (dashboard on http://localhost:8140):
    ros2 launch robot_feasibility_test engine.launch.py \\
        wrench_topic:=/right_arm_force_torque_sensor_broadcaster/wrench \\
        dashboard_port:=8140
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


_FALLBACKS = {
    "joint_states_topic": "/joint_states",
    "wrench_topic": "",
    "controller_manager": "/controller_manager",
    "controller_name": "",
    "send_rate": 200.0,
    "default_limit_deg": 8.0,
    "report_dir": "~/.ros/robot_feasibility_test/runs",
    "status_rate": 10.0,
}


def _defaults():
    try:
        from cct_common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not import cct_common.config_manager: "
                f"{type(exc).__name__}: {exc})")
    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not load config: {type(exc).__name__}: {exc})")
    if not cfg.has("robot_feasibility_test"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'robot_feasibility_test:' section in {cfg.config_path})")
    sec = cfg.section("robot_feasibility_test")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()
    args = [DeclareLaunchArgument(k, default_value=str(d[k])) for k in _FALLBACKS]
    args.append(DeclareLaunchArgument(
        "dashboard_port", default_value="",
        description="if set (e.g. 8140), also launch the optional web dashboard "
                    "on this port; empty => engine only (headless)"))
    log = LogInfo(msg=f"[feasibility_test_engine] config: {source}")
    params = {k: LaunchConfiguration(k) for k in _FALLBACKS}
    node = Node(
        package="robot_feasibility_test",
        executable="engine_node",
        name="feasibility_test_engine",
        output="screen",
        emulate_tty=True,
        parameters=[params],
    )

    # Optionally bring up the dashboard (a thin client of the engine) in the same
    # launch when ``dashboard_port`` is given. It shares the engine's report_dir
    # and targets the engine node started above.
    dash_launch = os.path.join(
        get_package_share_directory("robot_feasibility_test"),
        "launch", "dashboard.launch.py")
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dash_launch),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("dashboard_port"), "' != ''"])),
        launch_arguments={
            "port": LaunchConfiguration("dashboard_port"),
            "engine_node": "feasibility_test_engine",
            "report_dir": LaunchConfiguration("report_dir"),
        }.items())

    return LaunchDescription([log, *args, node, dashboard])
