"""Launch the FPC test dashboard.

Run this AFTER the robot is up (so /robot_description and /controller_manager
exist). Defaults are read from ``config/robot_config.yaml`` under
``fpc_test_dashboard:`` (via the ``cct_common`` package) when present, with
hard-coded fallbacks so it works on a fresh checkout.

Examples::

    ros2 launch fpc_test_dashboard dashboard.launch.py
    ros2 launch fpc_test_dashboard dashboard.launch.py port:=9140
    ros2 launch fpc_test_dashboard dashboard.launch.py \\
        wrench_topic:=/right_arm_force_torque_sensor_broadcaster/wrench
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FALLBACKS = {
    "host": "0.0.0.0",
    "port": 8140,
    "joint_states_topic": "/joint_states",
    "wrench_topic": "",
    "controller_manager": "/controller_manager",
    "send_rate": 200.0,
    "default_limit_deg": 8.0,
    "report_dir": "~/.ros/fpc_test_dashboard/runs",
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
    if not cfg.has("fpc_test_dashboard"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'fpc_test_dashboard:' section in {cfg.config_path})")
    sec = cfg.section("fpc_test_dashboard")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()
    args = [DeclareLaunchArgument(k, default_value=str(d[k])) for k in _FALLBACKS]
    log = LogInfo(msg=f"[fpc_test_dashboard] config: {source}; port={d['port']}")
    params = {k: LaunchConfiguration(k) for k in _FALLBACKS}
    node = Node(
        package="fpc_test_dashboard",
        executable="dashboard_node",
        name="fpc_test_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=[params],
    )
    return LaunchDescription([log, *args, node])
