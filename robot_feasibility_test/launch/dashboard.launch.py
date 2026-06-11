"""Launch the feasibility test DASHBOARD (optional web UI; thin client of the engine).

The dashboard does not touch the robot -- it is a pure client of
``feasibility_test_engine``. Run the engine first (``engine.launch.py``), then
launch this any time to monitor / drive it; stop or reload it without
interrupting a run. Defaults are read from ``config/robot_config.yaml`` under
``robot_feasibility_test:`` (via ``cct_common``) when present.

Examples::

    ros2 launch robot_feasibility_test dashboard.launch.py
    ros2 launch robot_feasibility_test dashboard.launch.py port:=9140
    ros2 launch robot_feasibility_test dashboard.launch.py engine_node:=feasibility_test_engine
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FALLBACKS = {
    "host": "0.0.0.0",
    "port": 8140,
    "engine_node": "feasibility_test_engine",
    "report_dir": "~/.ros/robot_feasibility_test/runs",
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
    log = LogInfo(msg=f"[feasibility_test_dashboard] config: {source}; port={d['port']}")
    params = {k: LaunchConfiguration(k) for k in _FALLBACKS}
    node = Node(
        package="robot_feasibility_test",
        executable="dashboard_node",
        name="feasibility_test_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=[params],
    )
    return LaunchDescription([log, *args, node])
