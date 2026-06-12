"""Launch the cct_pose_commander web dashboard (independent of the commander).

The dashboard is a thin HTTP/ROS client: it monitors a running commander's
``~/status`` and drives it via ``~/enable`` / ``~/disable`` / ``~/target_pose``.
The commander runs fine without it. Port + base_frame defaults come from the
toolkit's centralized config (``cct_pose_commander:`` in
``cct_common/config/toolkit_defaults.yaml``); CLI args override.

    ros2 launch cct_pose_commander dashboard.launch.py            # :8180
    ros2 launch cct_pose_commander dashboard.launch.py \
        commander_ns:=/cct_pose_commander_left base_frame:=base_link port:=8181
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_FALLBACKS = {"dashboard_port": 8180, "dashboard_base_frame": "base_link"}


def _defaults():
    try:
        from cct_common.config_manager import get_config  # type: ignore
        cfg = get_config()
        if cfg.has("cct_pose_commander"):
            sec = cfg.section("cct_pose_commander")
            return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
                    f"loaded from {cfg.config_path}")
        return (dict(_FALLBACKS), "FALLBACK (no 'cct_pose_commander:' section)")
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS), f"FALLBACK ({type(exc).__name__}: {exc})")


def generate_launch_description():
    d, source = _defaults()
    args = [
        DeclareLaunchArgument("port", default_value=str(d["dashboard_port"])),
        DeclareLaunchArgument("commander_ns",
                              default_value="/cct_pose_commander_right"),
        DeclareLaunchArgument("base_frame",
                              default_value=str(d["dashboard_base_frame"])),
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

    return LaunchDescription(
        args + [LogInfo(msg=f"[cct_pose_commander dashboard] config: {source}"),
                node])
