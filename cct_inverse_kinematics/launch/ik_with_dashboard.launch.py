"""Launch the headless IK solver AND the web dashboard together.

Launch-arg defaults come from the toolkit's centralized config
(``cct_inverse_kinematics:`` in ``cct_common/config/toolkit_defaults.yaml``);
CLI args override. Solver tuning stays in the package ``ik_defaults.yaml``.

Example::

    ros2 launch cct_inverse_kinematics ik_with_dashboard.launch.py port:=8160
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_FALLBACKS = {"base_frame": "", "dashboard_port": 8160}


def _defaults():
    try:
        from cct_common.config_manager import get_config  # type: ignore
        cfg = get_config()
        if cfg.has("cct_inverse_kinematics"):
            sec = cfg.section("cct_inverse_kinematics")
            return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
                    f"loaded from {cfg.config_path}")
        return (dict(_FALLBACKS), "FALLBACK (no 'cct_inverse_kinematics:' section)")
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS), f"FALLBACK ({type(exc).__name__}: {exc})")


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("cct_inverse_kinematics")
    params = os.path.join(pkg, "config", "ik_defaults.yaml")
    d, source = _defaults()

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        DeclareLaunchArgument("base_frame", default_value=str(d["base_frame"])),
        DeclareLaunchArgument("port", default_value=str(d["dashboard_port"])),
        LogInfo(msg=f"[cct_inverse_kinematics] config: {source}"),
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
