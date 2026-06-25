"""Launch the canonical-URDF source: aux_frame_manager (+ optional dashboard).

This is the building block that makes the FZI Cartesian controllers read their
URDF from a single latched topic. Run it AFTER the basic robot bringup (which
publishes the manufacturer URDF on ``/robot_description``) and BEFORE / alongside
the Cartesian controllers (which must be configured with
``urdf_from_topic:=true`` and ``robot_description_topic:=/cartesian/robot_description``).

    ros2 launch aux_frame_manager cartesian_urdf_source.launch.py \
        aux_frames:='ft_sensor_link:link_6; compliance_link:ft_sensor_link'

The FZI controllers validate their own ``robot_base_link`` -> ``end_effector_link``
chain at on_configure, so no separate pre-activation check runs here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# Hard-coded fallbacks, used when the central config (cct_common -> the
# 'aux_frame_manager:' section of robot_config.yaml / toolkit_defaults.yaml)
# is missing or omits a key. Values are strings because they become launch
# argument default_values; the node coerces types from its own declarations.
_FALLBACKS = {
    "base_urdf_topic": "/robot_description",
    "output_topic": "/cartesian/robot_description",
    "update_robot_state_publisher": "true",
    "robot_state_publisher_name": "robot_state_publisher",
    "config_file": "",
    "robot_base_link": "base_link",
    "dashboard_port": "",
}


def _defaults():
    """Return (defaults_dict, source_str) from the central config.

    Reads the ``aux_frame_manager:`` section via ``cct_common`` and overlays
    it on ``_FALLBACKS``; on any failure returns the fallbacks plus a string
    explaining why, so the launch still works on a fresh checkout.
    """
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
                f"FALLBACK (could not load config: "
                f"{type(exc).__name__}: {exc})")
    if not cfg.has("aux_frame_manager"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'aux_frame_manager:' section in {cfg.config_path})")
    sec = cfg.section("aux_frame_manager")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    args = [
        DeclareLaunchArgument("base_urdf_topic",
                              default_value=str(d["base_urdf_topic"])),
        DeclareLaunchArgument("output_topic",
                              default_value=str(d["output_topic"])),
        DeclareLaunchArgument("update_robot_state_publisher",
                              default_value=str(d["update_robot_state_publisher"])),
        DeclareLaunchArgument("robot_state_publisher_name",
                              default_value=str(d["robot_state_publisher_name"])),
        DeclareLaunchArgument("config_file",
                              default_value=str(d["config_file"])),
        # Inline frames for quick CLI tests: rcl-safe compact specs
        # 'name:parent[:x,y,z[:r,p,yw]]' separated by ';'. CLI-only convenience
        # (NOT config-sourced); merged on top of the config aux_frames list.
        DeclareLaunchArgument("aux_frames", default_value=""),
        # base_frame for the optional dashboard's TF lookups.
        DeclareLaunchArgument("robot_base_link",
                              default_value=str(d["robot_base_link"])),
        # Optional dashboard (3D view + live editor); started when set.
        DeclareLaunchArgument("dashboard_port",
                              default_value=str(d["dashboard_port"])),
    ]

    log = LogInfo(msg=(
        f"[aux_frame_manager] config: {source}; "
        f"base='{d['base_urdf_topic']}' -> canonical='{d['output_topic']}'"))

    manager = Node(
        package="aux_frame_manager",
        executable="aux_frame_manager",
        name="aux_frame_manager",
        output="screen",
        parameters=[{
            "base_urdf_topic": LaunchConfiguration("base_urdf_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "update_robot_state_publisher":
                LaunchConfiguration("update_robot_state_publisher"),
            "robot_state_publisher_name":
                LaunchConfiguration("robot_state_publisher_name"),
            "config_file": LaunchConfiguration("config_file"),
            "aux_frames": LaunchConfiguration("aux_frames"),
        }],
    )

    dashboard = Node(
        package="aux_frame_manager",
        executable="aux_frame_dashboard",
        name="aux_frame_dashboard",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("dashboard_port"),
                              "' != ''"])),
        parameters=[{
            "port": LaunchConfiguration("dashboard_port"),
            "manager_ns": "/aux_frame_manager",
            "base_frame": LaunchConfiguration("robot_base_link"),
            "canonical_topic": LaunchConfiguration("output_topic"),
            "base_urdf_topic": LaunchConfiguration("base_urdf_topic"),
        }],
    )

    return LaunchDescription([log] + args + [manager, dashboard])
