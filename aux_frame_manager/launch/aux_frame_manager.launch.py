"""Launch the aux_frame_manager: publish the canonical augmented URDF.

Assumes the basic robot bringup is already running (manufacturer URDF on
``/robot_description``). This node appends the configured aux frames and
publishes the canonical URDF on a latched topic for the FZI controllers.

    ros2 launch aux_frame_manager aux_frame_manager.launch.py
    ros2 launch aux_frame_manager aux_frame_manager.launch.py \
        aux_frames:='[{name: tcp, parent: link_6, xyz: [0,0,0.1]}]'

Pass ``dashboard_port`` to also start the optional web dashboard (3D view of
the canonical URDF with the added aux frames highlighted, plus a live editor):

    ros2 launch aux_frame_manager aux_frame_manager.launch.py dashboard_port:=8160
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
    "dashboard_port": "",
    "base_frame": "base_link",
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
        # Inline frames for quick CLI tests (NOT config-sourced); merged on top
        # of the config aux_frames list.
        DeclareLaunchArgument("aux_frames", default_value=""),
        DeclareLaunchArgument(
            "dashboard_port", default_value=str(d["dashboard_port"]),
            description="If set, also start the web dashboard on this port."),
        DeclareLaunchArgument(
            "base_frame", default_value=str(d["base_frame"]),
            description="Root TF frame the dashboard expresses link poses in."),
    ]

    log = LogInfo(msg=(
        f"[aux_frame_manager] config: {source}; "
        f"base='{d['base_urdf_topic']}' -> canonical='{d['output_topic']}'; "
        f"base_frame='{d['base_frame']}'"))

    node = Node(
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

    # Optional dashboard — only started when dashboard_port is non-empty.
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
            "base_frame": LaunchConfiguration("base_frame"),
            "canonical_topic": LaunchConfiguration("output_topic"),
            "base_urdf_topic": LaunchConfiguration("base_urdf_topic"),
        }],
    )

    return LaunchDescription([log] + args + [node, dashboard])
