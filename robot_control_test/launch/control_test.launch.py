"""Launch the robot_control_test web bench.

Run AFTER the robot is up (so ``/robot_description``, ``/joint_states`` and
``/controller_manager`` exist). The node serves a 3D dashboard that discovers
the robot's controllers and lets you activate one and jog it to verify the
robot moves correctly.

    ros2 launch robot_control_test control_test.launch.py            # :8200
    ros2 launch robot_control_test control_test.launch.py \
        dashboard_port:=8201 base_frame:=base_link tip_frame:=tool0

Defaults come from the toolkit's centralized config (a ``robot_control_test:``
section in ``robot_config.yaml`` / ``cct_common`` ``toolkit_defaults.yaml``)
when present; CLI args always override.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Hard-coded fallbacks (strings: they become launch-argument default_values;
# the node coerces types from its own parameter declarations). Used when the
# central config is missing or omits a key, so a fresh checkout still works.
_FALLBACKS = {
    "dashboard_port": "8200",
    "controller_manager": "/controller_manager",
    "robot_description_topic": "/robot_description",
    "joint_states_topic": "/joint_states",
    "base_frame": "base_link",
    "tip_frame": "",
    "max_joint_speed": "0.5",
    "send_rate": "100.0",
}


def _defaults():
    """Return (defaults_dict, source_str) from the central config.

    Reads the ``robot_control_test:`` section via ``cct_common`` and overlays it
    on ``_FALLBACKS``; on any failure returns the fallbacks plus a string
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
    if not cfg.has("robot_control_test"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'robot_control_test:' section in "
                f"{cfg.config_path})")
    sec = cfg.section("robot_control_test")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    args = [DeclareLaunchArgument(k, default_value=str(d[k]))
            for k in _FALLBACKS]

    log = LogInfo(msg=(
        f"[robot_control_test] config: {source}; "
        f"dashboard_port={d['dashboard_port']}, "
        f"controller_manager={d['controller_manager']}, "
        f"base_frame={d['base_frame']}"))

    node = Node(
        package="robot_control_test",
        executable="control_test_node",
        name="robot_control_test",
        output="screen",
        parameters=[{
            "dashboard_port": LaunchConfiguration("dashboard_port"),
            "controller_manager": LaunchConfiguration("controller_manager"),
            "robot_description_topic":
                LaunchConfiguration("robot_description_topic"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "tip_frame": LaunchConfiguration("tip_frame"),
            "max_joint_speed": LaunchConfiguration("max_joint_speed"),
            "send_rate": LaunchConfiguration("send_rate"),
        }],
    )

    return LaunchDescription([log] + args + [node])
