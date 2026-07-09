"""Launch the cartesian-controller web dashboard.

Defaults are loaded from ``config/robot_config.yaml`` under
``cartesian_controller_dashboard:`` (via the ``cct_common`` package) when
present, with hard-coded fallbacks so the launch still works on a
fresh checkout.

The dashboard is a pure UI / monitoring layer.  It does not launch
controllers, wrench relays, or safety supervisors -- those live in
:package:`cartesian_control_manager`.

Examples::

    ros2 launch cartesian_controller_dashboard dashboard.launch.py
    ros2 launch cartesian_controller_dashboard dashboard.launch.py port:=9120
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FALLBACKS = {
    "orchestrator_ns":     "/cartesian_control_manager",
    # Node that owns the wrench deadband (the ft_sensor_gravity_compensation
    # preprocessing node).  Empty -> the dashboard falls back to editing the
    # orchestrator's own deadband (legacy).  Dual-arm setups pass the per-arm
    # node explicitly, e.g. deadband_node_ns:=/ft_sensor_gravity_compensation_left.
    "deadband_node_ns":    "",
    "controller_name":     "cartesian_force_controller",
    "wrench_topic":        "/ft_sensor/wrench_compensated",
    "joint_states_topic":  "/joint_states",
    # TCP pose display: dashboard looks up ``base_frame -> tool_frame``
    # via TF.  Robot-neutral defaults (the ROS-Industrial ``base_link`` /
    # ``tool0`` conventions).  Override in the
    # ``cartesian_controller_dashboard:`` section of your
    # ``config/robot_config.yaml`` -- e.g. a robot with an aux tool tip
    # sets ``tool_frame: compliance_link`` (a frame added at bringup).
    "base_frame":          "base_link",
    "tool_frame":          "tool0",
    # Namespace of the aux_frame_manager (single-writer of the canonical
    # URDF).  The "Tool frames" editor reads / writes the frame list under
    # the standard ``aux_frame_manager:`` config section and routes live
    # saves through its <ns>/set_aux_frames so topic-mode FZI controllers
    # swap chain live.
    "aux_frame_manager_ns": "/aux_frame_manager",
    "service_timeout_sec": 2.0,
    "host":                "0.0.0.0",
    "port":                8120,
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
                f"FALLBACK (could not load config: "
                f"{type(exc).__name__}: {exc})")
    if not cfg.has("cartesian_controller_dashboard"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'cartesian_controller_dashboard:' section in "
                f"{cfg.config_path})")
    sec = cfg.section("cartesian_controller_dashboard")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()

    args = [
        DeclareLaunchArgument(key, default_value=str(d[key]))
        for key in _FALLBACKS.keys()
    ]
    log = LogInfo(msg=(
        f"[cartesian_controller_dashboard] config: {source}; "
        f"port={d['port']}"))
    parameters = {key: LaunchConfiguration(key) for key in _FALLBACKS.keys()}
    node = Node(
        package="cartesian_controller_dashboard",
        executable="dashboard_node",
        name="cartesian_controller_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=[parameters],
    )
    return LaunchDescription([log, *args, node])
