"""Launch the gravity-compensation node + its web dashboard.

Defaults are taken from ``config/robot_config.yaml`` under
``ft_sensor_gravity_compensation`` (via the ``cct_common`` package). CLI overrides
win.

The web dashboard is opt-in: it starts only when ``dashboard_port`` is a
positive port. Leaving it unset (the default ``0``) runs headless.

Examples:
  # headless -- dashboard_port defaults to 0 (disabled)
  ros2 launch ft_sensor_gravity_compensation compensation.launch.py
  # enable the calibration web UI on port 8100
  ros2 launch ft_sensor_gravity_compensation compensation.launch.py \\
      dashboard_port:=8100
  ros2 launch ft_sensor_gravity_compensation compensation.launch.py \\
      input_topic:=/ft_sensor/wrench_raw \\
      output_topic:=/ft_sensor/wrench_compensated \\
      sensor_frame:=ft_sensor_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FALLBACKS = {
    "input_topic":  "/ft_sensor/wrench_raw",
    "output_topic": "/ft_sensor/wrench_compensated",
    "world_frame":  "base_link",
    "sensor_frame": "tool0",
    "reliability":  "best_effort",
    "publish_when_no_tf": False,
    "storage_path": "~/.ros/ft_sensor_gravity_compensation/end_effectors.yaml",
    "host": "0.0.0.0",
    # dashboard_port: 0 disables the web dashboard; any positive port
    # enables it (replaces the legacy enable_dashboard bool + port pair).
    "dashboard_port": 0,
    "gravity": 9.80665,
    "tf_timeout": 0.05,
    "tf_max_age": 1.0,
    # Per-axis soft deadband on the compensated wrench (N / N*m). Read from the
    # (optionally per-instance) config section as a list and passed straight to
    # the node's double_array parameter. Robot-neutral default = no deadband.
    "force_deadband":  [0.0, 0.0, 0.0],
    "torque_deadband": [0.0, 0.0, 0.0],
}


def _defaults(section_name: str = "ft_sensor_gravity_compensation"):
    """Return ``(defaults_dict, source_str)`` for ``section_name``.

    Mirrors the pattern used by the other packages in this workspace: try to
    load the central config, but fall back to ``_FALLBACKS`` so the launcher
    still works on a fresh checkout. The source string is logged at launch so
    operators can see whether the central config was honoured.
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
    sec = cfg.section(section_name) if cfg.has(section_name) else None
    if sec is None:
        return (dict(_FALLBACKS),
                f"FALLBACK (no '{section_name}:' section in {cfg.config_path})")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path} [{section_name}]")


def _launch_bool(value):
    return str(value).lower()


def _build(context, *_args, **_kwargs):
    """Resolve ``instance_name`` -> node name + config section + deadband."""
    instance = LaunchConfiguration("instance_name").perform(context).strip()
    suffix = f"_{instance}" if instance else ""
    section_name = f"ft_sensor_gravity_compensation{suffix}"
    node_name = f"ft_sensor_gravity_compensation{suffix}"

    # The deadband comes from the (per-instance) config section as a LIST, so it
    # can be passed straight to the node's double_array parameter (a launch-arg
    # string can't be coerced into a double[]).
    d, source = _defaults(section_name)
    fdb = d.get("force_deadband",  [0.0, 0.0, 0.0])
    tdb = d.get("torque_deadband", [0.0, 0.0, 0.0])

    # Per-instance end-effector store by default, so two arms don't share one
    # file; an explicit ``storage_path:=`` still wins.
    storage = LaunchConfiguration("storage_path").perform(context)
    if instance and storage == _FALLBACKS["storage_path"]:
        storage = ("~/.ros/ft_sensor_gravity_compensation/"
                   f"{instance}_end_effectors.yaml")

    log = LogInfo(msg=(
        f"[ft_sensor_gravity_compensation] node={node_name}; config: {source}; "
        f"force_deadband={list(fdb)} torque_deadband={list(tdb)}"))

    node = Node(
        package="ft_sensor_gravity_compensation",
        executable="compensation_node",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic":  LaunchConfiguration("input_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "world_frame":  LaunchConfiguration("world_frame"),
            "sensor_frame": LaunchConfiguration("sensor_frame"),
            "reliability":  LaunchConfiguration("reliability"),
            "publish_when_no_tf": LaunchConfiguration("publish_when_no_tf"),
            "storage_path": storage,
            "host":         LaunchConfiguration("host"),
            "dashboard_port": LaunchConfiguration("dashboard_port"),
            "gravity":      LaunchConfiguration("gravity"),
            "tf_timeout":   LaunchConfiguration("tf_timeout"),
            "tf_max_age":   LaunchConfiguration("tf_max_age"),
            "force_deadband":  [float(x) for x in fdb],
            "torque_deadband": [float(x) for x in tdb],
        }],
    )
    return [log, node]


def generate_launch_description() -> LaunchDescription:
    # Build-time defaults from the single (non-instanced) section, used only for
    # the CLI-argument defaults. ``instance_name`` selects the per-instance
    # section + node name + deadband at launch time (see _build).
    d, _source = _defaults()

    args = [
        DeclareLaunchArgument(
            "instance_name", default_value="",
            description="Suffix for BOTH the node name and the config section, "
                        "e.g. instance_name:=right -> node "
                        "ft_sensor_gravity_compensation_right reading the "
                        "ft_sensor_gravity_compensation_right section (its "
                        "force_deadband / torque_deadband)."),
        DeclareLaunchArgument("input_topic",  default_value=str(d["input_topic"])),
        DeclareLaunchArgument("output_topic", default_value=str(d["output_topic"])),
        DeclareLaunchArgument("world_frame",  default_value=str(d["world_frame"])),
        DeclareLaunchArgument("sensor_frame", default_value=str(d["sensor_frame"])),
        DeclareLaunchArgument(
            "reliability", default_value=str(d["reliability"]),
            description="best_effort or reliable (matches the publisher's QoS)"),
        DeclareLaunchArgument(
            "publish_when_no_tf",
            default_value=_launch_bool(d["publish_when_no_tf"]),
            description="if true, publish bias-only wrench when TF is stale"),
        DeclareLaunchArgument("storage_path", default_value=str(d["storage_path"])),
        DeclareLaunchArgument(
            "host", default_value=str(d["host"]),
            description="HTTP bind address; 0.0.0.0 = LAN-visible"),
        DeclareLaunchArgument(
            "dashboard_port", default_value=str(d["dashboard_port"]),
            description="TCP port for the calibration web UI; 0 (the "
                        "default) disables the dashboard. Set e.g. "
                        "dashboard_port:=8100 to enable it."),
        DeclareLaunchArgument("gravity", default_value=str(d["gravity"])),
        DeclareLaunchArgument("tf_timeout", default_value=str(d["tf_timeout"])),
        DeclareLaunchArgument("tf_max_age", default_value=str(d["tf_max_age"])),
    ]

    return LaunchDescription([*args, OpaqueFunction(function=_build)])
