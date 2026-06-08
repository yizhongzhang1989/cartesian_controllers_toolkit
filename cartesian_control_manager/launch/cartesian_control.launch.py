"""Launch the Cartesian-control orchestrator + spawn FZI's controllers (inactive).

Defaults are loaded from ``config/robot_config.yaml`` under
``cartesian_control_manager:`` (via the ``cct_common`` package) when present,
with hard-coded fallbacks so the launch still works on a fresh checkout.

The launch spawns each FZI Cartesian controller plugin into the live
``controller_manager`` in the **inactive** state.  The orchestrator
node will then activate / deactivate the **selected** one on engage /
disengage via ``switch_controller``.  By default three controllers are
loaded:

* ``cartesian_force_controller``
* ``cartesian_motion_controller``
* ``cartesian_compliance_controller``

The default selection is ``cartesian_force_controller``; switch live
via ``ros2 param set /cartesian_control_manager active_controller_name
<name>`` (must be disengaged first), or use the
``cartesian_controller_dashboard`` UI.

The FZI controller YAML (joint names, base/EE frames, PD gains) is
**not** shipped by this package -- it is robot-specific.  The launch
takes a ``(fzi_controller_yaml_package, fzi_controller_yaml_relpath)``
pair pointing into a per-robot bringup package; the path is resolved
at launch time via ``ament_index``.

Multi-instance setups (e.g. dual-arm robots) use the ``instance_name``
launch argument.  When set to a non-empty string ``<i>``:

* the orchestrator node is named ``cartesian_control_manager_<i>``;
* defaults are loaded from the YAML section
  ``cartesian_control_manager_<i>:`` (falling back to the legacy
  ``cartesian_control_manager:`` section if absent);
* the default FZI controller catalogue and the default JTC name are
  suffixed with ``_<i>`` so two orchestrators can share a single
  ``/controller_manager`` without name collisions.

Examples::

    ros2 launch cartesian_control_manager cartesian_control.launch.py
    ros2 launch cartesian_control_manager cartesian_control.launch.py \\
        active_controller_name:=cartesian_compliance_controller
    ros2 launch cartesian_control_manager cartesian_control.launch.py \\
        fzi_controller_yaml_package:=ur_robot_bringup \\
        fzi_controller_yaml_relpath:=config/fzi_preset.yaml
    ros2 launch cartesian_control_manager cartesian_control.launch.py \\
        instance_name:=left \\
        fzi_controller_yaml_package:=g1_bringup \\
        fzi_controller_yaml_relpath:=config/fzi_preset.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Catalogue of FZI Cartesian-controller plugins this launch file knows
# how to spawn.  Each entry is ``(name, kind, plugin_type)`` where
# ``kind`` is consumed by the orchestrator (drives which heartbeat
# topics it publishes) and ``plugin_type`` is the pluginlib class the
# spawner needs to register the controller with controller_manager.
_FZI_CONTROLLERS = [
    ("cartesian_force_controller",       "force",
     "cartesian_force_controller/CartesianForceController"),
    ("cartesian_motion_controller",      "motion",
     "cartesian_motion_controller/CartesianMotionController"),
    ("cartesian_compliance_controller",  "compliance",
     "cartesian_compliance_controller/CartesianComplianceController"),
]


_FALLBACKS = {
    # multi-instance identity ----------------------------------------------
    # Empty -> single-instance setup (legacy behaviour).  Set to a short
    # identifier (e.g. "left" / "right") to namespace the orchestrator
    # node, suffix the spawned FZI controllers, and read defaults from
    # the YAML section ``cartesian_control_manager_<instance>:``.
    "instance_name":         "",
    # connectivity ---------------------------------------------------------
    "wrench_topic":          "/ft_sensor/wrench_compensated",
    "joint_states_topic":    "/joint_states",
    "controller_manager_ns": "/controller_manager",
    "engaged_default":       False,
    # FZI controller wiring ------------------------------------------------
    # Default JTC name 'joint_trajectory_controller' matches ros2_control's
    # convention; per-robot configs override (Duco uses 'arm_1_controller',
    # UR uses 'scaled_joint_trajectory_controller').
    "active_controller_name":  "cartesian_force_controller",
    "fzi_jtc_controller_name": "joint_trajectory_controller",
    "fzi_target_frame":        "tool0",
    "fzi_target_rate_hz":      10.0,
    "fzi_service_timeout_sec": 2.0,
    # FZI controller YAML location.  Resolved at launch time as
    # ``get_package_share_directory(<package>) / <relpath>``.
    # Each per-robot bringup package ships its own preset.
    "fzi_controller_yaml_package": "duco_robot_bringup",
    "fzi_controller_yaml_relpath": "config/fzi_preset.yaml",
    # target_wrench setpoint published by the heartbeat (fallback when
    # no external publisher is active).  Interpreted by FZI in the
    # end-effector frame (hand_frame_control:=true, default), or the
    # robot base frame (hand_frame_control:=false).  All-zero ==
    # pure free-drive (sensor-only -> compliance to operator pushes).
    "target_wrench_force_x":   0.0,
    "target_wrench_force_y":   0.0,
    "target_wrench_force_z":   0.0,
    "target_wrench_torque_x":  0.0,
    "target_wrench_torque_y":  0.0,
    "target_wrench_torque_z":  0.0,
    # external high-rate target_wrench input.  When non-empty, the
    # orchestrator subscribes BEST_EFFORT and forwards each incoming
    # WrenchStamped immediately (no rate limiting) to every consumer,
    # after clamping to max_wrench_*.  Intended for teleop devices
    # publishing 50-125 Hz (e.g. SpaceMouse).  If no message arrives
    # for external_target_wrench_timeout_sec, the parameter setpoint
    # above takes over via the heartbeat.  Empty = disabled.
    "external_target_wrench_topic":       "",
    "external_target_wrench_timeout_sec": 0.2,
    # supervisor + state publish ------------------------------------------
    "loop_rate_hz":            50.0,
    "state_publish_rate_hz":   5.0,
    # safety ---------------------------------------------------------------
    "max_wrench_force":          80.0,
    "max_wrench_torque":         10.0,
    "engage_max_joint_velocity": 0.05,
    "ft_stale_after":            0.25,
    "joint_states_stale_after":  0.25,
}


def _defaults(section_name: str = "cartesian_control_manager"):
    """Load defaults for ``section_name`` from ``robot_config.yaml``.

    Returns ``(dict_of_defaults, source_string)``.  The source string is
    human-readable provenance for the launch log.  Falls back to
    ``_FALLBACKS`` (with a description of why) whenever:

    * the ``cct_common`` package cannot be imported,
    * the config file cannot be loaded,
    * the requested section is absent (with a further fallback to the
      legacy ``cartesian_control_manager:`` section before giving up).
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
    if cfg.has(section_name):
        sec = cfg.section(section_name)
        return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
                f"loaded {section_name!r} from {cfg.config_path}")
    # Per-instance section missing; try the legacy single-instance section
    # so dual-arm setups can opt into instance suffixes without forcing
    # a YAML schema change.
    legacy = "cartesian_control_manager"
    if section_name != legacy and cfg.has(legacy):
        sec = cfg.section(legacy)
        return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
                f"FALLBACK to legacy {legacy!r} section in {cfg.config_path} "
                f"(no {section_name!r} section)")
    return (dict(_FALLBACKS),
            f"FALLBACK (no {section_name!r} section in {cfg.config_path})")


def _bool(v):
    return str(v).lower()


def _coerce(value: str, like):
    """Coerce a CLI launch-arg string back to the type of ``like``.

    ``LaunchConfiguration.perform()`` always yields a string, but the
    orchestrator node declares each parameter with a typed default
    (bool / int / float / str).  Passing an overridden value through as a
    raw string makes rclpy raise ``InvalidParameterTypeException`` (e.g.
    ``max_wrench_force`` declared DOUBLE but given STRING ``'100.0'``).
    This casts the string back so CLI overrides -- and the conservative
    string overrides forwarded by ``cartesian_control_real.launch.py`` --
    keep the parameter's declared type.
    """
    if isinstance(like, bool):
        return str(value).strip().lower() in ("1", "true", "yes", "on")
    if isinstance(like, int) and not isinstance(like, bool):
        return int(value)
    if isinstance(like, float):
        return float(value)
    return value


# Keys that the orchestrator node does NOT consume directly; they're only
# read by the launch file itself (either to find the FZI YAML or to drive
# the instance-naming logic).
_LAUNCH_ONLY_KEYS = (
    "instance_name",
    "fzi_controller_yaml_package",
    "fzi_controller_yaml_relpath",
)


def generate_launch_description() -> LaunchDescription:
    # Declare launch args from the legacy section's defaults so that
    # ``ros2 launch ... --show-args`` keeps showing the same values as
    # before this change.  Instance-specific overrides happen inside the
    # OpaqueFunction below, where ``instance_name`` is known.
    d, source = _defaults()

    args = []
    for key, default in _FALLBACKS.items():
        if isinstance(default, bool):
            args.append(DeclareLaunchArgument(key, default_value=_bool(d[key])))
        else:
            args.append(DeclareLaunchArgument(key, default_value=str(d[key])))

    log = LogInfo(msg=(
        f"[cartesian_control_manager] legacy-section config: {source}"))

    def _build_instance(context, *_args, **_kwargs):
        """Assemble the node + spawners with instance-aware names.

        Runs at launch time so ``instance_name`` (and every other
        ``LaunchConfiguration``) can be resolved to its final value.
        """
        instance = LaunchConfiguration(
            "instance_name").perform(context).strip()
        suffix = f"_{instance}" if instance else ""

        # Re-load defaults from the instance-specific section so dual-arm
        # setups can keep per-arm tuning in their own YAML section.
        section_name = (f"cartesian_control_manager{suffix}" if instance
                        else "cartesian_control_manager")
        section_defaults, section_source = _defaults(section_name)

        # Per-instance suffixed catalogue.  Both the spawner targets and
        # the orchestrator's ``available_controllers`` parameter use the
        # suffixed names so two managers can coexist on a shared
        # /controller_manager without colliding.
        catalogue = [
            (f"{name}{suffix}", kind, plugin_type)
            for name, kind, plugin_type in _FZI_CONTROLLERS
        ]

        # Resolve FZI YAML path (shared across instances; FZI's spawner
        # reads its own controller's section from the same file).
        yaml_pkg = LaunchConfiguration(
            "fzi_controller_yaml_package").perform(context)
        yaml_relpath = LaunchConfiguration(
            "fzi_controller_yaml_relpath").perform(context)
        try:
            pkg_share = get_package_share_directory(yaml_pkg)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"[cartesian_control_manager] could not resolve FZI YAML "
                f"package '{yaml_pkg}': {type(exc).__name__}: {exc}") from exc
        fzi_yaml = os.path.join(pkg_share, yaml_relpath)
        if not os.path.isfile(fzi_yaml):
            raise RuntimeError(
                f"[cartesian_control_manager] FZI YAML not found at "
                f"{fzi_yaml} (resolved from package={yaml_pkg!r} "
                f"relpath={yaml_relpath!r})")

        # Resolve every parameter.  Rule:
        #   * If the CLI value differs from the launch-arg default (the
        #     legacy-section value), the operator explicitly overrode it
        #     -- pass that through.
        #   * Otherwise, prefer the per-instance section value (which may
        #     itself fall back to _FALLBACKS).
        # This lets the per-arm YAML section be the source of truth for
        # values the operator did not override on the CLI, while
        # preserving CLI ergonomics (operator overrides always win).
        parameters: dict = {}
        for key, fallback in _FALLBACKS.items():
            if key in _LAUNCH_ONLY_KEYS:
                continue
            cli_value = LaunchConfiguration(key).perform(context)
            legacy_default = _bool(d[key]) if isinstance(fallback, bool) \
                else str(d[key])
            if cli_value != legacy_default:
                parameters[key] = _coerce(cli_value, fallback)
            else:
                parameters[key] = section_defaults.get(key, fallback)

        # The orchestrator catalogue must match the spawned controllers.
        parameters["available_controllers"] = [n for n, _, _ in catalogue]
        parameters["controller_kinds"] = [k for _, k, _ in catalogue]
        parameters["instance_name"] = instance

        # Default ``active_controller_name`` / ``fzi_jtc_controller_name``
        # also follow the suffix convention -- but only if the operator
        # did not override them on the CLI and the per-instance section
        # did not pin them either.
        legacy_active = str(d["active_controller_name"])
        if (parameters.get("active_controller_name") == legacy_active
                and instance):
            parameters["active_controller_name"] = f"{legacy_active}{suffix}"
        legacy_jtc = str(d["fzi_jtc_controller_name"])
        if (parameters.get("fzi_jtc_controller_name") == legacy_jtc
                and instance):
            parameters["fzi_jtc_controller_name"] = f"{legacy_jtc}{suffix}"

        node_name = (f"cartesian_control_manager{suffix}" if instance
                     else "cartesian_control_manager")

        return [
            LogInfo(msg=(
                f"[cartesian_control_manager] instance={instance!r} "
                f"node_name={node_name!r} section_config: {section_source}; "
                f"controllers={[n for n,_,_ in catalogue]}; "
                f"FZI YAML={fzi_yaml}")),
            # Pre-load each FZI controller into the live controller_manager
            # (inactive).  Our orchestrator activates the *selected* one
            # on engage via the SwitchController service.  All controllers
            # share the same YAML config (controller_manager picks the
            # section keyed by the controller's own name -- suffixed when
            # an instance is set, so the per-robot YAML must have one
            # section per spawned controller name).
            *[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        name,
                        "-c", LaunchConfiguration("controller_manager_ns"),
                        "-p", fzi_yaml,
                        "-t", plugin_type,
                        "--inactive",
                    ],
                    output="screen",
                )
                for name, _kind, plugin_type in catalogue
            ],
            Node(
                package="cartesian_control_manager",
                executable="cartesian_control_node",
                name=node_name,
                output="screen",
                emulate_tty=True,
                parameters=[parameters],
            ),
        ]

    return LaunchDescription([
        log,
        *args,
        OpaqueFunction(function=_build_instance),
    ])
