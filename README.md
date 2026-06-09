# cartesian_controllers_toolkit

Robot-agnostic ROS 2 (Humble) packages that wrap FZI's
[cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)
into a turn-key Cartesian compliance / force / motion stack.  Designed to
be dropped into any ROS 2 workspace as a git submodule and reused across
different robot platforms — the per-robot bringup package supplies the
URDF, joints, sensor topics, and the FZI YAML preset.

## Packages

| package | purpose | runtime port |
|---|---|---|
| `cct_common` | centralised config loader (`config/robot_config.yaml`) and shared XML / URDF helpers. Named `cct_common` so the toolkit drops into any workspace as a submodule without colliding with a host `common` package | -- |
| `cartesian_control_manager` | spawns FZI's `cartesian_force_controller` / `cartesian_motion_controller` / `cartesian_compliance_controller` (all inactive), relays the wrench, optionally publishes a `target_wrench` heartbeat (off by default — see `publish_target_wrench`), exposes engage / disengage `Trigger` services and runs a safety supervisor | -- |
| `cartesian_controller_dashboard` | optional web UI (stdlib `http.server`): engage / disengage, controller selection, live gain tuning, optional tool-frame editing | `8120` |
| `ft_sensor_gravity_compensation` | subscribes to a raw wrench topic + `/tf`, publishes a gravity-compensated wrench, ships its own calibration UI | `8100` |
| `ft_sensor_dashboard` | sensor-agnostic web UI for any `geometry_msgs/WrenchStamped` topic | `8080` |

## How to use

The toolkit is intended to be consumed as a git submodule under your
workspace's `external/` (or wherever fits your layout). Colcon discovers
the packages by recursive base-path search — no extra configuration
needed.

```bash
cd <your_workspace>
git submodule add https://github.com/yizhongzhang1989/cartesian_controllers_toolkit.git external/cartesian_controllers_toolkit
git submodule update --init --recursive

# colcon will pick up all 5 packages automatically
colcon build --symlink-install
```

Per-robot wiring lives entirely in your workspace's `config/robot_config.yaml`
plus a bringup package that ships an `fzi_preset.yaml` describing the
robot's KDL chain + initial gains. See the
[`cartesian_control_manager` README](cartesian_control_manager/README.md)
for the full list of keys.

If no `robot_config.yaml` is present, the toolkit loads its packaged
`toolkit_defaults.yaml` (in `cct_common/config/`) so every package still
starts with sensible defaults. The full lookup order — explicit
`ROBOT_CONFIG_PATH`, then the workspace config, then the packaged default —
is documented in the [`cct_common` README](cct_common/README.md#config-resolution-order).

## Porting to a new robot

The toolkit packages never change between robots — only the consuming
workspace's config and one per-robot preset do. Concretely:

1. **Have a working `ros2_control` stack** for your arm: a URDF reachable
   as `robot_description`, a running `controller_manager`, and a
   `JointTrajectoryController` (JTC) that moves the arm. The toolkit
   swaps this JTC with an FZI controller on engage.

2. **(Optional) add aux frames.** FZI's force/compliance controllers
   need the F/T frame (and usually a dedicated tool/`compliance_link`)
   to exist in the KDL chain. Declare them once under
   `config/robot_config.yaml::<your_bringup>.aux_frames`; the
   `cct_common.urdf_loader` helper appends them to the URDF at bring-up.
   With zero offsets the tip pose is unchanged, so JTC behaviour is
   identical.

3. **Copy the preset template** into your bringup package and edit the
   joint names + frame links to match your URDF:
   ```bash
   cp $(ros2 pkg prefix cartesian_control_manager)/share/cartesian_control_manager/config/fzi_preset.example.yaml \
      src/<your_bringup>/config/fzi_preset.yaml
   ${EDITOR:-nano} src/<your_bringup>/config/fzi_preset.yaml
   ```
   (Source is also at
   [`cartesian_control_manager/config/fzi_preset.example.yaml`](cartesian_control_manager/config/fzi_preset.example.yaml).)

4. **Add the toolkit sections to `config/robot_config.yaml`.** At minimum
   a `cartesian_control_manager:` block pointing the manager at your
   preset and your JTC, plus `ft_sensor_gravity_compensation:` and
   `cartesian_controller_dashboard:` if you use them:
   ```yaml
   cartesian_control_manager:
     wrench_topic: "/ft_sensor/wrench_compensated"
     joint_states_topic: "/joint_states"
     fzi_jtc_controller_name: "joint_trajectory_controller"  # your JTC
     fzi_target_frame: "tool0"                               # your tool link
     fzi_controller_yaml_package: "<your_bringup>"
     fzi_controller_yaml_relpath: "config/fzi_preset.yaml"
   ```
   Keys not listed fall back to the launch file's `_FALLBACKS`; see the
   [`cartesian_control_manager` README](cartesian_control_manager/README.md)
   for the full schema.

5. **Bring it up** in order — robot, F/T + gravity compensation, then the
   manager:
   ```bash
   ros2 launch <your_bringup> <robot>.launch.py
   ros2 launch ft_sensor_gravity_compensation compensation.launch.py
   ros2 launch cartesian_control_manager cartesian_control_real.launch.py
   ```

6. **Verify, then engage.** Check the three FZI controllers loaded
   inactive (`ros2 control list_controllers`) and that a compensated
   wrench is flowing, then:
   ```bash
   ros2 service call /cartesian_control_manager/engage std_srvs/srv/Trigger
   ```

## Supported robots

The toolkit itself is robot-agnostic. Currently exercised against:

* Duco GCR5_910 (in [`yizhongzhang1989/duco_control`](https://github.com/yizhongzhang1989/duco_control))
* Universal Robots (planned)

## License

MIT (same as the wrapped FZI fork).
