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
| `cartesian_control_manager` | spawns FZI's `cartesian_force_controller` / `cartesian_motion_controller` / `cartesian_compliance_controller` (all inactive), relays the wrench, publishes a zero `target_wrench` heartbeat, exposes engage / disengage `Trigger` services and runs a safety supervisor | -- |
| `cartesian_controller_dashboard` | optional FastAPI web UI: engage / disengage, controller selection, live gain tuning, optional tool-frame editing | `8120` |
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

## Supported robots

The toolkit itself is robot-agnostic. Currently exercised against:

* Duco GCR5_910 (in [`yizhongzhang1989/duco_control`](https://github.com/yizhongzhang1989/duco_control))
* Universal Robots (planned)

## License

MIT (same as the wrapped FZI fork).
