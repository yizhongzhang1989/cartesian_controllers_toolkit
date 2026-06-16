# aux_frame_manager

Single-writer owner of the **canonical augmented `robot_description`** for the
FZI Cartesian controllers. It consumes the manufacturer URDF from the basic
robot bringup (unchanged), appends the configured **auxiliary fixed frames**
(FT-sensor / compliance / operation-origin links), and publishes the one
canonical URDF on a **latched topic** that the controllers read as their *only*
URDF source. This removes the previous runtime URDF divergence (controller
manager vs robot_state_publisher vs each controller's private param) and lets
aux frames be added **without a custom bringup**.

```
basic bringup ──/robot_description──▶ aux_frame_manager ──/cartesian/robot_description (latched)──▶ FZI controllers
 (manufacturer URDF, unchanged)        strip+augment+validate        │  (urdf_from_topic:=true, single source)
                                       single writer                  └─▶ robot_state_publisher (mirror, TF/RViz)
```

## Why a topic (not the controller_manager URDF)

In ros2_control (Humble 2.53.1) the controller_manager's URDF is **immutable**
after first load (`ResourceManager ... Ignoring attempt to reload`). The FZI
controllers in this fork were therefore changed to read their URDF from a
latched topic (`urdf_from_topic:=true`): they defer the kinematic-chain build
until the URDF arrives and gate activation on it. So the canonical topic is the
single, consistent source — offset edits and frame additions published here
reach every controller (and TF, via the RSP mirror).

## Frame sources (Req: config file AND direct argument)

* **Config file** — `aux_frames_section` names a top-level key in
  `config/robot_config.yaml` whose `aux_frames:` list (each
  `{name, parent, xyz, rpy}`) is read via `cct_common`.
* **Direct argument** — `aux_frames` is an **rcl-safe compact** string of
  `name:parent[:x,y,z[:r,p,yw]]` specs separated by `;` (a bracketed YAML/JSON
  string does *not* survive the rcl parameter parser). It overrides/extends the
  config-file frames (override by name, append new).
* **Live edit** — publish a JSON frame list on `~/set_aux_frames`
  (`std_msgs/String`) to change offsets or add/remove frames at runtime.

## Run

```bash
# after the basic bringup is up (publishes /robot_description):
ros2 launch aux_frame_manager cartesian_urdf_source.launch.py \
    end_effector_link:=compliance_link \
    aux_frames:='ft_sensor_link:link_6; compliance_link:ft_sensor_link'
# manager alone:
ros2 launch aux_frame_manager aux_frame_manager.launch.py
```

Then configure the FZI controllers with:
```yaml
<controller_name>:
  ros__parameters:
    urdf_from_topic: true
    robot_description_topic: "/cartesian/robot_description"
    robot_base_link: "base_link"
    end_effector_link: "compliance_link"   # may be an aux frame!
    ft_sensor_ref_link: "ft_sensor_link"
    # joints / command_interfaces / solver ... as usual
```

## Nodes

* **`aux_frame_manager`** — builds and publishes the canonical URDF; mirrors it
  to `robot_state_publisher` (`update_robot_state_publisher:=true`). Sole writer.
* **`aux_frame_guard`** — verifies the configured endpoint/reference frames are
  present **and in-chain** (`base->ee`, exactly FZI's `getChain` +
  `robotChainContains` constraint), latches `<topic>_ready` (Bool), and prints
  an actionable error naming missing frames if not — instead of FZI's cryptic
  "robot_description is empty".
* **`aux_frame_dashboard`** *(optional)* — a small web UI (Three.js 3D canvas)
  that draws the canonical robot and **highlights the added aux frames** (orange
  marker + triad + label) versus the original links (grey meshes), plus a live
  editor to add / edit / remove frames at runtime. It is a thin client of the
  manager: it reads the canonical + base URDF topics and `~/status`, takes link
  poses from **TF** (no extra FK dependency), and drives `~/set_aux_frames`. Off
  by default; start it by passing `dashboard_port` to either launch file.

```bash
# manager + dashboard on http://localhost:8160
ros2 launch aux_frame_manager aux_frame_manager.launch.py \
    aux_frames:='op_tip:compliance_link:0,0,0.05' dashboard_port:=8160
```

## Key parameters (`aux_frame_manager`)

| param | default | meaning |
|---|---|---|
| `base_urdf_topic` | `/robot_description` | manufacturer URDF in |
| `output_topic` | `/cartesian/robot_description` | canonical URDF out (latched) |
| `update_robot_state_publisher` | `true` | mirror canonical URDF to RSP (one TF truth) |
| `aux_frames_section` | `""` | config-file section holding the `aux_frames` list |
| `config_file` | `""` (auto) | path to the config YAML |
| `aux_frames` | `""` | compact `name:parent[:x,y,z[:r,p,yw]]` specs, `;`-separated |

## Loop safety

The manager may push the canonical URDF to RSP, which re-publishes it on the
base topic. `build_canonical_urdf` **strips** any managed aux frames from the
incoming URDF before augmenting, so re-processing its own echo yields identical
bytes and the loop terminates. Re-processing is idempotent (unit-tested).

## Tests

```bash
python3 -m pytest external/cartesian_controllers_toolkit/aux_frame_manager/test -q
```
Pure-logic offline tests (merge/validate/strip/augment/chain) — no ROS needed.
