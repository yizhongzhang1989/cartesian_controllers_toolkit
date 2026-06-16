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
 (bare URDF, the bringup's input)      strip+augment+validate        │  (urdf_from_topic:=true, single source)
                                       single writer                  └─▶ robot_state_publisher ──/robot_description──▶ TF / RViz / MoveIt
                                                                          (mirror: RSP re-publishes the AUGMENTED URDF)
```

> **What ends up on `/robot_description`?** The *augmented* URDF, not the bare
> one. The bringup hands RSP the bare URDF at startup, but the manager then
> mirrors its canonical (augmented) URDF back into RSP via `set_parameters`
> (`update_robot_state_publisher:=true`, the default), so RSP re-publishes the
> augmented URDF on `/robot_description`. In steady state **both**
> `/robot_description` and `/cartesian/robot_description` carry the same
> augmented URDF — the bare URDF only exists transiently at startup and inside
> the manager (its stripped base copy). The two topics differ by **owner /
> purpose**, not content: `/robot_description` is RSP's (drives TF, RViz,
> MoveIt, FT-frame lookups); `/cartesian/robot_description` is the manager's
> latched single-source for the controllers. Set
> `update_robot_state_publisher:=false` to leave `/robot_description` bare (then
> only `/cartesian/robot_description` has the aux frames, and TF will not).

## Why a topic (not the controller_manager URDF)

In ros2_control (Humble 2.53.1) the controller_manager's URDF is **immutable**
after first load (`ResourceManager ... Ignoring attempt to reload`). The FZI
controllers in this fork were therefore changed to read their URDF from a
latched topic (`urdf_from_topic:=true`): they defer the kinematic-chain build
until the URDF arrives and gate activation on it. So the canonical topic is the
single, consistent source — offset edits and frame additions published here
reach every controller (and TF, via the RSP mirror).

## Interfaces — what controls the URDF

The canonical URDF is driven entirely through **ROS topics + parameters**; the
node exposes no services of its own (it *calls* `robot_state_publisher`'s
`set_parameters`). All `~/...` names below resolve under the node, i.e.
`/aux_frame_manager/...` by default.

### Topics

| dir | topic (default) | type | QoS | purpose |
|---|---|---|---|---|
| sub | `base_urdf_topic` = `/robot_description` | `std_msgs/String` | latched¹ | manufacturer URDF in (the base to augment) |
| **sub** | **`~/set_aux_frames`** | **`std_msgs/String`** (JSON) | depth 10 | **live control: replace the whole aux-frame list** |
| pub | `output_topic` = `/cartesian/robot_description` | `std_msgs/String` | latched¹ | the canonical augmented URDF out — controllers read this |
| pub | `~/status` | `std_msgs/String` (JSON) | latched¹ | last action / current frames / health |

¹ latched = `RELIABLE` + `TRANSIENT_LOCAL`, depth 1 — a late subscriber still
receives the current value.

The node additionally **calls** `/<robot_state_publisher>/set_parameters`
(`rcl_interfaces/srv/SetParameters`) to mirror the canonical URDF into TF/RViz
when `update_robot_state_publisher:=true`.

### `~/set_aux_frames` — the live control interface

This is **the** interface for operating the URDF at runtime. Publish a **JSON
array** of frame objects; the message **replaces the entire managed aux-frame
list** (it is not an incremental patch), so:

* **edit an offset** → send the full list with that frame's `xyz` / `rpy` changed;
* **add a frame** → send the list with the new frame included;
* **remove a frame** → send the list without it (an empty array `[]` removes all,
  making the canonical URDF equal to the bare base).

Each frame object is `{"name": str, "parent": str, "xyz": [x,y,z], "rpy": [r,p,y]}`
— `name` and `parent` are required; `xyz` (metres) and `rpy` (radians) default to
`[0,0,0]`. `parent` must be a base-URDF link or another aux frame. Frames may be
listed in **any order** — a child may precede its parent; the manager
topologically sorts them before building.

```bash
# Set ft_sensor_link 5 cm above link_6, keep compliance_link on it.
# (full list — this REPLACES whatever frames the manager currently holds)
ros2 topic pub --once /aux_frame_manager/set_aux_frames std_msgs/msg/String \
  '{data: "[{\"name\":\"ft_sensor_link\",\"parent\":\"link_6\",\"xyz\":[0,0,0.05]},{\"name\":\"compliance_link\",\"parent\":\"ft_sensor_link\"}]"}'

# Remove every aux frame (canonical URDF collapses to the base):
ros2 topic pub --once /aux_frame_manager/set_aux_frames std_msgs/msg/String '{data: "[]"}'
```

On each message the manager strips its previously-managed frames from the stored
base, re-augments with the new list, **validates** (parent exists, no cycle, no
duplicate, chain reachable), then republishes `output_topic` and re-mirrors to
RSP. An **invalid** list is rejected: the previous canonical URDF is kept and
`~/status` reports the error.

> The dashboards are just clients of this topic. The aux_frame 3D dashboard
> (`dashboard_port`) and the `cartesian_controller_dashboard` "Tool frames"
> panel both publish here; the latter also writes the values back to
> `robot_config.yaml` so they persist to the next launch.

### `~/status` — feedback

Latched `std_msgs/String` JSON, republished on every (re)build:

```json
{"message": "ok: published canonical URDF with frames [ft_sensor_link, compliance_link]",
 "output_topic": "/cartesian/robot_description",
 "base_topic": "/robot_description",
 "aux_frames": ["ft_sensor_link", "compliance_link"],
 "have_canonical": true,
 "mirror_to_rsp": true}
```

`message` starts with `ok:` on success or `error:` when a frame set was rejected
(the canonical output is then unchanged). `aux_frames` is the list of frame
names currently in the canonical URDF — read it to confirm an edit landed.

### Startup / static frame sources

For frames known at launch time (instead of, or in addition to, live edits):

* **config file** — `aux_frames_section` names a top-level key in
  `config/robot_config.yaml` whose `aux_frames:` list (each
  `{name, parent, xyz, rpy}`) is read via `cct_common`;
* **direct argument** — `aux_frames` is an **rcl-safe compact** string of
  `name:parent[:x,y,z[:r,p,yw]]` specs separated by `;` (a bracketed YAML/JSON
  string does *not* survive the rcl parameter parser). It overrides/extends the
  config-file frames (override by name, append new).

A live `~/set_aux_frames` message supersedes both for the rest of the session.

### Guard output (`aux_frame_guard`)

| dir | topic | type | purpose |
|---|---|---|---|
| pub | `<output_topic>_ready` = `/cartesian/robot_description_ready` | `std_msgs/Bool` | latched `true` once the endpoint/reference frames are present **and** in the `base->ee` chain |

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

Set at launch (read once at startup); the live `~/set_aux_frames` topic is the
runtime control surface.

| param | default | meaning |
|---|---|---|
| `base_urdf_topic` | `/robot_description` | manufacturer URDF in |
| `output_topic` | `/cartesian/robot_description` | canonical URDF out (latched) |
| `update_robot_state_publisher` | `true` | mirror canonical URDF to RSP (one TF truth) |
| `robot_state_publisher_name` | `robot_state_publisher` | RSP node whose `robot_description` is mirrored |
| `aux_frames_section` | `""` | config-file section holding the `aux_frames` list |
| `config_file` | `""` (auto) | path to the config YAML (`""` → `cct_common` auto-resolve) |
| `aux_frames` | `""` | compact `name:parent[:x,y,z[:r,p,yw]]` specs, `;`-separated |

`aux_frame_guard` parameters: `robot_description_topic`
(`/cartesian/robot_description`), `robot_base_link` (`base_link`),
`end_effector_link` (`""`), `required_frames` (`['']`).

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
