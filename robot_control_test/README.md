# robot_control_test

An interactive **web bench to verify a robot works correctly with each of its
ros2_control controllers**, with a **3D canvas** showing the live robot status.

It is a thin HTTP/ROS client (it imports no controller internals): it discovers
the controllers on `/controller_manager`, lets you activate one, and sends it
safe, speed-limited test commands while the 3D view shows the arm move. Use it
to answer *"is this controller wired up and does the robot actually move the way
I expect?"* before building anything on top of it.

It uses **TF for forward kinematics** (so it needs no Pinocchio or other
kinematics library), depending only on `rclpy` and standard ROS 2 message / TF
packages. That keeps it self-contained, so it drops into any ros2_control robot
workspace.

## What it does

1. Reads the robot's links / movable joints from **`/robot_description`** (URDF)
   and the live per-link poses from **TF** (`base_frame → link`) to draw the
   robot in 3D (meshes + skeleton + a TCP triad) with a live **joint-angle
   panel** (one colour-coded bar per joint, labelled with the real URDF joint
   name and ordered along the kinematic chain — base → tip — regardless of the
   `/joint_states` publish order; in degrees, or mm for a prismatic joint) and a
   joint-state freshness readout. The viewer renders **STL** and **COLLADA
   (`.dae`)** meshes (so both Duco and UR robots show full geometry); a robot
   whose URDF ships some other mesh format shows the kinematic skeleton instead,
   and the **mesh** toggle disables itself automatically (labelled
   *mesh (unsupported)*).
2. Discovers every controller on **`/controller_manager`** and classifies it:

   | kind | example plugin type | how you drive it |
   |---|---|---|
   | **joint trajectory** | `*/JointTrajectoryController`, `ur_controllers/ScaledJointTrajectoryController` | per-joint sliders / ± nudge → a timed `trajectory_msgs/JointTrajectory` on `/<ctrl>/joint_trajectory` |
   | **forward position** | `forward_command_controller/ForwardCommandController`, `position_controllers/JointGroupPositionController` | per-joint sliders / ± → a velocity-ramped `std_msgs/Float64MultiArray` streamed to `/<ctrl>/commands` |
   | **cartesian motion** | `cartesian_motion_controller/CartesianMotionController` | X/Y/Z/RX/RY/RZ jog of a `geometry_msgs/PoseStamped` on `/<ctrl>/target_frame` (seeded from the current TCP) |
   | **cartesian compliance** | `cartesian_compliance_controller/CartesianComplianceController` | Cartesian jog **and** a target wrench |
   | **cartesian force** | `cartesian_force_controller/CartesianForceController` | a `geometry_msgs/WrenchStamped` setpoint on `/<ctrl>/target_wrench` |

3. **Engaging** a controller activates it via `switch_controller` and
   deactivates any active controller that claims the same command interfaces, so
   the mutual exclusion ros2_control requires is handled for you.
4. Drives the engaged controller with **speed-limited** commands and shows the
   live joint angles / TCP pose so you can confirm correct, safe motion.

### The 3D view

A floating **3D View** panel (top-right) reports the link / joint counts and the
currently **selected** link and its **parent**, with toggles for **mesh**,
**labels** and per-link **frames**, plus **Fit view** (and a collapse button).
Drag to orbit, wheel to zoom. **Click a link** — its mesh or its label — to
select it: the mesh is highlighted, a thick RGB triad is drawn at that link's
frame, and its parent is shown in the panel. Click empty space or press **Esc**
to deselect. Turning **frames** on additionally draws a thin triad at every link.

## Safety

- The node **commands the real robot** — keep the workspace clear and an e-stop
  in reach. It starts with nothing engaged.
- Joint motion is bounded by `max_joint_speed` (rad/s): JTC moves are timed from
  it, forward-position streams are ramped to it.
- Forward-position and Cartesian streams start from / are seeded by the robot's
  **current** pose, so engaging does not jump the arm; the first jog moves it.
- **Stop** holds the arm at its current measured pose; **Disengage** deactivates
  the controller.

## Requirements & build

- **ROS 2** (tested on Humble) with `ros2_control`, and a robot already up (real
  or `use_fake_hardware:=true`) so `/robot_description`, `/joint_states` and
  `/controller_manager` exist.
- A modern **browser** (WebGL2 + ES-module `importmap` support) for the
  dashboard.
- No extra Python packages: the node uses only `rclpy`, standard ROS 2
  messages, `tf2_ros` and `ament_index_python`; the Three.js viewer is vendored
  under `static/vendor/`.

Build it in any colcon workspace and source the overlay:

```bash
cd ~/ros2_ws                       # your workspace
colcon build --packages-select robot_control_test --symlink-install
source install/setup.bash
```

## Run

Bring the robot up first (real or `use_fake_hardware:=true`), then:

```bash
ros2 launch robot_control_test control_test.launch.py        # http://localhost:8200
```

Open `http://localhost:8200`, pick a controller → **Engage** → jog it.

### Launch / node parameters

| arg / param | default | meaning |
|---|---|---|
| `dashboard_port` | `8200` | dashboard HTTP port |
| `controller_manager` | `/controller_manager` | where to discover / switch controllers |
| `robot_description_topic` | `/robot_description` | URDF source for the 3D model |
| `joint_states_topic` | `/joint_states` | live joint feedback |
| `base_frame` | `base_link` | TF frame the 3D view / Cartesian targets are expressed in |
| `tip_frame` | `""` | TCP frame (empty → the URDF's last leaf link) |
| `max_joint_speed` | `0.5` | joint speed limit (rad/s) for JTC moves + FPC ramps |
| `send_rate` | `100.0` | forward-position stream rate (Hz) |

These defaults are built into the launch file, so the package runs standalone
anywhere. (Optionally, if a `cct_common` config package exposing `get_config()`
is on the path, a `robot_control_test:` section there supplies the defaults
instead; when it is absent the built-in values above are used.) CLI args always
override.

## ROS interfaces

The node (`/robot_control_test`) uses only standard ros2_control topics and
services — it defines **no custom messages**. Configurable names are shown with
their default in parentheses.

| direction | name | type | purpose |
|---|---|---|---|
| subscribe | `robot_description_topic` (`/robot_description`) | `std_msgs/String` | URDF → 3D model + joint limits |
| subscribe | `joint_states_topic` (`/joint_states`) | `sensor_msgs/JointState` | live joint feedback |
| TF | `base_frame` → each link, → `tip_frame` | tf2 | per-link poses for the 3D view + TCP triad |
| service client | `<controller_manager>/list_controllers` | `controller_manager_msgs/ListControllers` | discover + classify controllers |
| service client | `<controller_manager>/switch_controller` | `controller_manager_msgs/SwitchController` | engage / disengage (`BEST_EFFORT`) |
| publish | `/<ctrl>/joint_trajectory` | `trajectory_msgs/JointTrajectory` | joint-trajectory moves |
| publish | `/<ctrl>/commands` | `std_msgs/Float64MultiArray` | forward-position stream |
| publish | `/<ctrl>/target_frame` | `geometry_msgs/PoseStamped` | Cartesian motion / compliance target |
| publish | `/<ctrl>/target_wrench` | `geometry_msgs/WrenchStamped` | Cartesian force / compliance setpoint |

The command publishers are created for the **engaged** controller only.

## Architecture

`control_test_node.py` is a single node: a `ThreadingHTTPServer` (daemon thread)
serving the dashboard, plus rclpy on a `MultiThreadedExecutor` with a
`ReentrantCallbackGroup` so the synchronous `switch_controller` call issued from
the HTTP handler thread never deadlocks the ROS spin. The web assets live in
`robot_control_test/static/` (`index.html`, `dashboard.css`, `dashboard.js`, the
Three.js `viewer.js`, and the vendored Three.js bundle). The dashboard defaults
to port 8200; override it with `dashboard_port` if something else already uses
that port.

### HTTP API

The browser drives the node through a small JSON API on the same port, so you
can script it from `curl` or another tool as well:

| method + path | body | effect |
|---|---|---|
| `GET /api/state` | — | full snapshot: `have_model`, `links`, `link_tf`, `movable_joints`, `joint_values`, `js_age`, `tcp`, `controllers`, mesh flags |
| `GET /mesh?pkg=<pkg>&path=<rel>` | — | mesh proxy: resolves `package://` via `ament_index_python`, serves STL / COLLADA |
| `POST /api/engage`, `/api/disengage` | `{name}` | switch the active controller |
| `POST /api/joint/set`, `/api/joint/jog` | `{positions}` / `{joint, delta}` | joint targets / single-joint nudge |
| `POST /api/joint/sync`, `/api/joint/stop` | — | seed sliders from the current pose / hold |
| `POST /api/cart/jog`, `/api/cart/reset` | `{axis, delta}` / — | Cartesian jog / reset to TCP |
| `POST /api/wrench/set`, `/api/wrench/zero` | `{force, torque}` / — | target wrench / zero it |
| `GET /`, `/vendor/*`, `*.css` `*.js` `*.html` | — | dashboard static assets |

## License

MIT — see the `<license>` tag in `package.xml`.
