# robot_control_test

An interactive **web bench to verify a robot works correctly with each of its
ros2_control controllers**, with a **3D canvas** showing the live robot status.

It is a thin HTTP/ROS client (it imports no controller internals): it discovers
the controllers on `/controller_manager`, lets you activate one, and sends it
safe, speed-limited test commands while the 3D view shows the arm move. Use it
to answer *"is this controller wired up and does the robot actually move the way
I expect?"* before building anything on top of it.

It is modelled on
[`ikt_pose_commander`](../../inverse_kinematics_toolkit/ikt_pose_commander) and
the `aux_frame_manager` dashboard, but uses **TF for forward kinematics** (no
Pinocchio / no `ikt_core` dependency), so it stays inside
`cartesian_controllers_toolkit`.

## What it does

1. Reads the robot's links / movable joints from **`/robot_description`** (URDF)
   and the live per-link poses from **TF** (`base_frame → link`) to draw the
   robot in 3D (meshes + skeleton + a TCP triad). The viewer renders **STL**
   meshes; robots whose URDF ships another format (e.g. UR's COLLADA `.dae`)
   show the kinematic skeleton instead, and the **mesh** toggle disables itself
   automatically (labelled *mesh (unsupported)*).
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

## Safety

- The node **commands the real robot** — keep the workspace clear and an e-stop
  in reach. It starts with nothing engaged.
- Joint motion is bounded by `max_joint_speed` (rad/s): JTC moves are timed from
  it, forward-position streams are ramped to it.
- Forward-position and Cartesian streams start from / are seeded by the robot's
  **current** pose, so engaging does not jump the arm; the first jog moves it.
- **Stop** holds the arm at its current measured pose; **Disengage** deactivates
  the controller.

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

Defaults may also be set in a `robot_control_test:` section of the toolkit
config (`robot_config.yaml`); CLI args override.

## Architecture

`control_test_node.py` is a single node: a `ThreadingHTTPServer` (daemon thread)
serving the dashboard, plus rclpy on a `MultiThreadedExecutor` with a
`ReentrantCallbackGroup` so the synchronous `switch_controller` call issued from
the HTTP handler thread never deadlocks the ROS spin. The web assets live in
`robot_control_test/static/` (`index.html`, `dashboard.css`, `dashboard.js`, the
Three.js `viewer.js`, and the vendored Three.js bundle). Port 8200 keeps it
clear of the other toolkit dashboards (8080 / 8100 / 8120 / 8140 / 8160 / 8180).
