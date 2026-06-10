# fpc_test_dashboard

A web platform to **test a robot's forward-position-control (FPC) capability**
before deploying host-side admittance / Cartesian force control on top of it.

Layering admittance control on a robot that has **no built-in force module**
works only if its position-passthrough interface can faithfully follow a
streamed reference. This dashboard measures exactly that: it drives the robot's
`ForwardCommandController` through the same kinds of command profiles an
admittance controller will produce, and shows you the tracking quality, latency
and vibration so you know whether (and how) the robot is usable.

## What it does

1. Reads the robot's movable joints from **`/robot_description`** (the URDF).
2. Discovers **forward-position controllers** on `/controller_manager` and the
   ordered joints each one commands. Both
   `forward_command_controller/ForwardCommandController` (e.g. Duco) and
   `position_controllers/JointGroupPositionController` (e.g. Universal Robots)
   are recognised.
3. You pick a controller and a subset of its joints in the web UI.
4. For each selected joint it runs an automatic battery and plots it live:

   | test | what the command is | why it matters |
   |---|---|---|
   | **Smooth** | a continuous sine refreshed every control tick | best case: the arm's intrinsic tracking lag + ripple |
   | **Stair** | the same sine **discretised into uniform `stair_step`° steps** (a visible staircase) | each sharp riser is a one-tick jump that excites resonance — what stepped / low-rate (teleop / admittance) commands do |
   | **Smoothed** | the staircase run through a 2-pole low-pass first | how much a host-side command smoother rounds the risers and removes the vibration |

5. Records the **actual joint motion** from `/joint_states` and, if you provide a
   `wrench_topic`, the **wrist wrench** — the *independent ground truth* for real
   end-effector vibration (admittance control reads this same signal, so its
   cleanliness is what ultimately matters).
6. Reports per-segment metrics: **pos HF** (joint ripple), **force HF** (real
   vibration), **lag** (phase delay) and **RMSE** (tracking error).
7. **Saves every run** as a self-contained report you can review later (see
   [Saved runs](#saved-runs)).

## Usage

Launch **after** the robot is up (so `/robot_description` and
`/controller_manager` exist):

```bash
ros2 launch fpc_test_dashboard dashboard.launch.py \
    wrench_topic:=/right_arm_force_torque_sensor_broadcaster/wrench
# then open http://localhost:8140
```

In the page: pick the controller → tick the joints → (optionally tweak
parameters) → **Run test**. The trace plots command vs. actual live; the results
table fills in as each segment finishes. **Stop** aborts and ramps back.

## Saved runs

Every run is persisted (like `temp/fp_control_test/exp1_ros2_control`) so you can
check it later — no need to keep the page open. For each run the dashboard writes
a timestamped folder under `report_dir`
(default `~/.ros/fpc_test_dashboard/runs/<timestamp>_<controller>/`):

| file | contents |
|---|---|
| `report.html` | **self-contained** report: per-joint matplotlib plots (command vs. actual, with the wrench-vibration trace overlaid), a colour-coded metrics table, and a per-joint capability **verdict**. Open it in any browser; all images are embedded. |
| `run.npz` | raw arrays (`t` / `cmd` / `act` / `force` per segment) for your own re-analysis. |
| `manifest.json` | the config + per-segment metrics (drives the history list). |

In the web UI, the **Saved runs** panel lists past runs newest-first with their
overall verdict; **Open ↗** shows the report, **Data** downloads the `.npz`. When
a run finishes a banner links straight to the new report. The plots match the
style of the `exp1_ros2_control` report (commanded vs. actual per segment + error
/ vibration), so the dashboard is a drop-in, live replacement for that offline
script.

## Parameters

| param | default | meaning |
|---|---|---|
| `host` / `port` | `0.0.0.0` / `8140` | HTTP bind |
| `joint_states_topic` | `/joint_states` | actual-position feedback |
| `wrench_topic` | `""` | optional 6-axis wrench (force ground truth); empty = off |
| `controller_manager` | `/controller_manager` | where to discover FPCs |
| `controller_name` | `""` | pin one controller by name; empty = auto-discover every forward-position controller. When set, the plugin-type allowlist is bypassed for that controller (escape hatch for vendor FPC subclasses with an unrecognised type), but it must still command only `<joint>/position`. |
| `send_rate` | `200.0` | command publish + analysis rate (Hz) |
| `default_limit_deg` | `8.0` | per-joint safety envelope around the start pose |
| `report_dir` | `~/.ros/fpc_test_dashboard/runs` | where each run's report + raw data is saved |

## Safety

* Only the **one** joint under test moves; all others hold their start position.
* Every commanded joint is hard-clamped to **`baseline ± limit`** (default ±8°).
* Smoothstep ramps in/out of every segment; **Stop**, errors and disconnects ramp
  back to baseline and restore the controllers that were active before the run.
* The run refuses to start if `/joint_states` is stale or the baseline is unknown.

> Always keep the workspace clear and an e-stop within reach while testing — the
> dashboard commands real joint motion.

## How to read the results

* **Smooth** at the noise floor with low lag → the FPC tracks cleanly; the robot
  is a good candidate for host-side admittance control.
* **Stair** above the floor (elevated force HF / pos HF) → raw low-rate streaming
  vibrates; you need to smooth the command before it reaches the FPC.
* **Smoothed** back at the floor with only a small added lag → a host-side
  command low-pass (cutoff ≈ 10–15 Hz) is the fix; pick the highest cutoff whose
  force HF is back at the floor for the least latency.
