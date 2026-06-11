# robot_feasibility_test

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
   | **Step** | one clean step 0 → amp | closed-loop **step response**: overshoot %, rise (10→90 %), 2 %-band settling |
   | **Resonance** | a small near-step (≤ 2°), then watch the ring-down | structural **natural frequency `f_n`** (FFT of the ring-down) + damping `ζ` (log-decrement) — the hard ceiling on a stable force bandwidth |
   | **Sweep** | a slow exponential chirp `f0 → f1` | the empirical **Bode response**: −3 dB **bandwidth**, phase lag, and (from the wrench) the true mechanical resonance |

5. Records the **actual joint motion** from `/joint_states` and, if you provide a
   `wrench_topic`, the **wrist wrench** — the *independent ground truth* for real
   end-effector vibration (admittance control reads this same signal, so its
   cleanliness is what ultimately matters).
6. Reports per-segment metrics: **pos HF** (joint ripple), **force HF** (real
   vibration), **lag** (phase delay), **RMSE** (tracking error), plus the
   dynamics metrics above (`f_n`, `ζ`, overshoot, settle, −3 dB BW, phase),
   the **HOLD DC error** + at-rest **wrench bias / noise floor**, and the
   command-stream **timing jitter**.
7. Synthesises a **force-control readiness scorecard** (see
   [Readiness scorecard](#readiness-scorecard)) — the headline
   *Suitable / Marginal / Unsuitable* verdict.
8. **Saves every run** as a self-contained report you can review later (see
   [Saved runs](#saved-runs)).

## Architecture: headless engine + optional dashboard

The test **engine** is the main task and runs **headless**; the web **dashboard**
is an **optional** client. Three ways to drive it:

```bash
# 1) Headless engine (the main task) — runs the battery, saves reports, no UI.
ros2 launch robot_feasibility_test engine.launch.py \
    wrench_topic:=/right_arm_force_torque_sensor_broadcaster/wrench

# 2) Optional web dashboard (a thin client of the engine) — monitor / drive live.
ros2 launch robot_feasibility_test dashboard.launch.py    # then open http://localhost:8140

# 3) Headless CLI — scripted / CI runs against a running engine.
ros2 run robot_feasibility_test feasibility_test \
    --controller right_arm_forward_position_controller --joints right_arm_joint4 \
    --tests smooth,step,resonance,sweep --amp 4 --resonance-deg 2 \
    --sweep-deg 1.0 --sweep-band 0.3,8.0 --seg 12 --limit 8
```

The engine exposes a small ROS API: a `run_config` string parameter (JSON), the
`~/run` and `~/stop` services (`std_srvs/Trigger`), and a `~/status` topic
(`std_msgs/String` JSON @ 10 Hz). The dashboard sets `run_config`, calls `~/run`,
subscribes to `~/status`, and serves saved reports from `report_dir`; it degrades
gracefully to *"engine not connected"* when the engine is absent and reconnects
when it returns.

## Readiness scorecard

From the measured metrics the tool predicts a **maximum stable admittance
bandwidth** `f_bw ≈ min(f_n / 3, 1/(2π·τ_loop))` (τ_loop = actuation lag +
wrench latency), grades each criterion against the thresholds in
[`improvement_plan.md`](improvement_plan.md) §6, recommends a safe starting
bandwidth (~70 % of the ceiling), and emits a robot-level
**Suitable / Marginal / Unsuitable** verdict that names the limiting factor
(e.g. *"soft structure (f_n ≈ 4.7 Hz) → cap force BW ≈ 1.6 Hz → prefer the arm's
native in-arm force mode"*). The keystone `f_n` is taken as the **minimum of the
two corroborating estimators** — the position ring-down and the wrench-chirp
ETFE — i.e. the lowest lightly-damped structural mode, which is the binding
constraint on force bandwidth.

## Using the web dashboard

Launch the engine and the dashboard (see
[Architecture](#architecture-headless-engine--optional-dashboard) above) **after**
the robot is up (so `/robot_description` and `/controller_manager` exist), then
open `http://localhost:8140`.

In the page: pick the controller → tick the joints → (optionally tweak
parameters) → **Run test**. The trace plots command vs. actual live; the results
table fills in as each segment finishes. **Stop** aborts and ramps back.

## Saved runs

Every run is persisted (like `temp/fp_control_test/exp1_ros2_control`) so you can
check it later — no need to keep the page open. For each run the dashboard writes
a timestamped folder under `report_dir`
(default `~/.ros/robot_feasibility_test/runs/<timestamp>_<controller>/`):

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
| `report_dir` | `~/.ros/robot_feasibility_test/runs` | where each run's report + raw data is saved |

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
* **Step**: low overshoot (≤ 3 %) and short settling → a well-damped servo;
  large overshoot / long settle → expect limit-cycling under a force loop.
* **Resonance** / **Sweep**: a **high `f_n` / wide −3 dB bandwidth** is what
  permits a high force-control bandwidth. A low `f_n` (soft structure) is the
  single biggest limiter — the scorecard caps the recommended bandwidth at
  ≈ `f_n / 3`. If the predicted `f_bw` is below ~2 Hz, host-side admittance is
  not worth it on that arm and you should use its native in-arm force mode.
* The **scorecard** rolls all of this into one verdict and names the limiting
  factor, so it is the first thing to read.

