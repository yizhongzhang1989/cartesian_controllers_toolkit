# robot_feasibility_test — Improvement Plan

> **Status:** proposal for review. The ultimate goal is to turn this package from
> a *command-smoothing tester* into an **integrated robot-readiness test platform**:
> when a new arm arrives, run one guided battery and get a verdict —
> **Suitable / Marginal / Unsuitable for host-side force control** — with the
> limiting factor named and recommended starting gains.
>
> This file is structured so it can be executed autonomously in a fresh session
> (like `shaper_plan.md` was for the shaper rework): each phase has concrete
> tasks, metric definitions, pass/fail thresholds, and the safety protocol.

---

## 1. Why this matters (motivation)

This session proved empirically that **forward-position-control (FPC) quality
decides force-control robustness**, and that the *decisive* predictors were **not**
in the current dashboard:

* the HOLD **DC offset** (~0.2°, shaper-induced) → admittance reads it as a
  persistent force error;
* **friction hysteresis** → dead-zone hunting / limit cycles;
* the **~5.3 Hz structural resonance** → set the hard ceiling on force-loop
  bandwidth (the "G2 vs G4" wall).

All three were measured by **standalone scripts** in `temp/fp_control_test/`
(`hyst_probe.py`, `resonance_probe.py`, `shaper_eval.py`), not by the dashboard.
The lesson: **characterise the robot's intrinsic servo first, then decide.** This
plan folds those probes in and adds the missing characterisation so the package
answers the real question.

---

## 2. Where the package is today (baseline)

**Files:** `dashboard_node.py` (ROS node + HTTP server + runner), `test_logic.py`
(pure-Python core), `report.py` (matplotlib + self-contained HTML + per-joint
verdict), `static/` (web UI), `launch/`, `README.md`.

**Current battery** (per selected joint, others held; ±limit clamp; ramps):

| segment | command | metrics today |
|---|---|---|
| HOLD | hold pose | `pos_hf`, `force_hf` |
| SMOOTH | continuous 0.2 Hz sine | `pos_hf`, `force_hf`, `lag_ms`, `rmse`, `amp` |
| STAIR | amplitude-quantised staircase | same |
| SMOOTHED | staircase + 2-pole LP | same |

**Strengths to keep:** URDF joint discovery; FPC auto-discovery + auto-switch
(JTC↔FPC) + restore; safety clamp + smoothstep ramps + stale-`/joint_states`
refusal; live Canvas plot; saved self-contained HTML reports + `run.npz` +
`manifest.json`; per-joint verdict; waveform-agnostic metrics (xcorr lag,
mean-centered HF RMS).

**What it answers well:** *"does host command smoothing help, and how bad is
stair vibration?"* (a teleop / command-shaping question).

**What it does NOT answer:** *"what are this robot's intrinsic servo properties,
and is there enough stability margin to close an admittance loop?"* (the
force-control-suitability question).

**Structural weakness (to fix in Phase 0):** the package is a **monolith** —
`FpcTestDashboard(Node)` fuses three concerns in one process: the ROS node, the
test **engine** (`_run_battery`), and the **HTTP dashboard** (`_start_http()` is
called unconditionally in the constructor). Consequences:

* the dashboard is **not optional** — you cannot run the automatic test battery
  headless (CI, scripted multi-robot runs, autonomous sessions) without also
  standing up the web server;
* the dashboard is **not independent** — it has no life of its own; it cannot be
  started/stopped/reloaded, or run on another host, without restarting the
  engine (and thus interrupting a test);
* the engine's only API is *internal Python method calls* from the HTTP handler,
  so nothing else (CLI, another UI, a supervising script) can drive or observe a
  run.

The test **engine** should be the main task and run on its own; the **dashboard**
should be an optional, independent client for monitoring + control.

---

## 3. Gap analysis

| Property | Why it predicts force-control suitability | Today |
|---|---|---|
| Command vibration (stair vs smooth) | teleop streaming cleanliness | ✅ |
| Actuation lag (cmd→motion) | phase loss in the loop | ⚠️ one 0.2 Hz point only |
| HOLD ripple (AC `pos_hf`/`force_hf`) | resting noise the loop amplifies | ✅ (AC only) |
| **Structural resonance f_n + ζ** | **sets max stable force bandwidth** | ❌ standalone (`resonance_probe.py`) |
| **HOLD steady-state DC error** | persistent force error / drift | ❌ standalone (`hyst_probe.py`) |
| **Backlash / friction hysteresis** | dead-zone → limit-cycle hunting | ❌ standalone (`hyst_probe.py`) |
| **Wrench latency** (force→sensor) | other half of total loop delay | ❌ not measured |
| **Wrench noise floor / bias / drift** | the signal the loop closes on | ❌ not measured |
| **Frequency response (Bode: BW + phase margin)** | the real "can it track a stream" answer | ❌ single frequency |
| **Timing jitter / state freshness** | variable latency = worse margin | ❌ (have `exp9`) |
| **Cartesian / TCP + multi-joint coupling** | force control is Cartesian, not per-joint | ❌ single-joint only |
| **Predicted stability margin + recommended gains** | the actual go/no-go | ❌ (only a vibration verdict) |

---

## 4. Measuring base vibration frequency (the keystone) — feasible

Already prototyped in `temp/fp_control_test/exp11_shaper/resonance_probe.py`:
command a small (≤2°) near-step on one joint, record the **ring-down** in
position **and** wrench at 200 Hz, then **FFT the detrended ringing → f_n** and
**log-decrement of the decaying envelope → ζ**. On Realman this measured
**f_n ≈ 5.3 Hz** (≈12.7 Hz in the wrench). Promote to a first-class test, measured
three complementary ways:

1. **Active step** (per joint) — what the probe does now.
2. **Passive tap test** — operator taps the end-effector; capture the free
   ring-down from the wrench (payload-aware, *no* commanded motion → safest).
3. **Vs. pose** — resonance shifts with configuration; sample a few poses
   (stretched vs folded) and report the range.

`f_n` is the keystone metric: achievable force bandwidth ≈ a fraction of `f_n`,
and after subtracting loop latency it drives the suitability verdict.

---

## 5. Phased plan

### Phase 0 — Architecture: decouple the test engine from the dashboard (do first)

> **STATUS: ✅ DONE (validated on the real robot, right arm).** The engine
> (`engine_node.py`, node `feasibility_test_engine`) is now the headless main task
> and the dashboard (`dashboard_node.py`) is an optional pure client. Engine API:
> `run_config` string param (JSON) + `~/run` & `~/stop` (`std_srvs/Trigger`) +
> `~/status` (`std_msgs/String` JSON @ 10 Hz). A `feasibility_test` CLI drives it
> headless. Launch split: `engine.launch.py` (always), `dashboard.launch.py`
> (opt), `test.launch.py dashboard:=true|false` (default false). Validated:
> headless CLI battery + dashboard-driven run both saved "good" reports; the
> dashboard degrades to `engine_offline` when the engine is killed and
> reconnects when it returns.
>
> **Finding (fixed):** the engine set status `done` *before* `save_run`
> populated `last_report`, so clients reacting to the terminal status missed the
> report path (a race). Fixed by saving the report under a transient `saving`
> status, then flipping to `done`/`stopped` with `last_report` set in the same
> lock block. Re-validated end-to-end (CLI + dashboard both print the report id).

Make the **engine** the standalone main task and the **dashboard** an optional,
independent client. This is foundational: every later phase adds tests to the
engine and views to the dashboard, so the split must exist first.

* **0a · Split into two nodes / entry points.**
  * **`test_engine_node`** (the main task): owns URDF parse, controller
    discovery + switch + restore, the runner thread, safety, metrics
    (`test_logic`), and report saving (`report.py`). **No HTTP.** Runs headless.
  * **`dashboard_node`** (optional): the web UI only. Holds **no** test logic and
    **never** touches the hardware directly; it is a pure client of the engine's
    API. Can be started, stopped, reloaded, or run on another host independently
    of the engine and without interrupting a run.
* **0b · Give the engine a ROS API** (its public contract, used by both the
  dashboard and any CLI / supervisor):
  * services `~/run` (start a battery from a config) and `~/stop`;
  * a `~/status` topic (mode, progress, live trace tail, latest results,
    last-saved report) published continuously;
  * a `~/results` / saved-run query (or reuse the on-disk `report_dir`).
  * Start ROS-native but pragmatic: a JSON-string `std_msgs/String` status topic
    + `std_srvs`-style run/stop is acceptable for v1; a typed `.msg`/`.srv` is a
    later nicety (see open questions).
* **0c · Headless CLI front-end** (`feasibility_test` console script): calls the engine
  services to run a named battery on chosen joints and waits for completion —
  for CI, scripted multi-robot characterisation, and autonomous sessions. No web
  server involved.
* **0d · Launch files reflect the split:**
  * `engine.launch.py` — the main task (default; what you run for a real test);
  * `dashboard.launch.py` — optional UI, launched separately;
  * optional convenience `test.launch.py` with `dashboard:=true|false` (default
    **false** so the dashboard is genuinely opt-in) that brings up engine (+ UI).
* **0e · Dashboard becomes a thin client:** the web UI subscribes to `~/status`
  and calls `~/run`/`~/stop`; it must degrade gracefully when the engine is
  absent (show "engine not connected"), and reconnect when it reappears.
* **0f · Keep `test_logic.py` pure** (no ROS, unit-testable) and have **both** the
  engine and the report importer use it — the single source of truth for
  waveforms, metrics, and (later) the scorecard.

> Result: `ros2 launch robot_feasibility_test engine.launch.py` runs the full
> automatic battery and saves reports with **no** dashboard; add
> `ros2 launch robot_feasibility_test dashboard.launch.py` any time to monitor /
> drive it live; or `ros2 run robot_feasibility_test feasibility_test --joints ...`
> for a headless scripted run. (DONE: the package was renamed `robot_feasibility_test`
> — engine-first, with the dashboard as a clearly-optional sub-component.)

### Phase 1 — Fold the three proven probes into the dashboard (highest value, code exists)

> **STATUS: ✅ DONE (1a, 1c, hold-DC part of 1b; validated on the real robot).**
> Implemented as pure functions in `test_logic.py` and wired through the engine
> + report with **no** dashboard dependency:
> * **`kind="step"`** — single clean step; `step_metrics()` → overshoot %, rise
>   (10→90 %), 2 %-band settle. Measured on `right_arm_joint4`: overshoot 3.4 %,
>   rise 75 ms, settle 340 ms.
> * **`kind="resonance"`** — small near-step (default 2°, clamped to the limit);
>   `estimate_resonance()` → `f_n` (FFT peak in 4–80 Hz) + `ζ` (log-decrement on
>   the **positive** ring-down peaks, one per period). Measured on joint4:
>   **f_n ≈ 4.25 Hz**. The report verdict now prints a structural line, e.g.
>   *"f_n ≈ 4.2 Hz → keep admittance bandwidth ≲ 1.4 Hz (soft)"*.
> * **HOLD DC error** — `analyze_segment` now reports `dc_err` (mean drift from
>   baseline) for the HOLD floor. Measured −0.12° on joint4 (consistent with the
>   known ~0.2° host command-shaper offset).
> * Offline self-tests confirm the math against synthetic signals: f_n within an
>   FFT bin, ζ exact for 0.05/0.10, step overshoot 37.1 % vs 37.2 % theory.
>
> **Findings / refinements deferred to a later pass:**
> * The active small-tap `ζ` is unreliable on a well-damped servo — a gentle 2°
>   step barely rings (joint4 overshoot 0.6 %), so the envelope is tiny and ζ
>   underestimates (read 0.003). `f_n` is robust; for ζ prefer the **wrench**
>   ring-down and/or map it from the larger `step` overshoot. **TODO 1a-bis.**
> * Resonance is **pose/joint dependent** — joint4 reads ~4.25 Hz vs the ~5.3 Hz
>   measured earlier on another joint/pose. The planned **vs-pose** + **passive
>   tap** modes (1a) and the **up/down hysteresis** staircase (rest of 1b) are
>   still **TODO**.

Migrate logic from `temp/` into `test_logic.py` + new dashboard test types so
there is **one tool**, not dashboard + scattered scripts.

* **1a · Resonance test** ← `resonance_probe.py`. New `kind="resonance"`: small
  step, record ring-down (pos + wrench), `estimate_resonance()` → `f_n`, `ζ`.
  Add active + passive-tap modes. Plot the ring-down + its FFT in the report.
* **1b · Hold accuracy / hysteresis** ← `hyst_probe.py`. Extend HOLD to drive an
  up/down staircase and record **signed DC error** + **hysteresis** (approach
  from below vs above) per level. New metrics `hold_err_deg`, `hold_hyst_deg`.
* **1c · Step response** ← `shaper_eval.py`. New `kind="step"`: overshoot %,
  settle time (2% band), onset smoothness (peak Δpos/tick, no single-tick jump).
* **1d** Retire / thin the `temp/` scripts once parity is confirmed (keep as
  reference); the dashboard becomes the single entry point.

### Phase 2 — Add the missing robot-characterisation tests

> **STATUS: 🟡 2a DONE + 2b (at-rest wrench) DONE + 2d (jitter) DONE (validated
> on the real robot); 2c + 2b-contact-latency TODO.**
> **2a — frequency sweep:** `kind="sweep"` drives an exponential (constant-Q)
> log-chirp `f0→f1` (default 0.3→6 Hz, small amplitude, clamped). `estimate_bode()`
> computes the ETFE `H = FFT(act)/FFT(cmd)` over the excited band → **−3 dB
> bandwidth**, resonant-peak freq/gain (only when the gain genuinely rises above
> the DC reference; a monotonic low-pass returns `f_peak = NaN`), and the
> **phase lag at the bandwidth**. It also computes a **wrench-referenced
> resonance** via a proper per-axis wrench/command ETFE on the most-excited force
> axis (NOT a raw |F| FFT, which a first attempt showed is biased to low
> frequency by the log-chirp dwell + magnitude nonlinearity → spurious 2.6 Hz).
> Offline-validated vs synthetic 2nd-order systems (ζ=0.5 → BW 6.2 Hz; ζ=0.1 →
> BW 7.6 Hz / peak 4.94 Hz / +12 dB; ζ=0.9 → no peak). Live on `right_arm_joint4`:
> **−3 dB BW ≈ 4.8 Hz**, phase ≈ −176° at BW.
> **2b — at-rest wrench characterisation:** the HOLD segment (no motion) now also
> reports `wrench_bias_n` (mean |F|) and `wrench_noise_n` (combined per-axis std)
> — the signal a host force loop closes on. Graded in the scorecard (< 0.1 N
> excellent / < 0.5 N usable) and a reason is emitted when the floor is high.
> Live: bias ≈ 0.47 N, noise ≈ 0.12 N (marginal). *(2b contact-step **wrench
> latency** is still TODO — needs a stiff surface + force-trip.)*
> **2d — command-stream timing jitter:** every segment now reports `dt_jitter_ms`
> (std of the actual send-loop tick interval) + `dt_max_ms` (worst deviation),
> computed for free from the recorded monotonic tick times. Graded (< 1 ms good
> / < 3 ms usable vs the 5 ms / 200 Hz tick); variable command latency erodes the
> host force loop's phase margin. *(Full `/joint_states` age distribution from
> `exp9` is a later nicety; the send-loop jitter is the host-controllable part.)*
>
> **Key finding — how to estimate the keystone f_n reliably (validated over 7+
> live runs):** three resonance estimators were compared run-to-run on joint4:
> * **position ring-down** (`estimate_resonance` FFT of the post-step encoder
>   ringing): ~4.1–6.3 Hz — usable but noisy (a small 2° tap barely rings on a
>   well-damped servo, so ζ is unreliable and f_n scatters).
> * **wrench chirp ETFE** (the sweep): ~4.8–5.6 Hz — **the tightest, most
>   repeatable** estimator, and the most physical (a force loop closes on the
>   wrench, and the chirp properly excites the band).
> * **wrench ring-down FFT** (post-step free decay of |F|): **unreliable** —
>   read 11, 11, then 2.1 Hz across runs and matched neither other estimator;
>   FFT-ing a short noisy free-decay is ill-conditioned. **Removed** from the
>   keystone (it was dragging the verdict via a spurious low value).
> The scorecard now takes **`f_n = min(position ring-down, wrench chirp ETFE)`**
> — the **lowest corroborated** lightly-damped mode is the binding constraint on
> force bandwidth. This lands repeatably at **~4.7–4.8 Hz** (consistent with the
> session's 5.3 Hz), verdict **Unsuitable**, predicted `f_bw ≈ 1.1–1.6 Hz`. NOTE:
> the wrench ring-down's ~11 Hz is a real *higher* second mode — averaging it in
> would blend distinct modes, which is why `min()` over corroborated estimators
> (not a mean/median over all) is the right rule.

* **2a · Frequency sweep (chirp / stepped-sine)** per joint → **Bode magnitude
  + phase** → **−3 dB bandwidth** and **phase at gain-crossover** (replaces the
  single 0.2 Hz lag point; this is the real tracking-capability measurement).
* **2b · Wrench characterisation** (no motion): noise-floor RMS at rest, bias,
  gravity-comp residual vs pose; and **wrench latency** via a **contact step**
  (command gently into a stiff surface; delay between motion onset in
  `/joint_states` and force onset in the wrench).
* **2c · Backlash / min-commandable motion**: tiny reversal staircase →
  dead-zone width / smallest motion that produces real movement.
* **2d · Timing & freshness** ← `exp9`: control-loop period stability (jitter)
  and `/joint_states` age distribution.

### Phase 3 — Cartesian / TCP space (force control is Cartesian)
* **3a** Drive the **end-effector along a Cartesian axis** (via the motion
  controller, or IK in the node) and measure tracking/lag/vibration in task space.
* **3b** **Multi-joint coordinated** move to expose coupling / Jacobian
  conditioning near the pose. Report worst-axis behaviour.

### Phase 4 — Suitability model + scorecard (the headline deliverable)

> **STATUS: \u2705 DONE (first version, validated live on the real robot).**
> `test_logic.force_control_scorecard()` grades a joint's measured metrics
> (`f_n`, `\u03b6`, actuation lag, step overshoot, hold DC error) against the \u00a76
> table, predicts a **max stable admittance bandwidth**
> `f_bw = min(f_n/3, 1/(2\u03c0\u00b7\u03c4_loop))` (\u03c4_loop = actuation lag + optional wrench
> latency), recommends a safe starting bandwidth (~70 % of the ceiling), and
> emits a **Suitable / Marginal / Unsuitable** verdict naming the limiting
> factor. `rollup_scorecards()` gives the robot-level call (worst joint).
> `report.py` renders a headline "Force-control readiness" panel + per-joint
> table; the verdict is also in `manifest.json` and `list_runs`. Live result on
> `right_arm_joint4`: **Unsuitable, f_bw \u2248 1.5 Hz**, *"soft structure (f_n \u2248 4.5
> Hz) + hold DC 0.12\u00b0 + lag 70 ms \u2192 prefer the arm's native in-arm force mode"*
> \u2014 matching every standalone finding this session. Offline-validated against
> synthetic good (Suitable, 8 Hz) / marginal (Marginal, 3.5 Hz) robots.
>
> **Still TODO (later passes):** 4a uses the **estimated** loop latency (no
> wrench latency yet \u2014 needs Phase 2b contact step) and an estimated bandwidth
> (Phase 2a freq sweep will give a **measured** \u22123 dB BW + phase margin to
> replace/augment the `f_n/3` rule). 4c (guarded live admittance sweep) not done.

* **4a \u00b7 Closed-loop estimate**: from measured `f_n`, `\u03b6`, total loop latency
  (actuation lag + wrench latency), and noise floor, compute a **predicted max
  stable admittance bandwidth** and **phase margin**, and **recommend M/D/K
  gains** + a safe starting bandwidth.
* **4b · Readiness scorecard**: weighted criteria → **Suitable / Marginal /
  Unsuitable**, naming the limiting factor (e.g. *"limited by 5.3 Hz resonance →
  cap force BW ≈ 3 Hz"* or *"high hold offset → fix shaper first"* or *"use the
  arm's native in-arm force mode"*). Robot-level, not just per-joint.
* **4c · (Optional, guarded) live admittance bandwidth sweep**: raise admittance
  gain until onset of oscillation (auto-tripped on `force_hf`), to find the
  *real* stability limit and **validate the Phase-4a prediction**. Heavily
  safety-gated (see §6).

### Phase 5 — Platform features ("a new robot arrives")
* **5a · Robot profile**: saved descriptor (name, DOF, command-interface type +
  units, limits, joints-to-test, wrench topic, base/tool frames) so the tool
  adapts to *any* arm, not just the FPC plugin types it recognises today.
* **5b · Guided wizard**: connect → discover → run full battery on representative
  joints + TCP → scorecard, in one flow with clear safety prompts.
* **5c · Cross-robot comparison view**: turn saved-runs history into a benchmark
  (Duco vs UR vs Realman side-by-side on every metric + verdict).

---

## 6. Metric definitions & suggested pass/fail (force-control readiness)

Per joint unless noted. Thresholds are **starting points for review**, calibrated
against this session's data (Realman ~5.3 Hz / ~0.2° legacy offset; UR/Duco good).

| metric | definition | good | marginal | unsuitable |
|---|---|---|---|---|
| `f_n` (resonance) | FFT peak of post-step ring-down | ≥ 20 Hz | 8–20 Hz | < 8 Hz |
| `zeta` | log-decrement damping | ≥ 0.1 | 0.03–0.1 | < 0.03 |
| hold DC error | settled \|act−cmd\| | ≤ 0.02° | ≤ 0.1° | > 0.1° |
| hold hysteresis | up- vs down-approach gap | ≤ 0.02° | ≤ 0.1° | > 0.1° |
| actuation lag | xcorr cmd→motion | ≤ 30 ms | ≤ 60 ms | > 60 ms |
| wrench latency | force-onset − motion-onset | ≤ 15 ms | ≤ 40 ms | > 40 ms |
| −3 dB bandwidth | from chirp Bode | ≥ 15 Hz | 5–15 Hz | < 5 Hz |
| HOLD `force_hf` | resting wrench noise (>2 Hz RMS) | low floor | — | high |
| backlash | reversal dead-zone | ≤ 0.02° | ≤ 0.1° | > 0.1° |

**Headline rule of thumb:** achievable stable force bandwidth ≈ `min(f_n/3,
1/(2π·τ_loop))`. If that is < ~2 Hz, host-side admittance is not worth it on this
arm → recommend the arm's native in-arm force mode.

---

## 7. SAFETY PROTOCOL (mandatory — all driven tests)

* Every commanded joint hard-clamped to **baseline ± limit**; only the tested
  joint(s) move; all others hold. Smoothstep ramps in/out. Refuse on stale
  `/joint_states`.
* Resonance/step tests use **small** steps (≤ 2°); contact / wrench-latency tests
  approach a surface **gently** and are auto-tripped on a force limit.
* On stop / error / disconnect: ramp to baseline, restore the controllers that
  were active before the run.
* Auto-discover then **restore** controller state; never leave an arm on FPC/force
  after a test.
* The optional live admittance sweep (4c) must: start at low gain, raise slowly,
  auto-disengage on `force_hf` threshold or motion limit, and require explicit
  operator opt-in. Keep an e-stop in reach.
* Health-check the arm before/after driven tests (enable flags, error flags).

---

## 8. Sequencing & deliverable

1. **Phase 0 first** — decouple engine ↔ dashboard. Foundational: it changes the
   process/launch model every later phase builds on, and it is cheap to do now
   (mostly moving existing code, not new behaviour). After it, the automatic test
   runs headless and the dashboard is optional.
2. **Phase 1** — fold in the three validated probes (resonance + DC hold error
   were the decisive metrics this session) as engine test-kinds.
3. **Phase 2** — bandwidth + wrench characterisation feed the model.
4. **Phase 4** scorecard is the headline, but depends on 1–2 producing inputs.
5. **Phases 3 & 5** make it a true multi-robot platform.

Each test added must: extend `test_logic.py` (pure-Python, unit-testable), run in
the **engine** (headless-capable), publish to the engine `~/status`/results API,
surface in the **optional** web UI, render in the saved HTML report, and
contribute to the scorecard. Keep the "one engine, optional dashboard, saved
reports, safe by default" properties.

---

## 9. Open questions for the reviewer

* **Engine ↔ dashboard API (Phase 0):** JSON-string `std_msgs/String` status +
  generic run/stop services for v1 (fast, no new interfaces pkg), or a typed
  `.msg`/`.srv` interface from the start (cleaner, but adds a `*_interfaces`
  package)? Should the dashboard talk to the engine **purely over ROS**, or keep
  the HTTP server and have it proxy the engine's ROS API to the browser?
* **Package naming (Phase 0):** ✅ RESOLVED — the package was renamed to
  `robot_feasibility_test` (engine-first), with the dashboard as a
  clearly-optional sub-component; nodes are `feasibility_test_engine` /
  `feasibility_test_dashboard` and the CLI command is `feasibility_test`.
* Is the optional **live admittance sweep (4c)** wanted, or is the *predicted*
  margin (4a) sufficient and safer?
* Cartesian tests (Phase 3) — drive via the FZI motion controller, or do IK
  inside the node to stay self-contained?
* Should the scorecard live in this package, or be a shared report the
  `cartesian_control_manager` also consumes for auto-gain-seeding?
