"""Pure-Python core for the robot feasibility test platform (no ROS dependency).

Holds everything that can be unit-tested without a robot:

* :func:`parse_urdf_joints`  -- extract movable joints (+ limits) from a URDF.
* :class:`TwoPoleLowPass`    -- the command smoother under test (matches the one
                                in rm_control's write()).
* :func:`build_plan`         -- the per-joint test battery (smooth / stair / smoothed).
* :func:`waveform_offset`    -- the commanded offset for a segment at tick ``k``.
* :func:`analyze_segment`    -- tracking metrics (RMSE, lag, vibration) from the
                                recorded command vs. actual (and optional wrench).

The "tests" mirror the conditions a host-side admittance / Cartesian force
controller will put the forward-position interface through:

* SMOOTH    -- a continuous reference refreshed every control tick. This is the
               best case and shows the arm's intrinsic tracking lag + ripple.
* STAIR     -- the same sine discretised into uniform steps (a visible staircase
               of ``stair_step`` risers). Each riser is a one-tick jump whose
               broadband content excites the arm's resonance -- exactly what a
               stepped / low-rate (teleop or admittance) command stream does.
* SMOOTHED  -- the STAIR command run through a 2-pole low-pass first (the fix):
               shows how much a host-side command smoother rounds the risers and
               recovers clean tracking.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

# Joint types that actually move a single DOF and can be tracked by an FPC.
_MOVABLE = ("revolute", "continuous", "prismatic")


# ---------------------------------------------------------------------------
# URDF parsing
# ---------------------------------------------------------------------------
@dataclass
class JointInfo:
    name: str
    type: str
    lower: Optional[float] = None     # rad (revolute/continuous) or m (prismatic)
    upper: Optional[float] = None
    velocity: Optional[float] = None  # limit, rad/s or m/s
    is_prismatic: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name, "type": self.type,
            "lower": self.lower, "upper": self.upper,
            "velocity": self.velocity, "is_prismatic": self.is_prismatic,
        }


def parse_urdf_joints(urdf_xml: str) -> List[JointInfo]:
    """Return the movable joints of a URDF, in document order.

    Fixed/mimic joints are skipped (an FPC can't drive them). ``continuous``
    joints report no position limit; ``revolute``/``prismatic`` report theirs
    when present. Robust to a missing ``<limit>`` element.
    """
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"expected <robot> root, got <{root.tag}>")
    out: List[JointInfo] = []
    for j in root.findall("joint"):
        jtype = (j.get("type") or "").lower()
        if jtype not in _MOVABLE:
            continue
        if j.find("mimic") is not None:
            continue  # driven by another joint; not independently commandable
        name = j.get("name")
        if not name:
            continue
        lim = j.find("limit")
        lower = upper = vel = None
        if lim is not None:
            def _f(key):
                v = lim.get(key)
                return float(v) if v is not None else None
            lower, upper, vel = _f("lower"), _f("upper"), _f("velocity")
        out.append(JointInfo(
            name=name, type=jtype, lower=lower, upper=upper, velocity=vel,
            is_prismatic=(jtype == "prismatic")))
    return out


# ---------------------------------------------------------------------------
# command smoother under test (mirrors rm_control's two cascaded one-pole LP)
# ---------------------------------------------------------------------------
class TwoPoleLowPass:
    """Two cascaded one-pole low-pass filters. ``cutoff_hz <= 0`` is a pass-through.

    ``beta = 1 - exp(-2*pi*fc*dt)`` per step; identical to the smoother applied
    in the rm_control hardware ``write()`` loop, so dashboard results predict the
    live behaviour.
    """

    def __init__(self, cutoff_hz: float, dt: float, x0: float = 0.0):
        self.enabled = cutoff_hz > 0.0
        self.beta = (1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)) if self.enabled else 1.0
        self.s1 = x0
        self.s2 = x0

    def reset(self, x0: float) -> None:
        self.s1 = x0
        self.s2 = x0

    def step(self, x: float) -> float:
        if not self.enabled:
            return x
        self.s1 += self.beta * (x - self.s1)
        self.s2 += self.beta * (self.s1 - self.s2)
        return self.s2


# ---------------------------------------------------------------------------
# test plan
# ---------------------------------------------------------------------------
@dataclass
class Segment:
    name: str
    kind: str            # "hold"|"smooth"|"stair"|"smoothed"|"step"|"resonance"|"sweep"
    duration: float      # seconds
    refresh_hz: float    # command-value refresh rate (>= send rate => every tick)
    lp_hz: float = 0.0   # host low-pass cutoff applied to the command (0 = off)
    amp_override: Optional[float] = None  # per-segment amplitude (else params.amp)
    step_delay: float = 0.7  # for step/resonance: seconds of HOLD before the step
    f0: float = 0.3      # sweep start frequency (Hz)
    f1: float = 6.0      # sweep end frequency (Hz)


@dataclass
class TestParams:
    amp: float = 0.07           # rad (~4 deg) or m for prismatic
    freq: float = 0.2           # Hz
    seg_seconds: float = 9.0
    send_rate: float = 200.0    # Hz (publish + analysis grid)
    stair_step: float = 0.0262  # rad (~1.5 deg) -- staircase riser height
    lp_hz: float = 10.0         # smoother cutoff for the SMOOTHED segment
    resonance_amp: Optional[float] = None  # native units; small tap (else amp)
    sweep_amp: Optional[float] = None      # native units; small sweep (else amp/2)
    sweep_f0: float = 0.3       # frequency-sweep start (Hz)
    sweep_f1: float = 6.0       # frequency-sweep end (Hz)
    include: Sequence[str] = field(
        default_factory=lambda: ("smooth", "stair", "smoothed"))


def build_plan(p: TestParams) -> List[Segment]:
    """The per-joint segment battery (a short HOLD floor first, then the tests).

    Beyond the tracking tests (smooth/stair/smoothed), two single-step dynamics
    tests probe the open-loop *structure* of the joint -- the properties a host
    force/admittance controller has to live inside:

    * ``step``      -- one clean step 0->amp; measures rise time, overshoot and
                       settling (the closed-loop step response of the servo).
    * ``resonance`` -- a small near-step whose **ring-down** reveals the
                       structural natural frequency ``f_n`` (FFT peak) and
                       damping ``zeta`` (log-decrement). ``f_n`` is the hard
                       ceiling on a stable admittance bandwidth.
    """
    segs: List[Segment] = [Segment("HOLD", "hold", min(5.0, p.seg_seconds), p.send_rate)]
    if "smooth" in p.include:
        segs.append(Segment("SMOOTH", "smooth", p.seg_seconds, p.send_rate))
    if "stair" in p.include:
        segs.append(Segment("STAIR", "stair", p.seg_seconds, p.send_rate))
    if "smoothed" in p.include:
        segs.append(Segment("SMOOTHED", "smoothed", p.seg_seconds, p.send_rate, p.lp_hz))
    if "step" in p.include:
        # enough post-step time to see overshoot + settle (>= 3 s tail)
        dur = max(4.0, min(8.0, p.seg_seconds))
        segs.append(Segment("STEP", "step", dur, p.send_rate, step_delay=0.7))
    if "resonance" in p.include:
        dur = max(5.0, min(8.0, p.seg_seconds))
        ramp = p.resonance_amp if (p.resonance_amp and p.resonance_amp > 0) else p.amp
        segs.append(Segment("RESONANCE", "resonance", dur, p.send_rate,
                            amp_override=ramp, step_delay=0.7))
    if "sweep" in p.include:
        # a slow log-chirp through the structural band; needs several seconds at
        # the low end to resolve f0. Small amplitude (sweeps through resonance).
        dur = max(10.0, p.seg_seconds)
        samp = p.sweep_amp if (p.sweep_amp and p.sweep_amp > 0) else max(p.amp / 2.0, 0.0)
        segs.append(Segment("SWEEP", "sweep", dur, p.send_rate,
                            amp_override=samp, f0=p.sweep_f0, f1=p.sweep_f1))
    return segs


def waveform_offset(seg: Segment, k: int, dt: float, amp: float,
                    freq: float, stair_step: float = 0.0) -> float:
    """Commanded offset (from baseline) for segment ``seg`` at tick ``k``.

    * ``smooth``           -- a continuous sine (best-case reference).
    * ``stair``/``smoothed`` -- the SAME sine discretised into uniform steps of
      height ``stair_step`` (a visible staircase). Each level crossing is a
      one-tick jump -- a sharp riser whose broadband content excites the arm's
      resonance, which is exactly what stepped / low-rate commands do. The
      caller runs ``smoothed`` through a :class:`TwoPoleLowPass` to round the
      risers (the fix), so it is *not* low-passed here.
    """
    if seg.kind == "hold":
        return 0.0
    eff_amp = seg.amp_override if (seg.amp_override is not None) else amp
    if seg.kind in ("step", "resonance"):
        # a single clean rising step from baseline at ``step_delay`` seconds.
        return eff_amp if (k * dt) >= seg.step_delay else 0.0
    if seg.kind == "sweep":
        # exponential (constant-Q) chirp f0 -> f1 over the segment; the phase is
        # the analytic integral of an exponentially-rising instantaneous freq.
        t = k * dt
        T = max(seg.duration, 1e-6)
        f0 = max(seg.f0, 1e-3)
        f1 = max(seg.f1, f0 + 1e-3)
        ratio = f1 / f0
        phase = 2.0 * math.pi * f0 * T * (ratio ** (t / T) - 1.0) / math.log(ratio)
        return eff_amp * math.sin(phase)
    base = eff_amp * math.sin(2.0 * math.pi * freq * (k * dt))
    if seg.kind == "smooth":
        return base
    # stair / smoothed: quantise the sine into discrete levels.
    step = stair_step if (stair_step and stair_step > 0.0) else (eff_amp / 3.0)
    q = round(base / step) * step
    if q > eff_amp:
        q = eff_amp
    elif q < -eff_amp:
        q = -eff_amp
    return q


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------
def _np():
    import numpy as np  # local import so the module imports without numpy present
    return np


def _highpass_rms(y, fs: float, fcut: float) -> float:
    np = _np()
    y = np.asarray(y, dtype=float)
    if y.size < 8:
        return 0.0
    win = max(1, int(round(fs / fcut)))
    if win % 2 == 0:
        win += 1
    lp = np.convolve(y, np.ones(win) / win, mode="same")
    return float(np.sqrt(np.mean((y - lp) ** 2)))


def _xcorr_lag_ms(cmd, act, dt: float, max_ms: float = 400.0) -> float:
    """Lag (ms) of ``act`` behind ``cmd`` via normalised cross-correlation.

    Waveform-agnostic (unlike a single-frequency phase fit), so it works for the
    smooth sine, the staircase and the smoothed staircase alike.
    """
    np = _np()
    c = np.asarray(cmd, float)
    a = np.asarray(act, float)
    n = int(min(c.size, a.size))
    if n < 8:
        return float("nan")
    c = c[:n] - c[:n].mean()
    a = a[:n] - a[:n].mean()
    if np.std(c) < 1e-9 or np.std(a) < 1e-9:
        return float("nan")
    max_lag = max(1, min(int(round((max_ms / 1000.0) / dt)), n - 4))
    best_k, best = 0, -1e18
    for k in range(0, max_lag):
        cc = c[:n - k]
        aa = a[k:n]
        denom = float(np.linalg.norm(cc) * np.linalg.norm(aa))
        if denom < 1e-12:
            continue
        v = float(np.dot(cc, aa) / denom)
        if v > best:
            best, best_k = v, k
    return best_k * dt * 1000.0


def _step_index(cmd) -> int:
    """Index of the rising step in a step/resonance command (0 -> amp)."""
    np = _np()
    c = np.asarray(cmd, float)
    cmax = float(np.nanmax(np.abs(c))) if c.size else 0.0
    if cmax < 1e-9:
        return 0
    above = np.where(np.abs(c) > 0.5 * cmax)[0]
    return int(above[0]) if above.size else 0


def estimate_resonance(y, fs: float, fmin: float = 4.0,
                       fmax: float = 80.0) -> Dict[str, float]:
    """Natural frequency + damping of a decaying ring-down ``y`` (post-step).

    * ``f_n`` -- the dominant FFT peak of the (detrended, windowed) ring-down,
      restricted to ``[fmin, fmax]`` Hz so the slow command move is ignored.
    * ``zeta`` -- viscous damping from the **log-decrement** of the successive
      envelope peaks: ``delta = mean(ln(A_i / A_{i+1}))`` per cycle and
      ``zeta = delta / sqrt(4*pi^2 + delta^2)``.

    Returns ``{f_n, zeta}`` in Hz / dimensionless (NaN if not resolvable).
    """
    np = _np()
    y = np.asarray(y, float)
    n = y.size
    out = {"f_n": float("nan"), "zeta": float("nan")}
    if n < 16 or fs <= 0:
        return out
    # detrend: subtract the settled value (mean of the last quarter)
    tail = y[int(0.75 * n):]
    y = y - (float(tail.mean()) if tail.size else float(y.mean()))
    # ---- f_n: FFT peak inside the band ----
    win = np.hanning(n)
    spec = np.abs(np.fft.rfft(y * win))
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    band = (freqs >= fmin) & (freqs <= fmax)
    if band.any() and spec[band].size:
        out["f_n"] = float(freqs[band][int(np.argmax(spec[band]))])
    # ---- zeta: log-decrement over decaying |peaks| ----
    # Use the POSITIVE peaks of the (zero-mean) ring-down: one maximum per
    # period, so successive ratios give the per-cycle decrement directly.
    # (|y| would have two humps per cycle and halve the estimate.)
    pk = [i for i in range(1, n - 1)
          if y[i] > y[i - 1] and y[i] >= y[i + 1] and y[i] > 0.0]
    if len(pk) >= 3:
        amps = y[pk]
        p0 = float(amps.max())
        # take the run of peaks above an 8% noise floor (the genuine ring-down)
        keep = amps[amps > 0.08 * p0]
        keep = keep[:min(12, keep.size)]
        if keep.size >= 3 and np.all(keep > 0):
            ln = np.log(keep)
            xs = np.arange(keep.size, dtype=float)
            slope = float(np.polyfit(xs, ln, 1)[0])
            delta = -slope                       # decrement per cycle
            if delta > 1e-6:
                out["zeta"] = float(delta / math.sqrt(4.0 * math.pi ** 2 + delta ** 2))
    return out


def estimate_bode(cmd, act, fs: float, f0: float, f1: float) -> Dict[str, Any]:
    """Empirical command->motion frequency response from a chirp segment.

    Computes the ETFE ``H(f) = FFT(act)/FFT(cmd)`` over the swept band, then:

    * ``bw_hz``        -- the -3 dB bandwidth: the highest frequency (scanning up
      from ``f0``) at which the smoothed gain is still >= 0.707 of the
      low-frequency (DC) gain, i.e. the first roll-off crossing past any
      resonant peak.
    * ``f_peak_hz`` / ``peak_gain_db`` -- the resonance peak in the response.
    * ``phase_at_bw_deg`` -- the command->motion phase lag at ``bw_hz`` (a phase
      margin proxy for the host force loop).

    Returns NaNs where the band has too little excitation to be trustworthy.
    """
    np = _np()
    c = np.asarray(cmd, float)
    a = np.asarray(act, float)
    n = int(min(c.size, a.size))
    out = {"bw_hz": float("nan"), "f_peak_hz": float("nan"),
           "peak_gain_db": float("nan"), "phase_at_bw_deg": float("nan")}
    if n < 64 or fs <= 0:
        return out
    c = c[:n] - c[:n].mean()
    a = a[:n] - a[:n].mean()
    win = np.hanning(n)
    C = np.fft.rfft(c * win)
    A = np.fft.rfft(a * win)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    band = (freqs >= f0) & (freqs <= f1)
    if band.sum() < 8:
        return out
    magC = np.abs(C)
    # only trust bins the chirp actually excited
    thr = 0.05 * float(magC[band].max()) if magC[band].size else 0.0
    good = band & (magC > thr)
    if good.sum() < 8:
        return out
    fb = freqs[good]
    H = A[good] / C[good]
    mag = np.abs(H)
    phase = np.unwrap(np.angle(H))
    # smooth the magnitude over frequency (moving average, ~7 bins)
    w = min(7, (mag.size // 2) * 2 + 1)
    if w >= 3:
        kk = np.ones(w) / w
        mag = np.convolve(mag, kk, mode="same")
    # DC/low-freq reference gain (median of the lowest 20 % of the band)
    nlow = max(2, int(0.2 * mag.size))
    ref = float(np.median(mag[:nlow]))
    if ref < 1e-9:
        return out
    rel = mag / ref
    # resonance peak -- only meaningful if the gain actually rises ABOVE the DC
    # reference (a genuine resonant bump). A monotonic low-pass has its max at
    # the low-frequency end, which is NOT a resonance -> report NaN for f_peak.
    ip = int(np.argmax(mag))
    peak_db = float(20.0 * math.log10(max(mag[ip] / ref, 1e-9)))
    out["peak_gain_db"] = peak_db
    if peak_db > 1.0 and fb[ip] > 1.2 * f0:
        out["f_peak_hz"] = float(fb[ip])
    # -3 dB bandwidth: first upward crossing below 0.707 after being above it
    thr707 = 1.0 / math.sqrt(2.0)
    bw = float(fb[-1])  # if it never drops, report the top of the band
    for i in range(1, rel.size):
        if rel[i] < thr707 and rel[i - 1] >= thr707:
            bw = float(fb[i])
            break
    out["bw_hz"] = bw
    # phase lag at the bandwidth frequency
    jb = int(np.argmin(np.abs(fb - bw)))
    out["phase_at_bw_deg"] = float(np.degrees(phase[jb]))
    return out


def step_metrics(t, act, fs: float, i0: int) -> Dict[str, Any]:
    """Closed-loop step-response metrics from actual ``act`` starting at ``i0``.

    rise (10->90 %), overshoot (% of step), and 2 %-band settling time (ms).
    """
    np = _np()
    t = np.asarray(t, float)
    a = np.asarray(act, float)
    n = a.size
    out: Dict[str, Any] = {}
    if n < 16 or i0 >= n - 4:
        return out
    pre = a[max(0, i0 - int(0.2 * fs)):i0]
    base = float(pre.mean()) if pre.size else float(a[0])
    final = float(a[int(0.75 * n):].mean())
    step = final - base
    if abs(step) < 1e-6:
        return out
    rising = step > 0
    post = a[i0:]
    peak = float(np.nanmax(post)) if rising else float(np.nanmin(post))
    out["overshoot_pct"] = max(0.0, (peak - final) / abs(step) * 100.0 if rising
                               else (final - peak) / abs(step) * 100.0)
    lo, hi = base + 0.1 * step, base + 0.9 * step

    def _cross(level: float) -> float:
        for i in range(i0, n):
            if (rising and a[i] >= level) or ((not rising) and a[i] <= level):
                return float(t[i])
        return float("nan")

    t10, t90 = _cross(lo), _cross(hi)
    out["rise_ms"] = ((t90 - t10) * 1000.0) if (t10 == t10 and t90 == t90) else float("nan")
    band = 0.02 * abs(step)
    settle = 0.0
    for i in range(n - 1, i0 - 1, -1):
        if abs(a[i] - final) > band:
            settle = float(t[i] - t[i0]) * 1000.0
            break
    out["settle_ms"] = settle
    out["step_size"] = abs(step)
    out["final"] = final
    return out


def analyze_segment(t, cmd, act, freq: float, fs: float,
                    force=None, kind: str = "",
                    meta_f0: Optional[float] = None,
                    meta_f1: Optional[float] = None) -> Dict[str, Any]:
    """Compute tracking metrics for one segment.

    Parameters
    ----------
    t, cmd, act : arrays of the segment's time, commanded offset, actual offset
                  (offsets from baseline; same units, rad or m).
    force : optional (N,3) array of the wrist force [Fx,Fy,Fz] over ``t`` -- the
            INDEPENDENT ground truth for real end-effector vibration.

    Returns a dict of floats (deg-based for readability if the joint is angular;
    the caller passes already-converted values, so units are whatever ``cmd``/
    ``act`` are in -- the dashboard converts rad->deg before calling for revolute
    joints).
    """
    np = _np()
    t = np.asarray(t, float)
    cmd = np.asarray(cmd, float)
    act = np.asarray(act, float)
    out: Dict[str, Any] = {"kind": kind, "n": int(t.size)}
    if t.size < 30:
        return {**out, "ok": False}
    t0 = t[0]
    fs = float(fs)

    # ---- command-stream timing jitter (loop-rate stability) --------------
    # ``t`` holds the ACTUAL monotonic tick times of the send loop, so the
    # spread of the per-tick interval around the nominal 1/fs is the host's
    # streaming jitter. Variable command latency erodes the phase margin of any
    # host force loop, so this is reported for every segment (computed for free).
    dts = np.diff(t)
    if dts.size:
        nominal = 1.0 / fs
        out["dt_jitter_ms"] = float(np.std(dts) * 1000.0)
        out["dt_max_ms"] = float(np.max(np.abs(dts - nominal)) * 1000.0)

    # ---- single-step dynamics tests (step / resonance) -------------------
    # These need the full transient, so they analyse the WHOLE segment on a
    # uniform grid (not the steady window that the tracking tests use).
    if kind in ("step", "resonance"):
        tg = np.arange(t0, t[-1], 1.0 / fs)
        if tg.size < 16:
            tg, ag, cg = t, act, cmd
        else:
            ag = np.interp(tg, t, act)
            cg = np.interp(tg, t, cmd)
        i0 = _step_index(cg)
        out.update(step_metrics(tg, ag, fs, i0))
        out["pos_hf"] = _highpass_rms(ag[i0:], fs, 2.0)
        if kind == "resonance":
            out.update(estimate_resonance(ag[i0:], fs))
        if force is not None:
            f = np.asarray(force, float)
            if f.ndim == 2 and f.shape[0] == t.size and f.shape[1] >= 3:
                fg = np.stack([np.interp(tg, t, f[:, c]) for c in range(3)], axis=1)
                win = max(1, int(round(fs / 2.0)) | 1)
                hp = np.stack([fg[:, c] - np.convolve(fg[:, c], np.ones(win) / win,
                              mode="same") for c in range(3)], axis=1)
                out["force_hf"] = float(np.sqrt(np.mean(np.sum(hp ** 2, axis=1))))
        out["ok"] = True
        return out

    # ---- frequency sweep (chirp) -> Bode bandwidth -----------------------
    if kind == "sweep":
        tg = np.arange(t0 + 0.5, t[-1] - 0.2, 1.0 / fs)  # drop ramp edges
        if tg.size < 64:
            out["ok"] = False
            return out
        ag = np.interp(tg, t, act)
        cg = np.interp(tg, t, cmd)
        out["pos_hf"] = _highpass_rms(ag, fs, 2.0)
        f0 = float(meta_f0) if (meta_f0 is not None) else 0.3
        f1 = float(meta_f1) if (meta_f1 is not None) else 6.0
        out.update(estimate_bode(cg, ag, fs, f0, f1))
        # wrench-referenced resonance from the chirp: use a proper transfer
        # function H_w(f) = FFT(F_axis)/FFT(cmd) on the most-excited force axis,
        # NOT a raw |F| FFT (which is biased to low freq by the log-chirp dwell
        # and the magnitude nonlinearity). The ETFE peak is the structural mode.
        if force is not None:
            f = np.asarray(force, float)
            if f.ndim == 2 and f.shape[0] == t.size and f.shape[1] >= 3:
                fg = np.stack([np.interp(tg, t, f[:, c]) for c in range(3)], axis=1)
                axis = int(np.argmax(np.std(fg, axis=0)))  # most-excited axis
                fa = fg[:, axis] - fg[:, axis].mean()
                cc = cg - cg.mean()
                w = np.hanning(fa.size)
                Fw = np.fft.rfft(fa * w)
                Cw = np.fft.rfft(cc * w)
                ff = np.fft.rfftfreq(fa.size, 1.0 / fs)
                band = (ff >= max(f0, 1.0)) & (ff <= f1)
                magC = np.abs(Cw)
                thr = 0.05 * float(magC[band].max()) if magC[band].size else 0.0
                good = band & (magC > thr)
                if good.sum() >= 8:
                    Hw = np.abs(Fw[good]) / np.abs(Cw[good])
                    fb = ff[good]
                    out["f_n_wrench"] = float(fb[int(np.argmax(Hw))])
                win2 = max(1, int(round(fs / 2.0)) | 1)
                fmag = np.sqrt(np.sum(fg[:, :3] ** 2, axis=1))
                hp = fmag - np.convolve(fmag, np.ones(win2) / win2, mode="same")
                out["force_hf"] = float(np.sqrt(np.mean(hp ** 2)))
        out["ok"] = True
        return out

    # steady window: drop the first second (ramp-in) and last 0.3 s
    a, b = t0 + 1.0, t[-1] - 0.3
    m = (t >= a) & (t <= b)
    tg = np.arange(a, b, 1.0 / fs) if (b - a) > 1.0 else t[m]
    act_g = np.interp(tg, t[m], act[m]) if m.sum() > 2 else act
    cmd_g = np.interp(tg, t[m], cmd[m]) if m.sum() > 2 else cmd
    out["pos_hf"] = _highpass_rms(act_g, fs, 2.0)        # real motion ripple
    # The HOLD segment holds station (no commanded motion), so lag / amplitude /
    # tracking RMSE are undefined -- report the vibration floor and the steady
    # DC error (mean drift of the actual from the commanded baseline).
    if kind == "hold":
        out["dc_err"] = float(np.mean(act_g))
        # Wrench characterisation at rest: the bias (mean |F|) and the broadband
        # noise floor (per-axis std, combined) are the signal a host force loop
        # closes on -- a high noise floor directly limits the usable force
        # resolution and the admittance gain. No motion is commanded here.
        if force is not None:
            f = np.asarray(force, float)
            if f.ndim == 2 and f.shape[0] == t.size and f.shape[1] >= 3:
                fw = f[m, :3] if m.sum() > 2 else f[:, :3]
                out["wrench_bias_n"] = float(np.linalg.norm(np.mean(fw, axis=0)))
                out["wrench_noise_n"] = float(
                    np.sqrt(np.sum(np.var(fw, axis=0))))
    if kind != "hold":
        dt = 1.0 / fs
        # Lag against the RECORDED command, so it is correct for any waveform
        # (smooth sine, staircase or smoothed staircase), not just a pure sine.
        lag = _xcorr_lag_ms(cmd_g, act_g, dt)
        out["lag_ms"] = lag
        # Delay-compensated tracking error: shift the actual back by the lag and
        # compare to the command, so transport delay alone is not counted as error.
        kk = int(round((lag / 1000.0) / dt)) if lag == lag else 0
        kk = max(0, min(kk, act_g.size - 5))
        err = act_g[kk:] - cmd_g[:act_g.size - kk]
        out["rmse"] = float(np.sqrt(np.mean(err ** 2)))
        # Actual amplitude achieved (peak-to-peak / 2); well below the commanded
        # amplitude means the motion was attenuated or clamped.
        out["amp"] = float((np.percentile(act_g, 99) - np.percentile(act_g, 1)) / 2.0)
    if force is not None:
        f = np.asarray(force, float)
        if f.ndim == 2 and f.shape[0] == t.size and f.shape[1] >= 3:
            fg = np.stack([np.interp(tg, t[m], f[m, c]) for c in range(3)], axis=1)
            hp = np.stack([fg[:, c] - np.convolve(
                fg[:, c], np.ones(max(1, int(round(fs / 2.0)) | 1)) /
                (max(1, int(round(fs / 2.0)) | 1)), mode="same")
                for c in range(3)], axis=1)
            out["force_hf"] = float(np.sqrt(np.mean(np.sum(hp ** 2, axis=1))))
    out["ok"] = True
    return out


# ---------------------------------------------------------------------------
# force-control readiness scorecard (Phase 4)
# ---------------------------------------------------------------------------
def _isnum(v: Any) -> bool:
    return isinstance(v, (int, float)) and not (math.isnan(v) or math.isinf(v))


def _grade(value: Optional[float], good: float, marg: float,
           higher_better: bool) -> Optional[int]:
    """Grade a metric -> 2 good / 1 marginal / 0 unsuitable (None if unknown)."""
    if not _isnum(value):
        return None
    if higher_better:
        return 2 if value >= good else (1 if value >= marg else 0)
    return 2 if value <= good else (1 if value <= marg else 0)


_GRADE_NAME = {2: "good", 1: "marginal", 0: "unsuitable"}


def force_control_scorecard(by_kind: Dict[str, Dict[str, Any]],
                            has_force: bool, unit: str = "deg",
                            wrench_latency_ms: Optional[float] = None
                            ) -> Dict[str, Any]:
    """Predict force-control readiness for one joint from its measured metrics.

    ``by_kind`` maps segment kind ("hold"/"smooth"/"step"/"resonance"/...) to its
    metrics dict (as produced by :func:`analyze_segment`). Returns a scorecard:

    * graded criteria (f_n, zeta, hold DC error, actuation lag, step overshoot);
    * a predicted **max stable admittance bandwidth** ``f_bw`` from the headline
      rule of thumb ``min(f_n/3, 1/(2*pi*tau_loop))`` (loop latency = actuation
      lag + optional wrench latency);
    * an overall **Suitable / Marginal / Unsuitable** verdict naming the limiting
      factor, and a recommended starting bandwidth.

    Thresholds follow the plan's §6 table; they are deg-based, so DC-error
    grading is skipped for non-angular (mm) joints.
    """
    res = by_kind.get("resonance", {})
    stp = by_kind.get("step", {})
    hold = by_kind.get("hold", {})
    smooth = by_kind.get("smooth", {})
    sweep = by_kind.get("sweep", {})

    f_n_pos = res.get("f_n")  # position ring-down peak (encoder view)
    swp_wrench = sweep.get("f_n_wrench")  # wrench chirp ETFE peak
    # Choosing the keystone f_n: the achievable stable force bandwidth is capped
    # by the LOWEST lightly-damped structural resonance, so take the MINIMUM of
    # the two CORROBORATING estimators -- the position ring-down and the wrench
    # chirp ETFE. On the Realman these agree tightly (~4.1-5.5 Hz across runs,
    # matching the session's measured 5.3 Hz). The wrench RING-DOWN FFT is
    # deliberately excluded from the keystone: FFT-ing a short, noisy free-decay
    # of a small tap proved unrepeatable (it read 11, 11, then 2.1 Hz across
    # runs and matched neither corroborating estimator); it is reported for
    # transparency (it can catch higher modes) but must not drive the verdict.
    f_n_cands = [(v, s) for v, s in (
        (f_n_pos, "position ring-down"),
        (swp_wrench, "wrench sweep"),
    ) if _isnum(v)]
    if not f_n_cands and _isnum(sweep.get("f_peak_hz")):
        f_n_cands = [(sweep.get("f_peak_hz"), "position sweep")]
    if f_n_cands:
        f_n, f_n_src = min(f_n_cands, key=lambda x: x[0])
    else:
        f_n, f_n_src = None, None
    zeta = res.get("zeta")
    lag = smooth.get("lag_ms")
    overshoot = stp.get("overshoot_pct")
    dc = hold.get("dc_err")
    dc_abs = abs(dc) if _isnum(dc) else None
    meas_bw = sweep.get("bw_hz")  # measured -3 dB command->motion bandwidth
    wrench_noise = hold.get("wrench_noise_n")  # resting force noise floor (N rms)
    wrench_bias = hold.get("wrench_bias_n")
    # streaming jitter: take the worst (max) across the segments that reported it
    jit_vals = [v.get("dt_jitter_ms") for v in by_kind.values()
                if _isnum(v.get("dt_jitter_ms"))]
    jitter_ms = max(jit_vals) if jit_vals else None

    grades: Dict[str, Optional[int]] = {
        "f_n": _grade(f_n, 20.0, 8.0, higher_better=True),
        "zeta": _grade(zeta, 0.1, 0.03, higher_better=True),
        "lag_ms": _grade(lag, 30.0, 60.0, higher_better=False),
        "overshoot_pct": _grade(overshoot, 3.0, 10.0, higher_better=False),
    }
    if _isnum(meas_bw):
        grades["bw_hz"] = _grade(meas_bw, 15.0, 5.0, higher_better=True)
    if _isnum(wrench_noise):
        # resting force noise floor: < 0.1 N excellent, < 0.5 N usable, else high
        grades["wrench_noise"] = _grade(wrench_noise, 0.1, 0.5, higher_better=False)
    if _isnum(jitter_ms):
        # streaming jitter vs a 5 ms (200 Hz) tick: < 1 ms good, < 3 ms usable
        grades["jitter"] = _grade(jitter_ms, 1.0, 3.0, higher_better=False)
    if unit == "deg":
        grades["hold_dc"] = _grade(dc_abs, 0.02, 0.1, higher_better=False)

    # ---- predicted stable admittance bandwidth ----
    tau_s = 0.0
    if _isnum(lag):
        tau_s += float(lag) / 1000.0
    if _isnum(wrench_latency_ms):
        tau_s += float(wrench_latency_ms) / 1000.0
    f_bw_struct = (f_n / 3.0) if _isnum(f_n) else None
    f_bw_lat = (1.0 / (2.0 * math.pi * tau_s)) if tau_s > 1e-6 else None
    f_bw_meas = float(meas_bw) if _isnum(meas_bw) else None
    cands = [(v, name) for v, name in
             ((f_bw_struct, "resonance"), (f_bw_lat, "loop latency"),
              (f_bw_meas, "measured tracking BW")) if v]
    f_bw = min(c[0] for c in cands) if cands else None
    limit_by = min(cands)[1] if cands else None

    # ---- overall verdict ----
    crit = [g for g in (grades.get("f_n"), grades.get("lag_ms"),
                        grades.get("hold_dc")) if g is not None]
    worst = min(crit) if crit else None
    reasons: List[str] = []
    if not _isnum(f_n):
        # the keystone resonance was not measured (no resonance/sweep test run),
        # so we cannot predict a force bandwidth -- do not claim Suitable.
        verdict, cls = "Inconclusive", ""
        reasons.append("resonance not measured (run the 'resonance' and/or "
                       "'sweep' test) \u2014 cannot predict force bandwidth")
    elif _isnum(f_bw) and f_bw < 2.0:
        verdict, cls = "Unsuitable", "bad"
        reasons.append(f"predicted force bandwidth \u2248 {f_bw:.1f} Hz (< 2 Hz) "
                       f"\u2014 host-side admittance not worth it; prefer the arm's "
                       f"native in-arm force mode")
    elif worst == 0:
        verdict, cls = "Unsuitable", "bad"
    elif worst == 1 or (_isnum(f_bw) and f_bw < 5.0):
        verdict, cls = "Marginal", "warn"
    elif worst == 2:
        verdict, cls = "Suitable", "good"
    else:
        verdict, cls = "Inconclusive", ""
    if _isnum(f_n) and f_n < 8.0:
        reasons.append(f"soft structure (f_n \u2248 {f_n:.1f} Hz) caps bandwidth")
    if grades.get("hold_dc") == 0 and _isnum(dc_abs):
        reasons.append(f"large hold DC error ({dc_abs:.2f}\u00b0) \u2014 fix the command "
                       f"shaper first")
    if grades.get("lag_ms") == 0 and _isnum(lag):
        reasons.append(f"high actuation lag ({lag:.0f} ms) erodes phase margin")
    if grades.get("wrench_noise") == 0 and _isnum(wrench_noise):
        reasons.append(f"high resting wrench noise ({wrench_noise:.2f} N rms) "
                       f"limits force resolution / max admittance gain")
    if grades.get("jitter") == 0 and _isnum(jitter_ms):
        reasons.append(f"high command-stream jitter ({jitter_ms:.1f} ms) adds "
                       f"variable latency")

    rec = None
    if _isnum(f_bw):
        # recommend a safe starting admittance bandwidth (~70 % of the ceiling)
        rec = max(0.2, round(0.7 * f_bw, 1))

    return {
        "verdict": verdict, "cls": cls,
        "grades": grades,
        "grade_names": {k: _GRADE_NAME.get(g) for k, g in grades.items()
                        if g is not None},
        "f_n": f_n, "zeta": zeta, "lag_ms": lag,
        "f_n_src": f_n_src,
        "f_n_pos": f_n_pos, "f_n_sweep": swp_wrench,
        "overshoot_pct": overshoot, "hold_dc": dc,
        "meas_bw_hz": f_bw_meas,
        "wrench_noise_n": wrench_noise, "wrench_bias_n": wrench_bias,
        "jitter_ms": jitter_ms,
        "f_bw_hz": f_bw, "f_bw_struct_hz": f_bw_struct,
        "f_bw_latency_hz": f_bw_lat, "tau_loop_ms": tau_s * 1000.0,
        "limited_by": limit_by,
        "recommended_bw_hz": rec,
        "reasons": reasons,
        "wrench_latency_ms": wrench_latency_ms,
    }


_VERDICT_RANK = {"Unsuitable": 0, "Marginal": 1, "Suitable": 2, "Inconclusive": 3}


def rollup_scorecards(per_joint: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Robot-level rollup: the worst per-joint verdict drives the overall call."""
    if not per_joint:
        return {"verdict": "Inconclusive", "cls": "", "limiting_joint": None,
                "f_bw_hz": None}
    def keyf(item):
        v = item[1].get("verdict", "Inconclusive")
        return (_VERDICT_RANK.get(v, 3), item[1].get("f_bw_hz") or 1e9)
    jname, sc = min(per_joint.items(), key=keyf)
    bws = [s.get("f_bw_hz") for s in per_joint.values() if _isnum(s.get("f_bw_hz"))]
    return {"verdict": sc.get("verdict"), "cls": sc.get("cls"),
            "limiting_joint": jname, "f_bw_hz": (min(bws) if bws else None),
            "reasons": sc.get("reasons", [])}
