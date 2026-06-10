"""Pure-Python core for the FPC test dashboard (no ROS dependency).

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
    kind: str            # "hold" | "smooth" | "stair" | "smoothed"
    duration: float      # seconds
    refresh_hz: float    # command-value refresh rate (>= send rate => every tick)
    lp_hz: float = 0.0   # host low-pass cutoff applied to the command (0 = off)


@dataclass
class TestParams:
    amp: float = 0.07           # rad (~4 deg) or m for prismatic
    freq: float = 0.2           # Hz
    seg_seconds: float = 9.0
    send_rate: float = 200.0    # Hz (publish + analysis grid)
    stair_step: float = 0.0262  # rad (~1.5 deg) -- staircase riser height
    lp_hz: float = 10.0         # smoother cutoff for the SMOOTHED segment
    include: Sequence[str] = field(
        default_factory=lambda: ("smooth", "stair", "smoothed"))


def build_plan(p: TestParams) -> List[Segment]:
    """The per-joint segment battery (a short HOLD floor first, then the tests)."""
    segs: List[Segment] = [Segment("HOLD", "hold", min(5.0, p.seg_seconds), p.send_rate)]
    if "smooth" in p.include:
        segs.append(Segment("SMOOTH", "smooth", p.seg_seconds, p.send_rate))
    if "stair" in p.include:
        segs.append(Segment("STAIR", "stair", p.seg_seconds, p.send_rate))
    if "smoothed" in p.include:
        segs.append(Segment("SMOOTHED", "smoothed", p.seg_seconds, p.send_rate, p.lp_hz))
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
    base = amp * math.sin(2.0 * math.pi * freq * (k * dt))
    if seg.kind == "smooth":
        return base
    # stair / smoothed: quantise the sine into discrete levels.
    step = stair_step if (stair_step and stair_step > 0.0) else (amp / 3.0)
    q = round(base / step) * step
    if q > amp:
        q = amp
    elif q < -amp:
        q = -amp
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


def analyze_segment(t, cmd, act, freq: float, fs: float,
                    force=None, kind: str = "") -> Dict[str, Any]:
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
    # steady window: drop the first second (ramp-in) and last 0.3 s
    a, b = t0 + 1.0, t[-1] - 0.3
    m = (t >= a) & (t <= b)
    tg = np.arange(a, b, 1.0 / fs) if (b - a) > 1.0 else t[m]
    act_g = np.interp(tg, t[m], act[m]) if m.sum() > 2 else act
    cmd_g = np.interp(tg, t[m], cmd[m]) if m.sum() > 2 else cmd
    out["pos_hf"] = _highpass_rms(act_g, fs, 2.0)        # real motion ripple
    # The HOLD segment holds station (no commanded motion), so lag / amplitude /
    # tracking RMSE are undefined -- report only the vibration floor.
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
