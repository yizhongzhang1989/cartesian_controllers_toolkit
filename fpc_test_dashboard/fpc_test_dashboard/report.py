"""Per-run report generation + history for the FPC test dashboard.

Every test run is persisted to disk so it can be reviewed later, in the same
spirit as ``temp/fp_control_test/exp1_ros2_control``:

* matplotlib plots (commanded vs. actual offset, plus tracking error and the
  independent wrench-vibration trace) rendered per joint/segment, and
* a self-contained ``report.html`` with every image embedded as base64 and a
  colour-coded metrics table + per-joint capability verdict, so the single file
  is enough to view the run with no external assets.

Layout on disk (under ``report_dir``, default ``~/.ros/fpc_test_dashboard/runs``)::

    <report_dir>/
      2026-06-10_14-23-05_right_arm_forward_position_controller/
        report.html       self-contained, embedded PNGs + tables
        run.npz           raw arrays (t/cmd/act/force per segment) for re-analysis
        manifest.json     config + per-segment metrics (used to list history)

This module is pure Python (numpy + matplotlib); it has no ROS dependency so it
can be unit-tested. matplotlib is imported lazily inside :func:`save_run` so that
:func:`list_runs` keeps working even where matplotlib is unavailable.
"""
from __future__ import annotations

import base64
import io
import json
import math
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

# colours (match the dashboard's dark theme but legible on white plot canvas)
_C_CMD = "#9aa4b2"
_C_ACT = "#2563eb"
_C_ERR = "#d33"
_C_FORCE = "#f59e0b"

_KIND_ORDER = {"hold": 0, "smooth": 1, "stair": 2, "smoothed": 3}


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _slug(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(s)).strip("_") or "run"


def _force_hf_trace(force: np.ndarray, fs: float) -> Optional[np.ndarray]:
    """Instantaneous wrench-vibration magnitude: |F - movavg(F)| over XYZ.

    Mirrors the high-pass used in :func:`test_logic.analyze_segment` so the
    plotted trace and the reported ``force_hf`` scalar are consistent.
    """
    if force is None:
        return None
    f = np.asarray(force, float)
    if f.ndim != 2 or f.shape[1] < 3 or f.shape[0] < 4:
        return None
    win = max(1, int(round(fs / 2.0)))
    if win % 2 == 0:
        win += 1
    k = np.ones(win) / win
    hp = np.empty((f.shape[0], 3))
    for c in range(3):
        hp[:, c] = f[:, c] - np.convolve(f[:, c], k, mode="same")
    return np.sqrt(np.sum(hp ** 2, axis=1))


def _fmt(v: Any, nd: int = 2) -> str:
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "&mdash;"
    return f"{float(v):.{nd}f}"


def _cls_ratio(r: Optional[float]) -> str:
    if r is None or math.isnan(r):
        return ""
    return "good" if r < 1.5 else ("warn" if r < 3.0 else "bad")


def _cls_lag(v: Optional[float]) -> str:
    if v is None or math.isnan(v):
        return ""
    return "good" if v < 60 else ("warn" if v < 150 else "bad")


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------
def _joint_figure(plt, joint: str, recs: List[Dict[str, Any]],
                  unit: str, has_force: bool):
    """One stacked figure for a joint: a row per segment, cmd vs actual (+force)."""
    recs = sorted(recs, key=lambda r: _KIND_ORDER.get(r.get("kind", ""), 9))
    n = len(recs)
    fig, axes = plt.subplots(n, 1, figsize=(9.2, max(1.0, 1.9 * n)),
                             squeeze=False)
    axes = axes[:, 0]
    for ax, r in zip(axes, recs):
        t = np.asarray(r["t"], float)
        cmd = np.asarray(r["cmd"], float)
        act = np.asarray(r["act"], float)
        m = r.get("metrics", {})
        ax.plot(t, cmd, color=_C_CMD, lw=1.3, label="command")
        ax.plot(t, act, color=_C_ACT, lw=1.1, label="actual")
        ax.set_ylabel(f"{unit}", fontsize=8)
        ax.grid(alpha=0.3)
        bits = [r["segment"]]
        if r.get("kind") != "hold":
            bits.append(f"lag {_txt(m.get('lag_ms'),0)} ms")
            bits.append(f"rmse {_txt(m.get('rmse'),2)} {unit}")
        bits.append(f"pos HF {_txt(m.get('pos_hf'),3)} {unit}")
        if has_force and m.get("force_hf") is not None:
            bits.append(f"force HF {_txt(m.get('force_hf'),3)} N")
        ax.set_title("   ·   ".join(bits), fontsize=8.5, loc="left")
        if has_force:
            fhf = _force_hf_trace(r.get("force"), float(r.get("fs", 200.0)))
            if fhf is not None and fhf.size == t.size:
                ax2 = ax.twinx()
                ax2.plot(t, fhf, color=_C_FORCE, lw=0.8, alpha=0.85)
                ax2.set_ylabel("|F|hf (N)", color=_C_FORCE, fontsize=7)
                ax2.tick_params(axis="y", labelsize=6, colors=_C_FORCE)
                ax2.set_ylim(bottom=0)
    axes[0].legend(loc="upper right", fontsize=7.5, ncol=2)
    axes[-1].set_xlabel("time in segment (s)")
    fig.suptitle(joint, fontsize=10, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    return fig


def _txt(v: Any, nd: int) -> str:
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "n/a"
    return f"{float(v):.{nd}f}"


def _fig_to_b64(plt, fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=92, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


# ---------------------------------------------------------------------------
# capability verdict (per joint)
# ---------------------------------------------------------------------------
def _verdict(recs: List[Dict[str, Any]], has_force: bool) -> Dict[str, Any]:
    """Compare each test's vibration to the joint's HOLD floor -> a one-line call."""
    key = "force_hf" if has_force else "pos_hf"
    by = {r.get("kind"): r.get("metrics", {}) for r in recs}
    floor = by.get("hold", {}).get(key)
    floor = floor if (floor and floor > 1e-9) else None

    def ratio(kind):
        v = by.get(kind, {}).get(key)
        if v is None or floor is None:
            return None
        return v / floor

    r_sm, r_st, r_smd = ratio("smooth"), ratio("stair"), ratio("smoothed")
    lag_sm = by.get("smooth", {}).get("lag_ms")
    # verdict text
    if r_sm is not None and r_sm < 1.6 and (lag_sm is None or lag_sm < 120):
        head = "GOOD — smooth tracks at the vibration floor"
        cls = "good"
    elif r_sm is not None and r_sm < 3.0:
        head = "FAIR — some ripple even on smooth"
        cls = "warn"
    else:
        head = "CHECK — elevated vibration / lag on smooth"
        cls = "bad"
    note = ""
    if r_st is not None and r_smd is not None:
        if r_st > 1.8 and r_smd < 1.6:
            note = "stair vibrates; host low-pass recovers it (use the smoother)"
        elif r_st > 1.8:
            note = "stair vibrates and the low-pass only partly helps"
        else:
            note = "stair already clean (low-rate streaming is tolerable)"
    return {"head": head, "cls": cls, "note": note,
            "floor": floor, "key": key,
            "r_smooth": r_sm, "r_stair": r_st, "r_smoothed": r_smd,
            "lag_smooth": lag_sm}


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------
def save_run(report_dir: str, run_meta: Dict[str, Any],
             records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Render + persist one run. Returns a metadata dict (id, name, paths, …).

    ``records`` is a list of per-(joint,segment) dicts with keys:
    ``joint, segment, kind, unit, fs, t, cmd, act, force(optional), metrics``
    where t/cmd/act are 1-D arrays (offsets, already in the display unit) and
    force is an (N,3) wrench array in Newtons (or ``None``).
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    root = Path(report_dir).expanduser()
    ctrl = str(run_meta.get("controller", "controller"))
    ts = time.localtime(run_meta.get("started", time.time()))
    run_id = f"{time.strftime('%Y-%m-%d_%H-%M-%S', ts)}_{_slug(ctrl)}"
    rundir = root / run_id
    rundir.mkdir(parents=True, exist_ok=True)

    has_force = bool(run_meta.get("wrench"))
    joints = list(dict.fromkeys(r["joint"] for r in records))  # ordered unique
    unit_by = {r["joint"]: r.get("unit", "deg") for r in records}

    # --- plots + verdicts per joint ---
    figs_b64: Dict[str, str] = {}
    verdicts: Dict[str, Dict[str, Any]] = {}
    for j in joints:
        recs = [r for r in records if r["joint"] == j]
        fig = _joint_figure(plt, j, recs, unit_by.get(j, "deg"), has_force)
        figs_b64[j] = _fig_to_b64(plt, fig)
        verdicts[j] = _verdict(recs, has_force)

    # --- raw arrays for later re-analysis ---
    arrays: Dict[str, np.ndarray] = {}
    seg_index = []
    for i, r in enumerate(records):
        pfx = f"{i}"
        arrays[f"{pfx}_t"] = np.asarray(r["t"], float)
        arrays[f"{pfx}_cmd"] = np.asarray(r["cmd"], float)
        arrays[f"{pfx}_act"] = np.asarray(r["act"], float)
        if r.get("force") is not None:
            arrays[f"{pfx}_force"] = np.asarray(r["force"], float)
        seg_index.append({"i": i, "joint": r["joint"], "segment": r["segment"],
                          "kind": r.get("kind"), "unit": r.get("unit", "deg"),
                          "fs": float(r.get("fs", 200.0))})
    try:
        np.savez_compressed(rundir / "run.npz", **arrays)
    except Exception:  # noqa: BLE001
        pass

    # --- manifest (drives the history list) ---
    metrics = [{"joint": r["joint"], "segment": r["segment"],
                "kind": r.get("kind"), "unit": r.get("unit", "deg"),
                **{k: r["metrics"].get(k) for k in
                   ("pos_hf", "force_hf", "lag_ms", "rmse", "amp")}}
               for r in records]
    manifest = {
        "id": run_id, "name": run_id,
        "time": run_meta.get("started", time.time()),
        "controller": ctrl,
        "joints": joints,
        "params": {k: run_meta.get(k) for k in
                   ("amp_deg", "freq_hz", "seg_seconds", "stair_step_deg",
                    "lp_hz", "limit_deg", "tests")},
        "wrench": has_force,
        "n_segments": len(records),
        "metrics": metrics,
        "verdicts": {j: {k: verdicts[j].get(k) for k in
                         ("head", "cls", "note")} for j in joints},
    }
    (rundir / "manifest.json").write_text(json.dumps(manifest, indent=2))

    # --- self-contained HTML ---
    html = _render_html(manifest, figs_b64, verdicts, unit_by, has_force)
    (rundir / "report.html").write_text(html, encoding="utf-8")

    return {"id": run_id, "name": run_id, "dir": str(rundir),
            "html": str(rundir / "report.html"),
            "time": manifest["time"], "joints": joints,
            "n_segments": len(records)}


def list_runs(report_dir: str, limit: int = 100) -> List[Dict[str, Any]]:
    """Return saved-run metadata (newest first) by scanning manifest.json files."""
    root = Path(report_dir).expanduser()
    out: List[Dict[str, Any]] = []
    if not root.is_dir():
        return out
    for d in root.iterdir():
        man = d / "manifest.json"
        if not (d.is_dir() and man.is_file()):
            continue
        try:
            m = json.loads(man.read_text())
        except Exception:  # noqa: BLE001
            continue
        # compact per-joint worst-case force/pos HF for the summary column
        worst = None
        key = "force_hf" if m.get("wrench") else "pos_hf"
        vals = [x.get(key) for x in m.get("metrics", []) if x.get("kind") != "hold"
                and isinstance(x.get(key), (int, float))]
        if vals:
            worst = max(vals)
        out.append({
            "id": m.get("id", d.name), "name": m.get("name", d.name),
            "time": m.get("time"), "controller": m.get("controller"),
            "joints": m.get("joints", []), "n_segments": m.get("n_segments"),
            "wrench": m.get("wrench", False),
            "worst_hf": worst, "worst_key": key,
            "verdicts": m.get("verdicts", {}),
            "has_html": (d / "report.html").is_file(),
            "has_npz": (d / "run.npz").is_file(),
        })
    out.sort(key=lambda r: r.get("time") or 0, reverse=True)
    return out[:limit]


def resolve_run_file(report_dir: str, run_id: str, fname: str) -> Optional[Path]:
    """Safely resolve ``<report_dir>/<run_id>/<fname>`` (no traversal). """
    if fname not in ("report.html", "run.npz", "manifest.json"):
        return None
    root = Path(report_dir).expanduser().resolve()
    target = (root / run_id / fname).resolve()
    try:
        target.relative_to(root)
    except ValueError:
        return None
    return target if target.is_file() else None


# ---------------------------------------------------------------------------
# HTML rendering
# ---------------------------------------------------------------------------
def _render_html(manifest, figs_b64, verdicts, unit_by, has_force) -> str:
    arm = manifest["controller"]
    p = manifest.get("params", {})
    when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(manifest["time"]))

    # metrics rows grouped by joint
    body_sections = []
    for j in manifest["joints"]:
        unit = unit_by.get(j, "deg")
        v = verdicts.get(j, {})
        rows = ""
        for mrow in manifest["metrics"]:
            if mrow["joint"] != j:
                continue
            is_hold = mrow.get("kind") == "hold"
            floor = v.get("floor")
            key = v.get("key", "force_hf" if has_force else "pos_hf")
            hf_val = mrow.get(key)
            ratio = (hf_val / floor) if (floor and hf_val and not is_hold) else None
            fhf = (f"<td class='{_cls_ratio(ratio)}'>{_fmt(mrow.get('force_hf'), 3)}</td>"
                   if has_force else "")
            rows += (
                f"<tr class='{mrow.get('kind','')}'>"
                f"<td class='seg'>{mrow['segment']}</td>"
                f"<td class='{'' if is_hold else _cls_ratio(ratio if key=='pos_hf' else None)}'>"
                f"{_fmt(mrow.get('pos_hf'), 3)}</td>"
                f"{fhf}"
                f"<td class='{_cls_lag(mrow.get('lag_ms'))}'>{_fmt(mrow.get('lag_ms'), 0)}</td>"
                f"<td>{_fmt(mrow.get('rmse'), 3)}</td>"
                f"<td>{_fmt(mrow.get('amp'), 2)}</td>"
                f"</tr>")
        fhf_hdr = "<th>force HF (N)</th>" if has_force else ""
        note = f" — {v.get('note')}" if v.get("note") else ""
        body_sections.append(f"""
        <section class="joint">
          <h2>{j} <span class="unit">({unit})</span></h2>
          <div class="verdict {v.get('cls','')}">{v.get('head','')}{note}</div>
          <table>
            <tr><th>segment</th><th>pos HF ({unit})</th>{fhf_hdr}
                <th>lag (ms)</th><th>RMSE ({unit})</th><th>amp ({unit})</th></tr>
            {rows}
          </table>
          <div class="plot"><img alt="{j} plot"
               src="data:image/png;base64,{figs_b64.get(j,'')}"/></div>
        </section>""")

    params_line = (
        f"amp {p.get('amp_deg')}° · freq {p.get('freq_hz')} Hz · "
        f"seg {p.get('seg_seconds')} s · stair step {p.get('stair_step_deg')}° · "
        f"LP {p.get('lp_hz')} Hz · clamp ±{p.get('limit_deg')}° · "
        f"tests {', '.join(p.get('tests') or [])}")

    legend = ("Colour key — vibration ratio vs. the joint's HOLD floor: "
              "&lt;1.5× good · &lt;3× fair · else high. "
              "lag: &lt;60 ms good · &lt;150 ms fair. "
              "force HF = independent wrench vibration (ground truth); "
              "pos HF = joint-motion ripple.")

    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FPC test report — {manifest['id']}</title>
<style>
  body{{font-family:-apple-system,Segoe UI,Roboto,sans-serif;margin:24px;
       color:#1b1f24;background:#f6f7f9}}
  h1{{margin:0 0 2px;font-size:1.3rem}} .sub{{color:#667;margin:0 0 4px}}
  .meta{{color:#556;font-size:13px;margin-bottom:10px}}
  .legend{{font-size:12px;color:#667;margin:6px 0 18px}}
  section.joint{{background:#fff;border:1px solid #e2e6ea;border-radius:10px;
       padding:14px 16px;margin:14px 0}}
  h2{{margin:0 0 8px;font-size:1.05rem}} .unit{{color:#889;font-weight:400;font-size:0.9rem}}
  .verdict{{display:inline-block;padding:4px 10px;border-radius:6px;font-size:13px;
       font-weight:600;margin-bottom:10px}}
  .verdict.good{{background:#e6f7ea;color:#176c2c}}
  .verdict.warn{{background:#fff6e0;color:#8a5a00}}
  .verdict.bad{{background:#fde7e7;color:#a12020}}
  table{{border-collapse:collapse;margin:6px 0 12px;font-size:13px}}
  th,td{{border:1px solid #e3e7ec;padding:4px 9px;text-align:right}}
  th{{background:#eef2f7}} td.seg{{text-align:left;font-family:monospace}}
  tr.hold td{{color:#889}}
  .good{{background:#e6f7ea}} .warn{{background:#fff6e0}} .bad{{background:#fde7e7}}
  .plot img{{max-width:980px;width:100%;border:1px solid #eaedf1;border-radius:8px}}
</style></head><body>
<h1>FPC test report</h1>
<p class="sub">Controller: <b>{arm}</b> · {when}</p>
<p class="meta">{params_line}</p>
<p class="legend">{legend}</p>
{''.join(body_sections)}
</body></html>"""
