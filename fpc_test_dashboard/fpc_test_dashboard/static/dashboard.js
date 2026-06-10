"use strict";
// FPC Test Dashboard front-end: populates joints/controllers from /api/info,
// drives a run via /api/run, polls /api/state for the live trace + results.
(() => {
  const $ = (id) => document.getElementById(id);
  const conn = $("conn"), statusEl = $("status"), prog = $("prog");
  const selCtrl = $("sel-ctrl"), jointList = $("joint-list"), wrenchState = $("wrench-state");
  const btnRun = $("btn-run"), btnStop = $("btn-stop");
  const tbody = document.querySelector("#results tbody");
  const plot = $("plot"), ctx = plot.getContext("2d");
  const runsList = $("runs-list"), runsDir = $("runs-dir"), reportBanner = $("report-banner");

  let info = null;           // /api/info snapshot
  let ctrlByName = {};       // name -> controller
  let unit = "deg";
  let lastReportId = null;   // track the newest saved run to surface a link

  // ---- setup -------------------------------------------------------------
  async function loadInfo() {
    try {
      const r = await fetch("/api/info"); info = await r.json();
      conn.textContent = "connected"; conn.className = "status ok";
    } catch (e) {
      conn.textContent = "disconnected"; conn.className = "status bad"; return;
    }
    ctrlByName = {};
    const prev = selCtrl.value;
    selCtrl.innerHTML = "";
    (info.controllers || []).forEach((c) => {
      ctrlByName[c.name] = c;
      const o = document.createElement("option");
      o.value = c.name;
      o.textContent = `${c.name}  [${c.state}, ${c.joints.length} joints]`;
      selCtrl.appendChild(o);
    });
    if (!info.controllers || info.controllers.length === 0) {
      const o = document.createElement("option");
      o.textContent = "(no ForwardCommandController found — launch the robot first)";
      o.disabled = true; selCtrl.appendChild(o);
    }
    if (prev && ctrlByName[prev]) selCtrl.value = prev;
    wrenchState.textContent = info.wrench_available ? "available" : "none";
    wrenchState.className = "badge " + (info.wrench_available ? "ok" : "");
    $("lg-force").hidden = !info.wrench_available;
    if (info.report_dir) runsDir.textContent = info.report_dir;
    // apply server defaults once
    if (!loadInfo._applied && info.defaults) {
      const d = info.defaults;
      $("p-amp").value = d.amp_deg; $("p-freq").value = d.freq_hz;
      $("p-seg").value = d.seg_seconds; $("p-stair").value = d.stair_step_deg;
      $("p-lp").value = d.lp_hz; $("p-limit").value = d.limit_deg;
      loadInfo._applied = true;
    }
    renderJoints();
  }

  function renderJoints() {
    const c = ctrlByName[selCtrl.value];
    // Only rebuild when the controller / joint set actually changes, so the
    // periodic loadInfo() refresh (every 3 s) doesn't wipe the user's ticks.
    const sig = c ? c.name + "|" + c.joints.join(",") : "";
    if (sig === renderJoints._sig) return;
    renderJoints._sig = sig;
    // Preserve any joints the user had already checked across a real rebuild.
    const checked = new Set(
      [...jointList.querySelectorAll(".j-cb:checked")].map((cb) => cb.value));
    jointList.innerHTML = "";
    if (!c) { jointList.innerHTML = "<em>select a controller…</em>"; return; }
    const byName = {}; (info.joints || []).forEach((j) => (byName[j.name] = j));
    c.joints.forEach((jn) => {
      const j = byName[jn] || {};
      const lab = document.createElement("label");
      const cb = document.createElement("input");
      cb.type = "checkbox"; cb.className = "j-cb"; cb.value = jn;
      if (checked.has(jn)) cb.checked = true;
      lab.appendChild(cb);
      const span = document.createElement("span");
      span.textContent = " " + jn + (j.is_prismatic ? " (lin)" : "");
      lab.appendChild(span);
      jointList.appendChild(lab);
    });
  }

  selCtrl.addEventListener("change", renderJoints);
  $("btn-all").addEventListener("click", () =>
    document.querySelectorAll(".j-cb").forEach((c) => (c.checked = true)));
  $("btn-none").addEventListener("click", () =>
    document.querySelectorAll(".j-cb").forEach((c) => (c.checked = false)));

  // ---- run / stop --------------------------------------------------------
  btnRun.addEventListener("click", async () => {
    const joints = [...document.querySelectorAll(".j-cb:checked")].map((c) => c.value);
    if (!joints.length) { alert("Select at least one joint."); return; }
    const tests = [...document.querySelectorAll(".t-test:checked")].map((c) => c.value);
    if (!tests.length) { alert("Select at least one test."); return; }
    const ampv = +$("p-amp").value, limv = +$("p-limit").value;
    if (ampv > limv) {
      alert(`Amplitude ${ampv}° exceeds the safety limit ${limv}°, so the motion ` +
            `would be clamped (flat-topped). Raise the Safety limit to ≥ ${ampv}° ` +
            `(clear the workspace first) or lower the amplitude.`);
      return;
    }
    const cfg = {
      controller: selCtrl.value, joints, tests,
      amp_deg: +$("p-amp").value, freq_hz: +$("p-freq").value,
      seg_seconds: +$("p-seg").value, stair_step_deg: +$("p-stair").value,
      lp_hz: +$("p-lp").value, limit_deg: +$("p-limit").value,
    };
    const r = await fetch("/api/run", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(cfg),
    });
    const j = await r.json();
    if (!j.ok) alert("Could not start: " + j.message);
    else { tbody.innerHTML = ""; resetPlot(); reportBanner.hidden = true; }
  });

  btnStop.addEventListener("click", () =>
    fetch("/api/stop", { method: "POST" }));

  // ---- live state polling ------------------------------------------------
  async function poll() {
    let s;
    try { s = await (await fetch("/api/state")).json(); }
    catch (e) { conn.textContent = "disconnected"; conn.className = "status bad"; return; }
    const running = s.status === "running";
    btnRun.disabled = running; btnStop.disabled = !running;
    statusEl.textContent = s.status + (s.message ? " — " + s.message : "");
    statusEl.className = "status " + (running ? "run" :
      (s.status === "error" ? "bad" : (s.status === "done" ? "ok" : "")));
    const p = s.progress || {};
    prog.textContent = p.n ? `${p.joint} · ${p.segment} (${p.i}/${p.n})` : "—";
    if (s.js_age != null && s.js_age > 0.5) {
      conn.textContent = "joint_states stale"; conn.className = "status bad";
    } else { conn.textContent = "connected"; conn.className = "status ok"; }
    drawPlot(s.live || [], p);
    renderResults(s.results || []);
    handleReport(s.last_report, s.status);
  }

  // ---- saved runs --------------------------------------------------------
  function handleReport(rep, status) {
    if (status === "error" && (!rep || rep.id !== lastReportId)) {
      // a failed run without a saved report: leave any prior banner alone
    }
    if (!rep || !rep.id) { return; }
    if (rep.id === lastReportId) return;        // already surfaced
    lastReportId = rep.id;
    reportBanner.hidden = false;
    reportBanner.className = "report-banner";
    const url = `/api/runs/${encodeURIComponent(rep.id)}/report.html`;
    reportBanner.innerHTML =
      `<span>✓ Report saved — <code>${rep.id}</code></span>` +
      `<a href="${url}" target="_blank" rel="noopener">Open report ↗</a>`;
    loadRuns();   // refresh the history list
  }

  async function loadRuns() {
    let data;
    try { data = await (await fetch("/api/runs")).json(); }
    catch (e) { return; }
    if (data.report_dir) runsDir.textContent = data.report_dir;
    renderRuns(data.runs || []);
  }

  function renderRuns(runs) {
    runsList.innerHTML = "";
    if (!runs.length) { runsList.innerHTML = "<em>no saved runs yet</em>"; return; }
    runs.forEach((r) => {
      const item = document.createElement("div");
      item.className = "run-item";
      const main = document.createElement("div");
      main.className = "ri-main";
      const when = r.time ? new Date(r.time * 1000).toLocaleString() : r.id;
      const joints = (r.joints || []).join(", ");
      const hf = (r.worst_hf != null)
        ? ` · worst ${r.worst_key === "force_hf" ? "force" : "pos"} HF ${(+r.worst_hf).toFixed(3)}`
        : "";
      // overall verdict = worst of the per-joint classes
      const vs = Object.values(r.verdicts || {});
      const rank = { good: 0, warn: 1, bad: 2 };
      let cls = "good", head = "";
      vs.forEach((v) => { if ((rank[v.cls] ?? 0) >= (rank[cls] ?? 0)) { cls = v.cls; head = v.head; } });
      main.innerHTML =
        `<span class="ri-name">${r.id}</span>` +
        `<span class="ri-sub">${when} · ${r.controller || ""} · ${joints}` +
        ` · ${r.n_segments || 0} segs${hf}</span>` +
        (vs.length ? `<span class="ri-verdict ${cls}">${head || ""}</span>` : "");
      const actions = document.createElement("div");
      actions.className = "ri-actions";
      const base = `/api/runs/${encodeURIComponent(r.id)}`;
      if (r.has_html)
        actions.innerHTML += `<a class="btnlink open" href="${base}/report.html" target="_blank" rel="noopener">Open ↗</a>`;
      if (r.has_npz)
        actions.innerHTML += `<a class="btnlink" href="${base}/run.npz">Data</a>`;
      item.appendChild(main); item.appendChild(actions);
      runsList.appendChild(item);
    });
  }

  // ---- results table -----------------------------------------------------
  function renderResults(rows) {
    if (rows.length === tbody.childElementCount) return;  // no change
    tbody.innerHTML = "";
    rows.forEach((r) => {
      const tr = document.createElement("tr");
      tr.className = r.kind || "";
      const u = r.unit || "deg";
      const fhf = r.force_hf != null ? r.force_hf.toFixed(3) : "—";
      const cells = [
        r.joint, r.segment,
        fmt(r.pos_hf, 3) + " " + u, fhf,
        fmt(r.lag_ms, 0), fmt(r.rmse, 3) + " " + u, fmt(r.amp, 2) + " " + u,
      ];
      cells.forEach((c, i) => {
        const td = document.createElement("td");
        td.textContent = c; tr.appendChild(td);
      });
      tbody.appendChild(tr);
    });
  }
  const fmt = (v, n) => (v == null || v !== v) ? "—" : (+v).toFixed(n);

  // ---- live plot ---------------------------------------------------------
  let plotMax = 5;
  function resetPlot() { plotMax = 5; ctx.clearRect(0, 0, plot.width, plot.height); }
  function drawPlot(live, prog) {
    const W = plot.width, H = plot.height;
    ctx.clearRect(0, 0, W, H);
    if (!live.length) return;
    // Each segment is shown on its own (the server clears the live buffer per
    // segment), so the x-axis is "time within the current segment".
    const tEnd = live[live.length - 1][0];
    const tStart = Math.max(0, tEnd - 12);
    const seg = live.filter((d) => d[0] >= tStart);
    let amax = 1;
    seg.forEach((d) => {
      if (d[1] != null) amax = Math.max(amax, Math.abs(d[1]));
      if (d[2] != null) amax = Math.max(amax, Math.abs(d[2]));
    });
    amax = Math.ceil(amax * 1.15);
    const x = (t) => ((t - tStart) / Math.max(0.1, tEnd - tStart)) * (W - 50) + 40;
    const y = (v) => H / 2 - (v / amax) * (H / 2 - 18);
    // grid
    ctx.strokeStyle = "#1c2230"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(40, H / 2); ctx.lineTo(W - 10, H / 2); ctx.stroke();
    ctx.fillStyle = "#5b6675"; ctx.font = "11px ui-sans-serif";
    ctx.fillText(`+${amax}`, 6, y(amax) + 10);
    ctx.fillText(`-${amax}`, 6, y(-amax) - 2);
    // command + actual
    line(seg, 1, "#9aa4b2", x, y);
    line(seg, 2, "#38bdf8", x, y);
    // current segment label (top-left) so the single-segment view is clear
    if (prog && prog.segment) {
      ctx.fillStyle = "#aeb6c2"; ctx.font = "12px ui-sans-serif";
      ctx.fillText(`${prog.joint || ""} · ${prog.segment}`, 44, 14);
    }
    // force HF on a secondary scale (right) if present
    if (info && info.wrench_available) {
      let fmax = 0.5; seg.forEach((d) => { if (d[3] != null) fmax = Math.max(fmax, d[3]); });
      const yf = (v) => H - 14 - (v / fmax) * (H - 40);
      ctx.strokeStyle = "#f59e0b"; ctx.lineWidth = 1.2; ctx.beginPath();
      let started = false;
      seg.forEach((d) => {
        if (d[3] == null) return;
        const px = x(d[0]), py = yf(d[3]);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
  }
  function line(seg, idx, color, x, y) {
    ctx.strokeStyle = color; ctx.lineWidth = 1.4; ctx.beginPath();
    let started = false;
    seg.forEach((d) => {
      if (d[idx] == null) { started = false; return; }
      const px = x(d[0]), py = y(d[idx]);
      if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  // ---- loops -------------------------------------------------------------
  $("btn-refresh-runs").addEventListener("click", loadRuns);
  loadInfo();
  loadRuns();
  setInterval(loadInfo, 3000);   // refresh controller catalogue
  setInterval(poll, 100);        // 10 Hz live update
  setInterval(loadRuns, 8000);   // refresh saved-run history
})();
