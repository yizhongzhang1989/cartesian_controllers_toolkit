"use strict";
// Control-panel logic for robot_control_test. Polls /api/state and renders the
// controller list plus the joint / Cartesian / wrench command cards for the
// engaged controller. The 3D canvas is owned by viewer.js (separate poll).

const $ = (id) => document.getElementById(id);

async function postJSON(url, body) {
  try {
    const r = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return await r.json();
  } catch (e) {
    return { ok: false, message: String(e) };
  }
}

function setMsg(text, ok) {
  const el = $("action-msg");
  el.textContent = text || "";
  el.className = "msg" + (ok === true ? " ok" : ok === false ? " err" : "");
}

function mkbtn(text, cls) {
  const b = document.createElement("button");
  b.type = "button";
  b.className = cls;
  b.textContent = text;
  return b;
}

// ---- unit helpers ---------------------------------------------------------
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const isAngular = (m) => !m || m.type === "revolute" || m.type === "continuous";
function fmtJ(v, m) {
  if (isAngular(m)) return (v * R2D).toFixed(1) + "°";
  return (v * 1000).toFixed(1) + "mm";
}
// human jog step (deg for angular, mm for prismatic) -> command units (rad / m)
function stepToCmd(m) {
  const s = parseFloat($("joint-step").value) || 0;
  return isAngular(m) ? s * D2R : s / 1000;
}

// ---- module state ---------------------------------------------------------
let _editing = false;          // user actively dragging a joint slider
let _lastCtrlSig = "";
let _lastJointSig = "";
let _lastCartName = "";
let _curSnap = null;
const jointRowEls = {};        // joint name -> row element
const movableByName = {};      // joint name -> {name,type,lower,upper}

// ---- poll + dispatch ------------------------------------------------------
async function poll() {
  let s;
  try {
    s = await (await fetch("/api/state")).json();
  } catch (e) {
    $("conn").textContent = "disconnected";
    $("conn").className = "pill pill-bad";
    return;
  }
  _curSnap = s;
  (s.movable_joints || []).forEach((m) => { movableByName[m.name] = m; });
  renderConn(s);
  renderControllers(s);
  renderJoint(s);
  renderCart(s);
  renderWrench(s);
}

function renderConn(s) {
  const ok = !!s.have_model;
  const good = s.cm_ok && ok;
  $("conn").textContent = !s.cm_ok ? "no controller_manager"
    : ok ? "connected" : "no robot model";
  $("conn").className = "pill " + (good ? "pill-good" : "pill-bad");
  if (s.tcp) {
    const p = s.tcp.xyz.map((v) => v.toFixed(3)).join(", ");
    const r = s.tcp.rpy.map((v) => (v * R2D).toFixed(1)).join(", ");
    $("tcp-read").textContent = `TCP ${s.tcp.tip}: [${p}] m · rpy [${r}]°`;
  } else {
    $("tcp-read").textContent = "no TCP (set tip_frame / TF)";
  }
  $("cm-status").textContent = s.cm_ok
    ? `${(s.controllers || []).length} on ${s.controller_manager}`
    : "controller_manager not found";
}

// ---- controllers ----------------------------------------------------------
const KIND_LABEL = {
  joint_trajectory: "joint trajectory",
  forward_position: "forward position",
  cartesian_motion: "cartesian motion",
  cartesian_compliance: "cartesian compliance",
  cartesian_force: "cartesian force",
  other: "other",
};

function renderControllers(s) {
  const eng = s.engaged || {};
  const engaged = new Set(
    [eng.joint, eng.cartesian, eng.wrench].filter(Boolean));
  const ctrls = s.controllers || [];
  const sig = ctrls.map((c) => c.name + ":" + c.state).join("|")
    + "#" + [...engaged].sort().join(",");
  if (sig === _lastCtrlSig) return;
  _lastCtrlSig = sig;

  const host = $("ctrl-list");
  host.innerHTML = "";
  if (!ctrls.length) {
    host.innerHTML = '<div class="muted sm">no controllers found — is the '
      + 'robot up?</div>';
    return;
  }
  for (const c of ctrls) {
    const isEng = engaged.has(c.name);
    const drivable = c.kind && c.kind !== "other";
    const row = document.createElement("div");
    row.className = "crow" + (c.state === "active" ? " is-active" : "")
      + (isEng ? " is-engaged" : "");
    row.innerHTML =
      `<div class="crow-head"><span class="cname">${c.name}</span>`
      + `<span class="cstate ${c.state === "active" ? "active" : "inactive"}">`
      + `${c.state}</span></div>`
      + `<div class="crow-meta">`
      + `<span class="kind ${drivable ? "" : "k-other"}">`
      + `${KIND_LABEL[c.kind] || c.kind}</span>`
      + `<span>${(c.joints || []).length} joints</span>`
      + `<span class="crow-btns"></span></div>`;
    const btns = row.querySelector(".crow-btns");
    if (isEng) {
      const b = mkbtn("Disengage", "btn sm btn-stop");
      b.onclick = () => doDisengage(c.name);
      btns.appendChild(b);
    } else if (drivable) {
      const b = mkbtn("Engage", "btn sm btn-go");
      b.onclick = () => doEngage(c.name);
      btns.appendChild(b);
    }
    host.appendChild(row);
  }
}

async function doEngage(name) {
  setMsg("engaging " + name + "…");
  const out = await postJSON("/api/engage", { name });
  setMsg(out.message || (out.ok ? "engaged" : "failed"), out.ok);
  resetSigs();
  poll();
}
async function doDisengage(name) {
  const out = await postJSON("/api/disengage", { name });
  setMsg(out.message || "disengaged", out.ok);
  resetSigs();
  poll();
}
function resetSigs() {
  _lastCtrlSig = ""; _lastJointSig = ""; _lastCartName = "";
}

// ---- joint control --------------------------------------------------------
function renderJoint(s) {
  const eng = s.engaged || {};
  const name = eng.joint;
  const card = $("card-joint");
  if (!name) { card.hidden = true; _lastJointSig = ""; return; }
  card.hidden = false;
  const isJtc = eng.joint_kind === "joint_trajectory";
  $("joint-ctrl-name").textContent = name + " · " + (eng.joint_kind || "");
  $("btn-joint-move").hidden = !isJtc;
  $("joint-mode-hint").textContent = isJtc
    ? "JointTrajectoryController: drag a slider then press Move, or ± to nudge "
    + "(each is a timed, speed-limited trajectory)."
    : "Forward-position controller: sliders and ± drive the robot live "
    + "(velocity-ramped to max_joint_speed).";

  const jnames = s.joint_names || [];
  const sig = jnames.join(",");
  if (sig !== _lastJointSig) {
    _lastJointSig = sig;
    buildJointRows(jnames);
  }
  const meas = s.joint_values || {};
  const tgt = s.joint_targets || {};
  for (const j of jnames) {
    const row = jointRowEls[j];
    if (!row) continue;
    const m = movableByName[j];
    const mv = row.querySelector(".jmeas");
    if (mv && j in meas) mv.textContent = fmtJ(meas[j], m);
    if (!_editing && j in tgt) {
      row.querySelector(".jslider").value = tgt[j];
      row.querySelector(".jval").textContent = fmtJ(tgt[j], m);
    }
  }
}

function buildJointRows(jnames) {
  const host = $("joint-rows");
  host.innerHTML = "";
  for (const k in jointRowEls) delete jointRowEls[k];
  for (const j of jnames) {
    const m = movableByName[j] || { type: "revolute" };
    const ang = isAngular(m);
    let lo = m.lower, hi = m.upper;
    if (lo === null || lo === undefined) lo = ang ? -Math.PI : -0.5;
    if (hi === null || hi === undefined) hi = ang ? Math.PI : 0.5;
    const row = document.createElement("div");
    row.className = "jrow";
    row.dataset.joint = j;
    row.innerHTML =
      `<span class="jname" title="${j}">${j}</span>`
      + `<input type="range" class="jslider" min="${lo}" max="${hi}" `
      + `step="0.001" value="0" />`
      + `<span class="jval">0°</span>`
      + `<button class="jnudge" data-d="-1">−</button>`
      + `<button class="jnudge" data-d="1">+</button>`;
    // measured readout sits between value and nudge buttons
    const jmeas = document.createElement("span");
    jmeas.className = "jmeas";
    jmeas.textContent = "—";
    row.insertBefore(jmeas, row.querySelector(".jnudge"));
    const sl = row.querySelector(".jslider");
    sl.addEventListener("input", () => onJointInput(j, m));
    sl.addEventListener("change", () => onJointChange(j, m));
    row.querySelectorAll(".jnudge").forEach((b) => {
      b.onclick = () => jointNudge(j, m, parseInt(b.dataset.d, 10));
    });
    host.appendChild(row);
    jointRowEls[j] = row;
  }
}

function jointTargetsFromUI() {
  const out = {};
  for (const j in jointRowEls) {
    const sl = jointRowEls[j].querySelector(".jslider");
    if (sl) out[j] = parseFloat(sl.value);
  }
  return out;
}
async function sendJointSet() {
  const out = await postJSON("/api/joint/set",
    { positions: jointTargetsFromUI() });
  if (!out.ok) setMsg(out.message || "joint set failed", false);
}

let _jtTimer = null, _jtLast = 0;
function onJointInput(j, m) {
  _editing = true;
  const row = jointRowEls[j];
  row.querySelector(".jval").textContent =
    fmtJ(parseFloat(row.querySelector(".jslider").value), m);
  const k = _curSnap && _curSnap.engaged && _curSnap.engaged.joint_kind;
  if (k === "forward_position") {                 // stream live, throttled
    const now = performance.now();
    if (now - _jtLast >= 50) { _jtLast = now; sendJointSet(); }
    else {
      clearTimeout(_jtTimer);
      _jtTimer = setTimeout(() => {
        _jtLast = performance.now(); sendJointSet();
      }, 50);
    }
  }
}
function onJointChange() {
  _editing = false;
  sendJointSet();             // FPC: final target · JTC: triggers a move
}
async function jointNudge(j, m, sign) {
  const delta = sign * stepToCmd(m);
  const out = await postJSON("/api/joint/jog", { joint: j, delta });
  if (!out.ok) setMsg(out.message || "jog failed", false);
}

// ---- Cartesian control ----------------------------------------------------
function renderCart(s) {
  const name = (s.engaged || {}).cartesian;
  const card = $("card-cart");
  if (!name) { card.hidden = true; _lastCartName = ""; return; }
  card.hidden = false;
  $("cart-ctrl-name").textContent = name;
  if (name !== _lastCartName) { _lastCartName = name; buildCartJog(); }
  const ct = s.cart_target;
  if (ct) {
    const p = ct.xyz.map((v) => v.toFixed(3)).join(", ");
    const r = ct.rpy.map((v) => (v * R2D).toFixed(1)).join(", ");
    $("cart-read").textContent = `target [${p}] m · rpy [${r}]°`;
  } else {
    $("cart-read").textContent = "target — press “Reset to TCP” to seed";
  }
}

function buildCartJog() {
  const host = $("cart-jog");
  host.innerHTML = "";
  const axes = [["X", "x", false], ["Y", "y", false], ["Z", "z", false],
                ["RX", "rx", true], ["RY", "ry", true], ["RZ", "rz", true]];
  for (const [label, axis, rot] of axes) {
    const wrap = document.createElement("div");
    wrap.className = "jog-axis";
    const tag = document.createElement("span");
    tag.className = "lbl-ax"; tag.textContent = label;
    const minus = mkbtn("−", "btn sm"), plus = mkbtn("+", "btn sm");
    minus.onclick = () => cartJog(axis, rot, -1);
    plus.onclick = () => cartJog(axis, rot, 1);
    wrap.appendChild(tag); wrap.appendChild(minus); wrap.appendChild(plus);
    host.appendChild(wrap);
  }
}
async function cartJog(axis, rot, sign) {
  const delta = rot
    ? sign * (parseFloat($("cart-step-deg").value) || 0) * D2R
    : sign * (parseFloat($("cart-step-mm").value) || 0) / 1000;
  const out = await postJSON("/api/cart/jog", { axis, delta });
  if (!out.ok) setMsg(out.message || "Cartesian jog failed", false);
}

// ---- wrench control -------------------------------------------------------
function renderWrench(s) {
  const name = (s.engaged || {}).wrench;
  const card = $("card-wrench");
  if (!name) { card.hidden = true; return; }
  card.hidden = false;
  $("wrench-ctrl-name").textContent = name;
}
function wrenchFromUI() {
  const n = (id) => parseFloat($(id).value) || 0;
  return {
    force: [n("w-fx"), n("w-fy"), n("w-fz")],
    torque: [n("w-tx"), n("w-ty"), n("w-tz")],
  };
}

// ---- static button wiring (elements always present) -----------------------
$("btn-joint-sync").onclick = async () => {
  const out = await postJSON("/api/joint/sync", {});
  setMsg(out.message || "synced", out.ok);
};
$("btn-joint-stop").onclick = async () => {
  const out = await postJSON("/api/joint/stop", {});
  setMsg(out.message || "stopped", out.ok);
};
$("btn-joint-move").onclick = () => sendJointSet();
$("btn-cart-reset").onclick = async () => {
  const out = await postJSON("/api/cart/reset", {});
  setMsg(out.message || "reset", out.ok);
};
$("btn-wrench-send").onclick = async () => {
  const out = await postJSON("/api/wrench/set", wrenchFromUI());
  setMsg(out.ok ? "wrench sent" : (out.message || "failed"), out.ok);
};
$("btn-wrench-zero").onclick = async () => {
  ["w-fx", "w-fy", "w-fz", "w-tx", "w-ty", "w-tz"].forEach((id) => {
    $(id).value = 0;
  });
  const out = await postJSON("/api/wrench/zero", {});
  setMsg(out.message || "zeroed", out.ok);
};

// kick off
poll();
setInterval(poll, 300);
