"use strict";
// IK dashboard client. Talks only to the dashboard HTTP API, which relays to
// ik_node over its ROS topics. Never commands the robot.

const DOF = ["x", "y", "z", "rx", "ry", "rz"];
const TEMPLATES = {
  pose:          [1, 1, 1, 1, 1, 1],
  point:         [1, 1, 1, 0, 0, 0],
  position_yaw:  [1, 1, 1, 0, 0, 1],
  axis_gaze:     [1, 1, 1, 1, 1, 0],
};

function el(id) { return document.getElementById(id); }

function buildStiffness() {
  const grid = el("stiff");
  grid.innerHTML = "";
  DOF.forEach((d, i) => {
    const cell = document.createElement("div");
    cell.className = "cell";
    cell.innerHTML = `<span>${d}: <b id="sv${i}">1.0</b></span>
      <input type="range" id="s${i}" min="0" max="1" step="0.01" value="1">`;
    grid.appendChild(cell);
  });
  DOF.forEach((d, i) => {
    el(`s${i}`).addEventListener("input", () => {
      el(`sv${i}`).textContent = Number(el(`s${i}`).value).toFixed(2);
    });
  });
}

function applyTemplate(name) {
  const t = TEMPLATES[name] || TEMPLATES.pose;
  t.forEach((v, i) => { el(`s${i}`).value = v; el(`sv${i}`).textContent = v.toFixed(2); });
}

function readStiffness() { return DOF.map((_, i) => Number(el(`s${i}`).value)); }

async function refreshState() {
  try {
    const r = await fetch("/api/state");
    const s = await r.json();
    const conn = el("conn");
    const st = s.status;
    if (st && st.have_model) {
      conn.textContent = `connected — ${st.dof} DOF`;
      conn.className = "pill pill-ok";
      populateFrames(st);
      renderArmAngles(st.arm_angles_now || {});
    } else {
      conn.textContent = "waiting for ik_node / robot_description";
      conn.className = "pill pill-bad";
    }
    el("status").textContent = JSON.stringify(st, null, 2);
  } catch (e) {
    el("conn").textContent = "dashboard offline";
    el("conn").className = "pill pill-bad";
  }
}

let framesPopulated = false;
function populateFrames(st) {
  if (framesPopulated) return;
  const chains = st.srs_chains || [];
  const psiSel = el("psi_chain");
  chains.forEach(c => {
    const o = document.createElement("option"); o.value = c; o.textContent = c;
    psiSel.appendChild(o);
  });
  // group selector mirrors the chains (right_arm / left_arm) as active sets
  const grpSel = el("group");
  chains.forEach(c => {
    const o = document.createElement("option"); o.value = c; o.textContent = c;
    grpSel.appendChild(o);
  });
  framesPopulated = true;
}

function renderArmAngles(aa) {
  const lines = Object.entries(aa).map(([k, v]) => `${k}: ψ = ${v.toFixed(4)} rad`);
  el("armangles").textContent = lines.join("\n") || "(no S-R-S chain reported)";
}

const GROUP_JOINTS = {
  right_arm: Array.from({length: 7}, (_, i) => `right_arm_joint${i + 1}`),
  left_arm: Array.from({length: 7}, (_, i) => `left_arm_joint${i + 1}`),
};

async function solve() {
  const frame = el("frame").value || "right_arm_Link7";
  const task = {
    frame,
    xyz: [Number(el("px").value), Number(el("py").value), Number(el("pz").value)],
    quat: [Number(el("qw").value), Number(el("qx").value),
           Number(el("qy").value), Number(el("qz").value)],
    stiffness: readStiffness(),
  };
  const req = { id: "dash" + Date.now(), tasks: [task] };
  const grp = el("group").value;
  if (grp && GROUP_JOINTS[grp]) req.active_joints = GROUP_JOINTS[grp];
  const pc = el("psi_chain").value;
  if (pc) req.arm_angles = [{ chain: pc, psi: Number(el("psi_des").value),
                             stiffness: Number(el("psi_k").value) }];

  el("verdict").textContent = "solving…";
  el("verdict").className = "verdict";
  try {
    const r = await fetch("/api/solve", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(req),
    });
    const sol = await r.json();
    renderResult(sol);
  } catch (e) {
    el("verdict").textContent = "request failed: " + e;
    el("verdict").className = "verdict bad";
  }
}

function renderResult(sol) {
  const v = el("verdict");
  if (sol.ok && sol.reachable) {
    v.textContent = "REACHABLE ✓"; v.className = "verdict ok";
  } else {
    v.textContent = `NOT REACHABLE — ${sol.reason || sol.error || "?"}`;
    v.className = "verdict bad";
  }
  const lines = [];
  if (sol.max_pos_err != null) lines.push(`pos err: ${(sol.max_pos_err * 1000).toFixed(2)} mm`);
  if (sol.max_ori_err != null) lines.push(`ori err: ${(sol.max_ori_err).toFixed(4)} rad`);
  if (sol.iters != null) lines.push(`iters: ${sol.iters}`);
  if (sol.manipulability != null) lines.push(`manip: ${sol.manipulability.toFixed(4)}`);
  if (sol.sigma_min != null) lines.push(`σ_min: ${sol.sigma_min.toFixed(4)}`);
  if (sol.blocking_joints && sol.blocking_joints.length)
    lines.push(`blocking: ${sol.blocking_joints.join(", ")}`);
  if (sol.arm_angles) lines.push(`ψ: ${JSON.stringify(sol.arm_angles)}`);
  if (sol.q) lines.push(`q: [${sol.q.map(x => x.toFixed(3)).join(", ")}]`);
  el("result").textContent = lines.join("\n");
}

window.addEventListener("DOMContentLoaded", () => {
  buildStiffness();
  el("template").addEventListener("change", e => applyTemplate(e.target.value));
  el("solve").addEventListener("click", solve);
  el("capture").addEventListener("click", () => {
    // "capture" just hints the user; without live FK in the UI we leave fields.
    alert("Enter a target pose, or use the CLI to capture the live FK pose.");
  });
  refreshState();
  setInterval(refreshState, 1000);
});
