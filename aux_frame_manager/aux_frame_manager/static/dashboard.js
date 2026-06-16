"use strict";
// Test interface for the aux_frame_manager dashboard.
//
// This is the non-3D half of the page: it shows the manager's live ~/status,
// lists the aux frames currently in the canonical URDF, and drives the
// manager's ~/set_aux_frames (via POST /api/set_frames) to add / edit / remove
// frames at runtime. All edits operate on the *live* frame list reported by the
// node, so what you see is what the manager currently has.
//
// The 3D viewer hands us each snapshot through window.__onSnapshot, and tells
// us about clicked links through window.__onPickLink.

const $ = (id) => document.getElementById(id);

let liveFrames = [];     // [{name, parent, xyz, rpy}] from the latest snapshot
let links = [];          // all canonical link names (parent choices)
let lastParentFill = ""; // signature to avoid rebuilding the <select> each poll

function fmt3(a) {
  return "[" + (a || [0, 0, 0]).map((v) => Number(v).toFixed(3)).join(", ") + "]";
}

async function postFrames(frames) {
  const r = await fetch("/api/set_frames", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ frames }),
  });
  return r.json();
}

function setMsg(id, text, kind) {
  const el = $(id); if (!el) return;
  el.textContent = text || "";
  el.className = "msg sm " + (kind || "muted");
}

// ---- form helpers -------------------------------------------------------
function readForm() {
  const num = (id) => { const v = parseFloat($(id).value); return isNaN(v) ? 0 : v; };
  return {
    name: ($("f-name").value || "").trim(),
    parent: $("f-parent").value || "",
    xyz: [num("f-x"), num("f-y"), num("f-z")],
    rpy: [num("f-roll"), num("f-pitch"), num("f-yaw")],
  };
}
function loadForm(f) {
  $("f-name").value = f.name || "";
  $("f-parent").value = f.parent || "";
  $("f-x").value = (f.xyz && f.xyz[0]) || 0;
  $("f-y").value = (f.xyz && f.xyz[1]) || 0;
  $("f-z").value = (f.xyz && f.xyz[2]) || 0;
  $("f-roll").value = (f.rpy && f.rpy[0]) || 0;
  $("f-pitch").value = (f.rpy && f.rpy[1]) || 0;
  $("f-yaw").value = (f.rpy && f.rpy[2]) || 0;
}

// merge one frame into the live list (replace by name), return new list
function mergeFrame(frame) {
  const out = liveFrames.filter((f) => f.name !== frame.name);
  out.push(frame);
  return out;
}

// ---- actions ------------------------------------------------------------
async function onAdd() {
  const f = readForm();
  if (!f.name) { setMsg("edit-msg", "name is required", "err"); return; }
  if (!f.parent) { setMsg("edit-msg", "pick a parent link", "err"); return; }
  setMsg("edit-msg", "sending…", "muted");
  const res = await postFrames(mergeFrame(f));
  setMsg("edit-msg", res.message || (res.ok ? "sent" : "failed"),
         res.ok ? "ok" : "err");
}
async function onRemove(name) {
  setMsg("ops-msg", "removing " + name + "…", "muted");
  const res = await postFrames(liveFrames.filter((f) => f.name !== name));
  setMsg("ops-msg", res.message || (res.ok ? "removed" : "failed"),
         res.ok ? "ok" : "err");
}
async function onClear() {
  if (!liveFrames.length) { setMsg("ops-msg", "already empty", "muted"); return; }
  setMsg("ops-msg", "clearing all…", "muted");
  const res = await postFrames([]);
  setMsg("ops-msg", res.message || (res.ok ? "cleared" : "failed"),
         res.ok ? "ok" : "err");
}
async function onReapply() {
  setMsg("ops-msg", "re-applying…", "muted");
  const res = await postFrames(liveFrames);
  setMsg("ops-msg", res.message || (res.ok ? "re-applied" : "failed"),
         res.ok ? "ok" : "err");
}

// ---- rendering ----------------------------------------------------------
function renderStatus(s) {
  const box = $("status-box"); if (!box) return;
  const st = s.status;
  if (!st) { box.textContent = "no status yet"; box.className = "status-box muted sm"; return; }
  const age = s.status_age != null ? s.status_age + "s ago" : "";
  const msg = String(st.message || "");
  const isErr = /^error/i.test(msg);
  box.className = "status-box sm " + (isErr ? "err" : "ok");
  const rows = [];
  rows.push(["state", isErr ? "error" : "ok"]);
  if (msg) rows.push(["message", msg]);
  if (Array.isArray(st.aux_frames)) rows.push(["frames", st.aux_frames.length]);
  if (st.output_topic) rows.push(["output", st.output_topic]);
  if (st.have_canonical != null) rows.push(["published", st.have_canonical ? "yes" : "no"]);
  if (st.mirror_to_rsp != null) rows.push(["mirror→rsp", st.mirror_to_rsp ? "yes" : "no"]);
  box.innerHTML = rows.map(
    ([k, v]) => `<div class="trow"><span class="k">${k}</span>`
      + `<span class="v">${String(v)}</span></div>`).join("")
    + (age ? `<div class="trow"><span class="k">updated</span>`
      + `<span class="v">${age}</span></div>` : "");
}

function renderList() {
  const host = $("edit-list"); if (!host) return;
  if (!liveFrames.length) {
    host.className = "edit-list muted sm";
    host.textContent = "no aux frames";
    return;
  }
  host.className = "edit-list";
  host.innerHTML = liveFrames.map((f) => {
    const safe = f.name.replace(/"/g, "&quot;");
    return `<div class="erow" data-name="${safe}">
      <div class="erow-head">
        <span class="dot"></span><b class="ename">${f.name}</b>
        <span class="muted">← ${f.parent}</span>
      </div>
      <div class="erow-num muted sm">xyz ${fmt3(f.xyz)} · rpy ${fmt3(f.rpy)}</div>
      <div class="erow-btns">
        <button class="mini" data-act="load">load</button>
        <button class="mini mini-warn" data-act="remove">remove</button>
      </div></div>`;
  }).join("");
  host.querySelectorAll(".erow").forEach((row) => {
    const name = row.getAttribute("data-name");
    const f = liveFrames.find((x) => x.name === name);
    row.querySelector('[data-act="load"]').onclick = () => {
      loadForm(f); if (window.__viewerSelect) window.__viewerSelect(name);
    };
    row.querySelector('[data-act="remove"]').onclick = () => onRemove(name);
  });
}

function fillParents(linkNames) {
  const sig = linkNames.join("|");
  if (sig === lastParentFill) return;
  lastParentFill = sig;
  const sel = $("f-parent"); if (!sel) return;
  const cur = sel.value;
  sel.innerHTML = '<option value="">— pick a link —</option>'
    + linkNames.map((l) => `<option value="${l}">${l}</option>`).join("");
  if (linkNames.includes(cur)) sel.value = cur;
}

// ---- snapshot hook (called by viewer.js) --------------------------------
window.__onSnapshot = (s) => {
  if (!s) return;
  liveFrames = (s.aux_frames || []).map((f) => ({
    name: f.name, parent: f.parent,
    xyz: f.xyz || [0, 0, 0], rpy: f.rpy || [0, 0, 0],
  }));
  // parent choices: base links only (aux frames can chain off any link, but
  // base links are the common case; include aux links too for chaining)
  links = s.links || [];
  fillParents(links);
  renderStatus(s);
  renderList();
  const sub = $("sub-line");
  if (sub) sub.textContent = (s.manager_ns || "?") + "  →  "
    + (s.canonical_topic || "?") + "   ·   base " + (s.base_frame || "?");
};

// when a link is clicked in 3D: aux → load it for editing; base → set parent
window.__onPickLink = (name, isAux, def) => {
  if (isAux && def) { loadForm(def); }
  else if (name) { const sel = $("f-parent"); if (sel) sel.value = name; }
};

// ---- wire buttons -------------------------------------------------------
$("btn-add").onclick = onAdd;
$("btn-load-sel").onclick = () => {
  const sel = $("sel-link") ? $("sel-link").textContent : "";
  const f = liveFrames.find((x) => x.name === sel);
  if (f) { loadForm(f); setMsg("edit-msg", "loaded " + sel, "muted"); }
  else setMsg("edit-msg", "select an aux frame in the 3D view first", "muted");
};
$("btn-apply").onclick = onReapply;
$("btn-clear").onclick = onClear;
