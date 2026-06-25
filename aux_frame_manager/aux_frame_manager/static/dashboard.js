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

// liveFrames now holds EVERY editable (fixed-joint) frame, each tagged
// source = "added" (this manager appended it) | "base" (already in the launch
// URDF). Added frames are managed via ~/set_aux_frames (the whole-list replace);
// any single frame's offset -- added OR base -- is edited via ~/edit_frame.
let liveFrames = [];     // [{name, parent, xyz, rpy, source}] from the snapshot
let links = [];          // all canonical link names (parent choices)
let lastParentFill = ""; // signature to avoid rebuilding the <select> each poll
let lastListSig = "";    // same idea for the editable-frames list (stable buttons)

function fmt3(a) {
  return "[" + (a || [0, 0, 0]).map((v) => Number(v).toFixed(3)).join(", ") + "]";
}

// HTML-escape dynamic text (frame names / error reasons) before inserting it
// into innerHTML, so a stray '<' in a URDF name or message can't break markup.
function esc(s) {
  return String(s == null ? "" : s)
    .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

// the managed (added) subset, stripped to the manager's frame schema -- the
// only frames ~/set_aux_frames may carry (sending a base frame here would make
// the manager augment a duplicate, so remove/clear/re-apply use this).
function addedOnly(keep) {
  return liveFrames
    .filter((f) => f.source === "added" && (!keep || keep(f)))
    .map((f) => ({ name: f.name, parent: f.parent, xyz: f.xyz, rpy: f.rpy }));
}

async function postFrames(frames) {
  const r = await fetch("/api/set_frames", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ frames }),
  });
  return r.json();
}

// edit ONE frame's offset (added or pre-existing); manager routes by name
async function postEdit(frame) {
  const r = await fetch("/api/edit_frame", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ frame }),
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

// ---- actions ------------------------------------------------------------
async function onAdd() {
  const f = readForm();
  if (!f.name) { setMsg("edit-msg", "name is required", "err"); return; }
  // an existing frame (added or pre-existing) is edited by offset only; a brand
  // new frame needs a parent to hang off.
  const existing = liveFrames.find((x) => x.name === f.name);
  if (!existing && !f.parent) {
    setMsg("edit-msg", "pick a parent link for a new frame", "err"); return;
  }
  setMsg("edit-msg", "sending…", "muted");
  const res = await postEdit(f);   // manager routes: managed / override / add
  setMsg("edit-msg", res.message || (res.ok ? "sent" : "failed"),
         res.ok ? "ok" : "err");
}
async function onRemove(name) {
  setMsg("ops-msg", "removing " + name + "…", "muted");
  // only added frames can be removed (base frames belong to the launch URDF)
  const res = await postFrames(addedOnly((f) => f.name !== name));
  setMsg("ops-msg", res.message || (res.ok ? "removed" : "failed"),
         res.ok ? "ok" : "err");
}
async function onClear() {
  if (!addedOnly().length) {
    setMsg("ops-msg", "no added frames to clear", "muted"); return;
  }
  setMsg("ops-msg", "clearing added frames…", "muted");
  const res = await postFrames([]);   // base frames stay; only added are cleared
  setMsg("ops-msg", res.message || (res.ok ? "cleared" : "failed"),
         res.ok ? "ok" : "err");
}
async function onReapply() {
  setMsg("ops-msg", "re-applying…", "muted");
  const res = await postFrames(addedOnly());
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
  const isPartial = /^partial/i.test(msg) || st.n_invalid > 0;
  box.className = "status-box sm " + (isErr ? "err" : isPartial ? "warn" : "ok");
  const rows = [];
  rows.push(["state", isErr ? "error" : isPartial ? "partial" : "ok"]);
  if (msg) rows.push(["message", msg]);
  if (Array.isArray(st.aux_frames)) rows.push(["valid frames", st.aux_frames.length]);
  if (st.n_invalid) rows.push(["invalid frames", st.n_invalid]);
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
    host.textContent = "no editable frames";
    lastListSig = "";
    return;
  }
  // The viewer polls /api/state every 200 ms and re-invokes this. Rebuilding the
  // list innerHTML each time destroyed the row buttons mid-click (so "load" took
  // several clicks to land). Only rebuild when the frames actually changed;
  // otherwise keep the existing DOM + click handlers stable.
  const sig = liveFrames.map((f) => [f.name, f.parent, (f.xyz || []).join(","),
    (f.rpy || []).join(","), f.source, f.valid, f.error].join("|")).join(";;");
  if (sig === lastListSig) return;
  lastListSig = sig;
  host.className = "edit-list";
  host.innerHTML = liveFrames.map((f) => {
    const safe = f.name.replace(/"/g, "&quot;");
    const base = f.source === "base";
    const invalid = f.valid === false;
    const cls = invalid ? "erow-invalid" : (base ? "erow-base" : "");
    const badge = invalid ? '<span class="badge badge-invalid">invalid</span>'
      : base ? '<span class="badge badge-base">pre-existing</span>'
             : '<span class="badge badge-added">added</span>';
    // invalid frames are managed (added) -> keep them removable/editable so the
    // operator can fix the parent or drop them; only base frames hide remove.
    const rm = base ? ""
      : '<button class="mini mini-warn" data-act="remove">remove</button>';
    const errLine = invalid
      ? `<div class="erow-err sm">⚠ ${esc(f.error || "invalid frame")}</div>` : "";
    return `<div class="erow ${cls}" data-name="${safe}">
      <div class="erow-head">
        <span class="dot"></span><b class="ename">${esc(f.name)}</b>
        <span class="muted">← ${esc(f.parent)}</span>${badge}
      </div>
      ${errLine}
      <div class="erow-num muted sm">xyz ${fmt3(f.xyz)} · rpy ${fmt3(f.rpy)}</div>
      <div class="erow-btns">
        <button class="mini" data-act="load">load</button>${rm}
      </div></div>`;
  }).join("");
  host.querySelectorAll(".erow").forEach((row) => {
    const name = row.getAttribute("data-name");
    const f = liveFrames.find((x) => x.name === name);
    row.querySelector('[data-act="load"]').onclick = () => {
      loadForm(f); if (window.__viewerSelect) window.__viewerSelect(name);
    };
    const rmb = row.querySelector('[data-act="remove"]');
    if (rmb) rmb.onclick = () => onRemove(name);
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
  const src = s.editable_frames || s.aux_frames || [];
  liveFrames = src.map((f) => ({
    name: f.name, parent: f.parent,
    xyz: f.xyz || [0, 0, 0], rpy: f.rpy || [0, 0, 0],
    source: f.source || "added",
    valid: f.valid !== false,     // invalid configured frames carry valid:false
    error: f.error || "",         // + the reason they couldn't be placed
  }));
  // parent choices: every canonical link (a frame may hang off any link)
  links = s.links || [];
  fillParents(links);
  renderStatus(s);
  renderList();
  const sub = $("sub-line");
  if (sub) sub.textContent = (s.manager_ns || "?") + "  →  "
    + (s.canonical_topic || "?") + "   ·   base " + (s.base_frame || "?");
};

// when a link is clicked in 3D: an editable frame loads for editing; any other
// link is offered as the parent for a new frame.
window.__onPickLink = (name, isEditable, def) => {
  const f = liveFrames.find((x) => x.name === name) || def;
  if (isEditable && f) { loadForm(f); }
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
