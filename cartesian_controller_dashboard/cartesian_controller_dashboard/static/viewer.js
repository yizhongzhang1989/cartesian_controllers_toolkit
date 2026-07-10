"use strict";
// 3D viewer for the cartesian_controller_dashboard. It renders the robot at its
// live pose (meshes + skeleton) plus a triad at the TCP / tip frame, so you can
// watch the arm move while you drive the FZI cartesian controllers.
//
// Forward kinematics is done server-side from TF: the node looks up
// base_frame -> link and sends the per-link 4x4 transforms in /api/viewer_state;
// the viewer only draws them -- no Pinocchio, no client-side FK. Meshes come
// from the URDF visuals (STL + COLLADA), fetched through the node's /mesh proxy.
//
// View options (checkboxes): meshes / labels / per-link frames. Click a link
// (its mesh or its label) to select it -- the selection is highlighted, a thick
// RGB triad is drawn at that link and its parent is shown; click empty space or
// press Esc to deselect. (The aux-frame marker code inherited from the shared
// viewer stays inert here, since this dashboard sends no aux frames.)
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";

const $ = (id) => document.getElementById(id);

const AUX_COLOR = 0xffb454;     // added aux frames (orange)
const AUX_COLOR_CSS = "#ffb454";
const BASE_COLOR = 0x34c3ff;    // pre-existing (launch-URDF) editable frames (cyan)

// Per-joint colours for the live joint-angle bars (J1=blue, J2=green, J3=orange,
// J4=red, J5=purple, J6=cyan; extra axes cycle) -- matches the other dashboards.
const JOINT_COLORS = ["#42a5f5", "#66bb6a", "#ffa726",
                      "#ef5350", "#ab47bc", "#26c6da"];
const RAD2DEG = 180 / Math.PI;

// ---- scene --------------------------------------------------------------
const canvas = $("viewer");
const labelsEl = $("labels");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1419);
const vw = () => canvas.clientWidth || (innerWidth - 380);
const vh = () => canvas.clientHeight || innerHeight;
const camera = new THREE.PerspectiveCamera(50, vw() / vh(), 0.01, 100);
camera.up.set(0, 0, 1);
camera.position.set(1.4, -1.4, 1.1);
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setSize(vw(), vh(), false);
renderer.setPixelRatio(devicePixelRatio);
renderer.outputColorSpace = THREE.SRGBColorSpace;
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.dampingFactor = 0.1;
controls.target.set(0, 0, 0.4); controls.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
scene.add(new THREE.HemisphereLight(0xb0d4f1, 0x404040, 0.85));
const d1 = new THREE.DirectionalLight(0xffffff, 1.2); d1.position.set(3, 5, 4); scene.add(d1);
const d2 = new THREE.DirectionalLight(0xffffff, 0.4); d2.position.set(-2, 3, -1); scene.add(d2);
const grid = new THREE.GridHelper(3, 30, 0x445, 0x334); grid.rotation.x = Math.PI / 2; scene.add(grid);
scene.add(new THREE.AxesHelper(0.25));   // base-frame triad at the origin
const tcpAxes = new THREE.AxesHelper(0.16);   // live TCP / tip-frame triad
tcpAxes.matrixAutoUpdate = false; tcpAxes.visible = false; scene.add(tcpAxes);

const solidMat = new THREE.MeshStandardMaterial({ color: 0x9fb4c4, metalness: 0.25, roughness: 0.6 });
const highlightMat = new THREE.MeshStandardMaterial({ color: AUX_COLOR, emissive: 0x6e3d00,
  emissiveIntensity: 0.6, metalness: 0.2, roughness: 0.5 });
const stlLoader = new STLLoader();
const colladaLoader = new ColladaLoader();
// url -> {kind:"stl"|"dae", obj, ready, waiting:[cb]}; obj = BufferGeometry (STL)
// or the loaded COLLADA scene Group (DAE). Loaded once per url, cloned per use.
const protoCache = {};
const meshItems = {};   // key(link#i) -> {link, local, solid, kind}
const frameAxes = {};   // link -> AxesHelper
const auxMarkers = {};  // aux link -> {group, line}
const labelPool = [];   // reusable label divs (clustered, not per-link)
let allLinks = [];      // link names from the snapshot
let auxSet = new Set(); // ADDED aux frames (the aux-only filter + orange marker)
let editDefs = {};      // EVERY editable frame -> {name,parent,xyz,rpy,source}
let meshLinks = new Set(); // links that already have a mesh (no marker needed)
let jointTree = [];     // [{parent, child, type}] for the skeleton lines
let didFit = false;

// ---- view options (checkboxes) ------------------------------------------
const opt = { mesh: true, labels: true, frames: false, auxOnly: false };
function readOpts() {
  if ($("show-mesh")) opt.mesh = $("show-mesh").checked;
  if ($("show-labels")) opt.labels = $("show-labels").checked;
  if ($("show-frames")) opt.frames = $("show-frames").checked;
  if ($("show-aux-only")) opt.auxOnly = $("show-aux-only").checked;
}
["show-mesh", "show-labels", "show-frames", "show-aux-only"].forEach((id) => {
  const el = $(id);
  if (el) el.addEventListener("change", () => { readOpts(); refreshStatic(); });
});

// Auto-detect whether the URDF actually carries mesh visuals. When it has none
// (e.g. a robot described with primitive boxes/cylinders only), force skeleton
// view: clear + disable the "mesh" checkbox so the toggle isn't a misleading
// no-op, grey its label, and note why. Re-enable it if a model WITH meshes
// later appears. Acts only on a state change, so it never fights a manual
// toggle while meshes are available.
let meshAvail = null, meshUnsup = null;
function applyMeshAvailability(has, unsupported) {
  if (has === meshAvail && unsupported === meshUnsup) return;
  meshAvail = has; meshUnsup = unsupported;
  const cb = $("show-mesh");
  if (!cb) return;
  cb.disabled = !has;
  cb.checked = has;
  opt.mesh = has;
  cb.title = has ? ""
    : unsupported
      ? "this URDF's meshes aren't a supported format (STL/COLLADA) \u2014 "
        + "the skeleton is shown"
      : "this URDF has no mesh visuals \u2014 skeleton view only";
  const lab = cb.closest("label");
  if (lab) lab.classList.toggle("opt-disabled", !has);
  const lbl = $("mesh-lbl");
  if (lbl) lbl.textContent = has ? "mesh"
    : unsupported ? "mesh (unsupported)" : "mesh (none)";
}

// ---- selection ----------------------------------------------------------
let selectedLink = "";
function parentOf(link) {
  // aux/editable frames carry their own parent; any other link's parent comes
  // from the joint tree (child -> parent). "" for the root link / unknown.
  if (editDefs[link] && editDefs[link].parent) return editDefs[link].parent;
  const j = jointTree.find((x) => x.child === link);
  return j ? j.parent : "";
}
function setSelected(link, notify) {
  selectedLink = link || "";
  if ($("sel-link")) $("sel-link").textContent = selectedLink || "—";
  if ($("sel-parent")) {
    $("sel-parent").textContent = (selectedLink && parentOf(selectedLink)) || "—";
  }
  // instant visual feedback (mesh highlight + thick selected frame) without
  // waiting for the next poll cycle
  const _tf = window.__lastLinkTf;
  if (_tf) { placeCurrent(_tf); placeSelFrame(_tf); }
  invalidate();
  if (notify && typeof window.__onPickLink === "function") {
    window.__onPickLink(selectedLink, !!editDefs[selectedLink],
                        editDefs[selectedLink] || null);
  }
}
window.__viewerSelect = (link) => setSelected(link, false);

// Mesh format is taken from the file extension. Direct URLs end in the ext
// (.stl); the server's /mesh proxy carries it in a `path=` query param
// (e.g. /mesh?pkg=ur_description&path=meshes/ur15/visual/base.dae).
function meshExt(url) {
  const m = /[?&]path=([^&]+)/.exec(url);
  const p = m ? decodeURIComponent(m[1]) : url.split("?")[0];
  const dot = p.lastIndexOf(".");
  return dot >= 0 ? p.slice(dot + 1).toLowerCase() : "";
}
// Load a mesh url once (STL -> BufferGeometry, COLLADA -> scene Group) and
// cache it; callers clone/instantiate per link. cb receives the cache entry.
function loadProto(url, cb) {
  const c = protoCache[url];
  if (c && c.ready) { cb(c); return; }
  if (c) { c.waiting.push(cb); return; }
  const entry = protoCache[url] = { kind: meshExt(url), obj: null, ready: false, waiting: [cb] };
  const done = (obj) => {
    entry.obj = obj; entry.ready = true;
    entry.waiting.forEach((f) => f(entry)); entry.waiting = [];
  };
  const fail = () => { entry.waiting = []; };   // load error: skeleton still shows
  if (entry.kind === "dae") {
    colladaLoader.load(url, (collada) => done(collada.scene), undefined, fail);
  } else {
    stlLoader.load(url, (g) => { g.computeVertexNormals(); done(g); }, undefined, fail);
  }
}
function localMatrix(xyz, rpy, scale) {
  const m = new THREE.Matrix4();
  m.makeRotationFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
  m.setPosition(xyz[0], xyz[1], xyz[2]);
  if (scale) m.scale(new THREE.Vector3(scale[0], scale[1], scale[2]));
  return m;
}
function rosMat(a) {
  return new THREE.Matrix4().set(
    a[0][0], a[0][1], a[0][2], a[0][3], a[1][0], a[1][1], a[1][2], a[1][3],
    a[2][0], a[2][1], a[2][2], a[2][3], a[3][0], a[3][1], a[3][2], a[3][3]);
}

// ---- pose smoothing -----------------------------------------------------
// The robot pose (link_tf) arrives from the server at a modest poll rate;
// applying it directly makes motion look stepped. Instead each poll sets a
// TARGET pose and the render loop eases the DISPLAYED pose toward it every
// frame (position lerp + quaternion slerp), so motion is smooth at the full
// frame rate no matter how often we poll. Each link's world transform is eased
// independently -- visually seamless for the small deltas between polls, and it
// needs no client-side FK (the server already did the FK via TF).
const SMOOTH_TAU = 0.06;               // easing time constant (s); smaller = snappier
const _ONE = new THREE.Vector3(1, 1, 1);
const _dm = new THREE.Matrix4();
const _cm = new THREE.Matrix4();
const _dp = new THREE.Vector3(), _dq = new THREE.Quaternion(), _dsc = new THREE.Vector3();
let targetPose = {};                   // link -> {pos:Vector3, quat:Quaternion}
const dispPose = {};                   // link -> {pos:Vector3, quat:Quaternion} (eased)
const dispTf = {};                     // link -> row-major 4x4 array (fed to the place* fns)
window.__lastLinkTf = dispTf;          // everything reads the live, smoothed pose

// On-demand rendering: the loop only does real work (place geometry + draw)
// when something changed -- a link is still easing, the camera is moving, or
// invalidate() was called (selection, toggle, resize, a mesh/texture finished
// loading). When the robot is still and nobody interacts, an idle canvas costs
// almost nothing, so many dashboards can share a screen.
let needsRender = true;
function invalidate() { needsRender = true; }
// a late-arriving mesh texture (e.g. the UR's diffuse map) must trigger a redraw
THREE.DefaultLoadingManager.onLoad = invalidate;

const POS_EPS = 1e-4;                  // ~0.1 mm: a link nearer than this is "settled"
const ANG_EPS = 5e-4;                  // ~0.03 deg
const _hm = new THREE.Matrix4();       // scratch world matrix (avoids per-frame alloc)
// Fill an existing Matrix4 from a row-major link_tf array (no allocation).
function rosMatInto(m, a) {
  return m.set(
    a[0][0], a[0][1], a[0][2], a[0][3], a[1][0], a[1][1], a[1][2], a[1][3],
    a[2][0], a[2][1], a[2][2], a[2][3], a[3][0], a[3][1], a[3][2], a[3][3]);
}
// Recompose an eased pose into a link's reusable row-major dispTf array.
function writeDispArray(link, pos, quat) {
  _cm.compose(pos, quat, _ONE);
  const e = _cm.elements;              // column-major -> row-major array
  let a = dispTf[link];
  if (!a) a = dispTf[link] = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]];
  a[0][0] = e[0]; a[0][1] = e[4]; a[0][2] = e[8];  a[0][3] = e[12];
  a[1][0] = e[1]; a[1][1] = e[5]; a[1][2] = e[9];  a[1][3] = e[13];
  a[2][0] = e[2]; a[2][1] = e[6]; a[2][2] = e[10]; a[2][3] = e[14];
}

// Latest server pose becomes the interpolation target. A link seen for the
// first time (or after a model change) SNAPS so it doesn't ease in from the
// origin; links that vanished are dropped.
function setTargetPose(linkTf) {
  const next = {};
  let changed = false;
  for (const link in linkTf) {
    rosMatInto(_dm, linkTf[link]);
    _dm.decompose(_dp, _dq, _dsc);
    next[link] = { pos: _dp.clone(), quat: _dq.clone() };
    if (!dispPose[link]) {                       // new link: snap + draw it once
      dispPose[link] = { pos: _dp.clone(), quat: _dq.clone() };
      writeDispArray(link, _dp, _dq);
      changed = true;
    }
  }
  for (const link in dispPose) {
    if (!next[link]) { delete dispPose[link]; delete dispTf[link]; changed = true; }
  }
  targetPose = next;
  if (changed) invalidate();                     // links added/removed -> redraw once
}

// Ease the displayed pose toward the target and refresh the row-major arrays
// the place* functions consume. Returns true while any link is actually moving;
// a link within POS_EPS/ANG_EPS of its target is left untouched (no work, no
// redraw) so a still robot is free. The deadband also means the redraw rate
// scales with how fast the robot moves -- full frame rate in motion, zero at rest.
function advanceInterp(dt) {
  const alpha = 1 - Math.exp(-dt / SMOOTH_TAU);
  let moving = false;
  for (const link in targetPose) {
    const t = targetPose[link];
    let d = dispPose[link];
    if (!d) { d = dispPose[link] = { pos: t.pos.clone(), quat: t.quat.clone() }; }
    if (d.pos.distanceTo(t.pos) < POS_EPS && d.quat.angleTo(t.quat) < ANG_EPS) continue;
    d.pos.lerp(t.pos, alpha);
    d.quat.slerp(t.quat, alpha);
    writeDispArray(link, d.pos, d.quat);
    moving = true;
  }
  return moving;
}

// Place all pose-dependent geometry (meshes, frames, aux, skeleton, TCP triad)
// at the smoothed pose. Called from the render loop only when the pose changed;
// labels are placed separately (they also move when only the camera moves).
function placeGeometry(tf) {
  placeCurrent(tf); placeAux(tf); placeFrames(tf); placeSelFrame(tf);
  updateSkeleton(tf, !opt.mesh || opt.auxOnly || !window.__hasMeshes);
  const tip = window.__tipFrame;
  if (tip && tf[tip]) { tcpAxes.visible = true; rosMatInto(tcpAxes.matrix, tf[tip]); }
  else if (tcpAxes.visible) { tcpAxes.visible = false; }
}

function ensureMeshes(visuals) {
  visuals.forEach((v, i) => {
    const key = v.link + "#" + i;
    if (meshItems[key] !== undefined) return;
    const item = { link: v.link, local: localMatrix(v.xyz, v.rpy, v.scale),
                   solid: null, kind: null, meshes: [] };
    meshItems[key] = item;
    loadProto(v.url, (entry) => {
      let obj; const meshes = [];
      if (entry.kind === "dae") {
        // ColladaLoader rotates a Z_UP asset by -90deg about X to fit three's
        // Y-up world (vertices are NOT converted). Our scene is ROS Z-up (we
        // place link_tf directly) and the mesh vertices are authored Z-up to
        // match the link frame, so we UNDO that up-axis tilt and place the
        // clone via rosMat*local exactly like an STL. The COLLADA sub-meshes
        // KEEP their native materials (e.g. the UR's grey body + blue joints);
        // each mesh's base material is stashed so the selection highlight can
        // swap to orange and back.
        const inner = entry.obj.clone(true);
        inner.rotation.set(0, 0, 0);
        inner.updateMatrix();
        inner.traverse((o) => {
          if (o.isMesh) {
            o.userData.link = v.link;
            o.userData.baseMat = o.material;   // native COLLADA colour
            meshes.push(o);
          }
        });
        obj = new THREE.Group(); obj.add(inner);
      } else {
        obj = new THREE.Mesh(entry.obj, solidMat);
        obj.userData.baseMat = solidMat;       // STL: neutral dashboard material
        meshes.push(obj);
      }
      obj.matrixAutoUpdate = false;
      obj.userData.link = v.link;          // for raycast → link lookup
      item.kind = entry.kind; item.solid = obj; item.meshes = meshes; scene.add(obj);
      invalidate();                        // a mesh just loaded -> draw it
    });
  });
}
function placeCurrent(linkTf) {
  for (const key in meshItems) {
    const it = meshItems[key]; if (!it.solid) continue;
    const lm = linkTf[it.link];
    const hide = !lm || !opt.mesh || opt.auxOnly;   // aux-only hides robot meshes
    if (hide) { if (it.solid.visible) it.solid.visible = false; continue; }
    it.solid.visible = true;
    // Selected link -> orange highlight; otherwise each mesh keeps its BASE
    // material (native COLLADA colours for .dae, neutral grey for STL) so UR
    // robots show their real colours while selection still highlights. Only
    // reassign on change to avoid needless GPU state churn.
    const hi = (it.link === selectedLink);
    for (const m of it.meshes) {
      const mat = hi ? highlightMat : m.userData.baseMat;
      if (m.material !== mat) m.material = mat;
    }
    it.solid.matrix.multiplyMatrices(rosMatInto(_hm, lm), it.local);   // reuse scratch
  }
}

// ---- aux-frame markers (sphere + triad + attachment line) ---------------
// Aux frames have no mesh, so we draw them explicitly: a highlighted sphere at
// the frame origin, an axes triad for orientation, and a line back to the
// parent link origin so the attachment point is obvious. This is the visual
// that makes "what did the manager add, and where" unmistakable.
function ensureAuxMarker(name) {
  if (auxMarkers[name]) return auxMarkers[name];
  const group = new THREE.Group();
  group.matrixAutoUpdate = false;
  const ball = new THREE.Mesh(new THREE.SphereGeometry(0.015, 16, 16),
    new THREE.MeshBasicMaterial({ color: AUX_COLOR }));
  ball.userData.link = name;             // aux frames are clickable too
  group.add(ball);
  group.add(new THREE.AxesHelper(0.12)); // bigger than per-link frames
  scene.add(group);
  const line = new THREE.Line(new THREE.BufferGeometry(),
    new THREE.LineDashedMaterial({ color: AUX_COLOR, dashSize: 0.02, gapSize: 0.012 }));
  line.frustumCulled = false; scene.add(line);
  auxMarkers[name] = { group, line, ball };
  return auxMarkers[name];
}
// world transform of an aux frame: prefer TF; else parent_tf @ local offset
function auxWorld(name, linkTf) {
  if (linkTf[name]) return rosMat(linkTf[name]);
  const def = editDefs[name];
  if (def && linkTf[def.parent]) {
    return rosMat(linkTf[def.parent]).multiply(localMatrix(def.xyz, def.rpy, null));
  }
  return null;
}
const _av = new THREE.Vector3();
// editable frames that have no mesh of their own need an explicit marker so
// they are visible AND clickable (e.g. a bare ft_sensor_link, added or baked in)
function markerNames() {
  return Object.keys(editDefs).filter((n) => !meshLinks.has(n));
}
function placeAux(linkTf) {
  const live = new Set(markerNames());
  for (const name of live) {
    const m = ensureAuxMarker(name);
    const w = auxWorld(name, linkTf);
    if (!w) { m.group.visible = false; m.line.visible = false; continue; }
    m.group.visible = true; m.group.matrix.copy(w);
    const isBase = (editDefs[name] || {}).source === "base";
    const col = name === selectedLink ? 0x39d353 : (isBase ? BASE_COLOR : AUX_COLOR);
    m.ball.material.color.setHex(col);
    m.line.material.color.setHex(col);
    // attachment line parent-origin -> aux-origin
    const def = editDefs[name];
    const p = def && linkTf[def.parent];
    if (p) {
      _av.setFromMatrixPosition(w);
      const seg = [p[0][3], p[1][3], p[2][3], _av.x, _av.y, _av.z];
      m.line.geometry.setAttribute("position",
        new THREE.Float32BufferAttribute(seg, 3));
      m.line.geometry.computeBoundingSphere();
      m.line.computeLineDistances();
      m.line.visible = true;
    } else {
      m.line.visible = false;
    }
  }
  // hide markers for frames that no longer exist
  for (const name in auxMarkers) {
    if (!live.has(name)) {
      auxMarkers[name].group.visible = false;
      auxMarkers[name].line.visible = false;
    }
  }
}

// ---- per-link coordinate frames (toggle) --------------------------------
function ensureFrames(linkTf) {
  for (const link in linkTf) {
    if (frameAxes[link]) continue;
    const ax = new THREE.AxesHelper(0.07);
    ax.matrixAutoUpdate = false; ax.visible = false;
    frameAxes[link] = ax; scene.add(ax);
  }
}
function placeFrames(linkTf) {
  for (const link in frameAxes) {
    const ax = frameAxes[link]; const lm = linkTf[link];
    // the selected link gets the THICK highlight frame instead of the thin one
    if (!lm || !opt.frames || link === selectedLink) { if (ax.visible) ax.visible = false; continue; }
    ax.visible = true; rosMatInto(ax.matrix, lm);
  }
}

// ---- selected-frame highlight (thick triad, shown only with frames) ------
// When the per-link frames are displayed, the SELECTED link's frame is drawn as
// a THICK RGB triad built from cylinders -- WebGL ignores line width, so this is
// how a "wider line" frame is achieved -- and drawn on top (depthTest off). The
// selected link's thin frame is hidden so only the wide one shows.
let selFrame = null;
function ensureSelFrame() {
  if (selFrame) return selFrame;
  const g = new THREE.Group();
  g.matrixAutoUpdate = false; g.visible = false;
  const LEN = 0.07, RAD = 0.006;
  const axis = (hex, dir) => {
    const geo = new THREE.CylinderGeometry(RAD, RAD, LEN, 14);
    geo.translate(0, LEN / 2, 0);                 // base at origin, extends +Y
    const mesh = new THREE.Mesh(geo,
      new THREE.MeshBasicMaterial({ color: hex, depthTest: false }));
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    mesh.renderOrder = 999;
    return mesh;
  };
  g.add(axis(0xff3b53, new THREE.Vector3(1, 0, 0)));  // X red
  g.add(axis(0x39d353, new THREE.Vector3(0, 1, 0)));  // Y green
  g.add(axis(0x3b82ff, new THREE.Vector3(0, 0, 1)));  // Z blue
  scene.add(g); selFrame = g; return g;
}
function placeSelFrame(linkTf) {
  const g = ensureSelFrame();
  if (!opt.frames || !selectedLink || !linkTf || !linkTf[selectedLink]) {
    g.visible = false; return;
  }
  g.visible = true; g.matrix.copy(rosMat(linkTf[selectedLink]));
}

// ---- per-link name labels (HTML overlay, clustered) ---------------------
const MERGE_PX = 18;     // screen-space merge radius (CSS px)
function getLabelDiv(i) {
  if (labelPool[i]) return labelPool[i];
  const d = document.createElement("div");
  d.className = "lbl";
  d.addEventListener("pointerdown", (e) => {
    e.stopPropagation();
    const links = d._links || [];
    if (!links.length) return;
    const idx = links.indexOf(selectedLink);
    setSelected(links[(idx + 1) % links.length], true);   // cycle within cluster
  });
  labelPool[i] = d; if (labelsEl) labelsEl.appendChild(d);
  return d;
}
const _lv = new THREE.Vector3();
function placeLabels(linkTf) {
  if (!labelsEl) return;
  if (!opt.labels) { for (const d of labelPool) d.style.display = "none"; return; }
  const w = canvas.clientWidth, h = canvas.clientHeight;
  // candidate frame origins: every link, plus aux frames via fallback world tf
  const origins = {};
  for (const link of allLinks) {
    if (opt.auxOnly && !auxSet.has(link)) continue;
    const lm = linkTf[link];
    if (lm) origins[link] = [lm[0][3], lm[1][3], lm[2][3]];
  }
  for (const name of auxSet) {
    if (origins[name]) continue;
    const w2 = auxWorld(name, linkTf);
    if (w2) { _lv.setFromMatrixPosition(w2); origins[name] = [_lv.x, _lv.y, _lv.z]; }
  }
  const hits = [];
  for (const link in origins) {
    _lv.set(origins[link][0], origins[link][1], origins[link][2]).project(camera);
    if (_lv.z > 1) continue;   // behind camera
    hits.push({ link, aux: auxSet.has(link),
                x: (_lv.x * 0.5 + 0.5) * w, y: (-_lv.y * 0.5 + 0.5) * h });
  }
  // greedy cluster by screen distance
  const clusters = [];
  for (const hit of hits) {
    let merged = false;
    for (const c of clusters) {
      if (Math.hypot(hit.x - c.x, hit.y - c.y) < MERGE_PX) {
        c.links.push(hit.link); c.aux = c.aux || hit.aux; merged = true; break;
      }
    }
    if (!merged) clusters.push({ x: hit.x, y: hit.y, links: [hit.link], aux: hit.aux });
  }
  let i = 0;
  for (; i < clusters.length; i++) {
    const c = clusters[i]; const d = getLabelDiv(i);
    d._links = c.links;
    d.textContent = c.links.join(", ");
    d.style.display = "block";
    d.style.left = c.x.toFixed(0) + "px";
    d.style.top = c.y.toFixed(0) + "px";
    d.classList.toggle("sel", c.links.includes(selectedLink));
    d.classList.toggle("lbl-aux", !!c.aux);   // colour aux-frame labels
  }
  for (; i < labelPool.length; i++) labelPool[i].style.display = "none";
}

// ---- skeleton (joint-connectivity lines + link dots) --------------------
let skelLines = null, skelDots = null;
function ensureSkeleton() {
  if (!skelLines) {
    skelLines = new THREE.LineSegments(new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x34c3ff }));
    skelLines.frustumCulled = false; scene.add(skelLines);
  }
  if (!skelDots) {
    skelDots = new THREE.Points(new THREE.BufferGeometry(),
      new THREE.PointsMaterial({ color: 0xe6e6e6, size: 0.022 }));
    skelDots.frustumCulled = false; scene.add(skelDots);
  }
}
function updateSkeleton(linkTf, show) {
  ensureSkeleton();
  skelLines.visible = show; skelDots.visible = show;
  if (!show) return;
  const segs = [];
  for (const j of jointTree) {
    const a = linkTf[j.parent], b = linkTf[j.child];
    if (!a || !b) continue;
    segs.push(a[0][3], a[1][3], a[2][3], b[0][3], b[1][3], b[2][3]);
  }
  skelLines.geometry.setAttribute("position",
    new THREE.Float32BufferAttribute(segs, 3));
  skelLines.geometry.computeBoundingSphere();
  const pts = [];
  for (const k in linkTf) { const m = linkTf[k]; pts.push(m[0][3], m[1][3], m[2][3]); }
  skelDots.geometry.setAttribute("position",
    new THREE.Float32BufferAttribute(pts, 3));
  skelDots.geometry.computeBoundingSphere();
}

function frameBox(linkTf) {
  const box = new THREE.Box3(); let any = false;
  for (const k in (linkTf || {})) {
    box.expandByPoint(new THREE.Vector3(linkTf[k][0][3], linkTf[k][1][3], linkTf[k][2][3])); any = true;
  }
  return any ? box : null;
}
function resetView(linkTf) {
  const box = frameBox(linkTf || window.__lastLinkTf);
  if (!box) return;
  const c = box.getCenter(new THREE.Vector3());
  const sz = box.getSize(new THREE.Vector3()).length() || 1.0;
  controls.target.copy(c);
  camera.position.set(c.x + sz, c.y - sz, c.z + sz * 0.7); controls.update();
  invalidate();
}
function fitView(linkTf) {
  if (didFit) return;
  if (!frameBox(linkTf)) return;
  resetView(linkTf); didFit = true;
}

// ---- click-to-select a link (raycast) -----------------------------------
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let downXY = null;
canvas.addEventListener("pointerdown", (e) => { downXY = [e.clientX, e.clientY]; });
canvas.addEventListener("pointerup", (e) => {
  if (!downXY) return;
  const moved = Math.hypot(e.clientX - downXY[0], e.clientY - downXY[1]);
  downXY = null;
  if (moved > 5) return;                       // it was an orbit drag, not a click
  const r = canvas.getBoundingClientRect();
  ndc.x = ((e.clientX - r.left) / r.width) * 2 - 1;
  ndc.y = -((e.clientY - r.top) / r.height) * 2 + 1;
  raycaster.setFromCamera(ndc, camera);
  const pick = [];
  for (const it of Object.values(meshItems)) if (it.solid && it.solid.visible) pick.push(it.solid);
  for (const m of Object.values(auxMarkers)) if (m.group.visible) pick.push(m.ball);
  // recursive so COLLADA sub-meshes (children of the link Group) are hit too
  const hit = raycaster.intersectObjects(pick, true)[0];
  let o = hit ? hit.object : null;
  while (o && !o.userData.link) o = o.parent;   // walk up to the link-tagged node
  // click a link -> select it; click empty space -> deselect
  setSelected(o && o.userData.link ? o.userData.link : "", true);
});

// Esc clears the selection too (ignored while typing in a form field).
window.addEventListener("keydown", (e) => {
  if (e.key !== "Escape" || !selectedLink) return;
  const t = document.activeElement;
  if (t && /^(INPUT|SELECT|TEXTAREA)$/.test(t.tagName)) return;
  setSelected("", true);
});

// Re-apply visibility/highlight to the static scene after a toggle change.
function refreshStatic() { invalidate(); }   // the render loop re-places everything

// ---- live joint-angle panel (read-only, in the 3D overlay) --------------
// Mirrors the robot_web_viewer joint bars: one coloured bar per movable joint,
// filled left/right of centre by the angle as a fraction of its (symmetric)
// limit span, with the value in degrees (or mm for a prismatic joint). This is
// purely a live readout -- the EDITABLE per-joint sliders live in the Joint
// control card and only appear once a joint controller is engaged.

// Depth-first link-chain order of the movable joints. Walks the kinematic tree
// (joint_tree parent->child link edges, kept in URDF order) from each root link
// (a link that is never a child), completing one branch fully before the next.
// So a MULTI-ROBOT system lists one robot's joints, then the next, and a robot
// with DIVERGING links finishes one sub-branch before the other. Fixed joints
// are traversed (to reach movable joints beyond them) but not listed.
function chainJointOrder(jointTree, movable) {
  const movableNames = (movable || []).map((m) => m.name);
  if (!jointTree || !jointTree.length) return movableNames;
  const movableSet = new Set(movableNames);
  const childJoints = {};       // parent link -> [joint, ...] in URDF order
  const childLinks = new Set();
  const links = new Set();
  for (const j of jointTree) {
    (childJoints[j.parent] = childJoints[j.parent] || []).push(j);
    childLinks.add(j.child);
    links.add(j.parent); links.add(j.child);
  }
  // roots = links that are never a joint child (>1 => several robots)
  const roots = [...links].filter((l) => !childLinks.has(l));
  const order = [];
  const seen = new Set();
  const visit = (link) => {
    if (seen.has(link)) return;          // guard against a malformed cycle
    seen.add(link);
    for (const j of (childJoints[link] || [])) {
      if (movableSet.has(j.name)) order.push(j.name);
      visit(j.child);                    // depth-first: finish this branch first
    }
  };
  for (const r of roots) visit(r);
  // append any movable joints not reached from a root (disconnected URDF)
  for (const n of movableNames) if (!order.includes(n)) order.push(n);
  return order;
}

const jbRowEls = {};
function buildJointBars(names) {
  const host = $("joint-bars"); if (!host) return;
  const sig = names.join(",");
  if (host.dataset.sig === sig) return;   // rebuild only when the joint set changes
  host.dataset.sig = sig;
  host.innerHTML = "";
  for (const k in jbRowEls) delete jbRowEls[k];
  names.forEach((name, i) => {
    const color = JOINT_COLORS[i % JOINT_COLORS.length];
    const row = document.createElement("div"); row.className = "jbrow";
    row.innerHTML =
      `<span class="jblabel" style="color:${color}" title="${name}">${name}</span>`
      + `<span class="jbbg"><span class="jbcenter"></span>`
      + `<span class="jbbar"></span></span>`
      + `<span class="jbval">\u2014</span>`;
    host.appendChild(row);
    jbRowEls[name] = { bar: row.querySelector(".jbbar"),
                       val: row.querySelector(".jbval"), color };
  });
}
// Order comes from the caller (kinematic link-chain, see chainJointOrder); the
// joint names are the real URDF names (which match the /joint_states topic).
// limits/type are looked up from the URDF movable-joint metadata (by name), and
// the live angle from joint_values.
function updateJointBars(names, movable, values) {
  const host = $("joint-bars"); if (!host) return;
  if (!names || !names.length) {
    host.innerHTML = '<div class="muted sm">waiting for /joint_states…</div>';
    host.dataset.sig = ""; return;
  }
  const meta = {};
  for (const m of (movable || [])) meta[m.name] = m;
  buildJointBars(names);
  for (const name of names) {
    const row = jbRowEls[name]; if (!row) continue;
    const m = meta[name] || {};
    const ang = m.type !== "prismatic";
    const v = Number((values && values[name]) ?? 0);
    let span = ang ? Math.PI : 0.5;
    if (m.lower != null && m.upper != null) {
      span = Math.max(Math.abs(m.lower), Math.abs(m.upper)) || span;
    }
    const frac = Math.max(-1, Math.min(1, v / span));
    const b = row.bar;
    b.style.background = row.color;
    if (frac >= 0) { b.style.left = "50%"; b.style.right = ""; }
    else { b.style.right = "50%"; b.style.left = ""; }
    b.style.width = (Math.abs(frac) * 50).toFixed(1) + "%";
    row.val.textContent = ang ? (v * RAD2DEG).toFixed(1) + "\u00b0"
                              : (v * 1000).toFixed(0) + " mm";
  }
}

// ---- poll ---------------------------------------------------------------
async function poll() {
  let s;
  try { s = await (await fetch("/api/viewer_state")).json(); } catch { return; }
  readOpts();
  if (s.have_model) {
    const ld = $("loading"); if (ld) ld.style.display = "none";
    const tf = s.link_tf || {};
    window.__hasMeshes = !!s.has_meshes;
    applyMeshAvailability(!!s.has_meshes, !!s.mesh_unsupported);
    allLinks = s.links || Object.keys(tf);
    auxSet = new Set(s.aux_links || []);
    editDefs = {};
    for (const f of (s.editable_frames || s.aux_frames || [])) editDefs[f.name] = f;
    meshLinks = new Set((s.visuals || []).map((v) => v.link));
    jointTree = s.joint_tree || [];
    if (s.has_meshes) ensureMeshes(s.visuals || []);
    ensureFrames(tf);
    window.__tipFrame = s.tip_frame || "";
    // hand the render frame (target frame_id) to the drag gizmo
    if (window.__ccOnState) window.__ccOnState(s);
    setTargetPose(tf);      // feed the smoother; the render loop eases + places the pose
    fitView(tf);
    if ($("n-links")) $("n-links").textContent = (s.links || []).length || "—";
    if ($("n-aux")) $("n-aux").textContent = (s.aux_links || []).length || 0;
    if ($("model-pill")) {
      $("model-pill").textContent = s.has_meshes ? "meshes" : "skeleton";
      $("model-pill").className = "pill pill-good";
    }
    // Joint-angle panel order = kinematic link-chain, depth-first: complete one
    // branch/robot fully before the next (handles multi-robot systems and
    // diverging links). Names are the real joint names (matching /joint_states);
    // values come from joint_values.
    const jOrder = chainJointOrder(jointTree, s.movable_joints || []);
    if ($("n-joints")) $("n-joints").textContent = jOrder.length || "\u2014";
    updateJointBars(jOrder, s.movable_joints || [], s.joint_values || {});
    if ($("js-age")) {
      const a = s.js_age;
      $("js-age").textContent = (a == null) ? "" : a.toFixed(1) + "s";
      $("js-age").classList.toggle("stale", a != null && a > 1.0);
    }
  } else {
    if ($("model-pill")) { $("model-pill").textContent = "no model"; $("model-pill").className = "pill"; }
    updateJointBars([], [], {});
  }
  // hand the snapshot to the test-interface script
  if (typeof window.__onSnapshot === "function") window.__onSnapshot(s);
}
if ($("fit")) $("fit").onclick = () => resetView();
if ($("vf-collapse")) $("vf-collapse").onclick = () => {
  const f = $("view-float"); const b = $("vf-collapse");
  const collapsed = f.classList.toggle("collapsed");
  b.textContent = collapsed ? "+" : "−";
  b.setAttribute("aria-expanded", String(!collapsed));
};
// ---- direct-manipulation drag gizmo (motion / compliance target pose) ----
// A Three.js TransformControls handle on a VISIBLE "target proxy" frame placed
// on the end-effector. Drag it (move / rotate) to command the active FZI
// motion / compliance controller's target_frame -- the same direct-manipulation
// UX as the ikt_pose_commander dashboard. The proxy does NOT follow the robot;
// it is re-centred on the current EE when the gizmo first appears (or via Snap)
// so the first drag doesn't jump. Everything is gated on ccVisible(): the
// controller must be ENGAGED, its kind motion/compliance, AND the operator must
// tick "Direct 3D drag" -- so the robot never moves from a stray click.
const ccProxy = new THREE.Object3D();
ccProxy.add(new THREE.AxesHelper(0.18));
ccProxy.visible = false;
scene.add(ccProxy);
const ccGizmo = new TransformControls(camera, renderer.domElement);
ccGizmo.setSize(0.9);
ccGizmo.setSpace("local");   // handles align with the target / EE axes
ccGizmo.attach(ccProxy);
ccGizmo.enabled = false;
ccGizmo.visible = false;
scene.add(ccGizmo);

let ccEnabled = false;    // operator toggle ("Direct 3D drag")
let ccEngaged = false;    // controller engaged (from /api/state)
let ccKind = "";          // active controller kind
let ccEeFrame = "";       // active controller end_effector_link
let ccRootFrame = "";     // viewer render frame (target frame_id; TF'd server-side)
let _ccInit = false;      // proxy has been placed on the EE at least once
let _ccWasVisible = false;
let _ccLastSend = 0;      // last stream POST time (ms)
let _ccPending = null;    // trailing pose to flush after the throttle window

function ccVisible() {
  return !!(ccEnabled && ccEngaged
    && (ccKind === "motion" || ccKind === "compliance")
    && ccEeFrame && window.__lastLinkTf && window.__lastLinkTf[ccEeFrame]);
}

// Re-centre the draggable frame on the controlled end-effector's live pose.
function ccSnapToEE() {
  const tf = window.__lastLinkTf;
  if (!ccEeFrame || !tf || !tf[ccEeFrame]) return false;
  rosMat(tf[ccEeFrame]).decompose(ccProxy.position, ccProxy.quaternion, ccProxy.scale);
  ccProxy.scale.set(1, 1, 1);
  ccProxy.updateMatrixWorld(true);
  _ccInit = true;
  invalidate();
  return true;
}

function ccApplyVisibility() {
  const vis = ccVisible();
  if (vis && !_ccWasVisible) ccSnapToEE();   // seed on appear so first drag holds
  _ccWasVisible = vis;
  ccGizmo.enabled = vis;
  ccGizmo.visible = vis;
  ccProxy.visible = vis;
  const hint = $("gizmo-hint");
  if (hint) {
    hint.textContent = vis
      ? "drag the triad on the tool \u2014 move / rotate to command the target"
      : (ccEnabled
          ? "engage a motion / compliance controller to drag"
          : "tick to grab the end-effector in 3D");
  }
  invalidate();
}

const _ccp = new THREE.Vector3(), _ccq = new THREE.Quaternion(), _ccs = new THREE.Vector3();
function ccTargetBody() {
  ccProxy.updateMatrixWorld(true);
  ccProxy.matrixWorld.decompose(_ccp, _ccq, _ccs);
  // three quaternion is (x,y,z,w); the backend wants [w,x,y,z]
  return { xyz: [_ccp.x, _ccp.y, _ccp.z],
           quat: [_ccq.w, _ccq.x, _ccq.y, _ccq.z],
           frame_id: ccRootFrame || "" };
}
function ccPost(body) {
  fetch("/api/target_pose", { method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body) }).catch(() => {});
}
// Stream drag poses at ~25 Hz with a trailing flush so the LAST pose of a drag
// always lands even if it arrived inside the throttle window.
function ccStream() {
  if (!ccVisible()) return;
  const now = performance.now();
  const body = ccTargetBody();
  if (now - _ccLastSend >= 40) {
    _ccLastSend = now; _ccPending = null; ccPost(body);
  } else {
    _ccPending = body;
    if (!ccStream._t) {
      ccStream._t = setTimeout(() => {
        ccStream._t = null;
        if (_ccPending) {
          _ccLastSend = performance.now();
          const b = _ccPending; _ccPending = null; ccPost(b);
        }
      }, 45);
    }
  }
}

ccGizmo.addEventListener("dragging-changed", (e) => {
  controls.enabled = !e.value;                 // don't orbit while dragging a handle
  if (!e.value && ccVisible()) ccStream();      // flush the final pose on release
});
ccGizmo.addEventListener("objectChange", () => { if (ccVisible()) ccStream(); });
ccGizmo.addEventListener("change", invalidate);

function ccSetMode(mode) {
  ccGizmo.setMode(mode);
  const mv = $("gizmo-move"), ro = $("gizmo-rotate");
  if (mv) mv.classList.toggle("sel", mode === "translate");
  if (ro) ro.classList.toggle("sel", mode === "rotate");
  invalidate();
}
if ($("gizmo-move")) $("gizmo-move").onclick = () => ccSetMode("translate");
if ($("gizmo-rotate")) $("gizmo-rotate").onclick = () => ccSetMode("rotate");
if ($("gizmo-enable")) $("gizmo-enable").addEventListener("change", (e) => {
  ccEnabled = !!e.target.checked; _ccInit = false; ccApplyVisibility();
});
ccSetMode("translate");

// Fed by dashboard.js on every /api/state + /api/live tick so the gizmo tracks
// the engaged state / active controller without polling itself.
window.__ccGizmo = {
  update(st) {
    if (!st) return;
    if (typeof st.engaged === "boolean") ccEngaged = st.engaged;
    if (typeof st.kind === "string") ccKind = st.kind;
    if (typeof st.eeFrame === "string" && st.eeFrame) ccEeFrame = st.eeFrame;
    ccApplyVisibility();
  },
  snapToEE: ccSnapToEE,
  setEnabled(b) {
    ccEnabled = !!b;
    const cb = $("gizmo-enable"); if (cb) cb.checked = ccEnabled;
    _ccInit = false; ccApplyVisibility();
  },
  setMode: ccSetMode,
};
// The viewer's own /api/viewer_state poll hands us the render frame (used as the
// target frame_id; the backend TF-transforms it into the controller base link).
window.__ccOnState = (s) => {
  if (s && s.base_frame) ccRootFrame = s.base_frame;
  if (ccVisible() && !_ccInit) ccSnapToEE();
};
poll(); setInterval(poll, 100);

// ---- render loop (on-demand) --------------------------------------------
// Resize is handled by a ResizeObserver instead of reading canvas.clientWidth
// every frame (which forces a synchronous layout reflow each frame).
let _pendingResize = true;
function applyResize() {
  _pendingResize = false;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const pr = renderer.getPixelRatio();
  if (canvas.width !== Math.round(w * pr) || canvas.height !== Math.round(h * pr)) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
}
new ResizeObserver(() => { _pendingResize = true; invalidate(); }).observe(canvas);

let _prevT = performance.now();
(function animate(now) {
  requestAnimationFrame(animate);
  const t = now || performance.now();
  const dt = Math.min(0.1, Math.max(0, (t - _prevT) * 0.001)); _prevT = t;
  if (_pendingResize) applyResize();
  const moving = advanceInterp(dt);        // ease poses; true while any link moves
  const camMoved = controls.update();      // true while the camera moves (drag / damping)
  const geomDirty = moving || needsRender;
  if (geomDirty || camMoved) {
    if (geomDirty) placeGeometry(dispTf);   // re-place the robot only when it moved
    placeLabels(dispTf);                    // labels reproject on robot OR camera move
    renderer.render(scene, camera);
    needsRender = false;
  }
})();
