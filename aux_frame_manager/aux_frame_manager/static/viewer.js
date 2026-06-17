"use strict";
// 3D viewer for the aux_frame_manager dashboard.
//
// Renders the robot from the *canonical* URDF meshes (the manager's output on
// /cartesian/robot_description) at the live pose taken from TF (server-side
// lookup base_frame -> link; no Pinocchio / no server FK), and clearly marks
// which frames are ORIGINAL (links present in the manufacturer base URDF,
// drawn as neutral grey meshes) versus ADDED aux frames (the links the manager
// injected — drawn as a highlighted orange sphere + triad + label, plus an
// attachment line to their parent link). Pre-existing fixed frames baked into
// the launch URDF (also editable here) get the same marker in cyan, so every
// editable frame is visible and clickable regardless of who created it.
//
// View options (checkboxes): meshes / labels / per-link frames / aux-only.
// Click a mesh to select it; the selection is echoed to the editor panel via
// window.__onPickLink so the test interface can load it.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const $ = (id) => document.getElementById(id);

const AUX_COLOR = 0xffb454;     // added aux frames (orange)
const AUX_COLOR_CSS = "#ffb454";
const BASE_COLOR = 0x34c3ff;    // pre-existing (launch-URDF) editable frames (cyan)

// ---- scene --------------------------------------------------------------
const canvas = $("viewer");
const labelsEl = $("labels");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1419);
const vw = () => canvas.clientWidth || (innerWidth - 360);
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

const solidMat = new THREE.MeshStandardMaterial({ color: 0x9fb4c4, metalness: 0.25, roughness: 0.6 });
const highlightMat = new THREE.MeshStandardMaterial({ color: AUX_COLOR, emissive: 0x6e3d00,
  emissiveIntensity: 0.6, metalness: 0.2, roughness: 0.5 });
const stlLoader = new STLLoader();
const geomCache = {};   // url -> {geom, waiting:[cb]}
const meshItems = {};   // key(link#i) -> {link, local, solid}
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

// ---- selection ----------------------------------------------------------
let selectedLink = "";
function setSelected(link, notify) {
  selectedLink = link || "";
  if ($("sel-link")) $("sel-link").textContent = selectedLink || "—";
  if (notify && typeof window.__onPickLink === "function") {
    window.__onPickLink(selectedLink, !!editDefs[selectedLink],
                        editDefs[selectedLink] || null);
  }
}
window.__viewerSelect = (link) => setSelected(link, false);

function getGeom(url, cb) {
  const c = geomCache[url];
  if (c && c.geom) { cb(c.geom); return; }
  if (c) { c.waiting.push(cb); return; }
  geomCache[url] = { geom: null, waiting: [cb] };
  stlLoader.load(url, (g) => {
    g.computeVertexNormals();
    geomCache[url].geom = g;
    geomCache[url].waiting.forEach((f) => f(g));
    geomCache[url].waiting = [];
  }, undefined, () => { /* load error: skeleton still shows */ });
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
function ensureMeshes(visuals) {
  visuals.forEach((v, i) => {
    const key = v.link + "#" + i;
    if (meshItems[key] !== undefined) return;
    const item = { link: v.link, local: localMatrix(v.xyz, v.rpy, v.scale), solid: null };
    meshItems[key] = item;
    getGeom(v.url, (geom) => {
      const s = new THREE.Mesh(geom, solidMat); s.matrixAutoUpdate = false;
      s.userData.link = v.link;          // for raycast → link lookup
      item.solid = s; scene.add(s);
    });
  });
}
function placeCurrent(linkTf) {
  for (const key in meshItems) {
    const it = meshItems[key]; if (!it.solid) continue;
    const lm = linkTf[it.link];
    const hide = !lm || !opt.mesh || opt.auxOnly;   // aux-only hides robot meshes
    if (hide) { it.solid.visible = false; continue; }
    it.solid.visible = true;
    it.solid.material = (it.link === selectedLink) ? highlightMat : solidMat;
    it.solid.matrix.copy(rosMat(lm).multiply(it.local));
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
    if (!lm || !opt.frames) { ax.visible = false; continue; }
    ax.visible = true; ax.matrix.copy(rosMat(lm));
  }
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
  const hit = raycaster.intersectObjects(pick, false)[0];
  if (hit && hit.object.userData.link) setSelected(hit.object.userData.link, true);
});

// Re-apply visibility/highlight to the static scene after a toggle change.
function refreshStatic() {
  const tf = window.__lastLinkTf;
  if (!tf) return;
  placeCurrent(tf); placeAux(tf); placeFrames(tf); placeLabels(tf);
  updateSkeleton(tf, !opt.mesh || opt.auxOnly || !window.__hasMeshes);
}

// ---- poll ---------------------------------------------------------------
async function poll() {
  let s;
  try { s = await (await fetch("/api/state")).json(); } catch { return; }
  readOpts();
  if (s.have_model) {
    const ld = $("loading"); if (ld) ld.style.display = "none";
    const tf = s.link_tf || {};
    window.__hasMeshes = !!s.has_meshes;
    allLinks = s.links || Object.keys(tf);
    auxSet = new Set(s.aux_links || []);
    editDefs = {};
    for (const f of (s.editable_frames || s.aux_frames || [])) editDefs[f.name] = f;
    meshLinks = new Set((s.visuals || []).map((v) => v.link));
    jointTree = s.joint_tree || [];
    if (s.has_meshes) ensureMeshes(s.visuals || []);
    ensureFrames(tf);
    placeCurrent(tf); placeAux(tf); placeFrames(tf); placeLabels(tf);
    updateSkeleton(tf, !opt.mesh || opt.auxOnly || !s.has_meshes);
    fitView(tf);
    window.__lastLinkTf = tf;
    if ($("n-links")) $("n-links").textContent = (s.links || []).length || "—";
    if ($("n-aux")) $("n-aux").textContent = (s.aux_links || []).length || 0;
    if ($("model-pill")) {
      $("model-pill").textContent = s.has_meshes ? "meshes" : "skeleton";
      $("model-pill").className = "pill pill-good";
    }
  } else {
    if ($("model-pill")) { $("model-pill").textContent = "no model"; $("model-pill").className = "pill"; }
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
poll(); setInterval(poll, 200);

// ---- render loop --------------------------------------------------------
function resizeToDisplay() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const pr = renderer.getPixelRatio();
  if (canvas.width !== Math.round(w * pr) || canvas.height !== Math.round(h * pr)) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
}
(function animate() {
  requestAnimationFrame(animate);
  resizeToDisplay(); controls.update(); renderer.render(scene, camera);
  if (window.__lastLinkTf) placeLabels(window.__lastLinkTf);
})();
