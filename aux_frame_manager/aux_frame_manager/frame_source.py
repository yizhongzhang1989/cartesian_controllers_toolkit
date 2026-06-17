"""Pure URDF aux-frame logic for ``aux_frame_manager`` (no rclpy).

Kept free of ROS so the merge/validate/augment pipeline can be unit-tested
offline. The node (``aux_frame_manager_node``) is a thin ROS wrapper over
:func:`build_canonical_urdf`.

Frame source precedence (Req 1 "config file AND direct argument"):
``merge_frames`` overlays inline/argument frames on top of the config-file
frames -- a same-named arg entry overrides the file entry; new arg entries are
appended. This gives "override or extend".

Feedback-loop safety: the manager may also push the augmented URDF to
``robot_state_publisher``, which then re-publishes it on the base topic. To stay
idempotent, :func:`build_canonical_urdf` first STRIPS any managed aux frames
from the incoming URDF (so an already-augmented echo collapses back to the
base) and then augments fresh. Re-processing canonical output yields the same
bytes, so the loop terminates.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

from cct_common.urdf_loader import augment_urdf, update_aux_frames


def _as3(value: Optional[Sequence[float]]) -> List[float]:
    if value is None:
        return [0.0, 0.0, 0.0]
    out = [float(v) for v in value]
    if len(out) != 3:
        raise ValueError(f"expected 3 numbers, got {out!r}")
    return out


def normalize_frame(entry: Mapping) -> Dict:
    """Return a canonical ``{name, parent, xyz[3], rpy[3]}`` dict.

    Raises ``ValueError`` on a missing/empty ``name`` or ``parent`` or a
    malformed ``xyz`` / ``rpy``.
    """
    if not isinstance(entry, Mapping):
        raise ValueError(f"aux frame must be a mapping, got {type(entry).__name__}")
    name = str(entry.get("name") or "").strip()
    parent = str(entry.get("parent") or "").strip()
    if not name:
        raise ValueError("aux frame requires a non-empty 'name'")
    if not parent:
        raise ValueError(f"aux frame '{name}' requires a non-empty 'parent'")
    return {
        "name": name,
        "parent": parent,
        "xyz": _as3(entry.get("xyz")),
        "rpy": _as3(entry.get("rpy")),
    }


def parse_inline_frames(spec) -> List[Dict]:
    """Parse the inline ``aux_frames`` argument into a list of frame dicts.

    Accepts an already-parsed list, or a YAML/JSON string (so it works both
    from a launch list-param and from a plain string CLI override). Returns
    ``[]`` for an empty/blank spec.
    """
    if spec is None:
        return []
    if isinstance(spec, str):
        s = spec.strip()
        if not s or s in ("[]", "null", "~"):
            return []
        try:
            data = yaml.safe_load(s)
        except yaml.YAMLError as exc:
            raise ValueError(f"could not parse inline aux_frames: {exc}") from exc
    else:
        data = spec
    if data is None:
        return []
    if isinstance(data, Mapping):
        data = [data]
    if not isinstance(data, (list, tuple)):
        raise ValueError("inline aux_frames must be a list of mappings")
    return [normalize_frame(e) for e in data]


def _parse_triple(text: str) -> List[float]:
    vals = [float(x) for x in str(text).split(",")]
    if len(vals) != 3:
        raise ValueError(f"expected 3 comma-separated numbers, got {text!r}")
    return vals


def parse_spec_string(spec) -> List[Dict]:
    """Parse rcl-param-friendly compact frame specs (the "direct argument").

    Format: ``name:parent[:x,y,z[:r,p,yw]]`` specs separated by ``;`` (or
    newlines). Deliberately uses NO brackets/braces/quotes so it survives the
    rcl parameter parser on the CLI, in a launch arg, and in a params file
    (where a bracketed YAML/JSON string does NOT parse). Example::

        ft_sensor_link:link_6; compliance_link:ft_sensor_link; \
        op_tip:compliance_link:0,0,0.05

    Returns ``[]`` for an empty spec. Accepts a string or a list of strings.
    """
    if spec is None:
        return []
    if isinstance(spec, (list, tuple)):
        items: List[str] = []
        for s in spec:
            items.extend(str(s).replace("\n", ";").split(";"))
    else:
        items = str(spec).replace("\n", ";").split(";")
    out: List[Dict] = []
    for raw in items:
        s = raw.strip()
        if not s:
            continue
        parts = [p.strip() for p in s.split(":")]
        if len(parts) < 2 or not parts[0] or not parts[1]:
            raise ValueError(f"aux frame spec {s!r} needs at least 'name:parent'")
        xyz = _parse_triple(parts[2]) if len(parts) > 2 and parts[2] else [0.0, 0.0, 0.0]
        rpy = _parse_triple(parts[3]) if len(parts) > 3 and parts[3] else [0.0, 0.0, 0.0]
        out.append(normalize_frame(
            {"name": parts[0], "parent": parts[1], "xyz": xyz, "rpy": rpy}))
    return out


def merge_frames(file_frames: Sequence[Mapping],
                 arg_frames: Sequence[Mapping]) -> List[Dict]:
    """Overlay ``arg_frames`` on ``file_frames`` (arg overrides by name, then
    appends new arg entries). Order: file order first, then new arg frames in
    their given order. Both inputs are normalised.
    """
    merged: List[Dict] = [normalize_frame(f) for f in (file_frames or [])]
    index = {f["name"]: i for i, f in enumerate(merged)}
    for raw in (arg_frames or []):
        f = normalize_frame(raw)
        if f["name"] in index:
            merged[index[f["name"]]] = f
        else:
            index[f["name"]] = len(merged)
            merged.append(f)
    return merged


def validate_frames(frames: Sequence[Mapping],
                    base_link_names: Sequence[str]) -> Tuple[bool, str]:
    """Validate that ``frames`` can be appended to a URDF with ``base_link_names``.

    Checks: unique non-empty names; no collision with an existing base link;
    every parent resolves to a base link or an EARLIER frame (this enforces the
    series ordering ``augment_urdf`` requires and rejects cycles / forward
    references). Returns ``(ok, message)``.
    """
    known = set(base_link_names or [])
    seen: set = set()
    for f in frames:
        try:
            nf = normalize_frame(f)
        except ValueError as exc:
            return False, str(exc)
        name, parent = nf["name"], nf["parent"]
        if name in seen:
            return False, f"duplicate aux frame name '{name}'"
        if name in (base_link_names or []):
            return False, f"aux frame '{name}' collides with an existing URDF link"
        if parent not in known:
            return False, (f"aux frame '{name}' parent '{parent}' is not an "
                           f"existing link or an earlier aux frame (check order)")
        seen.add(name)
        known.add(name)
    return True, "ok"


def _link_names(urdf_xml: str) -> List[str]:
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"expected <robot> root, got <{root.tag}>")
    return [ln.get("name") for ln in root.findall("link") if ln.get("name")]


def strip_aux_frames(urdf_xml: str, names: Sequence[str]) -> str:
    """Remove managed aux frames (``<link name=N>`` + the fixed ``<joint>`` whose
    child is ``N``) from a URDF, yielding the base. Idempotent: stripping a URDF
    that has none of ``names`` returns it unchanged (byte-for-byte for the parse
    round-trip). This is what makes :func:`build_canonical_urdf` loop-safe.
    """
    name_set = set(n for n in names if n)
    if not name_set:
        return urdf_xml
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"expected <robot> root, got <{root.tag}>")
    changed = False
    for link in list(root.findall("link")):
        if link.get("name") in name_set:
            root.remove(link)
            changed = True
    for joint in list(root.findall("joint")):
        child = joint.find("child")
        if child is not None and child.get("link") in name_set:
            root.remove(joint)
            changed = True
    if not changed:
        return urdf_xml
    return ET.tostring(root, encoding="unicode")


def order_frames(frames: Sequence[Mapping],
                 base_link_names: Sequence[str]) -> List[Dict]:
    """Topologically sort aux frames so each one follows what it hangs off.

    Aux frames may be supplied in ANY order -- a child can appear before its
    parent (e.g. a dashboard that moves an edited row to the end of its list,
    so editing the FIRST frame in a parent->child chain would otherwise push it
    after its own child). ``augment_urdf`` / ``validate_frames`` require the
    parent to come first, so reorder here. The sort is stable: a frame is
    emitted as soon as its parent (a base-URDF link, or an earlier aux frame)
    is available, otherwise input order is preserved.

    Raises ``ValueError`` if a frame's parent is neither a base link nor another
    aux frame (unknown parent), or if the aux frames form a cycle.
    """
    norm = [normalize_frame(f) for f in (frames or [])]
    if not norm:
        return []
    base = set(base_link_names or [])
    aux_names = {f["name"] for f in norm}
    for f in norm:
        if f["parent"] not in base and f["parent"] not in aux_names:
            raise ValueError(
                f"aux frame '{f['name']}' parent '{f['parent']}' is not an "
                f"existing link or another aux frame")
    ordered: List[Dict] = []
    placed = set(base)
    remaining = list(norm)
    while remaining:
        progressed = False
        still: List[Dict] = []
        for f in remaining:
            # a frame whose name was already placed earlier in the list is a
            # duplicate; leave it for validate_frames to reject (don't drop it)
            if f["parent"] in placed:
                ordered.append(f)
                placed.add(f["name"])
                progressed = True
            else:
                still.append(f)
        if not progressed:
            cyc = [f["name"] for f in remaining]
            raise ValueError(f"cyclic aux-frame parent chain among {cyc}")
        remaining = still
    return ordered


def build_canonical_urdf(incoming_urdf: str,
                         frames: Sequence[Mapping],
                         strip_extra: Optional[Sequence[str]] = None
                         ) -> Tuple[str, List[Dict]]:
    """Return ``(canonical_urdf, normalised_frames)``.

    STRIP any managed aux frames from ``incoming_urdf`` (collapses an
    already-augmented echo back to the base), VALIDATE, then AUGMENT fresh.
    ``strip_extra`` names are stripped too -- pass every frame name the manager
    has EVER added so a live remove/rename can't strand a stale frame (the
    incoming URDF may still carry it, e.g. an RSP echo). Raises ``ValueError``
    on an invalid frame set (caller must not publish).
    """
    norm = [normalize_frame(f) for f in (frames or [])]
    names = [f["name"] for f in norm]
    strip_names = set(names) | set(strip_extra or [])
    base = strip_aux_frames(incoming_urdf, strip_names)
    base_links = _link_names(base)
    # Order-independent: sort so each aux frame follows its parent before we
    # validate/augment (clients may send a child ahead of its parent).
    norm = order_frames(norm, base_links)
    ok, msg = validate_frames(norm, base_links)
    if not ok:
        raise ValueError(msg)
    return augment_urdf(base, norm), norm


def list_fixed_frames(urdf_xml: str) -> List[Dict]:
    """Return ``{name, parent, xyz, rpy}`` for every link held by a FIXED joint.

    These are exactly the frames whose static offset can be edited -- whether
    this manager appended them or they were already baked into the incoming
    (launch-time) URDF. Links carried by a movable joint (revolute / prismatic /
    continuous / floating / planar) are skipped: their pose comes from joint
    state, not a fixed offset, so editing an origin would be meaningless.
    """
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as exc:
        raise ValueError(f"could not parse URDF: {exc}") from exc
    if root.tag != "robot":
        raise ValueError(f"expected <robot> root, got <{root.tag}>")
    out: List[Dict] = []
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        c = joint.find("child")
        p = joint.find("parent")
        if c is None or p is None or not c.get("link") or not p.get("link"):
            continue
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(v) for v in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(v) for v in origin.get("rpy").split()]
        out.append({"name": c.get("link"), "parent": p.get("link"),
                    "xyz": xyz, "rpy": rpy})
    return out


def apply_overrides(urdf_xml: str,
                    overrides: Mapping[str, Mapping]
                    ) -> Tuple[str, List[str], List[str]]:
    """Rewrite the ``<origin>`` of existing FIXED-joint frames in place.

    ``overrides`` maps a frame (child-link) name to ``{xyz, rpy}``. This edits
    the offset of frames ALREADY present in ``urdf_xml`` (e.g. aux frames baked
    into the launch URDF) WITHOUT adding or restructuring anything -- unlike
    :func:`build_canonical_urdf`, which augments brand-new frames. It is the
    counterpart used by the manager so that *every* fixed frame is editable, not
    only the ones it appended.

    Returns ``(urdf, updated_names, missing_names)``. A name with no matching
    fixed joint is reported in ``missing`` and left untouched (idempotent, so
    re-applying the same overrides on the manager's own echo is a no-op).
    """
    if not overrides:
        return urdf_xml, [], []
    entries = [{"name": str(name),
                "xyz": _as3(spec.get("xyz") if isinstance(spec, Mapping) else None),
                "rpy": _as3(spec.get("rpy") if isinstance(spec, Mapping) else None)}
               for name, spec in overrides.items()]
    res = update_aux_frames(urdf_xml, entries)
    return res.urdf_xml, list(res.updated), list(res.missing)


def extract_chain_links(urdf_xml: str, base: str, ee: str) -> List[str]:
    """Return the ordered link names from ``base`` to ``ee``.

    Mirrors KDL ``getChain(base, ee)`` for a serial chain: walk parent joints
    from ``ee`` upward until ``base`` is reached. Raises ``ValueError`` if there
    is no path (what makes an FZI controller's on_configure fail) or a cycle.
    """
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"expected <robot> root, got <{root.tag}>")
    parent_of: Dict[str, str] = {}
    for joint in root.findall("joint"):
        c = joint.find("child")
        p = joint.find("parent")
        if c is not None and p is not None and c.get("link") and p.get("link"):
            parent_of[c.get("link")] = p.get("link")
    chain = [ee]
    seen = {ee}
    cur = ee
    while cur != base:
        nxt = parent_of.get(cur)
        if nxt is None:
            raise ValueError(
                f"no kinematic path from '{base}' to '{ee}' (stuck at '{cur}')")
        if nxt in seen:
            raise ValueError(f"cycle walking from '{ee}' toward '{base}'")
        chain.append(nxt)
        seen.add(nxt)
        cur = nxt
    chain.reverse()
    return chain


def check_frames_in_chain(urdf_xml: str, base: str, ee: str,
                          required: Sequence[str]) -> Tuple[bool, str]:
    """Verify every frame in ``required`` lies on the ``base``->``ee`` chain.

    This is exactly the FZI configure-time constraint (``getChain`` +
    ``robotChainContains``). Returns ``(ok, message)``.
    """
    try:
        chain = extract_chain_links(urdf_xml, base, ee)
    except ValueError as exc:
        return False, str(exc)
    chainset = set(chain)
    missing = [f for f in (required or []) if f and f not in chainset]
    if missing:
        return False, (f"frames not on the {base}->{ee} chain: {missing} "
                       f"(chain: {chain})")
    return True, "ok"
