"""Offline unit tests for aux_frame_manager.frame_source (no ROS)."""

import pytest

from aux_frame_manager.frame_source import (build_canonical_urdf, merge_frames,
                                            normalize_frame, order_frames,
                                            parse_inline_frames,
                                            parse_spec_string, strip_aux_frames,
                                            validate_frames)

BASE = (
    '<robot name="arm">'
    '<link name="base_link"/>'
    '<link name="link_6"/>'
    '<joint name="j6" type="revolute"><parent link="base_link"/>'
    '<child link="link_6"/></joint>'
    '</robot>'
)


def _links(urdf):
    import xml.etree.ElementTree as ET
    return [l.get("name") for l in ET.fromstring(urdf).findall("link")]


def test_normalize_defaults():
    f = normalize_frame({"name": "a", "parent": "link_6"})
    assert f == {"name": "a", "parent": "link_6", "xyz": [0, 0, 0], "rpy": [0, 0, 0]}


def test_normalize_requires_name_parent():
    with pytest.raises(ValueError):
        normalize_frame({"parent": "link_6"})
    with pytest.raises(ValueError):
        normalize_frame({"name": "a"})


def test_parse_inline_yaml_and_json_and_empty():
    assert parse_inline_frames("") == []
    assert parse_inline_frames("[]") == []
    y = parse_inline_frames("[{name: t, parent: link_6, xyz: [0,0,0.1]}]")
    assert y[0]["name"] == "t" and y[0]["xyz"] == [0, 0, 0.1]
    j = parse_inline_frames('[{"name":"t","parent":"link_6"}]')
    assert j[0]["parent"] == "link_6"


def test_parse_spec_string_compact():
    assert parse_spec_string("") == []
    specs = ("ft_sensor_link:link_6; compliance_link:ft_sensor_link; "
             "op_tip:compliance_link:0,0,0.05")
    frames = parse_spec_string(specs)
    assert [f["name"] for f in frames] == [
        "ft_sensor_link", "compliance_link", "op_tip"]
    assert frames[0]["parent"] == "link_6"
    assert frames[2]["xyz"] == [0.0, 0.0, 0.05]
    # with rpy too
    f2 = parse_spec_string("a:b:1,2,3:0,0,1.57")
    assert f2[0]["xyz"] == [1, 2, 3] and f2[0]["rpy"] == [0, 0, 1.57]
    # malformed -> ValueError
    with pytest.raises(ValueError):
        parse_spec_string("justname")


def test_merge_override_and_extend():
    file_frames = [{"name": "ft", "parent": "link_6", "xyz": [0, 0, 0]}]
    arg_frames = [{"name": "ft", "parent": "link_6", "xyz": [0, 0, 0.01]},
                  {"name": "comp", "parent": "ft"}]
    merged = merge_frames(file_frames, arg_frames)
    assert [f["name"] for f in merged] == ["ft", "comp"]
    assert merged[0]["xyz"] == [0, 0, 0.01]  # arg overrode file


def test_validate_series_ok_and_cycle_reject():
    frames = [{"name": "ft", "parent": "link_6"},
              {"name": "comp", "parent": "ft"}]
    ok, _ = validate_frames(frames, ["base_link", "link_6"])
    assert ok
    # forward reference (comp before ft) must fail
    bad = [{"name": "comp", "parent": "ft"}, {"name": "ft", "parent": "link_6"}]
    ok, msg = validate_frames(bad, ["base_link", "link_6"])
    assert not ok and "earlier aux frame" in msg


def test_validate_collision_with_existing_link():
    ok, msg = validate_frames([{"name": "link_6", "parent": "base_link"}],
                              ["base_link", "link_6"])
    assert not ok and "collides" in msg


def test_build_canonical_adds_frames():
    frames = [{"name": "ft_sensor_link", "parent": "link_6"},
              {"name": "compliance_link", "parent": "ft_sensor_link"}]
    canonical, norm = build_canonical_urdf(BASE, frames)
    links = _links(canonical)
    assert "ft_sensor_link" in links and "compliance_link" in links
    assert len(norm) == 2


def test_build_canonical_is_idempotent_loop_safe():
    """Re-processing canonical output yields identical bytes (RSP echo safe)."""
    frames = [{"name": "ft_sensor_link", "parent": "link_6"},
              {"name": "compliance_link", "parent": "ft_sensor_link"}]
    canon1, _ = build_canonical_urdf(BASE, frames)
    canon2, _ = build_canonical_urdf(canon1, frames)   # feed our own output back
    assert canon1 == canon2


def test_strip_removes_only_managed_frames():
    frames = [{"name": "ft_sensor_link", "parent": "link_6"}]
    canon, _ = build_canonical_urdf(BASE, frames)
    stripped = strip_aux_frames(canon, ["ft_sensor_link"])
    assert "ft_sensor_link" not in _links(stripped)
    assert "link_6" in _links(stripped) and "base_link" in _links(stripped)


def test_build_canonical_rejects_invalid():
    with pytest.raises(ValueError):
        build_canonical_urdf(BASE, [{"name": "x", "parent": "no_such_link"}])


def test_order_frames_child_before_parent():
    """A child supplied BEFORE its parent must be reordered (regression: a
    dashboard that moves the edited row to the end of the list sends
    [compliance_link, ft_sensor_link] when ft_sensor_link is edited)."""
    out_of_order = [
        {"name": "compliance_link", "parent": "ft_sensor_link"},
        {"name": "ft_sensor_link", "parent": "link_6"},
    ]
    ordered = order_frames(out_of_order, ["base_link", "link_6"])
    names = [f["name"] for f in ordered]
    assert names == ["ft_sensor_link", "compliance_link"]


def test_build_canonical_accepts_child_before_parent():
    """The full build must succeed regardless of input frame order."""
    out_of_order = [
        {"name": "compliance_link", "parent": "ft_sensor_link", "xyz": [0, 0, 0.3]},
        {"name": "ft_sensor_link", "parent": "link_6", "xyz": [0, 0, 0.05]},
    ]
    canon, norm = build_canonical_urdf(BASE, out_of_order)
    # canonical chain is correct and the returned frames are dependency-ordered
    assert [f["name"] for f in norm] == ["ft_sensor_link", "compliance_link"]
    from aux_frame_manager.frame_source import extract_chain_links
    chain = extract_chain_links(canon, "base_link", "compliance_link")
    assert chain == ["base_link", "link_6", "ft_sensor_link", "compliance_link"]


def test_order_frames_detects_cycle():
    with pytest.raises(ValueError):
        order_frames([{"name": "a", "parent": "b"},
                      {"name": "b", "parent": "a"}], ["base_link"])


def test_order_frames_unknown_parent():
    with pytest.raises(ValueError):
        order_frames([{"name": "a", "parent": "nope"}], ["base_link"])


def test_strip_extra_removes_since_removed_frames():
    """Live remove/rename: rebuilding from a URDF that still carries an old
    frame must drop it when strip_extra names it (regression)."""
    frames = [{"name": "ft_sensor_link", "parent": "link_6"},
              {"name": "op_tip", "parent": "ft_sensor_link"}]
    canon, _ = build_canonical_urdf(BASE, frames)
    assert "op_tip" in _links(canon)
    # now remove op_tip; feeding the OLD canonical back, op_tip must disappear
    # because strip_extra still names it (it is no longer in the frame list).
    new_frames = [{"name": "ft_sensor_link", "parent": "link_6"}]
    canon2, _ = build_canonical_urdf(canon, new_frames, strip_extra=["op_tip"])
    assert "op_tip" not in _links(canon2)
    assert "ft_sensor_link" in _links(canon2)


def test_strip_extra_handles_rename():
    frames = [{"name": "old_name", "parent": "link_6"}]
    canon, _ = build_canonical_urdf(BASE, frames)
    renamed = [{"name": "new_name", "parent": "link_6"}]
    canon2, _ = build_canonical_urdf(canon, renamed, strip_extra=["old_name"])
    links = _links(canon2)
    assert "new_name" in links and "old_name" not in links


def test_extract_chain_and_in_chain():
    from aux_frame_manager.frame_source import (check_frames_in_chain,
                                                extract_chain_links)
    frames = [{"name": "ft_sensor_link", "parent": "link_6"},
              {"name": "compliance_link", "parent": "ft_sensor_link"}]
    canon, _ = build_canonical_urdf(BASE, frames)
    chain = extract_chain_links(canon, "base_link", "compliance_link")
    assert chain == ["base_link", "link_6", "ft_sensor_link", "compliance_link"]
    ok, _ = check_frames_in_chain(canon, "base_link", "compliance_link",
                                  ["ft_sensor_link", "compliance_link"])
    assert ok


def test_in_chain_detects_missing_and_offchain():
    from aux_frame_manager.frame_source import check_frames_in_chain
    # a sibling branch off link_6 is NOT on the base->ee(=link_6) chain
    frames = [{"name": "ft_sensor_link", "parent": "link_6"}]
    canon, _ = build_canonical_urdf(BASE, frames)
    ok, msg = check_frames_in_chain(canon, "base_link", "link_6",
                                    ["ft_sensor_link"])
    assert not ok and "not on" in msg
    # unknown ee -> no path
    ok2, msg2 = check_frames_in_chain(canon, "base_link", "no_such", [])
    assert not ok2 and "no kinematic path" in msg2
