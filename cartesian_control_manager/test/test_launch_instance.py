"""Unit tests for the ``instance_name``-aware launch-file helpers.

The launch file is not a Python package member (it lives under
``launch/`` and is shipped via ``data_files``), so we load it from disk
with :mod:`importlib` to exercise its module-level helpers
(``_defaults`` + ``_FALLBACKS`` + ``_LAUNCH_ONLY_KEYS``).

We deliberately do not spin a :class:`LaunchContext` here -- the
``_build_instance`` OpaqueFunction's CLI-vs-default resolution is
covered by integration testing on the real workspace.  These tests
target the pure helpers so a regression in their contract is caught at
``colcon test`` time.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _load_launch_module():
    """Load ``launch/cartesian_control.launch.py`` as a one-off module.

    Cached on ``sys.modules`` under a synthetic name so repeated calls
    return the same object (and the launch file's import-time
    ``_defaults()`` only runs once).
    """
    name = "_test_cartesian_control_launch"
    if name in sys.modules:
        return sys.modules[name]
    here = Path(__file__).resolve()
    launch_file = (
        here.parent.parent / "launch" / "cartesian_control.launch.py")
    assert launch_file.is_file(), f"launch file missing: {launch_file}"
    spec = importlib.util.spec_from_file_location(name, str(launch_file))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)  # type: ignore[attr-defined]
    return module


def test_instance_name_declared_with_empty_default():
    """``instance_name`` must be a known fallback so the launch arg
    is auto-declared and ``--show-args`` lists it."""
    mod = _load_launch_module()
    assert "instance_name" in mod._FALLBACKS
    assert mod._FALLBACKS["instance_name"] == ""


def test_instance_name_is_launch_only():
    """``instance_name`` is consumed by the launch file (for naming) and
    forwarded as a parameter for observability -- but it must be listed
    in ``_LAUNCH_ONLY_KEYS`` so the per-parameter CLI/section resolution
    loop in ``_build_instance`` doesn't try to round-trip it."""
    mod = _load_launch_module()
    assert "instance_name" in mod._LAUNCH_ONLY_KEYS


def test_defaults_section_argument_falls_back_to_fallbacks():
    """When neither the requested section nor the legacy section exist
    (or the config file itself is missing), ``_defaults`` must return a
    copy of ``_FALLBACKS`` + a descriptive ``FALLBACK ...`` source
    string.  We test this with a guaranteed-missing section name."""
    mod = _load_launch_module()
    d, source = mod._defaults("__guaranteed_missing_section_for_unit_test__")
    assert isinstance(d, dict)
    # Must include every fallback key so the launch loop below has
    # something to read for each declared arg.
    for k in mod._FALLBACKS:
        assert k in d, f"missing fallback for {k!r}"
    assert source.startswith("FALLBACK")


def test_defaults_default_section_is_legacy_name():
    """``_defaults()`` (no arg) must still query the legacy
    ``cartesian_control_manager:`` section so back-compat is preserved
    for callers that don't know about instance naming yet."""
    mod = _load_launch_module()
    # Source string mentions either the legacy section name (loaded) or
    # 'FALLBACK' (no config file).  Either is acceptable; what we
    # forbid is the function silently switching to a different section.
    _, source = mod._defaults()
    assert ("cartesian_control_manager" in source
            or source.startswith("FALLBACK"))
