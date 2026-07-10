import math

import pytest

from cartesian_controller_dashboard.dashboard_node import (
    DashboardNode,
    _TUNABLES_BY_KIND,
    _VIRTUAL_MASS_PARAM,
)


class _DashboardStub:
    def _tunables_for_active(self):
        return [(_VIRTUAL_MASS_PARAM, "double")]

    def _active_kind(self):
        return "compliance"

    def _active_controller(self):
        return "cartesian_compliance_controller"

    def _set_param(self, controller, name, kind, value):
        return True, "ok"


def test_virtual_mass_is_available_for_every_controller_kind():
    for tunables in _TUNABLES_BY_KIND.values():
        assert (_VIRTUAL_MASS_PARAM, "double") in tunables


@pytest.mark.parametrize("value", [-1.0, 0.0, math.inf, math.nan, "bad"])
def test_virtual_mass_rejects_invalid_values(value):
    with pytest.raises(RuntimeError):
        DashboardNode.api_set_param(
            _DashboardStub(),
            {"name": _VIRTUAL_MASS_PARAM, "kind": "double", "value": value},
        )


def test_virtual_mass_accepts_positive_finite_value():
    result = DashboardNode.api_set_param(
        _DashboardStub(),
        {"name": _VIRTUAL_MASS_PARAM, "kind": "double", "value": 0.2},
    )

    assert result["ok"]
    assert result["value"] == 0.2
