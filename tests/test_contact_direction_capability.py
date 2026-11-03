import math
import importlib
from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def maximize_direction(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    return importlib.import_module('probe_post_support_contact_direction').maximize_direction


def test_directional_circle_boundary_optimum_has_primal_and_dual_witness(maximize_direction):
    result = maximize_direction([[0., 1.]], [.2], .35, [1., 0.])
    expected = math.sqrt(.35**2-.2**2)
    assert result['status'] == 'VERIFIED_FEASIBLE'
    assert abs(result['maximum_goal_velocity']-expected) < 1e-7
    assert result['dual_upper_bound'] >= expected-1e-7
    assert result['dual_upper_bound']-result['maximum_goal_velocity'] < 1e-6
    assert result['classification'] == 'POSITIVE_CAPABILITY'


def test_feasible_constraints_can_require_every_velocity_to_reverse(maximize_direction):
    result = maximize_direction([[-1., 0.]], [.1], .35, [1., 0.])
    assert result['status'] == 'VERIFIED_FEASIBLE'
    assert abs(result['maximum_goal_velocity']+.1) < 1e-7
    assert result['dual_upper_bound'] < -1e-7
    assert result['classification'] == 'MUST_REVERSE'
    np.testing.assert_allclose(np.array([[-1., 0.]]) @ result['u'], [.1], atol=1e-7)


def test_infeasible_half_plane_has_a_checked_speed_ball_separator(maximize_direction):
    result = maximize_direction([[-1., 0.]], [.4], .35, [1., 0.])
    assert result['status'] == 'VERIFIED_INFEASIBLE'
    assert result['classification'] == 'INFEASIBLE'
    assert abs(sum(result['separator_multipliers'])-1.) < 1e-7
    assert min(result['separator_multipliers']) >= 0.
    assert result['separator_upper_bound'] < -.049999
