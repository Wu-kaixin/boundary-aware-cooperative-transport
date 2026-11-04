"""A valid unconstrained saved state must remain analyzable after JSON export."""
from dataclasses import asdict
from pathlib import Path

import numpy as np

from dbact.safety.filter import SafetyFilterParams


def test_empty_serialized_matrix_keeps_two_velocity_columns(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    from probe_post_support_first_pusher_loss import counterfactual
    state = dict(agent_id='isolated', position=[0., 0.], nominal_argument=[.1, .05],
        u_nom=[.1, .05], neighbor_positions=[], inputs={}, A=[], b=[], kinds=[],
        params=asdict(SafetyFilterParams()), goal=[1., 0.])
    result = counterfactual(state)
    np.testing.assert_array_equal(result['command'], [.1, .05])
    assert result['A'] == [] and result['b'] == []
    assert result['hard_rows_equal'] is True
    assert result['aggregate_command_min_finite_residual'] is None
    assert result['capability']['classification'] == 'POSITIVE_CAPABILITY'
    assert result['capability']['maximum_goal_velocity'] == .3
