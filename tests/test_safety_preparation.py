"""Preparation is unexecuted; only the selected command counts as a solve."""
from dataclasses import asdict

import numpy as np
import pytest

from dbact.safety.filter import SafetyFilter, SafetyFilterParams


def conflict():
    safety = SafetyFilter(SafetyFilterParams(d_min=.28, r_safe=.195,
        gamma_obj=8., rho=.02, max_speed=.35, aggregate_frame_consistent=True))
    inputs = dict(position=np.array([0., .19]), nominal_velocity=np.zeros(2),
        neighbor_positions=[np.array([0., .47])],
        boundary_points=np.array([[-.02, 0.], [.02, 0.]]),
        boundary_normals=np.array([[0., 1.], [0., 1.]]))
    return safety, inputs


def prepare(safety, inputs):
    assert hasattr(safety, 'prepare_velocity'), 'Safety rows must be preparable before executing fallback'
    return safety.prepare_velocity(**inputs)


def test_infeasible_proposal_does_not_record_an_executed_scaled_fallback():
    safety, inputs = conflict()
    before = asdict(safety.stats)
    problem = prepare(safety, inputs)
    proposal = safety._attempt(problem.u_nom, problem.A, problem.b)
    assert not proposal.feasible
    assert asdict(safety.stats) == before
    result = safety.finish_velocity(problem)
    assert result.status == 'scaled_barrier' and result.barrier_scale == 0.
    assert safety.stats.solves == safety.stats.barrier_scalings == 1
    assert safety.stats.zero_input_feasible_failures == 1
    np.testing.assert_allclose(result.velocity, [0., 0.], atol=1e-12)


def test_finalization_preserves_hard_rows_after_other_problems_are_prepared():
    safety, inputs = conflict()
    problem = prepare(safety, inputs)
    other = prepare(safety, dict(position=[2., 2.], nominal_velocity=[.1, 0.]))
    result = safety.finish_velocity(problem)
    assert result.status == 'scaled_barrier'
    assert result.agent_residual_min >= -1e-7 and result.agent_rows == 1
    second = safety.finish_velocity(other)
    np.testing.assert_array_equal(second.velocity, [.1, 0.])
    assert safety.stats.solves == 2 and safety.stats.barrier_scalings == 1


def test_prepared_problem_cannot_be_counted_twice_or_use_changed_contract():
    safety, inputs = conflict()
    problem = prepare(safety, inputs)
    safety.params.rho = .03
    with pytest.raises(ValueError, match='parameters'):
        safety.finish_velocity(problem)
    assert safety.stats.solves == 0
    safety.params.rho = .02
    safety.finish_velocity(problem)
    with pytest.raises(ValueError, match='already'):
        safety.finish_velocity(problem)
    assert safety.stats.solves == 1


def test_preparation_copies_nominal_input_and_retains_full_original_rows():
    safety, inputs = conflict()
    problem = prepare(safety, inputs)
    inputs['nominal_velocity'][:] = 99.
    np.testing.assert_array_equal(problem.u_nom, [0., 0.])
    assert problem.agent_count == 1 and problem.wall_count == 0
    np.testing.assert_allclose(problem.A[0], [0., -.56], atol=1e-14)
    assert problem.b[1] > problem.b_no_margin[1] > 0.
    result = safety.finish_velocity(problem)
    np.testing.assert_array_equal(result.A_rows, problem.A)
    np.testing.assert_array_equal(result.b_original, problem.b_original)
