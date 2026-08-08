"""The exact planar QP solver, cross-checked against cvxpy.

The paper claims the safety filter is a hard QP with no slack variables, so the
solver has to be exactly that and not an iteration that happens to converge. The
cross-check is what makes the claim checkable by someone else.
"""

import numpy as np
import pytest

from dbact.qp2d import solve_min_norm_2d, solve_min_norm_2d_cvxpy

cvxpy = pytest.importorskip("cvxpy", reason="cvxpy is the independent cross-check backend")


def random_problem(rng, rows):
    u_nom = rng.normal(scale=0.4, size=2)
    A = rng.normal(size=(rows, 2))
    b = rng.normal(scale=0.3, size=rows) - 0.4  # bias towards feasible sets
    return u_nom, A, b, 0.3


def test_unconstrained_optimum_is_returned_unchanged():
    solution = solve_min_norm_2d(np.array([0.1, -0.2]), np.empty((0, 2)), np.empty(0), 0.3)
    assert solution.feasible
    assert solution.u == pytest.approx([0.1, -0.2])


def test_speed_cap_is_respected_and_radial():
    solution = solve_min_norm_2d(np.array([3.0, 4.0]), np.empty((0, 2)), np.empty(0), 0.5)
    assert np.linalg.norm(solution.u) == pytest.approx(0.5)
    assert solution.u == pytest.approx([0.3, 0.4])


def test_single_half_plane_projection_is_perpendicular():
    # require u_x >= 0.1 starting from u_x = -0.2
    solution = solve_min_norm_2d(np.array([-0.2, 0.05]), np.array([[1.0, 0.0]]), np.array([0.1]), 1.0)
    assert solution.feasible
    assert solution.u == pytest.approx([0.1, 0.05])


@pytest.mark.parametrize("rows", [1, 2, 3, 6, 12])
def test_matches_cvxpy_on_random_problems(rows):
    rng = np.random.default_rng(20260808 + rows)
    compared = 0
    for _ in range(120):
        u_nom, A, b, v_max = random_problem(rng, rows)
        exact = solve_min_norm_2d(u_nom, A, b, v_max)
        reference = solve_min_norm_2d_cvxpy(u_nom, A, b, v_max)
        assert exact.feasible == reference.feasible, "feasibility verdicts disagree"
        if not exact.feasible:
            continue
        compared += 1
        cost_exact = float(np.sum((exact.u - u_nom) ** 2))
        cost_reference = float(np.sum((reference.u - u_nom) ** 2))
        # The exact solver may only ever be better; allow solver tolerance.
        assert cost_exact <= cost_reference + 1e-6
        assert np.linalg.norm(exact.u) <= v_max + 1e-7
        assert np.all(A @ exact.u >= b - 1e-7)
    assert compared > 20, "test data degenerated to almost-all-infeasible problems"


def test_detects_genuine_infeasibility():
    # u_x >= 0.5 and u_x <= -0.5 simultaneously
    A = np.array([[1.0, 0.0], [-1.0, 0.0]])
    b = np.array([0.5, 0.5])
    assert not solve_min_norm_2d(np.zeros(2), A, b, 1.0).feasible
    assert not solve_min_norm_2d_cvxpy(np.zeros(2), A, b, 1.0).feasible


def test_detects_infeasibility_caused_by_the_speed_cap_alone():
    # u_x >= 2 is unreachable within ||u|| <= 1
    solution = solve_min_norm_2d(np.zeros(2), np.array([[1.0, 0.0]]), np.array([2.0]), 1.0)
    assert not solution.feasible


def test_zero_input_is_optimal_when_it_is_feasible_and_nominal():
    A = np.array([[1.0, 0.0], [0.0, 1.0]])
    b = np.array([-0.5, -0.5])
    solution = solve_min_norm_2d(np.zeros(2), A, b, 0.3)
    assert solution.feasible
    assert solution.u == pytest.approx([0.0, 0.0])


def test_degenerate_zero_rows_are_ignored():
    A = np.array([[0.0, 0.0], [1.0, 0.0]])
    b = np.array([-1.0, 0.05])
    solution = solve_min_norm_2d(np.array([0.0, 0.1]), A, b, 0.3)
    assert solution.feasible
    assert solution.u[0] == pytest.approx(0.05)
