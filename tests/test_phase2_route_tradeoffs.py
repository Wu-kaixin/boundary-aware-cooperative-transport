"""Analytic checks for reversal diagnosis without new physical episodes."""

from pathlib import Path

import numpy as np
import pytest
from conftest import load_script_module


@pytest.fixture(scope="module")
def analysis():
    with pytest.MonkeyPatch.context() as patch:
        patch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
        yield load_script_module("analyze_phase2_route_tradeoffs")


def test_nonreverse_projection_distinguishes_selection_from_feasibility(analysis):
    # Closest nominal projection onto y-x>=1 is (-.45,.55), yet (0,1)
    # is the unique closest nonreverse point. This is an exact analytic example.
    answer = analysis.nonreverse_projection(
        [[-1.0, 1.0]], [1.0], 3.0, [1.0, 0.0], [0.1, 0.0]
    )
    assert answer["status"] == "VERIFIED_FEASIBLE"
    np.testing.assert_allclose(answer["u"], [0.0, 1.0], atol=1e-7)
    assert answer["objective"] == pytest.approx(1.01, abs=1e-7)


def test_reverse_constraint_has_known_minimum_object_rhs_change(analysis):
    # x<=-.2 cannot coexist with x>=0 until the object RHS decreases by .2.
    answer = analysis.nonreverse_projection(
        [[-1.0, 0.0]], [0.2], 1.0, [1.0, 0.0], [0.3, 0.0]
    )
    assert answer["status"] == "VERIFIED_INFEASIBLE"
    relaxed = analysis.minimum_object_relaxation(
        [[-1.0, 0.0]], [0.2], [True], 1.0, [1.0, 0.0]
    )
    assert relaxed["status"] == "VERIFIED_FEASIBLE"
    assert relaxed["epsilon"] == pytest.approx(0.2, abs=1e-7)
    assert relaxed["dual_lower_bound"] == pytest.approx(0.2, abs=1e-7)


def test_diagnostic_never_relaxes_a_hard_row(analysis):
    answer = analysis.minimum_object_relaxation(
        [[-1.0, 0.0]], [0.2], [False], 1.0, [1.0, 0.0]
    )
    assert answer["status"] == "NO_FINITE_OBJECT_RELAXATION"


def test_empty_rows_preserve_nonreverse_nominal(analysis):
    answer = analysis.nonreverse_projection([], [], 1.0, [1.0, 0.0], [0.2, 0.1])
    assert answer["status"] == "VERIFIED_FEASIBLE"
    np.testing.assert_allclose(answer["u"], [0.2, 0.1], atol=1e-7)
    relaxed = analysis.minimum_object_relaxation([], [], [], 1.0, [1.0, 0.0])
    assert relaxed["status"] == "VERIFIED_FEASIBLE"
    assert relaxed["epsilon"] == pytest.approx(0.0, abs=1e-7)
