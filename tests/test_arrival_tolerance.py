"""The arrival gate, re-examined.

The legacy test is ``L <= J <= progress_max_ratio * L``: zero tolerance below the target and
40% above. That asymmetry is the biased estimator written into the acceptance criterion --
the contract's own docstring says the estimate is biased low "so the cargo always travels
somewhat past the target". Correcting the bias makes the band unsatisfiable from below.

``arrival_tolerance`` replaces the pair with a symmetric band around the target, taken from
``TransportTask.tolerance``, which the scenario already declares and the gate never consulted.
``None`` keeps the legacy behaviour exactly.
"""

from __future__ import annotations

import pytest

from dbact.contracts import ClosedLoopContract


def report(j: float, target: float = 1.0) -> dict:
    """A report that passes every gate except, possibly, the arrival one."""
    return {
        "J": j,
        "target_distance": target,
        "efficiency": 0.99,
        "direction_error_deg": 1.0,
        "max_cross_track": 0.01,
        "max_strict_coverage": 0.99,
        "final_speed": 0.0,
        "rotation_deg": 0.0,
        "detect_frame": 1,
        "contact_ready_frame": 2,
        "transport_frame": 3,
        "reached_frame": 4,
        "hold_frame": 5,
        "barrier_scalings": 0,
        "min_inter_agent_distance": 1.0,
        "d_min": 0.3,
        "solver_fallbacks": 0,
        "solver_infeasible": 0,
        "max_penetration": 0.0,
        "penetration_budget": 0.1,
        "min_signed_clearance": 0.1,
        "agents_inside": 0,
    }


def arrival_reasons(contract: ClosedLoopContract, j: float, target: float = 1.0) -> list[str]:
    verdict = contract.evaluate(report(j, target))
    return [r for r in verdict.reasons if "G500: J=" in r]


# --------------------------------------------------------------------------- #
# the legacy band, and why it is asymmetric
# --------------------------------------------------------------------------- #


def test_the_legacy_band_has_zero_tolerance_below_the_target():
    """One millimetre short fails; 39% past passes. That is the finding.

    No stopping controller can guarantee landing at or above a target without deliberately
    overshooting, so the legacy band is only satisfiable by a loop that overshoots -- which
    is exactly what the biased estimate produced.
    """
    legacy = ClosedLoopContract()
    assert legacy.arrival_tolerance is None
    assert arrival_reasons(legacy, 0.999) != []
    assert arrival_reasons(legacy, 1.000) == []
    assert arrival_reasons(legacy, 1.390) == []
    assert arrival_reasons(legacy, 1.401) != []


def test_the_legacy_band_is_unchanged_by_the_new_field():
    """Default ``None`` means every committed result keeps its verdict."""
    legacy = ClosedLoopContract()
    for j in (0.5, 0.99, 1.0, 1.2, 1.4, 1.5):
        explicit = ClosedLoopContract(arrival_tolerance=None)
        assert arrival_reasons(legacy, j) == arrival_reasons(explicit, j), j


# --------------------------------------------------------------------------- #
# the symmetric band
# --------------------------------------------------------------------------- #


def test_the_symmetric_band_treats_short_and_long_alike():
    """Stopping short and overshooting are the same failure of the same loop."""
    contract = ClosedLoopContract(arrival_tolerance=0.12)
    assert arrival_reasons(contract, 1.00) == []
    assert arrival_reasons(contract, 0.89) == []
    assert arrival_reasons(contract, 1.11) == []
    assert arrival_reasons(contract, 0.87) != []
    assert arrival_reasons(contract, 1.13) != []


def test_the_symmetric_band_names_the_side_and_the_amount():
    """Every failure carries the number that caused it, as the contract requires."""
    contract = ClosedLoopContract(arrival_tolerance=0.12)
    short = arrival_reasons(contract, 0.80)[0]
    assert "short of" in short and "0.2000" in short
    over = arrival_reasons(contract, 1.30)[0]
    assert "past" in over and "0.3000" in over
    assert "0.1200" in over


def test_the_symmetric_band_rejects_the_overshoot_the_legacy_band_accepted():
    """The band is tighter above, not looser -- this is not a relaxation.

    The legacy gate passed a 39% overshoot. On a 1 m task the symmetric 0.12 m band rejects
    anything past 12%, so adopting it *removes* passes at the top of the range as well as
    adding them at the bottom.
    """
    legacy = ClosedLoopContract()
    symmetric = ClosedLoopContract(arrival_tolerance=0.12)
    assert arrival_reasons(legacy, 1.35) == []
    assert arrival_reasons(symmetric, 1.35) != []
    # And it admits the case the legacy band could not: stopping just short.
    assert arrival_reasons(legacy, 0.95) != []
    assert arrival_reasons(symmetric, 0.95) == []


def test_the_tolerance_is_absolute_not_relative():
    """It comes from ``TransportTask.tolerance``, which is declared in metres.

    A relative band would re-introduce the distance dependence that made the cross-track gate
    tighten with task length (see CLOSED_LOOP_V2.md 7a). Arrival accuracy is a property of
    the stopping loop, not of how far it travelled.
    """
    contract = ClosedLoopContract(arrival_tolerance=0.12)
    assert arrival_reasons(contract, 0.90, target=1.0) == []
    assert arrival_reasons(contract, 2.40, target=2.5) == []
    assert arrival_reasons(contract, 0.85, target=1.0) != []
    assert arrival_reasons(contract, 2.35, target=2.5) != []


def test_the_tolerance_is_reported():
    assert ClosedLoopContract(arrival_tolerance=0.12).as_dict()["arrival_tolerance"] == 0.12
    assert ClosedLoopContract().as_dict()["arrival_tolerance"] is None


def test_j_min_is_a_different_gate_and_is_not_the_one_that_moved():
    """``evaluation.j_min`` asks whether the cargo moved at all, not whether it arrived.

    On the baseline the minimum J over twelve seeds is 1.133 m against a 0.15 m floor --
    7.6x of headroom -- so it cannot be what changed when the estimator was corrected. It is
    left alone deliberately; conflating it with the arrival band would be re-examining the
    wrong gate.
    """
    import numpy as np

    from dbact.contracts import DirectionalProgressContract

    c3 = DirectionalProgressContract()
    assert c3.j_min == pytest.approx(0.15)
    goal = np.array([1.0, 0.0])

    moved = c3.evaluate(np.zeros(2), np.array([1.133, 0.0]), goal)
    assert not any("J_min" in r for r in moved.reasons)

    barely = c3.evaluate(np.zeros(2), np.array([0.05, 0.0]), goal)
    assert any("J_min" in r for r in barely.reasons)
