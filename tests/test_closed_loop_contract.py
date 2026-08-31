"""C4/G500 - the closed-loop contract. Every gate fails closed."""

import pytest

from dbact.contracts import ClosedLoopContract


def passing_report(**overrides) -> dict:
    report = {
        "engine": "penalty",
        "target_distance": 0.50,
        "J": 0.55,
        "displacement": 0.556,
        "efficiency": 0.99,
        "direction_error_deg": 8.1,
        "max_cross_track": 0.06,
        "max_strict_coverage": 0.94,
        "rotation_deg": -1.2,
        "final_cargo_speed": 0.0,
        "holding": True,
        "first_detection_frame": 1,
        "contact_ready_frame": 96,
        "transport_frame": 112,
        "reached_frame": 340,
        "solver_fallbacks": 0,
        "solver_infeasible": 0,
        "barrier_scalings": 0,
        "min_barrier_scale": 1.0,
        "min_inter_agent_distance": 0.3402,
        "d_min": 0.34,
        "min_signed_clearance": 0.1101,
        "max_penetration": 0.049,
        "penetration_budget": 0.071,
    }
    report.update(overrides)
    return report


def reasons(**overrides) -> list[str]:
    return ClosedLoopContract().evaluate(passing_report(**overrides)).reasons


def test_a_complete_closed_loop_run_passes():
    verdict = ClosedLoopContract().evaluate(passing_report())
    assert verdict.success
    assert verdict.reasons == []


@pytest.mark.parametrize(
    "field",
    ["first_detection_frame", "contact_ready_frame", "transport_frame", "reached_frame"],
)
def test_a_stage_that_never_happened_fails_rather_than_being_skipped(field):
    problems = reasons(**{field: None})
    assert len(problems) == 1
    assert "never happened" in problems[0]


@pytest.mark.parametrize(
    "field,late",
    [
        ("first_detection_frame", 101),
        ("contact_ready_frame", 301),
        ("transport_frame", 351),
        ("reached_frame", 501),
    ],
)
def test_each_deadline_is_scored_against_the_shared_budget(field, late):
    problems = reasons(**{field: late})
    assert len(problems) == 1
    assert str(late) in problems[0]


def test_arriving_is_not_enough_the_run_must_also_stop():
    """A team that reaches the target and keeps pushing has not closed the loop."""
    problems = reasons(J=0.95)
    assert any("did not stop" in r for r in problems)
    assert reasons(holding=False) == ["G500: the run did not end in HOLD"]
    assert any("still drifting" in r for r in reasons(final_cargo_speed=0.09))


def test_short_of_the_target_fails_even_with_perfect_direction():
    problems = reasons(J=0.30, efficiency=1.0, direction_error_deg=0.0)
    assert any("< target L" in r for r in problems)


def quality(**overrides) -> list[str]:
    return ClosedLoopContract().evaluate(passing_report(**overrides)).metrics["quality_reasons"]


def test_trajectory_quality_is_reported_rather_than_gated():
    """T3 is a distribution with a rate, not a member of the conjunction. The
    thresholds are unchanged and every breach is still named; what changed is that
    a breach does not make the run invalid. Two of them are also the same quantity:
    max cross-track = J sin(direction error), so an absolute corridor changes
    meaning whenever the task distance does."""
    for field, value, needle in (
        ("efficiency", 0.5, "efficiency"),
        ("direction_error_deg", 41.0, "direction error"),
        ("max_cross_track", 0.4, "cross-track"),
        ("rotation_deg", 44.0, "rotated"),
    ):
        verdict = ClosedLoopContract().evaluate(passing_report(**{field: value}))
        assert verdict.success, f"{field} should not invalidate the run"
        assert not verdict.metrics["quality_ok"]
        assert any(needle in r for r in verdict.metrics["quality_reasons"])


def test_a_run_that_meets_every_quality_target_says_so():
    verdict = ClosedLoopContract().evaluate(passing_report())
    assert verdict.metrics["quality_ok"]
    assert verdict.metrics["quality_reasons"] == []


def test_a_run_that_never_enclosed_cannot_pass_on_displacement_alone():
    assert any("never enclosed" in r for r in reasons(max_strict_coverage=0.31))


def test_safety_is_part_of_the_criterion_not_a_separate_report():
    """T1 stays in the conjunction. A safety invariant is not a distribution."""
    assert any("min inter-agent distance" in r for r in reasons(min_inter_agent_distance=0.21))
    # ... but a deficit of nanometres on an exactly-binding barrier is the QP's
    # last bit of arithmetic, not a collision. Reporting it as one cost a whole
    # round of work chasing a safety regression that had never happened.
    assert ClosedLoopContract().evaluate(passing_report(min_inter_agent_distance=0.34 - 3e-8)).success
    assert any("entered the cargo" in r for r in reasons(min_signed_clearance=-0.02))
    assert any("max penetration" in r for r in reasons(max_penetration=0.12))


def test_solver_provenance_is_a_gate_and_zero_means_zero():
    assert any("solver fallback" in r for r in reasons(solver_fallbacks=1))
    assert any("QP infeasibility" in r for r in reasons(solver_infeasible=1))


def test_a_relaxation_that_is_only_renamed_still_fails_the_run():
    """The scaled-barrier tier keeps the inter-robot rows hard and gives up part
    of the object barrier's decrease rate. That is better than a projection
    fallback and it is still not a clean run, so it is scored rather than
    absorbed."""
    problems = reasons(barrier_scalings=7, min_barrier_scale=0.42)
    assert any("scaled-barrier" in r and "0.420" in r for r in problems)


def test_a_scripted_engine_is_rejected_outright():
    assert any("scripted" in r for r in reasons(engine="scripted"))


def test_a_missing_measurement_fails_rather_than_passes():
    """A criterion that cannot be evaluated cannot be defended."""
    for field in (
        "J",
        "efficiency",
        "direction_error_deg",
        "max_cross_track",
        "max_strict_coverage",
        "solver_fallbacks",
        "barrier_scalings",
        "min_inter_agent_distance",
        "min_signed_clearance",
        "max_penetration",
        "final_cargo_speed",
    ):
        report = passing_report()
        report.pop(field)
        verdict = ClosedLoopContract().evaluate(report)
        assert not verdict.success, f"missing {field!r} should fail closed"


def test_the_gates_are_reported_alongside_the_verdict():
    gates = ClosedLoopContract().as_dict()
    for field in ("detect_by", "contact_ready_by", "transport_by", "reach_by", "efficiency_min"):
        assert field in gates


def test_the_metrics_travel_with_the_verdict_so_a_failure_can_be_diagnosed():
    verdict = ClosedLoopContract().evaluate(passing_report(J=0.1))
    assert verdict.metrics["J"] == pytest.approx(0.1)
    assert verdict.metrics["target_distance"] == pytest.approx(0.5)
