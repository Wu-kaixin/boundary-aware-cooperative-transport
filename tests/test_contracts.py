import numpy as np
import pytest

from dbact.contracts import (
    ContactSafetyContract,
    ContractViolation,
    CoverageContract,
    DirectionalProgressContract,
    SolverContract,
)


def valid_contact_contract(**overrides) -> ContactSafetyContract:
    kwargs = dict(robot_radius=0.16, cage_offset=0.135, delta_max=0.05, gamma_obj=4.0, rho=0.05, d_min=0.34)
    kwargs.update(overrides)
    return ContactSafetyContract(**kwargs)


# --------------------------------------------------------------------------- #
# C1
# --------------------------------------------------------------------------- #


def test_c1_accepts_a_cage_offset_inside_the_contact_band():
    contract = valid_contact_contract()
    contract.assert_valid()
    assert contract.r_safe == pytest.approx(0.11)
    assert contract.contact_band == (pytest.approx(0.11), pytest.approx(0.16))
    assert contract.barrier_margin > 0.0


def test_c1_rejects_cage_offset_at_or_beyond_the_robot_radius():
    """d_c >= r_robot means zero penetration, so the contact force is identically
    zero and the cargo can never move -- while every safety metric looks perfect."""
    contract = valid_contact_contract(cage_offset=0.26)
    problems = contract.violations()
    assert any("identically zero" in p for p in problems)
    with pytest.raises(ContractViolation):
        contract.assert_valid()


def test_c1_rejects_cage_offset_inside_the_cbf_forbidden_region():
    contract = valid_contact_contract(cage_offset=0.10)
    assert any("chattering or deadlock" in p for p in contract.violations())


def test_c1_rejects_insufficient_barrier_margin():
    """gamma_obj (d_c - r_safe) must dominate rho or robots never reach the ring."""
    contract = valid_contact_contract(gamma_obj=1.0, rho=0.05)
    assert contract.barrier_margin < 0.0
    assert any("barrier margin" in p for p in contract.violations())


def test_c1_rejects_overlapping_robot_discs():
    contract = valid_contact_contract(d_min=0.20)
    assert any("overlap" in p for p in contract.violations())


def test_c1_omits_the_disc_check_when_d_min_is_not_supplied():
    valid_contact_contract(d_min=None).assert_valid()


# --------------------------------------------------------------------------- #
# C2
# --------------------------------------------------------------------------- #


def test_c2_rejects_an_auto_backend():
    """There is deliberately no 'auto': it is what turned a missing solver into a
    silent projection fallback while the write-up claimed a hard QP."""
    with pytest.raises(ContractViolation, match="no 'auto' backend"):
        SolverContract("auto")


@pytest.mark.parametrize("backend", ["qp", "cvxpy", "projection"])
def test_c2_accepts_the_three_explicit_backends(backend):
    assert SolverContract(backend).backend == backend


@pytest.mark.parametrize("backend", ["qp", "cvxpy"])
def test_c2_raises_when_a_requested_solver_fails(backend):
    with pytest.raises(ContractViolation, match="the solve failed"):
        SolverContract(backend).on_solver_failure("solver missing")


def test_c2_projection_backend_tolerates_failure_because_it_was_asked_for():
    SolverContract("projection").on_solver_failure("no solver")  # must not raise


# --------------------------------------------------------------------------- #
# C3
# --------------------------------------------------------------------------- #


def test_c3_rejects_reverse_displacement_that_the_old_criterion_accepted():
    """The pre-refactor test was ||dx|| >= threshold, which a 170-degree reverse
    displacement passes. The signed projection cannot."""
    contract = DirectionalProgressContract(j_min=0.15)
    goal = np.array([1.0, 0.0])
    reverse = 0.6230 * np.array([np.cos(np.radians(170.11)), np.sin(np.radians(170.11))])

    assert np.linalg.norm(reverse) > 0.5  # would have passed a magnitude test
    verdict = contract.evaluate(np.zeros(2), reverse, goal)
    assert not verdict.success
    assert verdict.metrics["directional_progress_J"] < 0.0
    assert any("directional progress" in r for r in verdict.reasons)


def test_c3_accepts_clean_forward_progress():
    contract = DirectionalProgressContract(j_min=0.15)
    verdict = contract.evaluate(
        np.zeros(2),
        np.array([0.5, 0.05]),
        np.array([1.0, 0.0]),
        min_signed_clearance=0.13,
        max_penetration=0.03,
        delta_max=0.05,
        solver_fallbacks=0,
        min_inter_agent_distance=0.35,
        d_min=0.34,
    )
    assert verdict.success and verdict.reasons == []


def test_c3_rejects_large_lateral_drift_above_the_displacement_gate():
    contract = DirectionalProgressContract(j_min=0.15, efficiency_min=0.7, displacement_gate=0.1)
    verdict = contract.evaluate(np.zeros(2), np.array([0.3, 0.5]), np.array([1.0, 0.0]))
    assert not verdict.success
    assert any("efficiency" in r for r in verdict.reasons)


def test_c3_does_not_apply_the_efficiency_gate_at_tiny_displacement():
    """Direction is close to undefined when the cargo has barely moved, so the
    ratio must not be evaluated there."""
    contract = DirectionalProgressContract(j_min=0.0, efficiency_min=0.7, displacement_gate=0.1)
    verdict = contract.evaluate(np.zeros(2), np.array([0.01, 0.05]), np.array([1.0, 0.0]))
    assert not any("efficiency" in r for r in verdict.reasons)


def test_c3_rejects_a_run_that_reached_the_goal_through_the_cargo():
    contract = DirectionalProgressContract(j_min=0.15)
    verdict = contract.evaluate(
        np.zeros(2), np.array([0.5, 0.0]), np.array([1.0, 0.0]), min_signed_clearance=-0.2
    )
    assert not verdict.success
    assert any("entered the cargo" in r for r in verdict.reasons)


def test_c3_rejects_a_run_with_solver_fallbacks():
    contract = DirectionalProgressContract(j_min=0.15)
    verdict = contract.evaluate(np.zeros(2), np.array([0.5, 0.0]), np.array([1.0, 0.0]), solver_fallbacks=3)
    assert any("fallback" in r for r in verdict.reasons)


def test_c3_reports_reasons_as_strings_not_a_bare_boolean():
    contract = DirectionalProgressContract(j_min=0.15)
    verdict = contract.evaluate(np.zeros(2), np.zeros(2), np.array([1.0, 0.0]))
    assert verdict.reasons and all(isinstance(r, str) for r in verdict.reasons)
    assert verdict.as_dict()["reasons"] == verdict.reasons


# --------------------------------------------------------------------------- #
# coverage contract
# --------------------------------------------------------------------------- #


def test_coverage_contract_holds_at_the_boundary_of_the_condition():
    CoverageContract(local_radius=0.8, comm_range=1.6).assert_valid()


def test_coverage_contract_rejects_a_local_radius_over_half_the_comm_range():
    with pytest.raises(ContractViolation, match="no longer exact"):
        CoverageContract(local_radius=0.9, comm_range=1.6).assert_valid()
