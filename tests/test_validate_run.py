"""The fail-closed run validator.

Default is rejection: a criterion that cannot be evaluated because a field is
missing must reject the run, not pass it.
"""

import copy
import importlib.util
import sys
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "validate_run", Path(__file__).resolve().parents[1] / "scripts" / "validate_run.py"
)
validate_run = importlib.util.module_from_spec(_spec)
sys.modules["validate_run"] = validate_run
_spec.loader.exec_module(validate_run)
validate = validate_run.validate


def valid_summary() -> dict:
    return {
        "provenance": {"git_sha": "abc1234", "config_hash": "0011223344556677", "seed": 0, "backend": "qp"},
        "engine": "penalty",
        "task_mode": "transport",
        "solver": {"solves": 1000, "fallbacks": 0, "infeasible": 0, "max_slack": 0.0, "statuses": {"optimal": 1000}},
        "contracts": {
            "C1": {
                "robot_radius": 0.16,
                "cage_offset": 0.135,
                "delta_max": 0.05,
                "r_safe": 0.11,
                "barrier_margin": 0.15,
            },
            "d_min": 0.34,
            "delta_max": 0.05,
            "discrete_overshoot": 0.02,
        },
        "min_inter_agent_distance": 0.35,
        "cargoes": {
            "cargo_0": {
                "min_signed_clearance": 0.12,
                "max_penetration": 0.04,
                "success": True,
                "failure_reasons": [],
            }
        },
    }


def test_a_complete_valid_summary_passes():
    assert validate(valid_summary()) == []


def test_missing_provenance_block_rejects():
    s = valid_summary()
    del s["provenance"]
    assert any("provenance" in r for r in validate(s))


@pytest.mark.parametrize("field", ["git_sha", "config_hash", "seed", "backend"])
def test_each_provenance_field_is_required(field):
    s = valid_summary()
    s["provenance"][field] = None
    assert any(field in r for r in validate(s))


def test_unknown_git_sha_rejects():
    s = valid_summary()
    s["provenance"]["git_sha"] = "unknown"
    assert any("git_sha" in r for r in validate(s))


def test_projection_backend_rejects_any_hard_qp_claim():
    s = valid_summary()
    s["provenance"]["backend"] = "projection"
    assert any("no hard-QP claim" in r for r in validate(s))


def test_scripted_engine_rejects():
    s = valid_summary()
    s["engine"] = "scripted"
    assert any("says nothing about transport" in r for r in validate(s))


def test_solver_fallback_rejects():
    s = valid_summary()
    s["solver"]["fallbacks"] = 3
    assert any("fallback" in r for r in validate(s))


def test_infeasibility_rejects():
    s = valid_summary()
    s["solver"]["infeasible"] = 1
    assert any("infeasibility" in r for r in validate(s))


def test_nonzero_slack_rejects():
    s = valid_summary()
    s["solver"]["max_slack"] = 1e-9
    assert any("hard QP" in r for r in validate(s))


def test_missing_solver_block_rejects():
    s = valid_summary()
    del s["solver"]
    assert any("solver" in r for r in validate(s))


def test_c1_violation_in_the_recorded_contract_rejects():
    s = valid_summary()
    s["contracts"]["C1"]["cage_offset"] = 0.26
    assert any("C1 violated" in r for r in validate(s))


def test_nonpositive_barrier_margin_rejects():
    s = valid_summary()
    s["contracts"]["C1"]["barrier_margin"] = 0.0
    assert any("barrier margin" in r for r in validate(s))


def test_inter_robot_violation_rejects():
    s = valid_summary()
    s["min_inter_agent_distance"] = 0.30
    assert any("inter-robot safety violated" in r for r in validate(s))


def test_negative_clearance_rejects():
    s = valid_summary()
    s["cargoes"]["cargo_0"]["min_signed_clearance"] = -0.01
    assert any("entered the cargo" in r for r in validate(s))


def test_penetration_beyond_budget_rejects():
    s = valid_summary()
    s["cargoes"]["cargo_0"]["max_penetration"] = 0.09
    assert any("max penetration" in r for r in validate(s))


def test_penetration_within_the_stated_discrete_overshoot_passes():
    """The barrier holds in continuous time; a fixed-step integrator can overshoot
    by one step of relative motion, and that allowance is stated rather than
    absorbed into delta_max."""
    s = valid_summary()
    s["cargoes"]["cargo_0"]["max_penetration"] = 0.055
    assert validate(s) == []


def test_unsuccessful_cargo_reports_its_own_reasons():
    s = valid_summary()
    s["cargoes"]["cargo_0"]["success"] = False
    s["cargoes"]["cargo_0"]["failure_reasons"] = ["C3: directional progress J=0.0621 m < J_min=0.1500 m"]
    reasons = validate(s)
    assert any("directional progress" in r for r in reasons)


def test_absent_success_flag_rejects_rather_than_passes():
    s = valid_summary()
    del s["cargoes"]["cargo_0"]["success"]
    del s["cargoes"]["cargo_0"]["failure_reasons"]
    assert any("success flag absent" in r for r in validate(s))


def test_no_cargo_results_rejects():
    s = valid_summary()
    s["cargoes"] = {}
    assert any("no cargo results" in r for r in validate(s))


def test_v3_frame_discovery_and_phase_deadlines_are_revalidated():
    s = valid_summary()
    s["steps"] = 500
    s["contracts"].update(
        {
            "frame_budget": 500,
            "require_initially_unobserved": True,
            "phase_deadlines": {
                "first_detection": 150,
                "first_enclosure": 300,
                "first_transport": 350,
                "first_hold": 500,
            },
        }
    )
    s["cargoes"]["cargo_0"].update(
        {
            "initial_detection_count": 0,
            "phase_frames": {
                "first_detection": 52,
                "first_enclosure": 168,
                "first_transport": 172,
                "first_hold": 259,
            },
        }
    )
    assert validate(s) == []

    s["cargoes"]["cargo_0"]["initial_detection_count"] = 1
    s["cargoes"]["cargo_0"]["phase_frames"]["first_detection"] = 151
    s["steps"] = 499
    reasons = validate(s)
    assert any("frame budget" in reason for reason in reasons)
    assert any("frame 0" in reason for reason in reasons)
    assert any("first_detection=151" in reason for reason in reasons)


def test_required_guarantee_certificate_and_runtime_map_witness_are_fail_closed():
    s = valid_summary()
    s["contracts"]["require_guarantee_certificate"] = True
    s["cargoes"]["cargo_0"]["guarantee_certificate"] = {
        "eligible": True,
        "failure_reasons": [],
        "checks": {"simple_polygon": {"passed": True}},
        "mapping": {"required_max_boundary_gap": 0.20},
        "runtime_map_witness": {"max_boundary_gap": 0.10},
        "runtime_eligible": True,
        "runtime_failure_reasons": [],
    }
    assert validate(s) == []

    s["cargoes"]["cargo_0"]["guarantee_certificate"]["runtime_map_witness"][
        "max_boundary_gap"
    ] = 0.21
    assert any("not epsilon-dense" in reason for reason in validate(s))


def test_an_empty_summary_rejects():
    assert validate({}) != []
