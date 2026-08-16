"""The ISSf margin re-derived against a moving boundary.

`docs/CLOSED_LOOP_V2.md` §3.1 rewrote the S1 *criterion* and said plainly that it did not
re-derive the constant. §6.3 and §7b.2 then measured the premise that constant rests on and
found it violated by 60.4% of cells, and by 36.7% with a **noiseless** sensor. These tests
pin the re-derivation.
"""

from __future__ import annotations

import math

import pytest

from dbact.guarantees import issf_margin_budget
from dbact_sim.scenarios import controller_params_from_config, load_yaml

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"


def baseline_budget(**overrides):
    p = controller_params_from_config(load_yaml(BASE_CONFIG))
    kwargs = dict(
        rho=p.rho,
        tangential_window=p.object_row_window,
        omega_max=p.max_object_yaw_rate,
        recovery_fraction=p.recovery_fraction,
        max_speed=p.max_speed,
    )
    kwargs.update(overrides)
    return issf_margin_budget(**kwargs)


def test_the_dropped_term_is_omega_times_the_tangential_window():
    """``|(dn/dt)^T (p - b)| <= omega_max * W``, which is why the window is load-bearing.

    For a rigid body ``dn/dt = omega R90 n``, so the dropped term is omega times the robot's
    tangential offset -- and the window is exactly the filter that bounds that offset.
    Without it the dropped term is unbounded and no rho exists at all.
    """
    budget = issf_margin_budget(
        rho=0.02, tangential_window=0.28, omega_max=0.80,
        recovery_fraction=0.6, max_speed=0.35,
    )
    assert budget["dropped_normal_rate_term"] == pytest.approx(0.224)
    assert budget["reachable_cap"] == pytest.approx(0.21)


def test_the_configured_margin_is_unsatisfiable_at_the_declared_yaw_bound():
    """The rotation term alone exceeds what a speed-limited robot can deliver.

    0.224 m/s required against a 0.210 m/s reachability cap, before the velocity error is
    even added. No value of rho works -- this is a statement about the actuator, not about
    tuning, and it is the reason the check is reported rather than gated.
    """
    budget = baseline_budget()
    assert budget["dropped_normal_rate_term"] > budget["reachable_cap"]
    assert budget["within_reachable_cap"] is False
    assert budget["sufficient"] is False
    assert budget["satisfiable"] is False
    assert budget["dropped_normal_rate_term"] / budget["rho_configured"] == pytest.approx(11.2, rel=1e-3)


def test_the_configured_margin_is_defensible_for_a_near_stationary_object():
    """rho = 0.02 covers rotation up to 4.09 deg/s, and the baseline turns 1400x slower.

    The point of reporting the regime rather than the verdict: the configured value is
    correct *for this object* and wrong as a general bound, and only stating the condition
    distinguishes the two.
    """
    budget = baseline_budget()
    covered = budget["omega_max_covered_by_rho"]
    assert covered == pytest.approx(0.02 / 0.28)
    assert math.degrees(covered) == pytest.approx(4.09, abs=0.01)

    # The measured baseline rotation is at most 0.086 deg over a ~30 s episode.
    measured_rate_deg_s = 0.086 / 30.0
    assert measured_rate_deg_s < math.degrees(covered) / 100.0

    # And with that actual rate the statement is satisfiable again.
    slow = baseline_budget(omega_max=math.radians(measured_rate_deg_s))
    assert slow["satisfiable"] is True


def test_the_two_disturbances_are_kept_apart():
    """The double-booking, made explicit.

    ``bounded_perception_and_motion_error`` checks ``velocity_error <= rho``, but
    ``velocity_error`` is the error in the *kept* term and rho was sized for the *dropped*
    one. The requirement is their sum, and this asserts the function adds them rather than
    letting one budget stand for both.
    """
    without = issf_margin_budget(
        rho=0.30, tangential_window=0.28, omega_max=0.10,
        recovery_fraction=0.6, max_speed=1.0,
    )
    with_error = issf_margin_budget(
        rho=0.30, tangential_window=0.28, omega_max=0.10,
        recovery_fraction=0.6, max_speed=1.0, velocity_error=0.05,
    )
    assert without["required_rho"] == pytest.approx(0.028)
    assert with_error["required_rho"] == pytest.approx(0.078)
    assert with_error["required_rho"] - without["required_rho"] == pytest.approx(0.05)
    assert without["satisfiable"] and with_error["satisfiable"]


def test_the_measured_velocity_error_makes_it_far_worse():
    """With the measured e_v the requirement is 0.87 m/s against a 0.21 m/s cap.

    The measured barrier-visible velocity error is ~0.65 m/s at its maximum on the baseline,
    so the dominant term is not the rotation at all -- it is the estimation error in the term
    rho was never budgeted for.
    """
    budget = baseline_budget(velocity_error=0.648)
    assert budget["required_rho"] == pytest.approx(0.872, abs=1e-3)
    assert budget["required_rho"] > 4.0 * budget["reachable_cap"]
    assert budget["satisfiable"] is False


def test_satisfiable_requires_both_bounds():
    """A margin can be large enough and undeliverable, or deliverable and too small."""
    too_small = issf_margin_budget(
        rho=0.01, tangential_window=0.28, omega_max=0.10,
        recovery_fraction=0.6, max_speed=1.0,
    )
    assert too_small["within_reachable_cap"] is True
    assert too_small["sufficient"] is False
    assert too_small["satisfiable"] is False

    undeliverable = issf_margin_budget(
        rho=5.0, tangential_window=0.28, omega_max=5.0,
        recovery_fraction=0.6, max_speed=0.35,
    )
    assert undeliverable["sufficient"] is True
    assert undeliverable["within_reachable_cap"] is False
    assert undeliverable["satisfiable"] is False


def test_the_certificate_reports_the_budget_without_gating_on_it():
    """Surfaced beside the checks, and deliberately not in ``domain_eligible``.

    Gating on it would make every run on this branch ineligible at a stroke. That is a true
    statement, and it is a different experiment from the ones already committed, so the
    decomposition is reported and the eligibility figures stay comparable.
    """
    from conftest import load_script_module

    import numpy as np

    from dbact.cargo import Cargo
    from dbact.guarantees import build_admissibility_certificate
    from dbact_sim.scenarios import (
        build_agents,
        contact_params_from_config,
        domain_from_config,
    )

    config = load_yaml("configs/sim/v2/shape_matrix.yaml")
    config, _ = load_script_module("run_arbitrary_shape_monte_carlo").build_case_config(
        config, "l_shape", 0, 0.40
    )
    cargo = Cargo("cargo_0", np.asarray(config["cargoes"][0]["vertices"], dtype=float),
                  surface_density=2.0)
    cert = build_admissibility_certificate(
        cargo=cargo,
        agents=build_agents(config, seed=0),
        domain=domain_from_config(config),
        goal_direction=np.array([1.0, 0.0]),
        target_distance=0.8,
        config=config,
        controller=controller_params_from_config(config),
        contact=contact_params_from_config(config),
        dt=float(config["dt"]),
    )

    budget = cert["issf_margin"]
    assert budget["satisfiable"] is False
    assert "issf_margin" not in cert["checks"]
    assert "issf_margin" not in cert["domain_failure_reasons"]
    # The instance is still domain-eligible, so the committed eligibility figures stand.
    assert cert["domain_eligible"] is True
