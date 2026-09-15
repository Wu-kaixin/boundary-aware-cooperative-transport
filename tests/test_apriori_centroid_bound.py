"""A priori centroid-error bound: constants, labels, and data-flow gates."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from dbact.apriori_centroid_bound import (
    assemble_prior,
    config_fingerprint,
    controller_grid_spacing,
    geometric_J_bound,
    kernel_lipschitz,
    moment_form_E_bound,
    oracle_midpoint_l1_bound,
    overlay_grid_resolution,
    plane_mass_upper,
    prior_J_bound,
    sampling_period_a,
    step_in_k0,
)


def _minimal_cfg():
    return {
        "dt": 0.05,
        "domain": {"xmin": 0.0, "xmax": 8.0, "ymin": 0.0, "ymax": 8.0},
        "agents": {"count": 16},
        "cargoes": [{"shape": "rectangle"}],
        "controller": {
            "grid_resolution": 20,
            "local_radius": 0.8,
            "kp_cage": 0.9,
            "max_speed": 0.35,
            "sigma": 0.2,
            "base_density": 0.001,
            "cage_offset": 0.105,
            "theorem_oracle_spacing": 0.04,
            "density_mode": "offset",
        },
    }


def test_sampling_period_positive_for_paper_defaults():
    a = sampling_period_a(0.9, 0.05)
    assert a == pytest.approx(2.0 / 0.9 - 0.05)
    assert a > 0
    with pytest.raises(ValueError):
        sampling_period_a(0.9, 3.0)


def test_controller_grid_spacing_shrinks_with_resolution():
    h20 = controller_grid_spacing(0.8, 20)
    h40 = controller_grid_spacing(0.8, 40)
    h80 = controller_grid_spacing(0.8, 80)
    assert h20 == pytest.approx(1.6 / 19)
    assert h40 < h20 < 0.1
    assert h80 < h40
    assert h80 == pytest.approx(1.6 / 79)


def test_oracle_l1_grows_with_spacing_independent_of_grid():
    tight = oracle_midpoint_l1_bound(7.2, 0.02, 0.2)
    loose = oracle_midpoint_l1_bound(7.2, 0.04, 0.2)
    assert tight.value > 0
    assert loose.value > tight.value
    assert tight.decreases_with_oracle_spacing
    assert not tight.decreases_with_controller_grid
    assert tight.forms_floor


def test_moment_form_avoids_tiny_mass_lower_bound():
    m_minus = 1.5e-5
    e_from_m_minus = (0.01**2) / m_minus
    bound = moment_form_E_bound(0.8, sum_abs_dm=0.01, sum_abs_dmu=0.008)
    assert bound == pytest.approx(2 * 0.8 * (0.008 + 0.8 * 0.01))
    assert bound < 0.05
    assert bound < e_from_m_minus / 10.0


def test_geometric_j_uses_total_mass_not_cellwise_ceiling():
    geom = geometric_J_bound(mass=1.87, mass_error=1e-10, umax=0.35)
    cellwise = 16 * 14.48 * 0.35**2
    assert geom["B_J_geom"] == pytest.approx((1.87 + 1e-10) * 0.35**2)
    assert geom["B_J_geom"] < 1.0
    assert geom["B_J_geom"] < cellwise / 10.0
    assert geom["not_used"] == "N * m_plus * u_max^2"


def test_prior_j_splits_h0_and_e_terms():
    out = prior_J_bound(b_h0=1.0, b_e=0.001, a=2.1722222222222225, k_steps=600, dt=0.05)
    assert out["B_J_prior"] == pytest.approx(out["term_2BH0_over_aKD"] + out["term_4BE_over_a2"])
    assert out["term_2BH0_over_aKD"] < 0.04
    assert out["status"] == "rigorous_on_K0"
    num = prior_J_bound(
        b_h0=1.0,
        b_e=0.001,
        a=2.1722222222222225,
        k_steps=600,
        dt=0.05,
        h0_status="numerical_a_priori",
        e_status="rigorous",
    )
    assert num["status"] == "numerical_a_priori"
    failed = prior_J_bound(
        b_h0=1.0,
        b_e=0.001,
        a=2.1722222222222225,
        k_steps=600,
        dt=0.05,
        h0_status="rigorous",
        e_status="rigorous",
        k0_holds=False,
    )
    assert failed["status"] == "constants_only"


def test_plane_mass_upper_beats_floor_only():
    m = plane_mass_upper(0.001, 64.0, 0.2, 7.2)
    assert m > 0.001 * 64
    assert m < 3.0


def test_overlay_grid_does_not_mutate_source():
    cfg = _minimal_cfg()
    out = overlay_grid_resolution(cfg, 80)
    assert cfg["controller"]["grid_resolution"] == 20
    assert out["controller"]["grid_resolution"] == 80
    assert config_fingerprint(cfg) != config_fingerprint(out)


def test_kernel_lipschitz_matches_wolfram():
    assert kernel_lipschitz(0.2) == pytest.approx(1.0 / (0.2 * np.sqrt(np.e)))


def test_k0_fail_closed_without_rho_flag():
    rec = {
        "solver_status": ["optimal"] * 2,
        "zero_input_feasible_with_rho": [True, False],
    }
    ok, bad = step_in_k0(rec, n_agents=2, a=2.17)
    assert not ok
    assert bad == [1]
    rec2 = {"solver_status": ["optimal", "optimal"]}
    ok2, bad2 = step_in_k0(rec2, n_agents=2, a=2.17)
    assert not ok2
    assert bad2 == [0, 1]


def test_fresh_cli_does_not_default_to_legacy_windows_artifacts():
    path = Path("scripts/run_multi_shape_static.py")
    if not path.exists():
        pytest.skip("run_multi_shape_static.py not present on this branch")
    text = path.read_text(encoding="utf-8")
    if "--fresh" not in text:
        pytest.skip("legacy multi-shape CLI flags not restored on this branch")
    assert "reuse-l-shape-dir" in text


def test_finalize_historical_assert_is_opt_in():
    path = Path("scripts/finalize_advisor_delivery.py")
    if not path.exists():
        pytest.skip("finalize_advisor_delivery.py not present")
    text = path.read_text(encoding="utf-8")
    if "historical-c-shape-seed5-assert" not in text:
        pytest.skip("historical C-shape assert flag not restored on this branch")
    assert "args.historical_c_shape_seed5_assert" in text


def test_assemble_prior_does_not_consume_trajectory_e():
    class StubObserver:
        def density(self, q, vertices):
            q = np.asarray(q, dtype=float).reshape(-1, 2)
            return np.full(len(q), 0.001)

        def total_mass(self, vertices):
            return 0.08

        def evaluate(self, positions, vertices):
            p = np.asarray(positions, dtype=float).reshape(-1, 2)
            mass = np.full(len(p), 0.005)
            return {
                "H": 1.0,
                "mass": mass,
                "centroid": p.copy(),
                "gradient": np.zeros_like(p),
            }

    cfg = yaml.safe_load(
        Path("configs/sim/theorem/static_rectangle_n16_oracle.yaml").read_text(encoding="utf-8")
    )
    vertices = np.array([[3.2, 3.4], [4.8, 3.4], [4.8, 4.6], [3.2, 4.6]], dtype=float)
    p0 = np.column_stack([np.linspace(3.0, 5.0, 16), np.full(16, 5.2)])
    prior = assemble_prior(
        cfg,
        vertices,
        p0,
        StubObserver(),
        observers_extra=None,
        k_steps=600,
        density_mesh=40,
        restrict_site_grid=3,
        restrict_local_mesh=8,
    )
    dumped = json.dumps(prior["B_E"])
    assert "E_bar" not in dumped
    assert prior["forbidden"].startswith("Do not substitute")
    assert prior["B_J"]["geometric"]["not_used"] == "N * m_plus * u_max^2"
    geom = prior["B_J"]["geometric"]["B_J_geom"]
    cellwise = 16 * 14.48 * 0.35**2
    assert geom < cellwise / 5.0
    assert prior["parameters"]["h_controller"] == pytest.approx(controller_grid_spacing(0.8, 20))
    assert prior["labels"]["E_bar_from_trajectory"] == "post_hoc"
    assert prior["B_J"]["prior_P0H_rigorousE"]["status"] != "rigorous_on_K0"
    assert prior["B_J"]["prior_certificate_crudeH_rigorousE"]["h0_status"] == "rigorous"
    assert prior["labels"]["H0_numerical_cannot_be_rigorous_on_K0"] is True
