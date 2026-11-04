"""Analytic information-contract checks; no simulation environment."""

from pathlib import Path

import pytest
from conftest import load_script_module


@pytest.fixture(scope="module")
def model_audit():
    with pytest.MonkeyPatch.context() as patch:
        patch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
        yield load_script_module("analyze_r1_model_feasibility")


def test_symmetric_uncertainty_has_sharp_clearance_contact_threshold(model_audit):
    result = model_audit.interval_width(0.13, 0.065, 0.03125, 0.02, 8.0)
    assert result["raw_width"] == pytest.approx(0.0025)
    assert result["reserved_width"] == pytest.approx(0.0, abs=1e-14)
    # Contact requires a strict upper endpoint, so equality is insufficient.
    assert result["robust_reserved_contact_possible"] is False


def test_hidden_protrusion_preserves_all_finite_ray_returns(model_audit):
    example = model_audit.hidden_protrusion()
    assert example["same_all_ray_ranges"] is True
    assert example["hit_count"] > 2
    assert example["signed_distances"][0] == pytest.approx(0.1, abs=1e-12)
    assert 0.0 < example["signed_distances"][1] < 0.03
    assert example["adjacent_chord_half_gap"] < 0.005


def test_gaussian_three_sigma_is_not_a_deterministic_bound(model_audit):
    result = model_audit.gaussian_allowance()
    assert result["single_draw_exceedance"] == pytest.approx(0.002699796063, abs=1e-12)
    assert 0.17 < result["at_least_one_exceedance_72_independent_draws"] < 0.18
