from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "theory" / "python"))

from generate_certificate import build_certificate  # noqa: E402
from theorem1_constants import (  # noqa: E402
    REQUIRED_LEDGER_FIELDS,
    CertificateError,
    aggregate_disturbance,
    density_error,
    kernel_constants,
    mass_bounds,
    quadrature_bounds,
    safety_filter_ceiling,
    sampled_data_bounds,
    stability_bounds,
    target_position_error,
)


def config() -> dict:
    return yaml.safe_load((ROOT / "theory" / "theorem1" / "theorem_mode.yaml").read_text(encoding="utf-8"))


def test_kernel_constants_match_wolfram_exact_values():
    sigma = 0.37
    constants = kernel_constants(sigma)
    assert constants.l1 == pytest.approx(2 * math.pi * sigma**2)
    assert constants.grad_l1 == pytest.approx(math.sqrt(2) * math.pi**1.5 * sigma)
    assert constants.grad_linf == pytest.approx(math.exp(-0.5) / sigma)


def test_mass_bounds_positive():
    c = config()
    kernel = kernel_constants(c["density"]["sigma"])
    bound = mass_bounds(
        base_density=c["density"]["base_density"],
        local_radius=c["geometry"]["local_radius"],
        separation=c["geometry"]["separation"],
        domain_width=c["domain"]["width"],
        domain_height=c["domain"]["height"],
        perimeter_max=c["geometry"]["perimeter_max"],
        kernel=kernel,
    )
    assert bound.minimum > 0
    assert bound.maximum >= bound.minimum


def test_quadrature_gate_fails_closed():
    c = config()
    kernel = kernel_constants(c["density"]["sigma"])
    bound = mass_bounds(
        base_density=c["density"]["base_density"],
        local_radius=c["geometry"]["local_radius"],
        separation=c["geometry"]["separation"],
        domain_width=c["domain"]["width"],
        domain_height=c["domain"]["height"],
        perimeter_max=c["geometry"]["perimeter_max"],
        kernel=kernel,
    )
    with pytest.raises(CertificateError, match="eta_m=.*>= m_min"):
        quadrature_bounds(local_radius=c["geometry"]["local_radius"], grid_resolution=4, mass=bound)


def test_error_monotonicity():
    kernel = kernel_constants(0.2)
    base = dict(
        kernel=kernel,
        perimeter_max=7.2,
        epsilon_xi=0.01,
        voxel_size=0.002,
        target_lipschitz=2.0,
        arc_step=0.002,
        weight_mismatch=0.001,
        local_radius=0.8,
        dropped_mass=0.001,
        tail_sigmas=4.0,
    )
    baseline = density_error(**base)[0]
    for name in ("epsilon_xi", "voxel_size", "arc_step", "weight_mismatch", "dropped_mass"):
        larger = dict(base)
        larger[name] *= 1.5
        assert density_error(**larger)[0] >= baseline
    assert target_position_error(0.02, 0.01, 0.2, 0.15) >= target_position_error(0.01, 0.01, 0.2, 0.15)
    assert aggregate_disturbance(
        centroid_gain=1.0,
        centroid_errors=[0.1, 0.1],
        zoh=0.01,
        numerical=0.01,
        tracking=0.01,
        safety_filter=0.02,
    ) >= aggregate_disturbance(
        centroid_gain=1.0,
        centroid_errors=[0.1, 0.1],
        zoh=0.01,
        numerical=0.01,
        tracking=0.01,
        safety_filter=0.01,
    )


def test_frozen_perfect_information():
    continuous = stability_bounds(
        centroid_gain=1.0,
        mass_maximum=2.0,
        mu=0.5,
        alpha=0.5,
        delta=0.0,
        nu_h=0.0,
    )
    discrete = sampled_data_bounds(
        centroid_gain=1.0,
        mass_minimum=1.0,
        a0=continuous.a0,
        mu=0.5,
        alpha=0.5,
        delta=0.0,
        nu_h=0.0,
        dt=0.1,
        gradient_lipschitz=1.0,
    )
    assert continuous.forcing == 0.0
    assert continuous.ultimate_radius == 0.0
    assert discrete.forcing == 0.0
    assert discrete.ultimate_radius == 0.0


def test_locality_gate():
    c = config()
    c["geometry"]["communication_radius"] = 1.59
    with pytest.raises(CertificateError, match="locality gate failed"):
        build_certificate(c)


def test_arc_length_required():
    c = config()
    c["map_error"]["arc_step"] = 0.0
    with pytest.raises(CertificateError, match="arc-length"):
        build_certificate(c)


def test_restriction_tail_radius_is_bound_to_the_density_configuration():
    c = config()
    c["density"]["influence_sigmas"] = c["map_error"]["tail_sigmas"] - 1.0
    with pytest.raises(CertificateError, match="influence_sigmas"):
        build_certificate(c)


def test_safety_norm_convention():
    agent, stacked = safety_filter_ceiling(9, 0.3, "euclidean_per_agent")
    assert agent == pytest.approx(0.6)
    assert stacked == pytest.approx(1.8)
    box_agent, box_stacked = safety_filter_ceiling(9, 0.3, "componentwise_box")
    assert box_agent == pytest.approx(2 * math.sqrt(2) * 0.3)
    assert box_stacked == pytest.approx(2 * math.sqrt(18) * 0.3)


def test_sampled_step_gate():
    with pytest.raises(CertificateError, match="sampled-data step gate failed"):
        sampled_data_bounds(
            centroid_gain=1.0,
            mass_minimum=1.0,
            a0=0.2,
            mu=0.5,
            alpha=0.5,
            delta=0.1,
            nu_h=0.0,
            dt=2.0,
            gradient_lipschitz=1.0,
        )


def test_sampled_bound_dominates_direct_one_step_upper():
    a0, a1, lh, dt, delta, nuh = 0.4, 0.7, 1.1, 0.2, 0.08, 0.03
    abar = a0 - lh * a1**2 * dt / 2
    c_delta = delta * (1 + lh * a1 * dt)
    bd = c_delta**2 / (2 * abar) + (lh / 2) * delta**2 * dt + nuh
    for x in (0.0, 0.05, 0.2, 1.0, 3.0):
        direct = -a0 * x**2 + delta * x + (lh * dt / 2) * (a1 * x + delta) ** 2 + nuh
        certified = -(abar / 2) * x**2 + bd
        assert direct <= certified + 1e-14


def test_certificate_schema_and_required_fields():
    certificate = build_certificate(config())
    schema = json.loads((ROOT / "theory" / "certificates" / "runtime_certificate.schema.json").read_text())
    assert certificate["schema_version"] == schema["properties"]["schema_version"]["const"]
    assert all(certificate["gates"].values())
    for item in certificate["constants"].values():
        assert set(item) == REQUIRED_LEDGER_FIELDS
        assert math.isfinite(item["value"])
        assert item["unit"]
        assert item["formula"]
        assert item["source_class"] in {"analytic", "assumption", "runtime", "wolfram_symbolic", "interval"}
        assert item["status"] in {"proved", "verified", "conditional", "pending", "blocked"}


def test_bias_gate():
    c = config()
    c["density"]["gap_gain"] = 0.1
    with pytest.raises(CertificateError, match="zero gap/explore"):
        build_certificate(c)
