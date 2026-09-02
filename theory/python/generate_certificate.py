"""Generate a numeric Theorem 1 certificate from a theorem-mode YAML file."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from theorem1_constants import (
    CertificateError,
    aggregate_disturbance,
    centroid_error,
    density_error,
    kernel_constants,
    ledger_entry,
    mass_bounds,
    quadrature_bounds,
    reference_rate,
    safety_filter_ceiling,
    sampled_data_bounds,
    stability_bounds,
    target_position_error,
)


def _require(mapping: dict[str, Any], key: str) -> Any:
    if key not in mapping:
        raise CertificateError(f"missing required config key: {key}")
    return mapping[key]


def build_certificate(config: dict[str, Any]) -> dict[str, Any]:
    if config.get("theorem_mode") is not True:
        raise CertificateError("config must set theorem_mode: true")
    if config.get("scope") != "enclose_local_cvt_only":
        raise CertificateError("Theorem 1 certificate scope must be enclose_local_cvt_only")

    n_agents = int(_require(config, "n_agents"))
    domain = _require(config, "domain")
    geometry = _require(config, "geometry")
    density = _require(config, "density")
    map_error = _require(config, "map_error")
    quadrature = _require(config, "quadrature")
    reference = _require(config, "reference")
    controller = _require(config, "controller")
    disturbance = _require(config, "disturbance")
    regularity = _require(config, "regularity")
    sampled = _require(config, "sampled_data")

    radius = float(_require(geometry, "local_radius"))
    comm = float(_require(geometry, "communication_radius"))
    if comm + 1e-12 < 2.0 * radius:
        raise CertificateError(f"locality gate failed: R_comm={comm} < 2 R_l={2.0*radius}")
    if quadrature.get("rule") != "midpoint":
        raise CertificateError("quadrature.rule must be midpoint")
    if float(density.get("gap_gain", 0.0)) != 0.0 or float(density.get("explore_gain", 0.0)) != 0.0:
        raise CertificateError("theorem mode requires zero gap/explore gain unless separately budgeted")
    if float(_require(map_error, "arc_step")) <= 0.0:
        raise CertificateError("theorem mode requires positive arc-length weights/arc_step")
    if float(_require(density, "influence_sigmas")) != float(_require(map_error, "tail_sigmas")):
        raise CertificateError(
            "density.influence_sigmas must equal map_error.tail_sigmas so the restrict() tail is auditable"
        )

    kernel = kernel_constants(_require(density, "sigma"))
    exi = target_position_error(
        _require(map_error, "boundary_position"),
        _require(map_error, "offset_distance"),
        _require(map_error, "normal_angle"),
        _require(geometry, "offset_max"),
    )
    ephi, tail = density_error(
        kernel=kernel,
        perimeter_max=_require(geometry, "perimeter_max"),
        epsilon_xi=exi,
        voxel_size=_require(map_error, "voxel_size"),
        target_lipschitz=_require(map_error, "target_lipschitz"),
        arc_step=_require(map_error, "arc_step"),
        weight_mismatch=_require(map_error, "weight_mismatch"),
        local_radius=radius,
        dropped_mass=_require(map_error, "dropped_mass"),
        tail_sigmas=_require(map_error, "tail_sigmas"),
    )
    mass = mass_bounds(
        base_density=_require(density, "base_density"),
        local_radius=radius,
        separation=_require(geometry, "separation"),
        domain_width=_require(domain, "width"),
        domain_height=_require(domain, "height"),
        perimeter_max=_require(geometry, "perimeter_max"),
        kernel=kernel,
        weight_mismatch=_require(map_error, "weight_mismatch"),
    )
    quad = quadrature_bounds(
        local_radius=radius,
        grid_resolution=int(_require(quadrature, "grid_resolution")),
        mass=mass,
    )
    ec = centroid_error(
        local_radius=radius,
        epsilon_phi=ephi,
        mass_minimum=mass.minimum,
        epsilon_quadrature=quad.epsilon,
        epsilon_geometric=_require(quadrature, "geometric_cell_error"),
    )
    per_agent_sf, stacked_sf = safety_filter_ceiling(
        n_agents, _require(controller, "max_speed"), _require(controller, "speed_norm")
    )
    sf = stacked_sf if disturbance.get("use_analytic_safety_ceiling") else float(
        _require(disturbance, "safety_filter")
    )
    delta = aggregate_disturbance(
        centroid_gain=_require(controller, "centroid_gain"),
        centroid_errors=[ec] * n_agents,
        zoh=_require(disturbance, "zoh"),
        numerical=_require(disturbance, "numerical"),
        tracking=_require(disturbance, "tracking"),
        safety_filter=sf,
    )
    vxi, nuphi, nuh = reference_rate(
        kernel=kernel,
        perimeter_max=_require(geometry, "perimeter_max"),
        object_speed_max=_require(reference, "object_speed_max"),
        object_yaw_rate_max=_require(reference, "object_yaw_rate_max"),
        object_radius=_require(geometry, "object_radius"),
        offset_max=_require(geometry, "offset_max"),
        offset_variation_length=_require(geometry, "offset_variation_length"),
        local_radius=radius,
    )
    stability = stability_bounds(
        centroid_gain=_require(controller, "centroid_gain"),
        mass_maximum=mass.maximum,
        mu=_require(regularity, "mu"),
        alpha=_require(regularity, "alpha"),
        delta=delta,
        nu_h=nuh,
    )
    discrete = sampled_data_bounds(
        centroid_gain=_require(controller, "centroid_gain"),
        mass_minimum=mass.minimum,
        a0=stability.a0,
        mu=_require(regularity, "mu"),
        alpha=_require(regularity, "alpha"),
        delta=delta,
        nu_h=nuh,
        dt=_require(sampled, "dt"),
        gradient_lipschitz=_require(sampled, "gradient_lipschitz"),
    )

    conditional = str(regularity.get("status", "conditional")) != "verified"
    entries = {
        "kernel_l1": ledger_entry(kernel.l1, "m^2", "K1", "wolfram_symbolic", "W-L1-01", "verified", conservative=False),
        "kernel_grad_l1": ledger_entry(kernel.grad_l1, "m", "K2", "wolfram_symbolic", "W-L1-02", "verified", conservative=False),
        "kernel_grad_linf": ledger_entry(kernel.grad_linf, "1/m", "K3", "wolfram_symbolic", "W-L1-03", "verified", conservative=False),
        "epsilon_xi": ledger_entry(exi, "m", "L1-xi", "runtime", "map_error", "conditional"),
        "epsilon_tail": ledger_entry(tail, "m^3", "L1-tail", "analytic", "04_lemma1_density_consistency.md", "proved"),
        "epsilon_phi": ledger_entry(ephi, "m^3", "L1", "analytic", "04_lemma1_density_consistency.md", "proved"),
        "m_min": ledger_entry(mass.minimum, "m^3", "L2-mass-floor", "analytic", "05_lemma2_centroid_perturbation.md", "proved"),
        "m_max": ledger_entry(mass.maximum, "m^3", "L2-mass-ceiling", "analytic", "05_lemma2_centroid_perturbation.md", "proved"),
        "grid_spacing": ledger_entry(quad.h, "m", "midpoint-h", "runtime", "LocalCVT.grid_resolution", "verified"),
        "eta_m": ledger_entry(quad.eta_mass, "m^3", "L2-eta-m", "analytic", "05_lemma2_centroid_perturbation.md", "proved"),
        "eta_a": ledger_entry(quad.eta_moment, "m^4", "L2-eta-a", "analytic", "05_lemma2_centroid_perturbation.md", "proved"),
        "epsilon_q": ledger_entry(quad.epsilon, "m", "L2-quadrature", "analytic", "05_lemma2_centroid_perturbation.md", "proved"),
        "epsilon_c": ledger_entry(ec, "m", "L2", "analytic", "05_lemma2_centroid_perturbation.md", "proved"),
        "safety_per_agent_ceiling": ledger_entry(per_agent_sf, "m/s", "P2-safety-agent", "analytic", "07_proposition2_disturbance_bound.md", "proved"),
        "safety_stacked_ceiling": ledger_entry(stacked_sf, "m/s", "P2-safety-stacked", "analytic", "07_proposition2_disturbance_bound.md", "proved"),
        "delta": ledger_entry(delta, "m/s", "P2", "runtime", "disturbance", "conditional"),
        "v_xi_max": ledger_entry(vxi, "m/s", "P3-target-rate", "runtime", "reference", "conditional"),
        "nu_phi": ledger_entry(nuphi, "m^3/s", "P3-a", "analytic", "08_proposition3_rate_and_regularity.md", "proved"),
        "nu_H": ledger_entry(nuh, "m^5/s", "P3-b", "analytic", "08_proposition3_rate_and_regularity.md", "proved"),
        "a0": ledger_entry(stability.a0, "1/(s*m^3)", "P1-a0", "analytic", "06_proposition1_ideal_local_cvt.md", "proved"),
        "lambda_H": ledger_entry(stability.lambda_h, "1/s", "T1-lambda", "analytic", "09_theorem1_practical_stability.md", "conditional" if conditional else "proved"),
        "B": ledger_entry(stability.forcing, "m^5/s", "T1-B", "analytic", "09_theorem1_practical_stability.md", "proved"),
        "ultimate_radius": ledger_entry(stability.ultimate_radius, "m", "T1-b", "analytic", "09_theorem1_practical_stability.md", "conditional" if conditional else "proved"),
        "sampled_abar": ledger_entry(discrete.abar, "1/(s*m^3)", "M7-abar", "analytic", "10_corollaries.md", "proved"),
        "sampled_Bd": ledger_entry(discrete.forcing, "m^5/s", "M7-Bd", "analytic", "10_corollaries.md", "proved"),
        "sampled_ultimate_radius": ledger_entry(discrete.ultimate_radius, "m", "M7-radius", "analytic", "10_corollaries.md", "conditional" if conditional else "proved"),
    }
    return {
        "schema_version": "1.0",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "scope": config["scope"],
        "evidence_status": "conditional" if conditional else "verified",
        "notes": config.get("notes", ""),
        "gates": {
            "locality": True,
            "arc_length_required": True,
            "midpoint_rule": True,
            "quadrature_denominator_positive": True,
            "sampled_step": True,
            "biases_disabled": True,
        },
        "constants": entries,
        "claims_deliberately_not_made": [
            "global PL",
            "formal caging",
            "unconditional transport success",
            "unconditional safety",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("theory/certificates/analytic_constants.json"),
    )
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    certificate = build_certificate(config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(certificate, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output}")
    print(f"status={certificate['evidence_status']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
