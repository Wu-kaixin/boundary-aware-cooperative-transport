"""Single-source constants and fail-closed gates for DBACT Theorem 1.

The functions here evaluate the formulas proved in ``theory/theorem1``.  They do
not prove assumptions, infer unavailable map error bounds, or turn a successful
numeric evaluation into evidence of caging or transport success.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable


class CertificateError(ValueError):
    """Raised when a theorem-mode premise or numerical gate is not satisfied."""


@dataclass(frozen=True)
class KernelConstants:
    l1: float
    grad_l1: float
    grad_linf: float


@dataclass(frozen=True)
class MassBounds:
    r0: float
    minimum: float
    maximum: float
    phi_max: float
    phi_lipschitz: float


@dataclass(frozen=True)
class QuadratureBounds:
    h: float
    midpoint_radius: float
    eta_mass: float
    eta_moment: float
    epsilon: float


@dataclass(frozen=True)
class StabilityBounds:
    a0: float
    lambda_h: float
    forcing: float
    ultimate_radius: float


@dataclass(frozen=True)
class SampledDataBounds:
    a1: float
    abar: float
    c_delta: float
    forcing: float
    contraction: float
    ultimate_radius: float


def _positive(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise CertificateError(f"{name} must be finite and positive, got {value!r}")
    return value


def _nonnegative(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise CertificateError(f"{name} must be finite and non-negative, got {value!r}")
    return value


def kernel_constants(sigma: float) -> KernelConstants:
    sigma = _positive("sigma", sigma)
    return KernelConstants(
        l1=2.0 * math.pi * sigma**2,
        grad_l1=math.sqrt(2.0) * math.pi ** 1.5 * sigma,
        grad_linf=math.exp(-0.5) / sigma,
    )


def target_position_error(
    boundary_position: float,
    offset_distance: float,
    normal_angle: float,
    offset_max: float,
) -> float:
    eb = _nonnegative("boundary_position", boundary_position)
    ed = _nonnegative("offset_distance", offset_distance)
    en = _nonnegative("normal_angle", normal_angle)
    dm = _nonnegative("offset_max", offset_max)
    if en > math.pi:
        raise CertificateError("normal_angle must lie in [0, pi]")
    return eb + ed + 2.0 * dm * math.sin(en / 2.0)


def density_error(
    *,
    kernel: KernelConstants,
    perimeter_max: float,
    epsilon_xi: float,
    voxel_size: float,
    target_lipschitz: float,
    arc_step: float,
    weight_mismatch: float,
    local_radius: float,
    dropped_mass: float,
    tail_sigmas: float,
) -> tuple[float, float]:
    pmax = _positive("perimeter_max", perimeter_max)
    exi = _nonnegative("epsilon_xi", epsilon_xi)
    voxel = _nonnegative("voxel_size", voxel_size)
    lt = _nonnegative("target_lipschitz", target_lipschitz)
    hs = _nonnegative("arc_step", arc_step)
    ew = _nonnegative("weight_mismatch", weight_mismatch)
    radius = _positive("local_radius", local_radius)
    mdrop = _nonnegative("dropped_mass", dropped_mass)
    ss = _nonnegative("tail_sigmas", tail_sigmas)
    tail = math.pi * radius**2 * mdrop * math.exp(-(ss**2) / 2.0)
    epsilon = (
        kernel.grad_l1
        * pmax
        * (exi + math.sqrt(2.0) * voxel + lt * hs / 2.0)
        + kernel.l1 * ew
        + tail
    )
    return epsilon, tail


def mass_bounds(
    *,
    base_density: float,
    local_radius: float,
    separation: float,
    domain_width: float,
    domain_height: float,
    perimeter_max: float,
    kernel: KernelConstants,
    weight_mismatch: float = 0.0,
) -> MassBounds:
    phi0 = _positive("base_density", base_density)
    radius = _positive("local_radius", local_radius)
    ds = _positive("separation", separation)
    width = _positive("domain_width", domain_width)
    height = _positive("domain_height", domain_height)
    pmax = _positive("perimeter_max", perimeter_max)
    ew = _nonnegative("weight_mismatch", weight_mismatch)
    r0 = min(radius, ds / 2.0, width / 2.0, height / 2.0)
    minimum = phi0 * math.pi * r0**2 / 4.0
    # The ideal mass ceiling uses P_max. Midpoint quadrature acts on the mapped
    # density, whose total nonnegative weight can be as large as P_max + E_w.
    maximum = math.pi * radius**2 * (phi0 + pmax)
    phi_max = phi0 + pmax + ew
    phi_lipschitz = (pmax + ew) * kernel.grad_linf
    return MassBounds(r0, minimum, maximum, phi_max, phi_lipschitz)


def quadrature_bounds(
    *,
    local_radius: float,
    grid_resolution: int,
    mass: MassBounds,
) -> QuadratureBounds:
    radius = _positive("local_radius", local_radius)
    n = int(grid_resolution)
    if n < 1:
        raise CertificateError("grid_resolution must be positive")
    # LocalCVT uses n midpoint cells across a bounding-box side of at most 2 R_l.
    h = 2.0 * radius / n
    rh = h / math.sqrt(2.0)
    area = math.pi * radius**2
    perimeter = 2.0 * math.pi * radius
    strip_area = 2.0 * perimeter * rh + math.pi * rh**2
    eta_m = mass.phi_lipschitz * rh * area + mass.phi_max * strip_area
    eta_a = (
        (mass.phi_max + radius * mass.phi_lipschitz) * rh * area
        + radius * mass.phi_max * strip_area
    )
    if eta_m >= mass.minimum:
        raise CertificateError(
            "midpoint quadrature certificate failed: "
            f"eta_m={eta_m:.12g} >= m_min={mass.minimum:.12g}"
        )
    epsilon = (eta_a + radius * eta_m) / (mass.minimum - eta_m)
    return QuadratureBounds(h, rh, eta_m, eta_a, epsilon)


def centroid_error(
    *,
    local_radius: float,
    epsilon_phi: float,
    mass_minimum: float,
    epsilon_quadrature: float,
    epsilon_geometric: float,
) -> float:
    radius = _positive("local_radius", local_radius)
    ephi = _nonnegative("epsilon_phi", epsilon_phi)
    mmin = _positive("mass_minimum", mass_minimum)
    eq = _nonnegative("epsilon_quadrature", epsilon_quadrature)
    eg = _nonnegative("epsilon_geometric", epsilon_geometric)
    return 2.0 * radius * ephi / mmin + eq + eg


def safety_filter_ceiling(n_agents: int, max_speed: float, convention: str) -> tuple[float, float]:
    n = int(n_agents)
    if n < 1:
        raise CertificateError("n_agents must be positive")
    umax = _positive("max_speed", max_speed)
    if convention == "euclidean_per_agent":
        per_agent = 2.0 * umax
        stacked = 2.0 * math.sqrt(n) * umax
    elif convention == "componentwise_box":
        per_agent = 2.0 * math.sqrt(2.0) * umax
        stacked = 2.0 * math.sqrt(2.0 * n) * umax
    else:
        raise CertificateError(f"unsupported speed_norm {convention!r}")
    return per_agent, stacked


def aggregate_disturbance(
    *,
    centroid_gain: float,
    centroid_errors: Iterable[float],
    zoh: float,
    numerical: float,
    tracking: float,
    safety_filter: float,
) -> float:
    gain = _positive("centroid_gain", centroid_gain)
    errors = [_nonnegative("centroid_error", item) for item in centroid_errors]
    if not errors:
        raise CertificateError("centroid_errors cannot be empty")
    remainder = sum(
        _nonnegative(name, value)
        for name, value in (
            ("zoh", zoh),
            ("numerical", numerical),
            ("tracking", tracking),
            ("safety_filter", safety_filter),
        )
    )
    return gain * math.sqrt(sum(item**2 for item in errors)) + remainder


def reference_rate(
    *,
    kernel: KernelConstants,
    perimeter_max: float,
    object_speed_max: float,
    object_yaw_rate_max: float,
    object_radius: float,
    offset_max: float,
    offset_variation_length: float,
    local_radius: float,
) -> tuple[float, float, float]:
    vxi = _nonnegative("object_speed_max", object_speed_max) + _nonnegative(
        "object_yaw_rate_max", object_yaw_rate_max
    ) * (
        _nonnegative("object_radius", object_radius)
        + _nonnegative("offset_max", offset_max)
        + _nonnegative("offset_variation_length", offset_variation_length)
    )
    nuphi = kernel.grad_l1 * _positive("perimeter_max", perimeter_max) * vxi
    nuh = 2.0 * _positive("local_radius", local_radius) ** 2 * nuphi
    return vxi, nuphi, nuh


def stability_bounds(
    *,
    centroid_gain: float,
    mass_maximum: float,
    mu: float,
    alpha: float,
    delta: float,
    nu_h: float,
) -> StabilityBounds:
    a0 = _positive("centroid_gain", centroid_gain) / (2.0 * _positive("mass_maximum", mass_maximum))
    mu = _positive("mu", mu)
    alpha = _positive("alpha", alpha)
    delta = _nonnegative("delta", delta)
    nu_h = _nonnegative("nu_h", nu_h)
    lambda_h = a0 * mu
    forcing = delta**2 / (2.0 * a0) + nu_h
    ultimate = math.sqrt(2.0 * forcing / (alpha * lambda_h))
    return StabilityBounds(a0, lambda_h, forcing, ultimate)


def sampled_data_bounds(
    *,
    centroid_gain: float,
    mass_minimum: float,
    a0: float,
    mu: float,
    alpha: float,
    delta: float,
    nu_h: float,
    dt: float,
    gradient_lipschitz: float,
) -> SampledDataBounds:
    a1 = _positive("centroid_gain", centroid_gain) / (2.0 * _positive("mass_minimum", mass_minimum))
    a0 = _positive("a0", a0)
    mu = _positive("mu", mu)
    alpha = _positive("alpha", alpha)
    delta = _nonnegative("delta", delta)
    nu_h = _nonnegative("nu_h", nu_h)
    dt = _positive("dt", dt)
    lh = _positive("gradient_lipschitz", gradient_lipschitz)
    threshold = 2.0 * a0 / (lh * a1**2)
    if not dt < threshold:
        raise CertificateError(f"sampled-data step gate failed: dt={dt:.12g} >= {threshold:.12g}")
    abar = a0 - lh * a1**2 * dt / 2.0
    contraction = abar * mu * dt
    if not 0.0 < contraction <= 1.0:
        raise CertificateError(
            "sampled-data contraction gate failed: require 0 < abar*mu*dt <= 1, "
            f"got {contraction:.12g}"
        )
    c_delta = delta * (1.0 + lh * a1 * dt)
    forcing = c_delta**2 / (2.0 * abar) + (lh / 2.0) * delta**2 * dt + nu_h
    ultimate = math.sqrt(2.0 * forcing / (alpha * abar * mu))
    return SampledDataBounds(a1, abar, c_delta, forcing, contraction, ultimate)


def ledger_entry(
    value: float,
    unit: str,
    formula: str,
    source_class: str,
    source_ref: str,
    status: str,
    *,
    conservative: bool = True,
    notes: str = "",
) -> dict[str, Any]:
    return {
        "value": float(value),
        "unit": unit,
        "source_class": source_class,
        "formula": formula,
        "source_ref": source_ref,
        "status": status,
        "conservative": bool(conservative),
        "notes": notes,
    }


REQUIRED_LEDGER_FIELDS = {
    "value",
    "unit",
    "source_class",
    "formula",
    "source_ref",
    "status",
    "conservative",
    "notes",
}
