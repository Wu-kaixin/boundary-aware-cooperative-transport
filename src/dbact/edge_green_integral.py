"""Exact-structure cell integrals for offset Gaussian mixtures via Green edges.

For an isotropic kernel ``k(q)=exp(-||q||^2/(2 sigma^2))`` the plane antiderivative

    F1(x,y) = sigma * sqrt(2 pi) / 2 * (1 + erf(x/(sigma*sqrt(2)))) * exp(-y^2/(2 sigma^2))

satisfies ``dF1/dx = k``.  Green then gives

    int_P k dA = oint_{dP} F1 dy.

First-moment identities (shifted to the kernel centre ``xi``) use

    int u_x k dA = oint -sigma^2 k dy,
    int u_y k dA = oint  sigma^2 k dx.

Disk cells are replaced by an inscribed regular ``n_gon`` clipped by the domain
and by Voronoi half-planes; the symmetric-difference area is charged explicitly
in ``apriori_centroid_bound``.  Edge integrals use composite trapezoid with an
explicit second-derivative majorant (not a requested ``epsabs``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.special import erf

_EPS = 1e-15


def _f1(x: np.ndarray, y: np.ndarray, sigma: float) -> np.ndarray:
    scale = sigma * math.sqrt(2.0)
    return (
        sigma
        * math.sqrt(2.0 * math.pi)
        * 0.5
        * (1.0 + erf(x / scale))
        * np.exp(-y * y / (2.0 * sigma * sigma))
    )


def _kernel(x: np.ndarray, y: np.ndarray, sigma: float) -> np.ndarray:
    return np.exp(-(x * x + y * y) / (2.0 * sigma * sigma))


def inscribed_disk_area_deficit(radius: float, n_gon: int) -> float:
    """Area(disk) - area(inscribed regular n-gon)."""
    r = float(radius)
    n = max(4, int(n_gon))
    return math.pi * r * r - 0.5 * n * r * r * math.sin(2.0 * math.pi / n)


def regular_inscribed_polygon(center: np.ndarray, radius: float, n_gon: int) -> np.ndarray:
    c = np.asarray(center, dtype=float).reshape(2)
    n = max(4, int(n_gon))
    ang = 2.0 * math.pi * np.arange(n) / n
    return c + float(radius) * np.column_stack((np.cos(ang), np.sin(ang)))


def clip_convex_polygon(polygon: np.ndarray, normal: np.ndarray, offset: float) -> np.ndarray:
    """Keep ``{q : q·normal <= offset}`` (Sutherland–Hodgman, convex)."""
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    if len(poly) == 0:
        return poly
    nrm = np.asarray(normal, dtype=float).reshape(2)
    out: list[np.ndarray] = []
    prev = poly[-1]
    prev_inside = float(prev @ nrm) <= offset + 1e-12
    for cur in poly:
        cur_inside = float(cur @ nrm) <= offset + 1e-12
        if cur_inside:
            if not prev_inside:
                denom = float((cur - prev) @ nrm)
                t = 0.0 if abs(denom) < _EPS else (offset - float(prev @ nrm)) / denom
                t = float(np.clip(t, 0.0, 1.0))
                out.append(prev + t * (cur - prev))
            out.append(cur.copy())
        elif prev_inside:
            denom = float((cur - prev) @ nrm)
            t = 0.0 if abs(denom) < _EPS else (offset - float(prev @ nrm)) / denom
            t = float(np.clip(t, 0.0, 1.0))
            out.append(prev + t * (cur - prev))
        prev = cur
        prev_inside = cur_inside
    if not out:
        return np.empty((0, 2))
    return np.asarray(out, dtype=float)


def clip_cell_polygon(
    center: np.ndarray,
    radius: float,
    neighbors: np.ndarray,
    domain: tuple[float, float, float, float],
    n_gon: int,
) -> np.ndarray:
    """Inscribed disk n-gon clipped by domain box and Voronoi half-planes."""
    poly = regular_inscribed_polygon(center, radius, n_gon)
    xmin, xmax, ymin, ymax = domain
    for normal, offset in (
        (np.array([1.0, 0.0]), xmax),
        (np.array([-1.0, 0.0]), -xmin),
        (np.array([0.0, 1.0]), ymax),
        (np.array([0.0, -1.0]), -ymin),
    ):
        poly = clip_convex_polygon(poly, normal, offset)
        if len(poly) == 0:
            return poly
    c = np.asarray(center, dtype=float).reshape(2)
    nbrs = np.asarray(neighbors, dtype=float).reshape(-1, 2) if len(neighbors) else np.empty((0, 2))
    for pj in nbrs:
        diff = pj - c
        poly = clip_convex_polygon(poly, diff, 0.5 * (float(pj @ pj) - float(c @ c)))
        if len(poly) == 0:
            return poly
    return poly


def polygon_area_centroid(polygon: np.ndarray) -> tuple[float, np.ndarray]:
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    if len(poly) < 3:
        return 0.0, np.zeros(2)
    x, y = poly[:, 0], poly[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y2 - x2 * y
    area = 0.5 * float(np.sum(cross))
    if abs(area) <= _EPS:
        return 0.0, poly.mean(axis=0)
    cx = float(np.sum((x + x2) * cross)) / (6.0 * area)
    cy = float(np.sum((y + y2) * cross)) / (6.0 * area)
    return abs(area), np.array([cx, cy], dtype=float)


@dataclass(frozen=True)
class EdgeQuadratureCertificate:
    """Composite-trapezoid remainder majorant for one scalar edge integrand."""

    panels: int
    m2_bound: float
    max_edge_length: float

    @property
    def one_edge_remainder(self) -> float:
        # |E| <= (b-a) * h^2 * M2 / 12 with h=(b-a)/m, so (b-a)^3 M2 / (12 m^2).
        L = float(self.max_edge_length)
        m = max(1, int(self.panels))
        return (L**3) * float(self.m2_bound) / (12.0 * m * m)


def trapezoid_edge_integral(
    values_at_nodes: np.ndarray,
    length: float,
) -> float:
    """Composite trapezoid on equally spaced samples along an edge of given length."""
    v = np.asarray(values_at_nodes, dtype=float).reshape(-1)
    if len(v) < 2:
        return 0.0
    trap = getattr(np, "trapezoid", None) or np.trapz
    return float(length) * float(trap(v, dx=1.0 / (len(v) - 1)))


def gaussian_mass_moment_over_polygon(
    polygon: np.ndarray,
    xi: np.ndarray,
    sigma: float,
    panels: int,
) -> tuple[float, np.ndarray]:
    """Return ``(int k dA, int (q-xi) k dA)`` over a positively oriented polygon."""
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    center = np.asarray(xi, dtype=float).reshape(2)
    if len(poly) < 3:
        return 0.0, np.zeros(2)
    # Orient CCW.
    area, _ = polygon_area_centroid(poly)
    if area == 0.0:
        return 0.0, np.zeros(2)
    # shoelace sign
    x, y = poly[:, 0], poly[:, 1]
    signed = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    if signed < 0.0:
        poly = poly[::-1]

    m = max(2, int(panels))
    mass = 0.0
    moment = np.zeros(2, dtype=float)
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length <= _EPS:
            continue
        ts = np.linspace(0.0, 1.0, m + 1)
        pts = a[None, :] + ts[:, None] * edge[None, :]
        u = pts - center[None, :]
        f1 = _f1(u[:, 0], u[:, 1], sigma)
        kk = _kernel(u[:, 0], u[:, 1], sigma)
        dy = edge[1]
        dx = edge[0]
        # int k = oint F1 dy, with dy constant * dt on the parameter interval.
        mass += trapezoid_edge_integral(f1, length) * (dy / length)
        moment[0] += trapezoid_edge_integral(-(sigma**2) * kk, length) * (dy / length)
        moment[1] += trapezoid_edge_integral((sigma**2) * kk, length) * (dx / length)
    return float(mass), moment


def mixture_mass_centroid_over_polygon(
    polygon: np.ndarray,
    targets: np.ndarray,
    weights: np.ndarray,
    sigma: float,
    floor: float,
    panels: int,
) -> tuple[float, np.ndarray, int]:
    """Mass and centroid of ``floor + sum w_j k(·-xi_j)`` on a polygon."""
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    area, area_c = polygon_area_centroid(poly)
    if area <= _EPS:
        return 0.0, np.zeros(2), 0
    mass = float(floor) * area
    moment = float(floor) * area * area_c
    tgt = np.asarray(targets, dtype=float).reshape(-1, 2)
    w = np.asarray(weights, dtype=float).reshape(-1)
    used = 0
    for xi, wj in zip(tgt, w):
        if abs(float(wj)) <= _EPS:
            continue
        m_j, mu_j = gaussian_mass_moment_over_polygon(poly, xi, sigma, panels)
        mass += float(wj) * m_j
        # int q k = int (u+xi) k = mu + xi * m
        moment += float(wj) * (mu_j + xi * m_j)
        used += 1
    if mass <= _EPS:
        return float(mass), area_c, used
    return float(mass), moment / mass, used


def edge_second_derivative_majorant(sigma: float) -> float:
    """Uniform majorant for ``|d^2/ds^2|`` of F1 and of ``sigma^2 k`` along unit-speed edges.

    Wolfram ``NMaximize`` on the closed form at ``sigma=0.2`` over a padded
    laboratory window gave ``max |g''| ≈ 12.53``.  Scaling by ``(0.2/sigma)^4``
    (two length derivatives on a Gaussian of width ``sigma``) and a safety factor
    ``15/12.53`` yields the majorant below.  Valid for ``sigma`` in the paper
    range near ``0.2``; outside that range re-run the maximisation.
    """
    s = max(float(sigma), 1e-6)
    return 15.0 * (0.2 / s) ** 4


def partition_edge_mass_moment_remainder(
    n_agents: int,
    n_gon: int,
    n_sources: int,
    sigma: float,
    radius: float,
    panels: int,
) -> dict[str, float]:
    """A priori sum of edge-trapezoid remainders over agents/sources/edges.

    Mass uses ``oint F1 dy`` (one scalar per edge).  The two moment components
    use ``oint k ds`` forms (two scalars).  Floor (constant) polygon integrals
    are exact by shoelace and contribute no edge remainder.
    """
    n = max(4, int(n_gon))
    chord = 2.0 * float(radius) * math.sin(math.pi / n)
    cert = EdgeQuadratureCertificate(
        panels=int(panels),
        m2_bound=edge_second_derivative_majorant(sigma),
        max_edge_length=chord,
    )
    one = cert.one_edge_remainder
    # Worst case: every agent polygon has n edges; every source touches every agent.
    n_edge_evals = int(n_agents) * n * int(n_sources)
    dm = n_edge_evals * one
    dmu = n_edge_evals * 2.0 * one  # two moment components
    return {
        "panels": float(cert.panels),
        "m2_bound": float(cert.m2_bound),
        "max_edge_length": float(cert.max_edge_length),
        "one_edge_remainder": float(one),
        "sum_abs_dm": float(dm),
        "sum_abs_dmu": float(dmu),
        "n_edge_evals": float(n_edge_evals),
    }


__all__ = [
    "EdgeQuadratureCertificate",
    "clip_cell_polygon",
    "edge_second_derivative_majorant",
    "gaussian_mass_moment_over_polygon",
    "inscribed_disk_area_deficit",
    "mixture_mass_centroid_over_polygon",
    "partition_edge_mass_moment_remainder",
    "polygon_area_centroid",
    "regular_inscribed_polygon",
]
