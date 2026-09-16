"""Exact-structure cell integrals for offset Gaussian mixtures via Green edges.

For an isotropic kernel ``k(q)=exp(-||q||^2/(2 sigma^2))`` the plane antiderivative

    F1(x,y) = sigma * sqrt(2 pi) / 2 * (1 + erf(x/(sigma*sqrt(2)))) * exp(-y^2/(2 sigma^2))

satisfies ``dF1/dx = k``.  Green then gives

    int_P k dA = oint_{dP} F1 dy.

First-moment identities (shifted to the kernel centre ``xi``) use

    int u_x k dA = oint -sigma^2 k dy,
    int u_y k dA = oint  sigma^2 k dx.

Disk cells are replaced by an inscribed regular ``n_gon`` clipped by the domain
and by Voronoi half-planes.  The symmetric-difference area is charged in
``apriori_centroid_bound``.  Edge integrals use composite trapezoid with an
*analytic* second-derivative majorant; panels are allocated from a frozen
maximum step ``h_max`` so that every *clipped* edge, including wall/Voronoi
chords up to length ``2 R``, meets a run-before budget.

Remainder identities use only a priori geometry of convex subsets of a disk:
perimeter ``<= 2 pi R``, diameter ``<= 2 R``.  They do not use trajectory
maxima.  Non-negative source weights enter as the factor ``W = sum w_j``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.special import erf

_EPS = 1e-15
_GOLDEN = 0.5 * (1.0 + math.sqrt(5.0))
# Analytic |d^2 F1 / ds^2| majorant at sigma = 1: golden/sqrt(e) + sqrt(2 pi).
# See theorem_and_proof.md §F1.  Valid for every unit direction and every (x,y).
_F1_M2_UNIT = _GOLDEN / math.sqrt(math.e) + math.sqrt(2.0 * math.pi)


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


def polygon_edge_lengths(polygon: np.ndarray) -> np.ndarray:
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    if len(poly) < 2:
        return np.zeros(0)
    edges = np.roll(poly, -1, axis=0) - poly
    return np.linalg.norm(edges, axis=1)


def unclipped_regular_chord(radius: float, n_gon: int) -> float:
    """Edge length of the *unclipped* inscribed regular n-gon. Not a clipped bound."""
    n = max(4, int(n_gon))
    return 2.0 * float(radius) * math.sin(math.pi / n)


def a_priori_max_edge_length(radius: float) -> float:
    """Any chord of a convex subset of the closed disk has length at most ``2 R``."""
    return 2.0 * float(radius)


def convex_disk_perimeter_bound(radius: float) -> float:
    """Perimeter of a convex subset of a disk of radius ``R`` is at most ``2 pi R``."""
    return 2.0 * math.pi * float(radius)


def analytic_m2_kernel(sigma: float) -> float:
    """Uniform majorant of ``|d^2 k / ds^2|`` along any unit-speed line.

    Let ``z = (u·τ)/σ``. Then ``d²k/ds² = k/σ² (z² - 1)`` and
    ``|d²k/ds²| ≤ exp(-z²/2) |z²-1| / σ²``. The 1-D function
    ``exp(-z²/2)|z²-1|`` attains maximum ``1`` at ``z=0`` (the only larger
    critical value is ``2 e^{-3/2} < 1``). Hence ``|d²k/ds²| ≤ 1/σ²``.
    Domain: ``σ > 0``, all positions, all unit directions. Units: 1/m².
    """
    s = float(sigma)
    if s <= 0.0:
        raise ValueError("sigma must be positive")
    return 1.0 / (s * s)


def analytic_m2_sigma2_kernel(sigma: float) -> float:
    """Uniform majorant of ``|d²(σ² k)/ds²|``. Equals 1, independent of σ.

    Follows from ``analytic_m2_kernel`` by multiplying by ``σ²``.
    Domain: ``σ > 0``. Dimensionless.
    """
    if float(sigma) <= 0.0:
        raise ValueError("sigma must be positive")
    return 1.0


def analytic_m2_f1(sigma: float) -> float:
    """Uniform majorant of ``|d² F1 / ds²|`` along any unit-speed line.

    Scaling: ``F1_σ(x,y) = σ F1_1(x/σ, y/σ)``, so the bound is ``C / σ`` with
    ``C = φ/√e + √(2π)`` and ``φ = (1+√5)/2``. Proof of ``C`` is in
    ``theorem_and_proof.md`` (Hessian of ``F1`` at ``σ=1``, Cauchy on the
    first-derivative term, Gaussian 1-D bound on ``|y²-1| e^{-y²/2}``).
    Domain: ``σ > 0``, all positions, all unit directions. Units: 1/m.
    """
    s = float(sigma)
    if s <= 0.0:
        raise ValueError("sigma must be positive")
    return _F1_M2_UNIT / s


def f1_second_derivative_majorant_constant() -> float:
    return float(_F1_M2_UNIT)


def edge_second_derivative_majorant(sigma: float) -> float:
    """Analytic uniform majorant covering both ``F1`` and ``σ² k`` second derivatives.

    This is *not* an NMaximize numerical envelope.  Because the two integrands
    have different scaling in ``σ``, callers that need a tight remainder should
    use ``analytic_m2_f1`` and ``analytic_m2_sigma2_kernel`` separately.
    """
    return max(analytic_m2_f1(sigma), analytic_m2_sigma2_kernel(sigma))


def panels_for_edge(length: float, h_max: float, min_panels: int = 2) -> int:
    """Number of trapezoid panels so the physical step is at most ``h_max``.

    ``h = length / panels ≤ h_max`` whenever ``panels = ceil(length / h_max)``.
    This rule is frozen before the run; it does not inspect future trajectories.
    The a priori remainder uses ``h_max`` itself, which dominates the realized
    ``h`` on every edge.
    """
    L = float(length)
    h = float(h_max)
    m0 = max(1, int(min_panels))
    if L <= _EPS:
        return m0
    if h <= 0.0:
        return m0
    return max(m0, int(math.ceil(L / h)))


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


def trapezoid_remainder_majorant(length: float, h_max: float, m2: float) -> float:
    """``|E| ≤ length * h_max² * M2 / 12`` for composite trapezoid with ``h ≤ h_max``."""
    return float(length) * float(h_max) ** 2 * float(m2) / 12.0


@dataclass(frozen=True)
class EdgeQuadratureCertificate:
    """Composite-trapezoid remainder majorant with a frozen maximum step."""

    h_max: float
    m2_f1: float
    m2_sigma2_k: float
    max_edge_length: float
    perimeter_bound: float

    @property
    def one_unit_mass_remainder_over_cell(self) -> float:
        """Mass remainder of one unit-weight kernel on one cell (∮ F1 dy)."""
        return trapezoid_remainder_majorant(self.perimeter_bound, self.h_max, self.m2_f1)

    @property
    def one_unit_moment_xi_l1_remainder_over_cell(self) -> float:
        """``|δμ_x|+|δμ_y|`` for kernel-centred first moments of one unit kernel."""
        one = trapezoid_remainder_majorant(self.perimeter_bound, self.h_max, self.m2_sigma2_k)
        return 2.0 * one


def _orient_ccw(poly: np.ndarray) -> np.ndarray:
    x, y = poly[:, 0], poly[:, 1]
    signed = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    if signed < 0.0:
        return poly[::-1]
    return poly


def gaussian_mass_moment_over_polygon(
    polygon: np.ndarray,
    xi: np.ndarray,
    sigma: float,
    panels: int | None = None,
    h_max: float | None = None,
    min_panels: int = 2,
) -> tuple[float, np.ndarray]:
    """Return ``(int k dA, int (q-xi) k dA)`` over a positively oriented polygon.

    If ``h_max`` is set, each clipped edge receives ``ceil(length / h_max)``
    panels (at least ``min_panels``).  A fixed ``panels`` value is used only
    when ``h_max`` is omitted (legacy tests).
    """
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    center = np.asarray(xi, dtype=float).reshape(2)
    if len(poly) < 3:
        return 0.0, np.zeros(2)
    area, _ = polygon_area_centroid(poly)
    if area == 0.0:
        return 0.0, np.zeros(2)
    poly = _orient_ccw(poly)

    mass = 0.0
    moment = np.zeros(2, dtype=float)
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length <= _EPS:
            continue
        if h_max is not None:
            m = panels_for_edge(length, float(h_max), min_panels=min_panels)
        else:
            m = max(2, int(panels if panels is not None else min_panels))
        ts = np.linspace(0.0, 1.0, m + 1)
        pts = a[None, :] + ts[:, None] * edge[None, :]
        u = pts - center[None, :]
        f1 = _f1(u[:, 0], u[:, 1], sigma)
        kk = _kernel(u[:, 0], u[:, 1], sigma)
        dy = edge[1]
        dx = edge[0]
        mass += trapezoid_edge_integral(f1, length) * (dy / length)
        moment[0] += trapezoid_edge_integral(-(sigma**2) * kk, length) * (dy / length)
        moment[1] += trapezoid_edge_integral((sigma**2) * kk, length) * (dx / length)
    return float(mass), moment


def project_to_disk(point: np.ndarray, center: np.ndarray, radius: float) -> tuple[np.ndarray, bool]:
    """Euclidean projection onto the closed disk ``B(center, radius)``."""
    p = np.asarray(point, dtype=float).reshape(2)
    c = np.asarray(center, dtype=float).reshape(2)
    r = float(radius)
    rel = p - c
    dist = float(np.linalg.norm(rel))
    if dist <= r + 1e-15:
        return p, False
    if dist <= _EPS:
        return c.copy(), False
    return c + rel * (r / dist), True


def mixture_mass_centroid_over_polygon(
    polygon: np.ndarray,
    targets: np.ndarray,
    weights: np.ndarray,
    sigma: float,
    floor: float,
    panels: int | None = None,
    cull_sigmas: float = 6.0,
    h_max: float | None = None,
    site: np.ndarray | None = None,
    radius: float | None = None,
    min_panels: int = 2,
    mass_floor: float = 1e-12,
) -> tuple[float, np.ndarray, int]:
    """Mass and centroid of ``floor + sum w_j k(·-xi_j)`` on a polygon.

    Sources with ``dist(xi, bbox(polygon)) > cull_sigmas * sigma`` are skipped;
    omitted mass is at most ``w_j * 2 pi sigma^2 exp(-cull_sigmas^2 / 2)`` and
    is charged in the prior via the Gaussian tail identity.  Runtime culling
    does not widen the partition-restrict certificate (that bound already uses
    the full weight sum ``W``).

    If ``site`` and ``radius`` are supplied, a non-positive numerical mass falls
    back to the site, and a finite-mass centroid is projected onto
    ``B(site, radius)``.  Projection onto a closed convex set containing the
    exact positive-density centroid is non-expansive, so it cannot increase
    ``||ĉ - c*||``.
    """
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    area, area_c = polygon_area_centroid(poly)
    site_arr = None if site is None else np.asarray(site, dtype=float).reshape(2)
    fallback = area_c if site_arr is None else site_arr.copy()
    if area <= _EPS:
        return 0.0, fallback.copy(), 0
    mass = float(floor) * area
    moment = float(floor) * area * area_c
    tgt = np.asarray(targets, dtype=float).reshape(-1, 2)
    w = np.asarray(weights, dtype=float).reshape(-1)
    used = 0
    lo = poly.min(axis=0)
    hi = poly.max(axis=0)
    cull_r = float(cull_sigmas) * float(sigma)
    for xi, wj in zip(tgt, w):
        if float(wj) <= _EPS:
            continue
        dx = float(max(lo[0] - xi[0], 0.0, xi[0] - hi[0]))
        dy = float(max(lo[1] - xi[1], 0.0, xi[1] - hi[1]))
        if dx * dx + dy * dy > cull_r * cull_r:
            continue
        m_j, mu_j = gaussian_mass_moment_over_polygon(
            poly, xi, sigma, panels=panels, h_max=h_max, min_panels=min_panels
        )
        mass += float(wj) * m_j
        # World-frame first moment: μ = μ_xi + xi m.  Robot-centred conversion
        # μ' = μ - p m is applied by the caller via (centroid - p); the prior
        # charges ||xi - p|| |δm| explicitly (see partition remainder).
        moment += float(wj) * (mu_j + xi * m_j)
        used += 1
    if mass <= float(mass_floor):
        return float(mass), fallback.copy(), used
    centroid = moment / mass
    if site_arr is not None and radius is not None:
        centroid, _projected = project_to_disk(centroid, site_arr, float(radius))
    return float(mass), centroid, used


def discrete_mixture_phi_max(
    phi0: float,
    weight_sum: float,
    n_edges: int,
    spacing: float,
    sigma: float,
) -> dict[str, float]:
    """Pointwise majorants of the *discrete* controller mixture.

    ``k ≤ 1`` gives ``φ ≤ φ0 + W``.  Sampling each of ``n_edges`` lines with
    spacing ``δ`` and weights ``δ`` additionally yields
    ``φ ≤ φ0 + n_edges (δ + σ √(2π))`` by comparing the discrete Gaussian ridge
    to ``1 + ∫_0^∞ exp(-x² λ) dx`` (see theorem_and_proof.md).  The continuous
    infinite-line bound ``σ √(2π)`` is *not* used alone: a discrete comb can
    overshoot it by up to ``δ``.
    """
    k1 = float(phi0) + float(weight_sum)
    line = float(phi0) + int(n_edges) * (float(spacing) + float(sigma) * math.sqrt(2.0 * math.pi))
    return {
        "phi_max_k_le_1": k1,
        "phi_max_discrete_lines": line,
        "phi_max_used": min(k1, line),
    }


def cull_partition_mass_bound(weight_sum: float, sigma: float, cull_sigmas: float) -> float:
    """Mass omitted by bbox culling on a truncated Voronoi partition.

    Same Gaussian plane-tail identity as restrict, with radius ``cull_sigmas``.
    Cells are disjoint, so there is no ``N`` factor.
    """
    s = float(cull_sigmas)
    return 2.0 * math.pi * float(sigma) ** 2 * math.exp(-0.5 * s * s) * float(weight_sum)


def aabb_disk_corner_reach(radius: float) -> float:
    """Farthest a point of ``bbox(P)`` can lie from ``p`` when ``P ⊂ B(p,R)``.

    The axis-aligned bounding box of the disk is the square of half-width ``R``,
    whose corners are at distance ``√2 R``.  ``bbox(P)`` is contained in that
    square, even if ``P`` itself never reaches the corner.
    """
    return math.sqrt(2.0) * float(radius)


def evaluated_source_reach(
    radius: float,
    cull_sigmas: float,
    sigma: float,
    *,
    max_offset: float | None = None,
    influence_sigmas: float | None = None,
) -> dict[str, float]:
    """A priori ``||xi - p||`` bound for sources the integrator actually evaluates.

    Counterexample to ``R + sσ``: take ``P`` with points near ``p+(R,0)`` and
    ``p+(0,R)``. Then ``bbox(P)`` contains the square corner at distance ``√2 R``,
    and a source with ``dist(xi, bbox(P)) = sσ`` just beyond that corner has
    ``||xi-p|| = √2 R + sσ > R + sσ``.

    ``density.restrict`` (when applied first) keeps raw boundary points with
    ``||b-p|| ≤ R + d_c + n_σ σ``. Kernel centres ``xi = b + offset n`` then
    satisfy ``||xi-p|| ≤ R + 2 d_c + n_σ σ``.
    """
    bbox_reach = aabb_disk_corner_reach(radius) + float(cull_sigmas) * float(sigma)
    invalid_old = float(radius) + float(cull_sigmas) * float(sigma)
    out: dict[str, float | str] = {
        "bbox_aabb_reach": float(bbox_reach),
        "invalid_R_plus_s_sigma": float(invalid_old),
        "used": float(bbox_reach),
        "rule": "sqrt(2) R + s sigma",
    }
    if max_offset is not None and influence_sigmas is not None:
        restrict_xi = (
            float(radius) + 2.0 * float(max_offset) + float(influence_sigmas) * float(sigma)
        )
        out["restrict_xi_reach"] = float(restrict_xi)
        if restrict_xi < bbox_reach:
            out["used"] = float(restrict_xi)
            out["rule"] = "min(sqrt(2) R + s sigma, R + 2 d_c + n_sig sigma)"
    return out  # type: ignore[return-value]


def partition_edge_mass_moment_remainder(
    n_agents: int,
    n_gon: int,
    n_sources: int,
    sigma: float,
    radius: float,
    panels: int,
    weight_sum: float | None = None,
    h_max: float | None = None,
    cull_sigmas: float = 6.0,
    max_offset: float | None = None,
    influence_sigmas: float | None = None,
) -> dict[str, float]:
    """A priori sum of edge-trapezoid remainders over agents and sources.

    The unclipped regular-n-gon chord ``2 R sin(π/n)`` is *not* a bound on
    clipped Voronoi/wall edges.  Those edges have length at most ``2 R``.
    The remainder is formed from:

    * analytic ``M2`` for ``F1`` and ``σ² k`` (not NMaximize);
    * perimeter ``≤ 2 π R`` per convex cell;
    * frozen ``h_max`` (or, if omitted, ``h_max = 2 R / panels``, which is the
      unique step that makes a diameter-length edge use ``panels`` panels);
    * non-negative weights through ``W = sum w_j``.  If ``weight_sum`` is
      omitted, ``W = n_sources`` (unit-weight majorant, conservative when
      true weights are arc lengths ``≪ 1``).

    Robot-centred first-moment error includes the missing conversion term
    ``||xi - p_i|| |δm|``.  ``P ⊂ B(p,R)`` and ``dist(xi, bbox(P)) ≤ sσ`` do
    **not** imply ``||xi-p|| ≤ R + sσ``: ``bbox(P)`` is an axis-aligned box and
    may contain the square corner at distance ``√2 R``.  The universal bound
    is ``||xi-p|| ≤ √2 R + sσ``.  If ``density.restrict`` ran first, kernel
    centres also satisfy ``||xi-p|| ≤ R + 2 d_c + n_σ σ``, and the certificate
    uses the minimum of the two.
    """
    r = float(radius)
    n = max(4, int(n_gon))
    legacy_chord = unclipped_regular_chord(r, n)
    max_edge = a_priori_max_edge_length(r)
    perim = convex_disk_perimeter_bound(r)
    if h_max is None:
        h = max_edge / float(max(1, int(panels)))
    else:
        h = float(h_max)
        if h <= 0.0:
            raise ValueError("h_max must be positive")
    w = float(n_sources) if weight_sum is None else float(weight_sum)
    if w < 0.0:
        raise ValueError("weight_sum must be nonnegative")
    m2_f1 = analytic_m2_f1(sigma)
    m2_s2 = analytic_m2_sigma2_kernel(sigma)
    cert = EdgeQuadratureCertificate(
        h_max=h,
        m2_f1=m2_f1,
        m2_sigma2_k=m2_s2,
        max_edge_length=max_edge,
        perimeter_bound=perim,
    )
    n_ag = int(n_agents)
    dm_unit = n_ag * w * cert.one_unit_mass_remainder_over_cell
    dmu_xi = n_ag * w * cert.one_unit_moment_xi_l1_remainder_over_cell
    reach_info = evaluated_source_reach(
        r,
        float(cull_sigmas),
        float(sigma),
        max_offset=max_offset,
        influence_sigmas=influence_sigmas,
    )
    reach_xi = float(reach_info["used"])
    dmu_robot = dmu_xi + reach_xi * dm_unit
    # Floating-point envelope: each edge sum has O(panels) addends of size
    # O(max|F1|) ≤ σ √(2π).  Charged separately from analytic truncation.
    panels_diam = panels_for_edge(max_edge, h, min_panels=2)
    n_edges_cell = n + 4 + max(0, n_ag - 1)
    float_one = float(n_edges_cell) * (panels_diam + 1) * 256.0 * np.finfo(float).eps * (
        float(sigma) * math.sqrt(2.0 * math.pi) + 1.0
    ) * max_edge
    float_dm = n_ag * w * float_one
    return {
        "panels_requested": float(panels),
        "h_max": float(h),
        "m2_f1": float(m2_f1),
        "m2_sigma2_k": float(m2_s2),
        "m2_bound": float(max(m2_f1, m2_s2)),
        "max_edge_length": float(max_edge),
        "legacy_unclipped_chord": float(legacy_chord),
        "perimeter_bound": float(perim),
        "weight_sum_W": float(w),
        "n_sources_majorant": float(n_sources),
        "used_unit_source_weights": float(1.0 if weight_sum is None else 0.0),
        "one_cell_unit_mass_remainder": float(cert.one_unit_mass_remainder_over_cell),
        "sum_abs_dm": float(dm_unit),
        "sum_abs_dmu_xi": float(dmu_xi),
        "xi_to_site_reach": float(reach_xi),
        "xi_to_site_reach_rule": str(reach_info["rule"]),
        "xi_to_site_reach_bbox": float(reach_info["bbox_aabb_reach"]),
        "xi_to_site_reach_invalid_old": float(reach_info["invalid_R_plus_s_sigma"]),
        "sum_abs_dmu": float(dmu_robot),
        "sum_abs_dmu_robot": float(dmu_robot),
        "float_sum_abs_dm": float(float_dm),
        "float_sum_abs_dmu": float(reach_xi * float_dm + n_ag * w * 2.0 * float_one),
        "cull_sigmas": float(cull_sigmas),
        "n_edge_evals": float(n_ag * n_edges_cell * max(int(n_sources), 1)),
        "certificate_status": "rigorous",
        "m2_status": "analytic",
        "note": (
            "Clipped-edge remainder uses diameter 2R and perimeter 2 pi R, "
            "not the unclipped regular-n-gon chord. M2 is analytic. "
            "Robot-centred moments include ||xi-p|| |dm| with a proved "
            "||xi-p|| reach (not R+sσ)."
        ),
        "reach_info": {k: (float(v) if isinstance(v, (int, float)) else v) for k, v in reach_info.items()},
    }


__all__ = [
    "EdgeQuadratureCertificate",
    "a_priori_max_edge_length",
    "analytic_m2_f1",
    "analytic_m2_kernel",
    "analytic_m2_sigma2_kernel",
    "clip_cell_polygon",
    "convex_disk_perimeter_bound",
    "cull_partition_mass_bound",
    "aabb_disk_corner_reach",
    "evaluated_source_reach",
    "discrete_mixture_phi_max",
    "edge_second_derivative_majorant",
    "f1_second_derivative_majorant_constant",
    "gaussian_mass_moment_over_polygon",
    "inscribed_disk_area_deficit",
    "mixture_mass_centroid_over_polygon",
    "panels_for_edge",
    "partition_edge_mass_moment_remainder",
    "polygon_area_centroid",
    "polygon_edge_lengths",
    "project_to_disk",
    "regular_inscribed_polygon",
    "unclipped_regular_chord",
]
