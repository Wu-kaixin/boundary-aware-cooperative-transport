"""A priori mass-weighted centroid-error bounds for static sampled theorem_mode.

This module turns the discrete-dissipation identity

    J_bar <= 2 H*(P0) / (a K Delta) + 4 E_bar / a^2

into a *prior* comparison by producing E_bar <= B_E from parameters, geometry
and initial conditions only. Trajectory maxima, means, fits and future states
are not inputs.

Status labels (kept in every returned dict):
    rigorous           -- proved under the stated hypotheses
    numerical_a_priori -- computed from geometry / P0 / a spatial site grid;
                          remainder is Lipschitz covering, not a trajectory
    post_hoc           -- uses a closed-loop record (forbidden as a prior)
    not_strict_certificate -- quadrature comparison only

Units follow the existing code convention: the offset kernel is dimensionless,
arc-length weights are metres, floor density is 1/m^2. Mass integrals therefore
mix those conventions exactly as ``LocalCVT`` and ``UniformOffsetObserver`` do.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

# Gaussian kernel k(q) = exp(-||q||^2 / (2 sigma^2)).
# max |grad k| = 1/(sigma sqrt(e));  int k dA = 2 pi sigma^2;
# int |grad k| dA = pi sigma sqrt(2 pi);
# int |d^2 k / dx^2| dA = 4 sqrt(2 pi / e)  (sigma-independent).
# Verified with Wolfram (session constants, sigma = 1/5 and sigma = 1).
SQRT_E = math.sqrt(math.e)
GAUSS_INT_ABS_FPP = 4.0 * math.sqrt(2.0 * math.pi / math.e)  # dimensionless


@dataclass(frozen=True)
class BoundTerm:
    """One summand of a prior, with formula / source / domain / units."""

    name: str
    value: float
    formula: str
    source: str
    domain: str
    units: str
    status: str
    decreases_with_controller_grid: bool
    decreases_with_oracle_spacing: bool
    forms_floor: bool
    notes: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def kernel_lipschitz(sigma: float) -> float:
    """max_q |grad_q exp(-||q||^2 / (2 sigma^2))| = 1/(sigma sqrt(e))."""
    if sigma <= 0.0:
        raise ValueError("sigma must be positive")
    return 1.0 / (float(sigma) * SQRT_E)


def kernel_plane_mass(sigma: float) -> float:
    """int_{R^2} exp(-||q||^2 / (2 sigma^2)) dA = 2 pi sigma^2."""
    return 2.0 * math.pi * float(sigma) ** 2


def kernel_grad_l1(sigma: float) -> float:
    """int_{R^2} |grad k| dA = pi sigma sqrt(2 pi)."""
    return math.pi * float(sigma) * math.sqrt(2.0 * math.pi)


def sampling_period_a(kc: float, dt: float) -> float:
    a = 2.0 / float(kc) - float(dt)
    if a <= 0.0:
        raise ValueError(f"a = 2/k_c - Delta = {a} must be positive")
    return a


def step_in_k0(record: dict, n_agents: int, a: float) -> tuple[bool, list[int]]:
    """A step is in K_0 iff every QP status is optimal, every rho-zero flag is true, and a>0.

    Missing ``zero_input_feasible_with_rho`` fails closed (not treated as True).
    """
    if float(a) <= 0.0:
        return False, list(range(int(n_agents)))
    status = [str(s) == "optimal" for s in record.get("solver_status", [])]
    if len(status) != int(n_agents):
        return False, list(range(int(n_agents)))
    if "zero_input_feasible_with_rho" not in record:
        return False, list(range(int(n_agents)))
    zr = [bool(z) for z in record["zero_input_feasible_with_rho"]]
    if len(zr) != int(n_agents):
        return False, list(range(int(n_agents)))
    bad = [i for i in range(int(n_agents)) if (not zr[i]) or (not status[i])]
    return (len(bad) == 0), bad


def controller_grid_spacing(local_radius: float, grid_resolution: int) -> float:
    """Endpoint-grid spacing on the axis-aligned box of the reach disk.

    LocalCVT uses linspace(p-R, p+R, n) when the disk is interior, so
    h = 2 R / (n-1). Domain clipping only *shrinks* the box, hence this h
    is an upper bound. Source: ``src/dbact/local_cvt.py`` ``cell_samples``.
    """
    n = max(4, int(grid_resolution))
    return 2.0 * float(local_radius) / (n - 1)


def polygon_perimeter(vertices: np.ndarray) -> float:
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    if len(v) < 2:
        return 0.0
    edges = np.roll(v, -1, axis=0) - v
    return float(np.sum(np.linalg.norm(edges, axis=1)))


def plane_mass_upper(phi0: float, domain_area: float, sigma: float, perimeter: float) -> float:
    """M <= phi0 |D| + (int k) L. Ridge mass on R^2 equals C_K L; floor on D."""
    return float(phi0) * float(domain_area) + kernel_plane_mass(sigma) * float(perimeter)


def h_star_crude_upper(local_radius: float, mass_upper: float) -> float:
    """H* <= R^2 M because the truncated integrand is at most R^2."""
    return float(local_radius) ** 2 * float(mass_upper)


def oracle_midpoint_l1_bound(perimeter: float, spacing: float, sigma: float) -> BoundTerm:
    """L1(R^2) bound on |phi_ctrl - phi*| from midpoint edge sampling.

    ``oracle_boundary_view`` places samples at edge midpoints with
    ds = length/count <= spacing. On each panel the first moment along the
    edge is exact, and Taylor with remainder gives

        int |int_panel k(q, xi(s)) ds - ds k(q, xi_mid)| dq
            <= sqrt(2 pi / e) * h^3 / 6

    because int |d^2 k / ds^2| dA = 4 sqrt(2 pi / e) and
    int_{-h/2}^{h/2} t^2 dt = h^3 / 12. Summing h^3 <= spacing^2 * L yields
    the displayed formula. Independent of the CVT grid. Vanishes as ds -> 0.
    """
    if spacing < 0.0 or perimeter < 0.0 or sigma <= 0.0:
        raise ValueError("oracle L1 bound requires nonnegative L, ds and positive sigma")
    coeff = math.sqrt(2.0 * math.pi / math.e) / 6.0
    value = coeff * float(perimeter) * float(spacing) ** 2
    return BoundTerm(
        name="oracle_midpoint_L1",
        value=value,
        formula="sqrt(2 pi / e) * L * delta^2 / 6",
        source="midpoint Taylor remainder + int |d^2 k/ds^2| dA = 4 sqrt(2 pi / e) (Wolfram)",
        domain="static oracle, uniform cage offset, frozen polygon, ds = theorem_oracle_spacing",
        units="same mixed mass units as int |phi_ctrl - phi*| dA",
        status="rigorous",
        decreases_with_controller_grid=False,
        decreases_with_oracle_spacing=True,
        forms_floor=True,
        notes=(
            "Floor w.r.t. CVT grid refinement: controller vs continuous edge measure "
            "does not vanish as n -> inf. Not a trajectory statistic."
        ),
    )


def restrict_pointwise_bound(perimeter: float, influence_sigmas: float) -> BoundTerm:
    """Uniform |phi_restrict - phi_full| on B(p, R).

    ``density.restrict`` drops boundary points with ||b_k - p|| > R + d_c + n_sig sigma.
    Every dropped kernel then satisfies ||q - xi_k|| > n_sig sigma on the CVT disk,
    so k < exp(-n_sig^2 / 2). Summing ds_k <= L gives a uniform pointwise bound.
    Independent of grid spacing; forms a floor unless influence_sigmas -> inf.

    Prefer ``restrict_partition_mass_bound`` for mass sums over a truncated Voronoi
    partition (no N-fold disk factor).
    """
    tail = math.exp(-0.5 * float(influence_sigmas) ** 2)
    value = float(perimeter) * tail
    return BoundTerm(
        name="restrict_pointwise_tail",
        value=value,
        formula="L * exp(-influence_sigmas^2 / 2)",
        source="src/dbact/boundary_density.py restrict(); Gaussian tail on the CVT disk",
        domain="any site p in D, queries q in B(p, R)",
        units="same as phi (mixed)",
        status="rigorous",
        decreases_with_controller_grid=False,
        decreases_with_oracle_spacing=False,
        forms_floor=True,
        notes="Pointwise only. Over-counts when used as N * value * area(disk).",
    )


def restrict_partition_mass_bound(
    weight_sum: float,
    sigma: float,
    influence_sigmas: float,
) -> BoundTerm:
    """Mass-sum restrict error on a truncated Voronoi partition.

    Hypotheses (checked against ``boundary_density.restrict``):
      1. Weights ``w_j >= 0`` and ``sum_j w_j = W`` (oracle: arc lengths, conf=1).
      2. Deletion uses raw boundary points ``b_j`` with
         ``||b_j - p_i|| > R + max_offset + s sigma`` (``s=influence_sigmas``).
      3. Kernel centres are ``xi_j = b_j + offset_j n_j``, so on ``q in B(p_i,R)``
         every dropped source satisfies ``||q - xi_j|| >= s sigma``.
      4. Cells ``Omega_i = V_i cap B(p_i,R)`` are pairwise disjoint up to null sets.

    Then for each dropped index ``j`` and each ``i`` that drops it,
    ``int_{Omega_i} k(q, xi_j) dq <= int_{||u||>= s sigma} k = 2 pi sigma^2 exp(-s^2/2)``,
    and summing over the disjoint cells that drop ``j`` cannot exceed that plane
    tail. Hence

        sum_i int_{Omega_i} |phi_full - phi_restrict,i| dq
            <= 2 pi sigma^2 exp(-s^2/2) * W.

    Wolfram check: ``Integrate[r Exp[-r^2/(2 sigma^2)],{r,s sigma,inf}]*2 pi``
    equals ``2 pi sigma^2 Exp[-s^2/2]``.
    """
    s = float(influence_sigmas)
    tail = math.exp(-0.5 * s * s)
    value = 2.0 * math.pi * float(sigma) ** 2 * tail * float(weight_sum)
    return BoundTerm(
        name="restrict_partition_tail_mass",
        value=value,
        formula="2 pi sigma^2 * exp(-s^2/2) * W",
        source="disjoint truncated Voronoi cells + Gaussian plane tail (Wolfram)",
        domain="static oracle/offset density; restrict by raw b_j; Omega_i partition",
        units="mass (mixed)",
        status="rigorous",
        decreases_with_controller_grid=False,
        decreases_with_oracle_spacing=False,
        forms_floor=True,
        notes="No N factor. Requires non-negative weights and the delete-vs-xi distance relation.",
    )


def line_edge_phi_max(phi0: float, sigma: float, n_edges: int) -> float:
    """Pointwise majorant: each infinite line contributes at most sigma*sqrt(2 pi)."""
    return float(phi0) + int(n_edges) * float(sigma) * math.sqrt(2.0 * math.pi)


def convex_cell_area_error(local_radius: float, h: float) -> float:
    """|N h^2 - Area(Omega)| for a convex subset of the reach disk.

    Omega_i = V_i cap B(p_i, R) is convex. Davenport-type lattice discrepancy:
    the symmetric difference between Omega and the union of h-squares meeting
    dOmega sits in a tube of width sqrt(2) h, plus four corner squares.
    Perim(Omega) <= 2 pi R (circle maximises perimeter among convex subsets of
    a disk). Endpoint linspace uses this same h.
    """
    perim = 2.0 * math.pi * float(local_radius)
    width = math.sqrt(2.0) * float(h)
    return perim * width + math.pi * width * width + 4.0 * float(h) ** 2


def grid_mass_moment_errors(
    phi_max: float,
    lip_phi: float,
    local_radius: float,
    h: float,
    n_agents: int,
) -> tuple[BoundTerm, BoundTerm]:
    """Per-cell Cartesian endpoint-grid mass and first-moment errors, then N-sum.

    Disks overlap, so grid errors are *not* disjoint; the N factor is required.
    Interior Lip term uses Area <= pi R^2. No 1/m_minus appears.
    """
    area = math.pi * float(local_radius) ** 2
    e_area = convex_cell_area_error(local_radius, h)
    e_m_one = float(phi_max) * e_area + float(lip_phi) * math.sqrt(2.0) * float(h) * area
    # First moment relative to p_i: |q-p_i| <= R on the true cell, <= R + sqrt(2) h
    # on a straddling square.
    reach = float(local_radius) + math.sqrt(2.0) * float(h)
    e_mu_one = reach * float(phi_max) * e_area + float(lip_phi) * math.sqrt(2.0) * float(h) * area * float(
        local_radius
    )
    n = int(n_agents)
    mass_term = BoundTerm(
        name="cvt_endpoint_grid_mass_sum",
        value=n * e_m_one,
        formula="N * (phi_max * e_area(h) + Lip(phi) * sqrt(2) h * pi R^2)",
        source="LocalCVT endpoint linspace + Davenport tube on convex V_i cap disk",
        domain=f"any legal configuration in D^N; h = 2R/(n_grid-1) = {h:.6g} m",
        units="mass (mixed)",
        status="rigorous",
        decreases_with_controller_grid=True,
        decreases_with_oracle_spacing=False,
        forms_floor=False,
        notes="O(h) from the cell boundary; interior Lip is also O(h). Overlap forces the N factor.",
    )
    moment_term = BoundTerm(
        name="cvt_endpoint_grid_moment_sum",
        value=n * e_mu_one,
        formula="N * ((R+sqrt(2)h) phi_max e_area + Lip(phi) sqrt(2) h pi R^3)",
        source="same as mass term, first moment relative to the site",
        domain=mass_term.domain,
        units="mass * metre",
        status="rigorous",
        decreases_with_controller_grid=True,
        decreases_with_oracle_spacing=False,
        forms_floor=False,
        notes="Grid centroids lie in conv(disk cap grid) subset B(p,R).",
    )
    return mass_term, moment_term


def moment_form_E_bound(
    local_radius: float,
    sum_abs_dm: float,
    sum_abs_dmu: float,
) -> float:
    """E = sum m*_i ||chat_i - c*_i||^2 <= 2 R (sum ||dmu'_i|| + R sum |dm_i|).

    Proof. Both centroids lie in B(p_i, R), so ||e_i|| <= 2 R and
        m*_i ||e_i||^2 <= 2 R * ||m*_i e_i||.
    Write m* (chat - c*) = (m* chat - mu*) and compare with the discrete
    first moment mu_hat = m_hat chat:
        m* e = (mu_hat - mu*) - chat (m_hat - m*),
    hence ||m* e|| <= ||dmu'|| + R |dm|. No division by m_minus.
    """
    r = float(local_radius)
    return 2.0 * r * (float(sum_abs_dmu) + r * float(sum_abs_dm))


def diameter_E_bound(local_radius: float, mass_upper: float) -> BoundTerm:
    return BoundTerm(
        name="E_diameter_4R2M",
        value=4.0 * float(local_radius) ** 2 * float(mass_upper),
        formula="4 R^2 M_upper",
        source="chat, c* in B(p_i,R) so ||e||<=2R; sum m* <= M",
        domain="any truncated partition of D",
        units="mass * metre^2",
        status="rigorous",
        decreases_with_controller_grid=False,
        decreases_with_oracle_spacing=False,
        forms_floor=True,
        notes="Resolution-independent; kept as a baseline, not the proposed bound.",
    )


def prior_J_bound(b_h0: float, b_e: float, a: float, k_steps: int, dt: float) -> dict[str, Any]:
    """B_J_prior = 2 B_H0 / (a K Delta) + 4 B_E / a^2. Requires a>0 and full-horizon K_0."""
    a = float(a)
    k_steps = int(k_steps)
    dt = float(dt)
    term_h = 2.0 * float(b_h0) / (a * k_steps * dt)
    term_e = 4.0 * float(b_e) / (a * a)
    return {
        "B_J_prior": term_h + term_e,
        "term_2BH0_over_aKD": term_h,
        "term_4BE_over_a2": term_e,
        "a": a,
        "K": k_steps,
        "dt": dt,
        "formula": "2 B_H0 / (a K Delta) + 4 B_E / a^2",
        "status": "rigorous_on_K0",
        "domain": "full window in K_0, a>0, sat+QP cascade of Theorem C",
    }


def geometric_J_bound(mass: float, mass_error: float, umax: float) -> dict[str, Any]:
    """B_J_geom = (M + eps_M) u_max^2. Uses total mass, not N m_plus u_max^2."""
    m_up = float(mass) + abs(float(mass_error))
    umax = float(umax)
    return {
        "B_J_geom": m_up * umax * umax,
        "M_used": float(mass),
        "eps_M": abs(float(mass_error)),
        "M_upper_for_bound": m_up,
        "u_max": umax,
        "formula": "(M_total + eps_M) * u_max^2",
        "not_used": "N * m_plus * u_max^2",
        "status": "rigorous",
        "domain": "J_k = sum m*_i ||u_i||^2 <= (sum m*_i) u_max^2 <= M_total u_max^2",
    }


def config_fingerprint(cfg: dict) -> str:
    payload = {
        "dt": cfg.get("dt"),
        "domain": cfg.get("domain"),
        "agents": cfg.get("agents"),
        "cargoes": cfg.get("cargoes"),
        "controller": cfg.get("controller"),
    }
    blob = json.dumps(payload, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def overlay_grid_resolution(cfg: dict, grid_resolution: int) -> dict:
    out = json.loads(json.dumps(cfg))  # JSON round-trip: plain dict / lists
    out.setdefault("controller", {})["grid_resolution"] = int(grid_resolution)
    return out


def build_controller_density(vertices: np.ndarray, cfg: dict, spacing: float):
    """Oracle discrete mixture used by the controller (no robot-dependent gap)."""
    from dbact.boundary_density import BoundaryAwareDensity, DensityParams
    from dbact.cargo import Cargo
    from dbact.theorem_mode import oracle_boundary_view

    verts = np.asarray(vertices, dtype=float).reshape(-1, 2)
    cargo = Cargo(object_id="prior", vertices=verts, movable=False)
    view = oracle_boundary_view([cargo], spacing=float(spacing))
    ctrl = cfg.get("controller", {})
    params = DensityParams(
        mode=str(ctrl.get("density_mode", "offset")),
        cage_offset=float(ctrl["cage_offset"]),
        sigma=float(ctrl["sigma"]),
        base_density=float(ctrl["base_density"]),
        gap_gain=0.0,
        explore_gain=0.0,
        lead_offset=None,
        influence_sigmas=float(ctrl.get("influence_sigmas", 3.0)),
    )
    return BoundaryAwareDensity.from_view(view, params, robot_positions=None, goal_direction=None)


def numerical_density_mismatch(
    vertices: np.ndarray,
    cfg: dict,
    observer,
    spacing: float,
    mesh: int = 240,
) -> dict[str, Any]:
    """Geometry-only L1 / Linf of |phi_ctrl - phi*| on D (not a trajectory).

    Remainder: Lip(delta_phi) <= Lip(phi_ctrl)+Lip(phi*) <= 2 L / (sigma sqrt(e)),
    so the mesh L1 underestimates the true L1 by at most Lip * (h_mesh/sqrt(2)) * |D|
    in the worst covering (each cell evaluated at a corner). Reported separately;
    consistency under mesh refinement is *not* promoted to a strict certificate.
    """
    domain = cfg["domain"]
    xmin, xmax = float(domain["xmin"]), float(domain["xmax"])
    ymin, ymax = float(domain["ymin"]), float(domain["ymax"])
    width, height = xmax - xmin, ymax - ymin
    area = width * height
    xs = np.linspace(xmin, xmax, int(mesh))
    ys = np.linspace(ymin, ymax, int(mesh))
    h = max(width / (mesh - 1), height / (mesh - 1))
    xx, yy = np.meshgrid(xs, ys)
    q = np.column_stack([xx.ravel(), yy.ravel()])
    density = build_controller_density(vertices, cfg, spacing)
    phi_ctrl = np.atleast_1d(density(q))
    phi_star = np.atleast_1d(observer.density(q, vertices))
    diff = np.abs(phi_ctrl - phi_star)
    cell = (width / (mesh - 1)) * (height / (mesh - 1))
    l1_mesh = float(np.sum(diff) * cell)
    linf = float(np.max(diff))
    perimeter = polygon_perimeter(vertices)
    sigma = float(cfg["controller"]["sigma"])
    lip = 2.0 * kernel_lipschitz(sigma) * perimeter
    l1_remainder = lip * (h / math.sqrt(2.0)) * area
    analytic = oracle_midpoint_l1_bound(perimeter, spacing, sigma)
    return {
        "status": "numerical_a_priori",
        "strict_certificate": False,
        "reason": "mesh Lipschitz remainder is an estimate, not a closed quadrature certificate",
        "mesh": int(mesh),
        "h_mesh": h,
        "L1_mesh": l1_mesh,
        "L1_remainder_lipschitz": l1_remainder,
        "L1_mesh_plus_remainder": l1_mesh + l1_remainder,
        "Linf_mesh": linf,
        "analytic_L1_bound": analytic.value,
        "analytic_covers_mesh": bool(l1_mesh <= analytic.value + 1e-12),
        "phi_ctrl_max_mesh": float(np.max(phi_ctrl)),
        "phi_star_max_mesh": float(np.max(phi_star)),
        "mean_abs_diff": float(np.mean(diff)),
    }


def numerical_restrict_mass_majorant(
    vertices: np.ndarray,
    cfg: dict,
    spacing: float,
    local_radius: float,
    site_grid: int = 12,
    local_mesh: int = 36,
) -> dict[str, Any]:
    """Max over a site grid of int_{disk} |phi_full - phi_restrict|.

    Site locations are a lattice in D, independent of the closed-loop trajectory.
    Remainder: Lip * h_site * pi R^2 at the worst cell, plus local-mesh Lip error.
    """
    domain = cfg["domain"]
    xmin, xmax = float(domain["xmin"]), float(domain["xmax"])
    ymin, ymax = float(domain["ymin"]), float(domain["ymax"])
    density = build_controller_density(vertices, cfg, spacing)
    r = float(local_radius)
    xs = np.linspace(xmin + r, xmax - r, int(site_grid))
    ys = np.linspace(ymin + r, ymax - r, int(site_grid))
    h_site = max((xmax - xmin - 2 * r) / max(site_grid - 1, 1), (ymax - ymin - 2 * r) / max(site_grid - 1, 1))
    worst = 0.0
    worst_p = None
    nloc = max(8, int(local_mesh))
    for x in xs:
        for y in ys:
            p = np.array([x, y], dtype=float)
            lo = p - r
            hi = p + r
            gx = np.linspace(lo[0], hi[0], nloc)
            gy = np.linspace(lo[1], hi[1], nloc)
            xx, yy = np.meshgrid(gx, gy)
            q = np.column_stack([xx.ravel(), yy.ravel()])
            rel = q - p[None, :]
            mask = np.sum(rel * rel, axis=1) <= r * r
            q = q[mask]
            if len(q) == 0:
                continue
            cell = ((hi[0] - lo[0]) / (nloc - 1)) * ((hi[1] - lo[1]) / (nloc - 1))
            full = np.atleast_1d(density(q))
            rest = np.atleast_1d(density.restrict(p, r)(q))
            mass = float(np.sum(np.abs(full - rest)) * cell)
            if mass > worst:
                worst = mass
                worst_p = [float(x), float(y)]
    perimeter = polygon_perimeter(vertices)
    sigma = float(cfg["controller"]["sigma"])
    lip = kernel_lipschitz(sigma) * perimeter
    remainder = lip * h_site * math.pi * r * r
    pointwise = restrict_pointwise_bound(
        perimeter, float(cfg["controller"].get("influence_sigmas", 3.0))
    )
    crude_mass = pointwise.value * math.pi * r * r
    return {
        "status": "numerical_a_priori",
        "strict_certificate": False,
        "site_grid": int(site_grid),
        "local_mesh": nloc,
        "max_disk_L1": worst,
        "worst_site": worst_p,
        "lipschitz_site_remainder": remainder,
        "max_disk_L1_plus_remainder": worst + remainder,
        "crude_one_cell_mass": crude_mass,
        "pointwise_phi_bound": pointwise.value,
    }


def observer_total_mass_with_error(observer, vertices: np.ndarray, mass_plane: float) -> dict[str, Any]:
    """Observer.total_mass uses scipy.quad (epsabs=1e-11), independent of polar ntheta.

    A rigorous upper bound is the plane mass. The quad residual is taken as 1e-10
    (one digit above the requested absolute tolerance) so B_J_geom remains valid.
    """
    m_obs = float(observer.total_mass(vertices))
    quad_eps = 1e-10
    if m_obs > mass_plane + 1e-8:
        # Plane mass clips only by dropping Gaussian tails outside D; observer
        # should not exceed it. Keep the larger number rather than silently clip.
        mass_for_geom = m_obs + quad_eps
        note = "observer mass exceeded plane majorant; using observer+eps for geometry"
    else:
        mass_for_geom = m_obs + quad_eps
        note = "M_geom = observer.total_mass + 1e-10; plane majorant is a coarser envelope"
    return {
        "M_observer": m_obs,
        "M_plane_upper": float(mass_plane),
        "quad_epsabs_requested": 1e-11,
        "eps_M_used": quad_eps,
        "M_for_geometric_J": mass_for_geom,
        "M_observer_le_plane": bool(m_obs <= mass_plane + 1e-8),
        "note": note,
        "status": "numerical_a_priori",
        "strict_certificate": False,
        "reason": "scipy.quad remainder is requested, not a proved enclosure",
    }


def observer_resolution_envelope(
    observers: dict[str, Any],
    positions: np.ndarray,
    vertices: np.ndarray,
) -> dict[str, Any]:
    """Compare H, g, m, c on the *same* site using several polar observers.

    All four fields come from one evaluate() call per observer. Differences are
    a numerical error assessment; they are not a strict certificate, even if
    they shrink under refinement.
    """
    evals = {}
    for name, obs in observers.items():
        ev = obs.evaluate(positions, vertices)
        evals[name] = {
            "H": float(ev["H"]),
            "mass_sum": float(np.sum(ev["mass"])),
            "centroid": np.asarray(ev["centroid"], dtype=float),
            "gradient": np.asarray(ev["gradient"], dtype=float),
        }
    names = list(evals)
    base = names[0]
    comparisons = {}
    for name in names[1:]:
        dH = abs(evals[name]["H"] - evals[base]["H"])
        dm = abs(evals[name]["mass_sum"] - evals[base]["mass_sum"])
        dc = float(np.max(np.linalg.norm(evals[name]["centroid"] - evals[base]["centroid"], axis=1)))
        dg = float(np.linalg.norm(evals[name]["gradient"] - evals[base]["gradient"]))
        comparisons[f"{name}_vs_{base}"] = {
            "abs_dH": dH,
            "abs_d_mass_sum": dm,
            "max_centroid_shift": dc,
            "gradient_l2": dg,
        }
    return {
        "status": "numerical_a_priori",
        "strict_certificate": False,
        "reason": "no_rigorous_quadrature_error_bound",
        "H": {k: v["H"] for k, v in evals.items()},
        "mass_sum": {k: v["mass_sum"] for k, v in evals.items()},
        "comparisons": comparisons,
        "note": "H, g, m, c taken from the same observer.evaluate per resolution.",
    }


def assemble_prior(
    cfg: dict,
    vertices: np.ndarray,
    positions0: np.ndarray,
    observer,
    observers_extra: dict[str, Any] | None,
    k_steps: int,
    density_mesh: int = 240,
    restrict_site_grid: int = 12,
    restrict_local_mesh: int = 36,
    geometry_cache: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Compute every prior constant *before* the closed-loop run is used."""
    ctrl = cfg["controller"]
    domain = cfg["domain"]
    xmin, xmax = float(domain["xmin"]), float(domain["xmax"])
    ymin, ymax = float(domain["ymin"]), float(domain["ymax"])
    area = (xmax - xmin) * (ymax - ymin)
    r = float(ctrl["local_radius"])
    n_grid = int(ctrl["grid_resolution"])
    n_agents = int(cfg["agents"]["count"])
    sigma = float(ctrl["sigma"])
    phi0 = float(ctrl["base_density"])
    kc = float(ctrl["kp_cage"])
    dt = float(cfg["dt"])
    umax = float(ctrl["max_speed"])
    spacing = float(ctrl["theorem_oracle_spacing"])
    influence = float(ctrl.get("influence_sigmas", 3.0))
    L = polygon_perimeter(vertices)
    h = controller_grid_spacing(r, n_grid)
    a = sampling_period_a(kc, dt)

    m_plane = plane_mass_upper(phi0, area, sigma, L)
    mass_info = observer_total_mass_with_error(observer, vertices, m_plane)
    b_h0_crude = h_star_crude_upper(r, m_plane)

    ev0 = observer.evaluate(positions0, vertices)
    h0_obs = float(ev0["H"])
    envelope = None
    if observers_extra:
        envelope = observer_resolution_envelope(
            {"eval": observer, **observers_extra}, positions0, vertices
        )
        dH = 0.0
        for row in envelope["comparisons"].values():
            dH = max(dH, float(row["abs_dH"]))
        b_h0_p0 = h0_obs + dH
        b_h0_p0_status = "numerical_a_priori"
    else:
        b_h0_p0 = h0_obs
        b_h0_p0_status = "numerical_a_priori"

    if geometry_cache is not None and "mismatch" in geometry_cache:
        mismatch = geometry_cache["mismatch"]
        restrict_num = geometry_cache["restrict"]
    else:
        mismatch = numerical_density_mismatch(
            vertices, cfg, observer, spacing, mesh=density_mesh
        )
        restrict_num = numerical_restrict_mass_majorant(
            vertices,
            cfg,
            spacing,
            r,
            site_grid=restrict_site_grid,
            local_mesh=restrict_local_mesh,
        )
        if geometry_cache is not None:
            geometry_cache["mismatch"] = mismatch
            geometry_cache["restrict"] = restrict_num
    phi_max_loose = phi0 + L
    n_edges = int(len(np.asarray(vertices, dtype=float).reshape(-1, 2)))
    phi_max_line = line_edge_phi_max(phi0, sigma, n_edges)
    # Mesh maxima are diagnostic only — never promoted to a global phi upper bound.
    phi_max_mesh = max(
        float(mismatch["phi_star_max_mesh"]),
        float(mismatch["phi_ctrl_max_mesh"]),
        phi0,
    )
    lip_phi = kernel_lipschitz(sigma) * L
    grid_m, grid_mu = grid_mass_moment_errors(phi_max_loose, lip_phi, r, h, n_agents)
    grid_m_mesh, grid_mu_mesh = grid_mass_moment_errors(phi_max_line, lip_phi, r, h, n_agents)

    oracle_l1 = oracle_midpoint_l1_bound(L, spacing, sigma)
    dm_den = oracle_l1.value
    dmu_den = r * oracle_l1.value

    # Restrict: partition-tail (rigorous) replaces N-fold disk tube.
    # W = L for static oracle (arc weights, confidence 1, no explore/gap).
    weight_sum = float(L)
    restrict_part = restrict_partition_mass_bound(weight_sum, sigma, influence)
    dm_restrict_rig = float(restrict_part.value)
    dmu_restrict_rig = r * dm_restrict_rig
    dm_restrict_num = min(
        dm_restrict_rig, n_agents * float(restrict_num["max_disk_L1"])
    )
    dmu_restrict_num = r * dm_restrict_num

    method = str(ctrl.get("integration_method", "endpoint_grid"))
    n_gon = int(ctrl.get("edge_n_gon", 256))
    panels = int(ctrl.get("edge_panels", 48))
    n_sources = int(math.ceil(L / max(spacing, 1e-9))) + n_edges

    from dbact.edge_green_integral import (
        inscribed_disk_area_deficit,
        partition_edge_mass_moment_remainder,
    )

    if method == "edge_green":
        deficit = inscribed_disk_area_deficit(r, n_gon)
        # Missing sets Omega_i \\ P_i are disjoint => sum areas <= N * deficit
        # still holds; using phi_max_line (proved), not mesh max.
        dm_grid = n_agents * phi_max_line * deficit
        dmu_grid = r * dm_grid
        edge_rem = partition_edge_mass_moment_remainder(
            n_agents, n_gon, n_sources, sigma, r, panels
        )
        dm_grid += float(edge_rem["sum_abs_dm"])
        dmu_grid += float(edge_rem["sum_abs_dmu"])
        dm_grid_mesh = dm_grid
        dmu_grid_mesh = dmu_grid
        grid_note = {
            "method": "edge_green",
            "n_gon": n_gon,
            "panels": panels,
            "area_deficit_one_disk": deficit,
            "phi_max_line_edges": phi_max_line,
            "n_edges": n_edges,
            "n_sources_majorant": n_sources,
            "edge_remainder": edge_rem,
            "status": "rigorous",
        }
    else:
        dm_grid = grid_m.value
        dmu_grid = grid_mu.value
        dm_grid_mesh = grid_m_mesh.value
        dmu_grid_mesh = grid_mu_mesh.value
        edge_rem = None
        grid_note = {
            "method": "endpoint_grid",
            "status": "rigorous",
            "note": "N-fold Davenport tube; typically inactive vs diameter at n<=80",
        }

    dm_obs_num = 0.0
    dmu_obs_num = 0.0
    if envelope is not None:
        dm_one = 0.0
        dc_one = 0.0
        for row in envelope["comparisons"].values():
            dm_one = max(dm_one, float(row["abs_d_mass_sum"]))
            dc_one = max(dc_one, float(row["max_centroid_shift"]))
        dm_obs_num = dm_one
        dmu_obs_num = r * dm_one + float(np.sum(ev0["mass"])) * dc_one

    dm_rig = dm_den + dm_restrict_rig + dm_grid
    dmu_rig = dmu_den + dmu_restrict_rig + dmu_grid
    b_e_moment = moment_form_E_bound(r, dm_rig, dmu_rig)
    b_e_diameter = diameter_E_bound(r, m_plane)
    b_e_rigorous = min(b_e_moment, float(b_e_diameter.value))
    b_e_active = (
        "diameter_4R2M" if float(b_e_diameter.value) <= b_e_moment + 1e-15 else "moment_form"
    )

    dm_num = dm_den + dm_restrict_num + dm_grid_mesh + dm_obs_num
    dmu_num = dmu_den + dmu_restrict_num + dmu_grid_mesh + dmu_obs_num
    b_e_moment_num = moment_form_E_bound(r, dm_num, dmu_num)
    b_e_numerical = min(b_e_moment_num, float(b_e_diameter.value))

    # Geometric J uses the analytic plane mass (strict), not scipy.quad.
    geom = geometric_J_bound(m_plane, 0.0, umax)
    geom["M_observer_quad"] = mass_info["M_observer"]
    geom["M_plane_used"] = m_plane
    geom["note"] = (
        "B_J_geom = M_plane * u_max^2 with M_plane = phi0|D| + 2 pi sigma^2 L. "
        "scipy.quad observer mass is recorded but not used in the certificate comparator."
    )
    b_h0_cert = b_h0_crude  # only the proved R^2 M_plane term
    j_cert = prior_J_bound(b_h0_crude, b_e_rigorous, a, k_steps, dt)
    j_p0h = prior_J_bound(b_h0_p0, b_e_rigorous, a, k_steps, dt)
    j_num = prior_J_bound(b_h0_p0, b_e_numerical, a, k_steps, dt)

    e_star = (geom["B_J_geom"] - j_cert["term_2BH0_over_aKD"]) * a * a / 4.0

    return {
        "labels": {
            "B_E_rigorous": "rigorous",
            "B_E_numerical_a_priori": "numerical_a_priori",
            "B_E_diameter": "rigorous",
            "B_H0_crude": "rigorous",
            "B_H0_P0": b_h0_p0_status,
            "B_J_geom": "rigorous",
            "M_observer_quad": "numerical_a_priori_not_certificate",
            "phi_max_mesh": "diagnostic_not_global_upper_bound",
            "E_bar_from_trajectory": "post_hoc",
        },
        "parameters": {
            "N": n_agents,
            "R": r,
            "k_c": kc,
            "dt": dt,
            "a": a,
            "K": int(k_steps),
            "u_max": umax,
            "sigma": sigma,
            "phi0": phi0,
            "grid_resolution": n_grid,
            "h_controller": h,
            "oracle_spacing": spacing,
            "influence_sigmas": influence,
            "perimeter_L": L,
            "domain_area": area,
            "phi_max_loose_phi0_plus_L": phi_max_loose,
            "phi_max_line_edges": phi_max_line,
            "phi_max_mesh_diagnostic": phi_max_mesh,
            "Lip_phi": lip_phi,
            "integration_method": method,
            "edge_n_gon": n_gon,
            "edge_panels": panels,
            "n_edges": n_edges,
            "weight_sum_W": weight_sum,
        },
        "mass": mass_info,
        "H0": {
            "H_observer_P0": h0_obs,
            "B_H0_crude_R2M": b_h0_crude,
            "B_H0_P0": b_h0_p0,
            "B_H0_certificate": b_h0_cert,
            "observer_envelope": envelope,
            "note": "Certificate B_H0 uses R^2 M_plane only. H*(P0)+polar envelope is numerical_a_priori.",
        },
        "density_mismatch": {
            "analytic": oracle_l1.as_dict(),
            "numerical_mesh": mismatch,
        },
        "restrict": {
            "pointwise": restrict_pointwise_bound(L, influence).as_dict(),
            "partition_mass": restrict_part.as_dict(),
            "numerical_site_grid": restrict_num,
            "legacy_N_fold_disk_mass": n_agents * float(restrict_num["crude_one_cell_mass"]),
        },
        "grid": {
            "h": h,
            "mass_loose_phimax": grid_m.as_dict(),
            "moment_loose_phimax": grid_mu.as_dict(),
            "mass_line_phimax": grid_m_mesh.as_dict(),
            "moment_line_phimax": grid_mu_mesh.as_dict(),
            "e_area_one_cell": convex_cell_area_error(r, h),
            "integration": grid_note,
        },
        "error_budget_rigorous": {
            "sum_abs_dm": dm_rig,
            "sum_abs_dmu": dmu_rig,
            "oracle_dm": dm_den,
            "restrict_dm": dm_restrict_rig,
            "grid_dm": dm_grid,
            "observer_dm": 0.0,
            "observer_dm_note": "polar observer remainder not rigorously closed; excluded from the certificate budget",
        },
        "error_budget_numerical_a_priori": {
            "sum_abs_dm": dm_num,
            "sum_abs_dmu": dmu_num,
            "oracle_dm": dm_den,
            "restrict_dm": dm_restrict_num,
            "grid_dm": dm_grid_mesh,
            "observer_dm": dm_obs_num,
            "observer_dmu": dmu_obs_num,
            "lipschitz_covering_remainders_not_used": {
                "density_L1_remainder": mismatch["L1_remainder_lipschitz"],
                "restrict_site_remainder": restrict_num["lipschitz_site_remainder"],
                "reason": "global Lip times domain area is too loose to be a useful majorant; recorded but not added to B_E",
            },
        },
        "B_E": {
            "diameter": b_e_diameter.as_dict(),
            "moment_rigorous_uncapped": b_e_moment,
            "moment_numerical_uncapped": b_e_moment_num,
            "moment_rigorous": b_e_rigorous,
            "moment_numerical_a_priori": b_e_numerical,
            "active_rigorous_bound": b_e_active,
            "E_star_to_meet_geometry_using_crude_H": e_star,
            "formula_moment": "2 R (sum ||dmu|| + R sum |dm|)",
            "note": (
                "Certificate uses min(moment form, 4 R^2 M_plane). "
                "With edge_green + partition restrict, moment form is intended to be active."
            ),
        },
        "B_J": {
            "geometric": geom,
            "prior_certificate_crudeH_rigorousE": j_cert,
            "prior_P0H_rigorousE": j_p0h,
            "prior_numerical_a_priori": j_num,
        },
        "beats_geometry": {
            "certificate_crudeH_rigorousE": bool(j_cert["B_J_prior"] < geom["B_J_geom"]),
            "P0H_rigorousE": bool(j_p0h["B_J_prior"] < geom["B_J_geom"]),
            "numerical_a_priori": bool(j_num["B_J_prior"] < geom["B_J_geom"]),
        },
        "grid_refinement_narrative": {
            "decreases": [
                "edge_green: inscribed n-gon deficit O(1/n_gon^2) and trapezoid panels",
                "endpoint_grid baseline: LocalCVT O(h) tube (usually inactive vs diameter)",
                "observer polar remainder if the evaluation grid is refined (not the controller grid)",
            ],
            "floors": [
                "oracle midpoint vs continuous edge measure (depends on theorem_oracle_spacing)",
                "density.restrict partition Gaussian tail at influence_sigmas (no N factor)",
                "controller discrete mixture vs observer erf line density: same kernel, different edge discretisation",
            ],
        },
        "forbidden": "Do not substitute trajectory max/mean E for B_E.",
    }


__all__ = [
    "BoundTerm",
    "assemble_prior",
    "config_fingerprint",
    "controller_grid_spacing",
    "geometric_J_bound",
    "kernel_lipschitz",
    "line_edge_phi_max",
    "moment_form_E_bound",
    "observer_resolution_envelope",
    "overlay_grid_resolution",
    "plane_mass_upper",
    "prior_J_bound",
    "restrict_partition_mass_bound",
    "restrict_pointwise_bound",
    "sampling_period_a",
    "step_in_k0",
]
