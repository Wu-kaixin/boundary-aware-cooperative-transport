"""S1 - safety layer: one QP carrying two families of barrier constraints.

Inter-robot rows use the pairwise distance barrier
``h_ij = ||p_i - p_j||^2 - d_min^2`` in *shared responsibility* form: each robot
takes half of the required decrease rate,

    2 (p_i - p_j)^T u_i  >=  -(gamma/2) h_ij ,

so no robot needs to know its neighbour's input. This is the construction of
Wang, Ames and Egerstedt (T-RO 2017); it is used here to support the feasibility
proposition and is not claimed as a contribution.

The legacy object-boundary mode uses tangent-plane rows
``h_k = n_k^T (p_i - b_k) - r_safe``.  It remains for ablation, but it cannot be
the arbitrary-shape safety certificate: intersecting several local "outside"
half-spaces over-constrains the free space at polygon corners.

The research mode instead reconstructs locally continuous line segments only
between bearing-adjacent returns whose gap and normal change are bounded.  It
selects the nearest valid point/segment feature ``q*`` and uses
``h = ||p_i-q*|| - r_safe - epsilon_b`` with radial gradient
``n = (p_i-q*)/||p_i-q*||``.  Discontinuities and corners are split rather than
bridged.  With the nearest feature fixed locally, the row is

    n_k^T u_i  >=  n_k^T v_obj - gamma_obj h_k + rho .

``rho`` is not a tuning margin. It bounds the measured boundary-point velocity
error, including unmodelled rotation, and makes the statement input-to-state
safe (ISSf). ``epsilon_b`` is the independently measured boundary-point error.
Both numerical values belong in the paper and are fail-closed runtime gates.

The tangential window
---------------------
A local plane represents the boundary only near the point it was fitted at, so
each row is admitted only while the robot stays inside a tangential window ``W``
around its own boundary point. Without that window the construction silently
becomes a convex-hull constraint: on a non-convex object the tangent plane of a
point half a metre away, around a corner, cuts through a robot that is in fact
0.36 m clear of the true boundary. Measured on the L shape, this alone made the
QP infeasible on 65 of 600 solves -- not a solver problem but a modelling one.

``W`` is also what bounds the effect of normal-estimate error: a normal wrong by
``eps_n`` misplaces the plane by at most ``W sin(eps_n)`` over the window, so the
window size is part of the barrier construction rather than an implementation
detail, and it is what lets the perception error budget be turned into a safety
margin.

The inner limit
---------------
A robot's map also holds points relayed by neighbours, including points on the
*far* face of a thin part of the object. A robot standing safely outside one face
is, by construction, on the inner side of the opposite face's plane, and that
plane then reports a large negative barrier for a robot that is in no danger at
all. On the L shape (leg thickness 0.405 m) this produced a demand to retreat at
full speed while a neighbour blocked the way -- infeasible, from a robot with
0.16 m of true clearance. Rows with ``n_k^T (p_i - b_k) < -inner_limit`` are
therefore describing a different part of the object rather than this robot's local
boundary, and are dropped. The limit is the robot radius: any larger negative
offset would mean the robot centre is past the boundary, at which point the run
has already failed its penetration invariant and is reported as such.

Feasibility certificate: with ``u_i = 0`` the inter-robot rows hold whenever
``h_ij >= 0`` and the object rows hold whenever
``gamma_obj h_k >= n_k^T v_obj + rho``. The QP is therefore feasible without any
slack variable, which is why none is present -- a soft quadratic penalty can
never drive a violation exactly to zero, so reported "zero violation" under a
soft filter would be an artefact of the weight.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .contracts import ContactSafetyContract, ContractViolation, SolverContract
from .qp2d import solve_min_norm_2d, solve_min_norm_2d_cvxpy


@dataclass
class SafetyFilterParams:
    d_min: float = 0.30
    gamma_agent: float = 6.0
    gamma_obj: float = 4.0
    rho: float = 0.05
    r_safe: float = 0.10
    boundary_error_bound: float = 0.0
    max_speed: float = 0.30
    backend: str = "qp"
    enable_object_rows: bool = True
    max_object_rows: int = 12
    object_row_range: float = 0.60
    object_row_window: float = 0.28
    object_row_inner_limit: float = 0.16
    object_barrier_geometry: str = "tangent_plane"
    object_active_tolerance: float = 0.02
    object_polyline_max_gap: float = 0.15
    object_polyline_max_normal_angle_deg: float = 30.0
    recovery_fraction: float = 0.6
    projection_iterations: int = 24


@dataclass
class SafetyFilterStats:
    """Per-run solver provenance. ``fallbacks`` must be zero for a valid run."""

    solves: int = 0
    fallbacks: int = 0
    infeasible: int = 0
    margin_relaxations: int = 0
    zero_input_feasible_checks: int = 0
    # Certificate failures: u = 0 does not satisfy the *barrier*. This is the
    # quantity the feasibility proposition is about.
    zero_input_feasible_failures: int = 0
    # Steps where u = 0 satisfies the barrier but not the ISSf margin band. Not a
    # certificate failure -- the band exists precisely to demand active retreat --
    # so it is counted separately rather than folded into the line above.
    inside_margin_band: int = 0
    max_slack: float = 0.0
    max_modification: float = 0.0
    statuses: dict[str, int] = field(default_factory=dict)

    def record_status(self, status: str) -> None:
        self.statuses[status] = self.statuses.get(status, 0) + 1

    def as_dict(self) -> dict:
        return {
            "solves": self.solves,
            "fallbacks": self.fallbacks,
            "infeasible": self.infeasible,
            "margin_relaxations": self.margin_relaxations,
            "zero_input_feasible": self.zero_input_feasible_failures == 0,
            "zero_input_feasible_checks": self.zero_input_feasible_checks,
            "zero_input_feasible_failures": self.zero_input_feasible_failures,
            "inside_margin_band": self.inside_margin_band,
            "max_slack": self.max_slack,
            "max_modification_norm": self.max_modification,
            "statuses": dict(self.statuses),
        }


@dataclass
class FilterResult:
    velocity: np.ndarray
    status: str
    object_rows: int
    agent_rows: int
    modification: float
    zero_input_feasible: bool
    max_full_margin_deficit: float
    max_barrier_deficit: float
    max_object_margin_deficit: float
    min_object_h: float
    max_object_velocity_projection: float


class SafetyFilter:
    """Combined inter-robot and object-boundary CBF-QP safety filter."""

    def __init__(self, params: SafetyFilterParams, contract: ContactSafetyContract | None = None):
        self.params = params
        self.solver = SolverContract(params.backend)
        self.contract = contract
        if contract is not None:
            contract.assert_valid()
            # The filter and the contract must agree on the same r_safe, otherwise
            # C1 is asserted about a number the controller never uses.
            if abs(contract.r_safe - params.r_safe) > 1e-9:
                raise ContractViolation(
                    f"safety filter r_safe={params.r_safe:.6f} disagrees with the C1 contract "
                    f"r_safe={contract.r_safe:.6f} (robot_radius - delta_max)"
                )
        self.stats = SafetyFilterStats()
        self.last_infeasible_problem: dict[str, np.ndarray] | None = None

    # ------------------------------------------------------------------ #
    # constraint assembly
    # ------------------------------------------------------------------ #

    def _agent_rows(
        self,
        position: np.ndarray,
        neighbor_positions: list[np.ndarray] | np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        if len(neighbor_positions) == 0:
            return np.empty((0, 2)), np.empty(0)
        p = np.asarray(position, dtype=float).reshape(2)
        q = np.asarray(neighbor_positions, dtype=float).reshape(-1, 2)
        diff = p[None, :] - q
        h = np.sum(diff * diff, axis=1) - self.params.d_min ** 2
        A = 2.0 * diff
        b = -0.5 * self.params.gamma_agent * h
        return A, b

    def _object_rows(
        self,
        position: np.ndarray,
        boundary_points: np.ndarray,
        boundary_normals: np.ndarray,
        object_velocity: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pts = np.asarray(boundary_points, dtype=float).reshape(-1, 2)
        if len(pts) == 0:
            return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        normals = np.asarray(boundary_normals, dtype=float).reshape(-1, 2)
        p = np.asarray(position, dtype=float).reshape(2)
        velocity_input = np.asarray(object_velocity, dtype=float)
        if velocity_input.shape == (2,):
            point_velocities = np.repeat(velocity_input[None, :], len(pts), axis=0)
        else:
            point_velocities = velocity_input.reshape(-1, 2)
            if len(point_velocities) != len(pts):
                raise ValueError("one object point velocity is required per boundary point")

        rel = p[None, :] - pts
        if self.params.object_barrier_geometry in {"point_distance", "polyline_distance"}:
            distances = np.linalg.norm(rel, axis=1)
            near = distances <= self.params.object_row_range
            if not np.any(near):
                return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
            candidate_points = pts[near]
            candidate_normals = normals[near]
            candidate_velocities = point_velocities[near]
            if self.params.object_barrier_geometry == "polyline_distance" and len(pts) >= 2:
                edges = np.diff(pts, axis=0)
                lengths = np.linalg.norm(edges, axis=1)
                cosine_limit = np.cos(np.radians(self.params.object_polyline_max_normal_angle_deg))
                aligned = np.sum(normals[:-1] * normals[1:], axis=1) >= cosine_limit
                valid = (
                    (lengths > 1e-9)
                    & (lengths <= self.params.object_polyline_max_gap)
                    & aligned
                )
                if np.any(valid):
                    starts = pts[:-1][valid]
                    segments = edges[valid]
                    denom = np.sum(segments * segments, axis=1)
                    fraction = np.clip(
                        np.sum((p[None, :] - starts) * segments, axis=1) / denom,
                        0.0,
                        1.0,
                    )
                    footpoints = starts + fraction[:, None] * segments
                    candidate_points = np.vstack([candidate_points, footpoints])
                    candidate_normals = np.vstack(
                        [candidate_normals, normals[:-1][valid] + normals[1:][valid]]
                    )
                    segment_velocities = point_velocities[1:] - point_velocities[:-1]
                    footpoint_velocities = (
                        point_velocities[:-1][valid]
                        + fraction[:, None] * segment_velocities[valid]
                    )
                    candidate_velocities = np.vstack(
                        [candidate_velocities, footpoint_velocities]
                    )
            candidate_rel = p[None, :] - candidate_points
            candidate_distances = np.linalg.norm(candidate_rel, axis=1)
            best = int(np.argmin(candidate_distances))
            distance = float(candidate_distances[best])
            if distance > 1e-12:
                normals = candidate_rel[best : best + 1] / distance
            else:
                fallback = candidate_normals[best]
                normals = fallback[None, :] / max(float(np.linalg.norm(fallback)), 1e-12)
            row_velocities = candidate_velocities[best : best + 1]
            effective_r_safe = self.params.r_safe + self.params.boundary_error_bound
            h = np.array([distance - effective_r_safe], dtype=float)
            if self.params.object_barrier_geometry == "point_distance":
                # Point-only ablations may expose the complete exact-tie active
                # set.  The polyline controller intentionally uses one nearest
                # differentiable feature.
                point_h = distances[near] - effective_r_safe
                active = point_h <= float(np.min(point_h)) + self.params.object_active_tolerance
                point_rel = rel[near][active]
                point_distances = distances[near][active]
                normals = point_rel / np.maximum(point_distances[:, None], 1e-12)
                h = point_h[active]
                row_velocities = point_velocities[near][active]
        else:
            if self.params.object_barrier_geometry != "tangent_plane":
                raise ValueError(
                    "object_barrier_geometry must be 'tangent_plane', 'point_distance' "
                    "or 'polyline_distance'"
                )
            normal_offset = np.sum(normals * rel, axis=1)
            tangential_offset = np.abs(normals[:, 0] * rel[:, 1] - normals[:, 1] * rel[:, 0])

            # Keep only rows whose local plane still describes the boundary near this
            # robot: within reach along the normal, and inside the tangential window.
            near = (
                (np.linalg.norm(rel, axis=1) <= self.params.object_row_range)
                & (tangential_offset <= self.params.object_row_window)
                & (normal_offset >= -self.params.object_row_inner_limit)
            )
            if not np.any(near):
                return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
            normals, normal_offset = normals[near], normal_offset[near]
            row_velocities = point_velocities[near]
            h = normal_offset - self.params.r_safe - self.params.boundary_error_bound

        if len(h) > self.params.max_object_rows:
            keep = np.argsort(h)[: self.params.max_object_rows]
            normals, h, row_velocities = normals[keep], h[keep], row_velocities[keep]

        # Saturate only the recovery term.  Capping the complete RHS would also
        # cap ``n.T v_obj + rho`` at h=0 and destroy forward invariance exactly on
        # the safe-set boundary.  The saturated recovery remains an extended
        # class-K response for h>=0, while avoiding an impossible demand after a
        # state has already crossed the boundary.
        cap = self.params.recovery_fraction * self.params.max_speed
        recovery = np.minimum(-self.params.gamma_obj * h, cap)
        velocity_projection = np.sum(normals * row_velocities, axis=1)
        rhs_barrier = velocity_projection + recovery
        rhs = rhs_barrier + self.params.rho
        return normals, rhs, rhs_barrier, h

    # ------------------------------------------------------------------ #
    # solve
    # ------------------------------------------------------------------ #

    def filter_velocity(
        self,
        position: np.ndarray,
        nominal_velocity: np.ndarray,
        neighbor_positions: list[np.ndarray] | np.ndarray = (),
        boundary_points: np.ndarray | None = None,
        boundary_normals: np.ndarray | None = None,
        object_velocity: np.ndarray | None = None,
    ) -> FilterResult:
        u_nom = np.asarray(nominal_velocity, dtype=float).reshape(2)
        speed = float(np.linalg.norm(u_nom))
        if speed > self.params.max_speed:
            u_nom = u_nom * (self.params.max_speed / speed)

        A_agent, b_agent = self._agent_rows(position, list(neighbor_positions))
        if not self.params.enable_object_rows or boundary_points is None or len(boundary_points) == 0:
            A_obj, b_obj, b_obj_barrier, h_obj = (
                np.empty((0, 2)),
                np.empty(0),
                np.empty(0),
                np.empty(0),
            )
        else:
            A_obj, b_obj, b_obj_barrier, h_obj = self._object_rows(
                position,
                boundary_points,
                boundary_normals if boundary_normals is not None else np.zeros_like(boundary_points),
                object_velocity if object_velocity is not None else np.zeros(2),
            )

        A = np.vstack([A_agent, A_obj]) if len(A_agent) or len(A_obj) else np.empty((0, 2))
        b = np.concatenate([b_agent, b_obj]) if len(b_agent) or len(b_obj) else np.empty(0)
        # Same rows with the ISSf robustness margin removed. Used only when the
        # margin itself is what makes the problem infeasible.
        b_no_margin = np.concatenate([b_agent, b_obj_barrier]) if len(b) else b

        # The certificate is about the barrier, so it is evaluated against the
        # margin-free right-hand side. Evaluating it with rho included would report
        # a certificate failure every time a robot sits in the ISSf band, which is
        # the intended operating point at the cage ring, not a violation.
        zero_feasible = bool(len(b_no_margin) == 0 or np.all(b_no_margin <= 1e-9))
        self.stats.zero_input_feasible_checks += 1
        if not zero_feasible:
            self.stats.zero_input_feasible_failures += 1
        elif len(b) and np.any(b > 1e-9):
            self.stats.inside_margin_band += 1

        self.stats.solves += 1
        u, status = self._solve(u_nom, A, b, b_no_margin)
        self.stats.record_status(status)

        deficits = b - A @ u if len(A) else np.empty(0)
        barrier_deficits = b_no_margin - A @ u if len(A) else np.empty(0)
        object_deficits = deficits[len(A_agent) :] if len(A_obj) else np.empty(0)
        recovery = np.minimum(
            -self.params.gamma_obj * h_obj,
            self.params.recovery_fraction * self.params.max_speed,
        )
        velocity_projection = b_obj_barrier - recovery if len(A_obj) else np.empty(0)

        modification = float(np.linalg.norm(u - u_nom))
        self.stats.max_modification = max(self.stats.max_modification, modification)
        return FilterResult(
            velocity=u,
            status=status,
            object_rows=len(A_obj),
            agent_rows=len(A_agent),
            modification=modification,
            zero_input_feasible=zero_feasible,
            max_full_margin_deficit=max(0.0, float(np.max(deficits))) if len(deficits) else 0.0,
            max_barrier_deficit=(
                max(0.0, float(np.max(barrier_deficits))) if len(barrier_deficits) else 0.0
            ),
            max_object_margin_deficit=(
                max(0.0, float(np.max(object_deficits))) if len(object_deficits) else 0.0
            ),
            min_object_h=float(np.min(h_obj)) if len(h_obj) else float("inf"),
            max_object_velocity_projection=(
                float(np.max(np.abs(velocity_projection))) if len(velocity_projection) else 0.0
            ),
        )

    def _solve(
        self, u_nom: np.ndarray, A: np.ndarray, b: np.ndarray, b_no_margin: np.ndarray
    ) -> tuple[np.ndarray, str]:
        """Two-tier solve, then fail.

        Tier 1 asks for the full constraint set including the ISSf margin ``rho``.
        Tier 2 drops ``rho`` from the object rows. That distinction matters: the
        safety property is ``h >= 0``, whereas ``rho`` is the robustness margin
        that absorbs the neglected ``d/dt(n)`` term. A robot wedged between two
        neighbours at exactly ``d_min`` while the margin band asks it to retreat
        has no feasible input -- but giving up the *margin* still keeps
        ``h_dot >= -gamma h``, so the barrier itself is intact. Relaxations are
        counted and reported rather than hidden, because the ISSf constant only
        holds for steps where the margin was actually enforced.

        Tier 3 -- infeasible even without the margin -- is a modelling failure
        rather than a solver failure, so it does not raise here. It is counted as
        a fallback, and C3 rejects any run whose fallback count is non-zero.
        """
        backend = self.params.backend
        if backend == "projection":
            return self._project(u_nom, A, b), "projection"

        for rhs, relaxed in ((b, False), (b_no_margin, True)):
            if backend == "cvxpy":
                try:
                    solution = solve_min_norm_2d_cvxpy(u_nom, A, rhs, self.params.max_speed)
                except Exception as exc:  # missing solver, numerical error, ...
                    self.solver.on_solver_failure(f"cvxpy raised {type(exc).__name__}: {exc}")
                    self.stats.fallbacks += 1
                    return self._project(u_nom, A, b), "fallback_projection"
            else:
                solution = solve_min_norm_2d(u_nom, A, rhs, self.params.max_speed)
            if solution.feasible:
                if relaxed:
                    self.stats.margin_relaxations += 1
                    return solution.u, "relaxed_margin"
                return solution.u, "optimal"
            if self.params.rho <= 0.0:
                break

        self.stats.infeasible += 1
        self.stats.fallbacks += 1
        self.last_infeasible_problem = {
            "u_nom": np.asarray(u_nom, dtype=float).copy(),
            "A": np.asarray(A, dtype=float).copy(),
            "b_full": np.asarray(b, dtype=float).copy(),
            "b_barrier": np.asarray(b_no_margin, dtype=float).copy(),
        }
        return self._project(u_nom, A, b), "fallback_projection"

    def _project(self, u_nom: np.ndarray, A: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Iterated half-plane projection. Explicit, inexact baseline only."""
        u = np.asarray(u_nom, dtype=float).reshape(2).copy()
        if len(A) == 0:
            return self._cap(u)
        norms2 = np.sum(A * A, axis=1)
        for _ in range(self.params.projection_iterations):
            for k in range(len(A)):
                if norms2[k] < 1e-12:
                    continue
                violation = b[k] - float(np.dot(A[k], u))
                if violation > 0.0:
                    u = u + (violation / norms2[k]) * A[k]
            u = self._cap(u)
        return u

    def _cap(self, u: np.ndarray) -> np.ndarray:
        speed = float(np.linalg.norm(u))
        if speed <= self.params.max_speed:
            return u
        return u * (self.params.max_speed / speed)


__all__ = ["SafetyFilter", "SafetyFilterParams", "SafetyFilterStats", "FilterResult"]
