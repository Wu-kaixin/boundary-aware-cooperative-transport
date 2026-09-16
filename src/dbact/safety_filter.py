"""S1 - safety layer: one QP carrying two families of barrier constraints.

Inter-robot rows use the pairwise distance barrier
``h_ij = ||p_i - p_j||^2 - d_min^2`` in *shared responsibility* form: each robot
takes half of the required decrease rate,

    2 (p_i - p_j)^T u_i  >=  -(gamma/2) h_ij ,

so no robot needs to know its neighbour's input. This is the construction of
Wang, Ames and Egerstedt (T-RO 2017); it is used here to support the feasibility
proposition and is not claimed as a contribution.

Object-boundary rows use one row per nearby observed boundary point. The true
barrier for a point cloud is a pointwise minimum and therefore nonsmooth, so it
is realised as the intersection of the corresponding half-spaces rather than as
a single smooth CBF -- the nonsmooth barrier construction of Glotfelter, Cortes
and Egerstedt (L-CSS 2017). With ``h_k = n_k^T (p_i - b_k) - r_safe``,

    n_k^T u_i  >=  n_k^T v_obj - gamma_obj h_k + rho .

``rho`` is not a tuning margin. It is the price of dropping the
``d/dt(n_k)^T (p_i - b_k)`` term, and it is what turns the exact CBF statement
into an input-to-state-safe (ISSf) one. Its numerical value belongs in the paper.

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

# A row ``a^T u >= b`` counts as active when the returned input sits on it to
# within the solver's own arithmetic. Diagnostic only -- nothing in the control
# path branches on this.
_ACTIVE_ROW_TOLERANCE = 1e-6


@dataclass
class SafetyFilterParams:
    d_min: float = 0.30
    gamma_agent: float = 6.0
    gamma_obj: float = 4.0
    rho: float = 0.05
    r_safe: float = 0.10
    max_speed: float = 0.30
    backend: str = "qp"
    enable_object_rows: bool = True
    max_object_rows: int = 12
    object_row_range: float = 0.60
    object_row_window: float = 0.28
    object_row_inner_limit: float = 0.16
    recovery_fraction: float = 0.6
    projection_iterations: int = 24
    # Rows whose normal disagrees with the robot's nearest row by more than this
    # angle describe a different face of the object. See ``_object_rows``.
    object_row_face_cosine: float = 0.26  # cos(75 deg)
    # Upper bound on the object-velocity estimate admitted into the barrier. This
    # is the ISSf disturbance bound, so it belongs in the configuration next to
    # ``rho`` rather than being whatever the estimator happened to produce.
    object_velocity_bound: float = 0.20
    # Number of bisection steps used by the last relaxation tier.
    barrier_scale_steps: int = 8
    # "aggregate" represents a face by one smooth weighted plane; "pointwise" keeps
    # one row per map sample, which is the pre-T4 construction and the ablation.
    # "nearest_feature" uses Euclidean distance to every polygon *segment* in the
    # one-step cover Φ(p) = {edges : dist(p,edge) ≤ sd(p)+2 u_max Δ} ∩ range.
    # Dist-to-segment is convex, so a discrete CBF row per covered edge proves
    # invariance of true signed distance, including feature switches.  Infinite
    # planes of non-nearest edges are forbidden (C5/L11/L17).
    object_row_mode: str = "aggregate"
    # Control period, used only to check that the sampled barrier condition is a
    # valid discrete-time CBF: ``gamma_obj * dt <= 1``.
    dt: float = 0.05
    # Sampled theorem_mode: wall rows keep P+ΔU inside D; forbidden fallback means
    # an empty feasible set aborts rather than projecting through the constraints.
    enable_wall_rows: bool = False
    domain: tuple[float, float, float, float] | None = None
    forbid_fallback: bool = False
    allow_object_barrier_scaling: bool = True
    # Theorem-mode option: if the hard (rho-free) object barrier admits u=0 but the
    # ISSf margin does not, clamp those object RHS entries up to 0 so that the
    # projection set used by the QP contains 0.  This is a per-row margin
    # relaxation, not a global rho reduction; agent/wall rows are untouched.
    clamp_margin_to_keep_zero: bool = False


@dataclass
class SafetyFilterStats:
    """Per-run solver provenance. ``fallbacks`` must be zero for a valid run."""

    solves: int = 0
    fallbacks: int = 0
    infeasible: int = 0
    margin_relaxations: int = 0
    # Steps that needed the scaled-barrier tier, and the smallest factor used.
    # The inter-robot rows stayed hard on every one of them; what was given up is
    # part of the object barrier's decrease rate, and that is what these report.
    barrier_scalings: int = 0
    min_barrier_scale: float = 1.0
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
            "barrier_scalings": self.barrier_scalings,
            "min_barrier_scale": self.min_barrier_scale,
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
    # Which rows the returned input is actually sitting on. A row count says how
    # many constraints were *written*; an active count says how many were binding,
    # and only the second distinguishes "the QP shaped this command" from "the QP
    # passed the nominal input through". Diagnostic output, not a control signal.
    agent_rows_active: int = 0
    object_rows_active: int = 0
    wall_rows: int = 0
    wall_rows_active: int = 0
    barrier_scale: float = 1.0
    agent_residual_min: float = 0.0
    wall_residual_min: float = 0.0
    object_residual_min: float = 0.0
    feasible: bool = True
    # ``zero_input_feasible`` is the barrier certificate (margin-free RHS).
    # ``zero_input_feasible_with_rho`` asks whether u=0 lies in the *final* QP set
    # including object ISSf rows with ``rho``. False + barrier-feasible means the
    # robot is inside the intended margin band (active retreat demanded).
    zero_input_feasible_with_rho: bool = True
    inside_margin_band: bool = False
    # Original (unclamped) object-inclusive RHS vs the RHS actually sent to the QP.
    b_original: np.ndarray | None = None
    b_effective: np.ndarray | None = None
    b_no_margin: np.ndarray | None = None
    A_rows: np.ndarray | None = None
    u_nominal: np.ndarray | None = None
    row_kinds: list[str] | None = None
    h_object: np.ndarray | None = None
    clamp_applied: bool = False
    original_zero_input_feasible_with_rho: bool = True
    empty_feasible_set: bool = False
    zero_not_in_F: bool = False



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
        if params.object_row_mode not in ("aggregate", "pointwise", "nearest_feature"):
            raise ContractViolation(
                f"object_row_mode must be 'aggregate', 'pointwise' or 'nearest_feature', "
                f"got {params.object_row_mode!r}"
            )
        # Discrete-time admissibility. ``h_{t+1} >= (1 - alpha) h_t`` with
        # ``alpha = gamma_obj * dt`` is the discrete-time CBF condition the sampled
        # row implements, and it is only a decrease condition for ``alpha <= 1``.
        # Above that the row asks for more decrease than one step can contain, and
        # the barrier is no longer a barrier -- it is a constraint the integrator
        # cannot honour, which is a modelling error rather than a solver one.
        if params.gamma_obj * params.dt > 1.0 + 1e-9:
            raise ContractViolation(
                f"discrete-time CBF admissibility violated: gamma_obj * dt = "
                f"{params.gamma_obj * params.dt:.4f} > 1 (gamma_obj={params.gamma_obj:.4f}, "
                f"dt={params.dt:.4f}). The sampled object row demands more decrease than a single "
                "step can deliver; lower gamma_obj or the control period"
            )
        self.stats = SafetyFilterStats()

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
        point_velocities: np.ndarray | None = None,
        trace: dict | None = None,
        obstacle_vertices: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Assemble the object-boundary rows.

        ``trace``, if provided, is filled with selection/aggregation internals.
        It is diagnostic only and is not read by the control path.

        ``point_velocities`` (T2) is the estimated velocity of the material point at
        each ``boundary_points`` entry. When it is ``None`` every row is built from
        the single translational ``object_velocity``, which is exactly v1's
        construction and the default. When it is supplied, each row carries its own
        point's velocity, because the barrier ``h_k = n_k^T (p_i - b_k) - r_safe``
        differentiates to ``n_k^T (u_i - v_{b_k})`` and ``v_{b_k}`` is the material
        point's velocity, not the body's translational one. On a rotating object the
        two differ by ``omega |b_k - c|``, which is largest on the widest part of the
        object -- where the pushing robots stand.
        """
        pts = np.asarray(boundary_points, dtype=float).reshape(-1, 2)
        if self.params.object_row_mode == "nearest_feature":
            verts = None if obstacle_vertices is None else np.asarray(obstacle_vertices, dtype=float).reshape(-1, 2)
            if verts is not None and len(verts) >= 3:
                return self._nearest_feature_rows(position, verts, object_velocity, trace=trace)
            if len(pts) == 0:
                if trace is not None:
                    trace.update({"n_raw": 0, "dropped_empty": True, "mode": "nearest_feature_no_samples"})
                return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
            return self._nearest_sample_rows(position, pts, object_velocity, trace=trace)
        if len(pts) == 0:
            if trace is not None:
                trace.update({"n_raw": 0, "n_near": 0, "n_same_face": 0, "dropped_empty": True})
            return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        normals = np.asarray(boundary_normals, dtype=float).reshape(-1, 2)
        p = np.asarray(position, dtype=float).reshape(2)
        v_obj = np.asarray(object_velocity, dtype=float).reshape(2)

        # The estimate is a measurement, and the barrier needs a *bounded*
        # disturbance. An estimator spike enters the right-hand side directly, so
        # an unclamped estimate lets a transient demand a retreat the robot cannot
        # perform: measured, spikes to 0.21 m/s produced rows demanding the full
        # recovery rate on faces the robot was 0.20 m clear of.
        speed = float(np.linalg.norm(v_obj))
        if speed > self.params.object_velocity_bound:
            v_obj = v_obj * (self.params.object_velocity_bound / speed)

        if point_velocities is None:
            row_velocity = None
        else:
            # The same ISSf disturbance bound, applied per row. It has to be the
            # per-point velocity that is bounded rather than the body's: the row's
            # right-hand side contains ``n_k^T v_{b_k}``, so bounding only the
            # translational part would leave ``omega |b_k - c|`` unbounded and the
            # ISSf constant would be stated over a quantity nothing constrains.
            row_velocity = np.asarray(point_velocities, dtype=float).reshape(-1, 2)
            if len(row_velocity) != len(pts):
                raise ValueError(
                    f"point_velocities has {len(row_velocity)} rows for {len(pts)} boundary "
                    "points; the barrier pairs each row with its own point's velocity, so a "
                    "length mismatch would silently pair a row with another point's motion"
                )
            magnitude = np.linalg.norm(row_velocity, axis=1)
            excess = magnitude > self.params.object_velocity_bound
            if np.any(excess):
                scale = np.ones(len(row_velocity))
                scale[excess] = self.params.object_velocity_bound / magnitude[excess]
                row_velocity = row_velocity * scale[:, None]

        rel = p[None, :] - pts
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
            if trace is not None:
                trace.update(
                    {
                        "n_raw": int(len(pts)),
                        "n_near": 0,
                        "n_same_face": 0,
                        "dropped_empty": True,
                        "near_mask": near.astype(bool).tolist(),
                    }
                )
            return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        near_pts = pts[near]
        normals, normal_offset = normals[near], normal_offset[near]
        distance = np.linalg.norm(rel[near], axis=1)
        if row_velocity is not None:
            row_velocity = row_velocity[near]

        # Face consistency. The tangential window bounds how far a plane is
        # extrapolated, but on a non-convex object two faces can both pass the
        # window from opposite sides of a corner, and their rows then demand
        # retreat in directions up to 180 degrees apart. No input satisfies both,
        # and the QP reports infeasible for a robot that is in no danger at all --
        # measured at 0.20 m of true clearance. The robot's own nearest return
        # names the face it is standing off; rows whose normal disagrees with it by
        # more than the face angle belong to a different face and are dropped.
        anchor_index = int(np.argmin(distance))
        anchor = normals[anchor_index]
        same_face = normals @ anchor >= self.params.object_row_face_cosine
        face_pts = near_pts[same_face]
        face_distance = distance[same_face]
        normals, normal_offset = normals[same_face], normal_offset[same_face]
        if len(normals) == 0:
            if trace is not None:
                trace.update(
                    {
                        "n_raw": int(len(pts)),
                        "n_near": int(np.count_nonzero(near)),
                        "n_same_face": 0,
                        "dropped_empty": True,
                        "anchor": anchor.tolist(),
                    }
                )
            return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        if row_velocity is not None:
            row_velocity = row_velocity[same_face]

        h = normal_offset - self.params.r_safe
        aggregate_trace: dict = {}
        if self.params.object_row_mode == "aggregate":
            normals, h, row_velocity = self._aggregate_face(
                p, face_pts, normals, face_distance, row_velocity, trace=aggregate_trace
            )
        elif len(h) > self.params.max_object_rows:
            keep = np.argsort(h)[: self.params.max_object_rows]
            normals, h = normals[keep], h[keep]
            if row_velocity is not None:
                row_velocity = row_velocity[keep]

        # Both right-hand sides are built from the uncapped expression and capped
        # afterwards. Deriving the margin-free one by subtracting ``rho`` from the
        # *capped* row is wrong whenever the cap binds: the cap is a limit on what
        # the actuator can do, not a term of the barrier, so the subtraction lands
        # on a number that no longer contains ``rho`` and tier 2 relaxes nothing.
        # That is why steps whose barrier was perfectly satisfiable at ``u = 0``
        # were still reaching the scaled tier.
        normal_velocity = (
            normals @ v_obj
            if row_velocity is None
            else np.einsum("ij,ij->i", normals, row_velocity)
        )
        demand = normal_velocity - self.params.gamma_obj * h
        rhs_uncapped = demand + self.params.rho
        rhs = self._cap_to_reachable(normals, rhs_uncapped)
        rhs_no_margin = self._cap_to_reachable(normals, demand)
        if trace is not None:
            witness = normals.sum(axis=0) if len(normals) else np.zeros(2)
            wnorm = float(np.linalg.norm(witness))
            witness_u = witness / wnorm if wnorm > 1e-9 else witness
            reachable = (
                self.params.recovery_fraction * self.params.max_speed * (normals @ witness_u)
                if len(normals)
                else np.empty(0)
            )
            trace.update(
                {
                    "n_raw": int(len(pts)),
                    "n_near": int(np.count_nonzero(near)),
                    "n_same_face": int(len(face_pts)),
                    "dropped_empty": False,
                    "anchor_index_in_near": int(anchor_index),
                    "anchor": np.asarray(anchor, dtype=float).tolist(),
                    "same_face_mask": np.asarray(same_face, dtype=bool).tolist(),
                    "near_points": np.asarray(near_pts, dtype=float).tolist(),
                    "h": np.asarray(h, dtype=float).tolist(),
                    "A": np.asarray(normals, dtype=float).tolist(),
                    "demand": np.asarray(demand, dtype=float).tolist(),
                    "rhs_uncapped": np.asarray(rhs_uncapped, dtype=float).tolist(),
                    "rhs_capped": np.asarray(rhs, dtype=float).tolist(),
                    "rhs_no_margin_uncapped": np.asarray(demand, dtype=float).tolist(),
                    "rhs_no_margin_capped": np.asarray(rhs_no_margin, dtype=float).tolist(),
                    "reachable": np.asarray(np.maximum(reachable, 0.0), dtype=float).tolist(),
                    "cap_active": np.asarray(rhs < np.asarray(rhs_uncapped) - 1e-15, dtype=bool).tolist(),
                    "aggregate": aggregate_trace,
                    "r_safe": float(self.params.r_safe),
                    "gamma_obj": float(self.params.gamma_obj),
                    "rho": float(self.params.rho),
                    "object_row_window": float(self.params.object_row_window),
                    "object_row_range": float(self.params.object_row_range),
                    "object_row_face_cosine": float(self.params.object_row_face_cosine),
                }
            )
            if aggregate_trace:
                for key, value in aggregate_trace.items():
                    trace.setdefault(key, value)
        return normals, rhs, rhs_no_margin, h

    def _vertex_is_reflex(self, vertices: np.ndarray, index: int) -> bool:
        """True if vertex ``index`` of a CCW polygon is a reflex (inner) corner."""
        v = np.asarray(vertices, dtype=float).reshape(-1, 2)
        n = len(v)
        prev_pt = v[(index - 1) % n]
        cur = v[index % n]
        nxt = v[(index + 1) % n]
        e0 = cur - prev_pt
        e1 = nxt - cur
        cross = float(e0[0] * e1[1] - e0[1] * e1[0])
        return bool(cross < -1e-12)

    def _edge_plane_row(self, position: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
        edge = np.asarray(b, dtype=float).reshape(2) - np.asarray(a, dtype=float).reshape(2)
        n = np.array([edge[1], -edge[0]], dtype=float)
        nrm = float(np.linalg.norm(n))
        if nrm <= 1e-15:
            n = np.array([1.0, 0.0])
            nrm = 1.0
        n = n / nrm
        h = float(np.dot(n, np.asarray(position, dtype=float).reshape(2) - np.asarray(a, dtype=float).reshape(2)))
        h -= self.params.r_safe
        return n, h

    def _finish_object_rows(
        self,
        normals: np.ndarray,
        h: np.ndarray,
        v_obj: np.ndarray,
        trace: dict | None,
        extra: dict | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if len(normals) == 0:
            if trace is not None:
                trace.update({"dropped_empty": True, **(extra or {})})
            return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        normals = np.asarray(normals, dtype=float).reshape(-1, 2)
        h = np.asarray(h, dtype=float).reshape(-1)
        normal_velocity = normals @ np.asarray(v_obj, dtype=float).reshape(2)
        demand = normal_velocity - self.params.gamma_obj * h
        rhs_uncapped = demand + self.params.rho
        rhs = self._cap_to_reachable(normals, rhs_uncapped)
        rhs_no_margin = self._cap_to_reachable(normals, demand)
        if trace is not None:
            trace.update(
                {
                    "mode": self.params.object_row_mode,
                    "h": h.tolist(),
                    "A": normals.tolist(),
                    "demand": np.asarray(demand, dtype=float).tolist(),
                    "rhs_uncapped": np.asarray(rhs_uncapped, dtype=float).tolist(),
                    "rhs_capped": np.asarray(rhs, dtype=float).tolist(),
                    "rhs_no_margin_uncapped": np.asarray(demand, dtype=float).tolist(),
                    "rhs_no_margin_capped": np.asarray(rhs_no_margin, dtype=float).tolist(),
                    "cap_active": np.asarray(rhs < np.asarray(rhs_uncapped) - 1e-15, dtype=bool).tolist(),
                    "r_safe": float(self.params.r_safe),
                    "gamma_obj": float(self.params.gamma_obj),
                    "rho": float(self.params.rho),
                    "dropped_empty": False,
                    **(extra or {}),
                }
            )
        return normals, rhs, rhs_no_margin, h

    def _feature_cover_reach(self) -> float:
        """Radius of the one-step feature cover: ``2 u_max Δ``.

        Any feature that can become nearest during a hold of length ``Δ`` at
        speed at most ``u_max`` currently satisfies
        ``dist(p, F) ≤ sd(p) + 2 u_max Δ``.  This is geometry, not a margin
        that replaces ``rho``.
        """
        return 2.0 * float(self.params.max_speed) * float(self.params.dt)

    def _nearest_feature_rows(
        self,
        position: np.ndarray,
        vertices: np.ndarray,
        object_velocity: np.ndarray,
        trace: dict | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Segment-distance CBF rows on the one-step feature cover Φ(p).

        Infinite-line extrapolation of one incident edge is the C5/L11/L17
        failure: a convex-corner robot is assigned the supporting plane of an
        adjacent edge, which cuts free space and can make ``h`` negative while
        the true signed distance stays above ``r_safe``.

        Each edge ``F`` contributes the Euclidean barrier
        ``h_F = dist(p, F) - r_safe`` with ``n_F = ∇ dist(p, F)`` (radial at an
        endpoint, outward normal on an edge interior).  Dist-to-segment is
        convex, so the sampled row is a discrete CBF for that edge.  Covering
        every edge with ``dist(p,F) ≤ sd(p) + 2 u_max Δ`` (and within
        ``object_row_range``) makes the minimizer — true unsigned distance to
        ``∂S`` — invariant as well, including during a hold that switches
        features.  A single nearest-feature row is not sufficient for that
        argument.

        Reflex two-plane infinite supporting planes are not used: in a concave
        crook they under-estimate true ``sd`` and can destroy ``0 ∈ F_ρ``.
        """
        from .geometry import closest_point_on_segment, ensure_ccw, signed_distance_and_gradient

        p = np.asarray(position, dtype=float).reshape(2)
        v = ensure_ccw(vertices)
        v_obj = np.asarray(object_velocity, dtype=float).reshape(2)
        nvert = len(v)
        sd, grad, _foot = signed_distance_and_gradient(p[None, :], v)
        sd0 = float(sd[0])
        h_true = sd0 - self.params.r_safe
        cover_reach = self._feature_cover_reach()
        # Outside: unsigned distance is sd.  Inside: include every in-range edge
        # so the recovery rows are still the true nearest segments.
        unsigned = sd0 if sd0 >= 0.0 else 0.0
        cover = unsigned + cover_reach
        rows_n: list[np.ndarray] = []
        rows_h: list[float] = []
        metas: list[dict] = []
        best_d = float("inf")
        best_i = 0
        best_t = 0.0
        best_q = v[0]
        for i in range(nvert):
            q, t = closest_point_on_segment(p, v[i], v[(i + 1) % nvert])
            d = float(np.linalg.norm(p - q))
            if d < best_d:
                best_d, best_i, best_t, best_q = d, i, t, q
            if d > self.params.object_row_range + 1e-15:
                continue
            if d > cover + 1e-15:
                continue
            h = d - self.params.r_safe
            if d <= 1e-15:
                n, _h_plane = self._edge_plane_row(p, v[i], v[(i + 1) % nvert])
            else:
                n = (p - q) / d
            duplicate = False
            for n_old, h_old in zip(rows_n, rows_h):
                if abs(h - h_old) <= 1e-12 and float(np.dot(n, n_old)) >= 1.0 - 1e-9:
                    duplicate = True
                    break
            if duplicate:
                continue
            rows_n.append(np.asarray(n, dtype=float).reshape(2))
            rows_h.append(float(h))
            metas.append(
                {
                    "edge_index": int(i),
                    "t": float(t),
                    "foot": np.asarray(q, dtype=float).tolist(),
                    "distance": d,
                    "h": float(h),
                    "n": np.asarray(n, dtype=float).reshape(2).tolist(),
                    "at_vertex": bool(t <= 1e-9 or t >= 1.0 - 1e-9),
                }
            )
        extra = {
            "feature_kind": "cover",
            "cover_reach": cover_reach,
            "cover_threshold": cover,
            "n_cover_rows": len(rows_h),
            "edge_index": int(best_i),
            "t": float(best_t),
            "foot": np.asarray(best_q, dtype=float).tolist(),
            "distance": float(best_d),
            "h_true": h_true,
            "sd_gradient": grad[0].tolist(),
            "n_bar": None if not rows_n else rows_n[0].tolist(),
            "offset_from_n_k": (
                None
                if not rows_n
                else float(np.dot(rows_n[0], p) - rows_h[0] - self.params.r_safe)
            ),
            "h_bar": float(min(rows_h)) if rows_h else h_true,
            "covered_edges": metas,
        }
        if best_d > self.params.object_row_range:
            extra["reason"] = "out_of_range"
            return self._finish_object_rows(np.empty((0, 2)), np.empty(0), v_obj, trace, extra=extra)
        if not rows_n:
            extra["reason"] = "empty_cover"
            return self._finish_object_rows(np.empty((0, 2)), np.empty(0), v_obj, trace, extra=extra)
        return self._finish_object_rows(
            np.vstack(rows_n), np.asarray(rows_h, dtype=float), v_obj, trace, extra=extra
        )

    def _nearest_sample_rows(
        self,
        position: np.ndarray,
        points: np.ndarray,
        object_velocity: np.ndarray,
        trace: dict | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Map-only fallback: disk barrier about the nearest sample, not its infinite plane."""
        p = np.asarray(position, dtype=float).reshape(2)
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        d = np.linalg.norm(pts - p[None, :], axis=1)
        k = int(np.argmin(d))
        dist = float(d[k])
        if dist > self.params.object_row_range:
            return self._finish_object_rows(
                np.empty((0, 2)), np.empty(0), object_velocity, trace, extra={"reason": "out_of_range"}
            )
        q = pts[k]
        if dist <= 1e-12:
            n = np.array([1.0, 0.0])
            h = -self.params.r_safe
        else:
            n = (p - q) / dist
            h = dist - self.params.r_safe
        extra = {
            "feature_kind": "sample_disk",
            "n_bar": n.tolist(),
            "offset_from_n_k": float(np.dot(n, q)),
            "h_bar": float(h),
            "foot": q.tolist(),
        }
        return self._finish_object_rows(n.reshape(1, 2), np.array([h]), object_velocity, trace, extra=extra)

    def _aggregate_face(
        self,
        position: np.ndarray,
        points: np.ndarray,
        normals: np.ndarray,
        distance: np.ndarray,
        point_velocities: np.ndarray | None = None,
        trace: dict | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """One smooth plane for the face, instead of one row per map sample.

        This is the discrete-time half of the barrier problem, and it is a
        *discontinuity* rather than a magnitude. The sampled condition
        ``n^T u >= n^T v_obj - gamma h + rho`` is a valid discrete-time CBF for
        ``gamma dt <= 1`` provided ``h`` evolves as ``h + dt n^T (u - v_obj)``. It
        does not. ``h`` is read off a *set* of map cells, and the set changes: a
        carve deletes the nearest cell, a new return creates one, and the row that
        was binding is replaced by a different row a whole voxel away. ``h`` then
        steps by up to the voxel size with the robot stationary, and the honest
        robust margin for that is ``rho = W/dt`` -- 0.25 m/s for a 0.0125 m jump at
        20 Hz, larger than the speed limit, which is why no amount of ``rho`` could
        buy feasibility.

        Aggregating removes the discontinuity at its source. The face is
        represented by a single confidence- and proximity-weighted plane,

            n_bar = normalize( sum_k g_k n_k ),
            d_bar = ( sum_k g_k n_k^T b_k ) / ( sum_k g_k ),
            h_bar = n_bar^T p - d_bar - r_safe,

        so adding or removing one cell moves the plane by ``O(g_k / sum g)``
        instead of switching which sample defines the constraint. The face filter
        has already restricted the set to one face, which is what makes a single
        plane the right summary rather than a convex-hull approximation of a
        non-convex object.

        It also leaves the object family trivially feasible: one half-plane
        intersected with the speed ball is non-empty whenever the reachability cap
        holds, so the only way to an empty set is a conflict with the inter-robot
        rows.

        The aggregated boundary-point velocity (T2)
        -------------------------------------------
        One plane needs one velocity, and the choice is not the weighted mean. The
        aggregated row stands for the whole face, so it must demand at least as much
        retreat as the fastest-approaching point on that face requires; a weighted
        mean would let a fast-approaching arc be averaged away by the slow cells
        beside it, which is the aggregate silently under-reporting the disturbance it
        exists to summarise. The maximum of ``n_bar^T v_k`` over the face is taken
        instead. On a purely translating object every ``v_k`` is equal and the maximum
        *is* the mean, so this is conservative only where rotation is present -- which
        is the only place it is doing anything at all.
        """
        if len(normals) == 0:
            return normals, np.empty(0), None
        scale = max(self.params.object_row_window, 1e-6)
        weight = np.exp(-0.5 * (distance / scale) ** 2)
        total = float(np.sum(weight))
        if total <= 1e-12:
            weight = np.ones(len(normals))
            total = float(len(normals))

        stacked = weight @ normals
        norm = float(np.linalg.norm(stacked))
        if norm <= 1e-9:
            if trace is not None:
                trace.update({"degenerate_normal_sum": True, "weight_sum": total})
            return np.empty((0, 2)), np.empty(0), None
        n_bar = stacked / norm
        offset = float(np.sum(weight * np.einsum("ij,ij->i", normals, points)) / total)
        h_bar = float(np.dot(n_bar, position)) - offset - self.params.r_safe
        alignment = float(norm / total) if total > 0.0 else 0.0

        aggregated_velocity = None
        if point_velocities is not None and len(point_velocities):
            worst = int(np.argmax(point_velocities @ n_bar))
            aggregated_velocity = point_velocities[worst].reshape(1, 2)
        if trace is not None:
            b_bar = np.sum(weight[:, None] * points, axis=0) / total
            n_bar_b = float(np.dot(n_bar, b_bar))
            trace.update(
                {
                    "weights": np.asarray(weight, dtype=float).tolist(),
                    "weight_sum": total,
                    "stacked_norm": norm,
                    "alignment_sum_gn_over_G": alignment,
                    "translation_defect": float(1.0 - alignment),
                    "n_bar": n_bar.tolist(),
                    "offset_from_n_k": offset,
                    "b_bar": np.asarray(b_bar, dtype=float).tolist(),
                    "offset_if_n_bar_times_b_bar": n_bar_b,
                    "offset_mismatch": float(offset - n_bar_b),
                    "h_bar": h_bar,
                    "face_points": np.asarray(points, dtype=float).tolist(),
                    "face_normals": np.asarray(normals, dtype=float).tolist(),
                    "face_distance": np.asarray(distance, dtype=float).tolist(),
                }
            )
        return n_bar.reshape(1, 2), np.array([h_bar]), aggregated_velocity

    def _cap_to_reachable(self, normals: np.ndarray, rhs: np.ndarray) -> np.ndarray:
        """Cap the object rows at what a speed-limited robot can actually deliver.

        A demand above the speed limit is not a stronger safety guarantee, it is an
        infeasible problem: the robot cannot retreat faster than ``v_max`` however
        the barrier is written, so a right-hand side above that turns a safety
        margin into a solver failure. The cap is stated against an explicit witness
        rather than a flat constant. With

            w = normalize(sum_k n_k)      the common retreat direction
            u* = f v_max w                f = recovery_fraction

        the input ``u*`` satisfies every object row whose right-hand side obeys
        ``r_k <= f v_max (n_k . w)``, so capping there leaves the object family
        feasible *by construction* and names the point that proves it. Rows facing
        away from ``w`` are capped at zero, which is the statement that a robot
        cannot simultaneously retreat from two opposing faces -- true, and better
        said in the constraint than discovered in the solver.
        """
        if len(normals) == 0:
            return np.asarray(rhs, dtype=float)
        witness = normals.sum(axis=0)
        norm = float(np.linalg.norm(witness))
        if norm <= 1e-9:
            return np.minimum(rhs, 0.0)
        witness = witness / norm
        reachable = self.params.recovery_fraction * self.params.max_speed * (normals @ witness)
        return np.minimum(rhs, np.maximum(reachable, 0.0))

    # ------------------------------------------------------------------ #
    # solve
    # ------------------------------------------------------------------ #

    def _wall_rows(self, position: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if not self.params.enable_wall_rows:
            return np.empty((0, 2)), np.empty(0)
        if self.params.domain is None:
            raise ValueError("enable_wall_rows requires an explicit rectangular domain")
        from .theorem_mode import wall_halfplanes

        return wall_halfplanes(position, self.params.domain, self.params.dt)

    def filter_velocity(
        self,
        position: np.ndarray,
        nominal_velocity: np.ndarray,
        neighbor_positions: list[np.ndarray] | np.ndarray = (),
        boundary_points: np.ndarray | None = None,
        boundary_normals: np.ndarray | None = None,
        object_velocity: np.ndarray | None = None,
        boundary_point_velocities: np.ndarray | None = None,
        obstacle_vertices: np.ndarray | None = None,
    ) -> FilterResult:
        u_nom = np.asarray(nominal_velocity, dtype=float).reshape(2)
        speed = float(np.linalg.norm(u_nom))
        if speed > self.params.max_speed:
            u_nom = u_nom * (self.params.max_speed / speed)

        A_agent, b_agent = self._agent_rows(position, list(neighbor_positions))
        A_wall, b_wall = self._wall_rows(position)
        have_map = boundary_points is not None and len(boundary_points) > 0
        have_poly = (
            obstacle_vertices is not None
            and len(np.asarray(obstacle_vertices, dtype=float).reshape(-1, 2)) >= 3
        )
        if not self.params.enable_object_rows or not (have_map or have_poly):
            A_obj, b_obj, b_obj_free, h_obj = np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        else:
            pts = (
                np.asarray(boundary_points, dtype=float)
                if have_map
                else np.empty((0, 2))
            )
            nrm = (
                np.asarray(boundary_normals, dtype=float)
                if boundary_normals is not None and have_map
                else np.zeros_like(pts)
            )
            A_obj, b_obj, b_obj_free, h_obj = self._object_rows(
                position,
                pts,
                nrm,
                object_velocity if object_velocity is not None else np.zeros(2),
                boundary_point_velocities,
                obstacle_vertices=obstacle_vertices,
            )

        # Stack order matters for active-row accounting and for the scaled-barrier
        # split: agent | wall | object. Walls are hard domain constraints and are
        # never scaled; only object rows may be scaled when allowed.
        self._agent_row_count = len(A_agent)
        self._wall_row_count = len(A_wall)
        blocks_A = [block for block in (A_agent, A_wall, A_obj) if len(block)]
        blocks_b = [block for block in (b_agent, b_wall, b_obj) if len(block)]
        A = np.vstack(blocks_A) if blocks_A else np.empty((0, 2))
        b = np.concatenate(blocks_b) if blocks_b else np.empty(0)
        # Same rows with the ISSf robustness margin removed. Used only when the
        # margin itself is what makes the problem infeasible.
        b_no_margin = (
            np.concatenate([b_agent, b_wall, b_obj_free]) if len(b) else b
        )
        b_original = np.asarray(b, dtype=float).copy()
        row_kinds = (
            ["agent"] * len(A_agent) + ["wall"] * len(A_wall) + ["object"] * len(A_obj)
        )

        clamped = False
        if (
            self.params.clamp_margin_to_keep_zero
            and len(b_obj)
            and len(b_obj_free) == len(b_obj)
        ):
            # Rows are a^T u >= b.  u=0 is feasible iff b <= 0.  When the hard
            # (rho-free) RHS already satisfies b_free <= 0 but the margin pushes
            # b > 0, clamp b down to 0 so the projection set contains 0 without
            # weakening the barrier below the rho-free level.
            # This *relaxes* the rho-margin rows.  It is not a proof that the
            # original a^T u >= b (with rho) still holds.
            n_pre = len(b_agent) + len(b_wall)
            for i in range(len(b_obj)):
                if float(b_obj_free[i]) <= 1e-9 and float(b[n_pre + i]) > 0.0:
                    b[n_pre + i] = 0.0
                    clamped = True
            if clamped:
                self.stats.margin_relaxations += 1

        # The certificate is about the barrier, so it is evaluated against the
        # margin-free right-hand side. Evaluating it with rho included would report
        # a certificate failure every time a robot sits in the ISSf band, which is
        # the intended operating point at the cage ring, not a violation.
        zero_feasible = bool(len(b_no_margin) == 0 or np.all(b_no_margin <= 1e-9))
        # Full QP right-hand side including ISSf margin ``rho`` on object rows.
        # Use the *original* (unclamped) b for the comparison baseline.
        original_zero_feasible_with_rho = bool(
            len(b_original) == 0 or np.all(b_original <= 1e-9)
        )
        zero_feasible_with_rho = bool(len(b) == 0 or np.all(b <= 1e-9))
        inside_margin = bool(zero_feasible and not original_zero_feasible_with_rho)
        self.stats.zero_input_feasible_checks += 1
        if not zero_feasible:
            self.stats.zero_input_feasible_failures += 1
        elif inside_margin:
            self.stats.inside_margin_band += 1

        self.stats.solves += 1
        u, status, scale = self._solve(u_nom, A, b, b_no_margin)
        self.stats.record_status(status)

        modification = float(np.linalg.norm(u - u_nom))
        self.stats.max_modification = max(self.stats.max_modification, modification)
        agent_active = wall_active = object_active = 0
        agent_res = wall_res = object_res = 0.0
        if len(A_agent):
            agent_residual = A_agent @ u - b_agent
            agent_active = int(np.count_nonzero(np.abs(agent_residual) <= _ACTIVE_ROW_TOLERANCE))
            agent_res = float(np.min(agent_residual))
        if len(A_wall):
            wall_residual = A_wall @ u - b_wall
            wall_active = int(np.count_nonzero(np.abs(wall_residual) <= _ACTIVE_ROW_TOLERANCE))
            wall_res = float(np.min(wall_residual))
        if len(A_obj):
            object_residual = A_obj @ u - b_obj
            object_active = int(np.count_nonzero(np.abs(object_residual) <= _ACTIVE_ROW_TOLERANCE))
            object_res = float(np.min(object_residual))
        return FilterResult(
            velocity=u,
            status=status,
            object_rows=len(A_obj),
            agent_rows=len(A_agent),
            modification=modification,
            zero_input_feasible=zero_feasible,
            agent_rows_active=agent_active,
            object_rows_active=object_active,
            wall_rows=len(A_wall),
            wall_rows_active=wall_active,
            barrier_scale=float(scale),
            agent_residual_min=agent_res,
            wall_residual_min=wall_res,
            object_residual_min=object_res,
            feasible=status not in ("infeasible", "fallback_projection"),
            zero_input_feasible_with_rho=original_zero_feasible_with_rho,
            inside_margin_band=inside_margin,
            b_original=b_original,
            b_effective=np.asarray(b, dtype=float).copy(),
            b_no_margin=np.asarray(b_no_margin, dtype=float).copy(),
            A_rows=np.asarray(A, dtype=float).copy() if len(A) else np.empty((0, 2)),
            u_nominal=np.asarray(u_nom, dtype=float).copy(),
            row_kinds=list(row_kinds),
            h_object=np.asarray(h_obj, dtype=float).copy() if len(h_obj) else np.empty(0),
            clamp_applied=bool(clamped),
            original_zero_input_feasible_with_rho=original_zero_feasible_with_rho,
            empty_feasible_set=bool(status in ("infeasible", "fallback_projection")),
            zero_not_in_F=bool(not original_zero_feasible_with_rho),
        )

    def _solve(
        self, u_nom: np.ndarray, A: np.ndarray, b: np.ndarray, b_no_margin: np.ndarray
    ) -> tuple[np.ndarray, str, float]:
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

        Tier 3 scales the object rows' right-hand side down by the largest factor
        that leaves the whole set feasible, found by bisection. That replaces the
        projection fallback, and the difference matters: the projection satisfies
        *nothing* exactly, so a single infeasible step used to put a robot inside
        the inter-robot barrier -- measured, the minimum separation fell to 0.218 m
        against a d_min of 0.34 -- and the next step then demanded a harder retreat
        that was infeasible again. Scaling keeps the inter-robot rows hard, keeps
        the object rows in the same direction, and records how much of the
        object-barrier decrease rate was given up, so the degradation is a number
        in the summary instead of a violated invariant.

        Tier 4 -- infeasible with the object rows dropped entirely -- means the
        inter-robot rows alone have no solution, which is a modelling failure
        rather than a solver failure. It is counted as a fallback, and the success
        contracts reject any run whose fallback count is non-zero.

        Under ``forbid_fallback`` (theorem_mode) tier 4 returns an explicit
        infeasible status with a zero command instead of projecting. The caller
        must abort; continuing would no longer be a certified hold update.
        """
        backend = self.params.backend
        if backend == "projection":
            if self.params.forbid_fallback:
                self.stats.infeasible += 1
                return np.zeros(2), "infeasible", 1.0
            return self._project(u_nom, A, b), "projection", 1.0

        for rhs, relaxed in ((b, False), (b_no_margin, True)):
            solution = self._attempt(u_nom, A, rhs)
            if solution is None:
                if self.params.forbid_fallback:
                    self.stats.infeasible += 1
                    self.stats.fallbacks += 1
                    return np.zeros(2), "infeasible", 1.0
                self.stats.fallbacks += 1
                return self._project(u_nom, A, b), "fallback_projection", 1.0
            if solution.feasible:
                if relaxed:
                    self.stats.margin_relaxations += 1
                    return solution.u, "relaxed_margin", 1.0
                return solution.u, "optimal", 1.0
            if self.params.rho <= 0.0:
                break

        if self.params.allow_object_barrier_scaling:
            scaled = self._scaled_barrier_solve(u_nom, A, b_no_margin)
            if scaled is not None:
                u, scale = scaled
                self.stats.barrier_scalings += 1
                self.stats.min_barrier_scale = min(self.stats.min_barrier_scale, scale)
                return u, "scaled_barrier", float(scale)

        # ``infeasible`` means no admissible input existed at all, which is what
        # the word has to mean for the gate on it to be worth anything. Steps that
        # needed a relaxation are counted under the relaxation that was used --
        # ``margin_relaxations`` or ``barrier_scalings`` -- and both are gated
        # separately, so nothing gets through by being renamed.
        self.stats.infeasible += 1
        if self.params.forbid_fallback:
            return np.zeros(2), "infeasible", 1.0
        self.stats.fallbacks += 1
        return self._project(u_nom, A, b), "fallback_projection", 1.0

    def _attempt(self, u_nom: np.ndarray, A: np.ndarray, rhs: np.ndarray):
        if self.params.backend == "cvxpy":
            try:
                return solve_min_norm_2d_cvxpy(u_nom, A, rhs, self.params.max_speed)
            except Exception as exc:  # missing solver, numerical error, ...
                self.solver.on_solver_failure(f"cvxpy raised {type(exc).__name__}: {exc}")
                return None
        return solve_min_norm_2d(u_nom, A, rhs, self.params.max_speed)

    def _scaled_barrier_solve(self, u_nom: np.ndarray, A: np.ndarray, b: np.ndarray):
        """Largest ``s`` in [0, 1] for which scaling the object rows is feasible."""
        agent_n = getattr(self, "_agent_row_count", 0)
        wall_n = getattr(self, "_wall_row_count", 0)
        split = agent_n + wall_n
        if split >= len(A):
            return None
        b_zero = b.copy()
        b_zero[split:] = np.minimum(b_zero[split:], 0.0)
        solution = self._attempt(u_nom, A, b_zero)
        if solution is None or not solution.feasible:
            return None

        best_u, best_scale = solution.u, 0.0
        low, high = 0.0, 1.0
        for _ in range(max(1, self.params.barrier_scale_steps)):
            mid = 0.5 * (low + high)
            rhs = b.copy()
            rhs[split:] = np.where(b[split:] > 0.0, mid * b[split:], b[split:])
            trial = self._attempt(u_nom, A, rhs)
            if trial is not None and trial.feasible:
                best_u, best_scale = trial.u, mid
                low = mid
            else:
                high = mid
        return best_u, best_scale

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
