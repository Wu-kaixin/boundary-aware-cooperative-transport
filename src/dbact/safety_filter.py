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
    object_row_mode: str = "aggregate"
    # Control period, used only to check that the sampled barrier condition is a
    # valid discrete-time CBF: ``gamma_obj * dt <= 1``.
    dt: float = 0.05


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
        if params.object_row_mode not in ("aggregate", "pointwise"):
            raise ContractViolation(
                f"object_row_mode must be 'aggregate' or 'pointwise', got {params.object_row_mode!r}"
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
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pts = np.asarray(boundary_points, dtype=float).reshape(-1, 2)
        if len(pts) == 0:
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
            return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        normals, normal_offset = normals[near], normal_offset[near]
        distance = np.linalg.norm(rel[near], axis=1)

        # Face consistency. The tangential window bounds how far a plane is
        # extrapolated, but on a non-convex object two faces can both pass the
        # window from opposite sides of a corner, and their rows then demand
        # retreat in directions up to 180 degrees apart. No input satisfies both,
        # and the QP reports infeasible for a robot that is in no danger at all --
        # measured at 0.20 m of true clearance. The robot's own nearest return
        # names the face it is standing off; rows whose normal disagrees with it by
        # more than the face angle belong to a different face and are dropped.
        anchor = normals[int(np.argmin(distance))]
        same_face = normals @ anchor >= self.params.object_row_face_cosine
        normals, normal_offset = normals[same_face], normal_offset[same_face]
        if len(normals) == 0:
            return np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)

        h = normal_offset - self.params.r_safe
        if self.params.object_row_mode == "aggregate":
            normals, h = self._aggregate_face(p, pts[near][same_face], normals, distance[same_face])
        elif len(h) > self.params.max_object_rows:
            keep = np.argsort(h)[: self.params.max_object_rows]
            normals, h = normals[keep], h[keep]

        # Both right-hand sides are built from the uncapped expression and capped
        # afterwards. Deriving the margin-free one by subtracting ``rho`` from the
        # *capped* row is wrong whenever the cap binds: the cap is a limit on what
        # the actuator can do, not a term of the barrier, so the subtraction lands
        # on a number that no longer contains ``rho`` and tier 2 relaxes nothing.
        # That is why steps whose barrier was perfectly satisfiable at ``u = 0``
        # were still reaching the scaled tier.
        demand = normals @ v_obj - self.params.gamma_obj * h
        rhs = self._cap_to_reachable(normals, demand + self.params.rho)
        rhs_no_margin = self._cap_to_reachable(normals, demand)
        return normals, rhs, rhs_no_margin, h

    def _aggregate_face(
        self, position: np.ndarray, points: np.ndarray, normals: np.ndarray, distance: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
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
        """
        if len(normals) == 0:
            return normals, np.empty(0)
        scale = max(self.params.object_row_window, 1e-6)
        weight = np.exp(-0.5 * (distance / scale) ** 2)
        total = float(np.sum(weight))
        if total <= 1e-12:
            weight = np.ones(len(normals))
            total = float(len(normals))

        stacked = weight @ normals
        norm = float(np.linalg.norm(stacked))
        if norm <= 1e-9:
            return np.empty((0, 2)), np.empty(0)
        n_bar = stacked / norm
        offset = float(np.sum(weight * np.einsum("ij,ij->i", normals, points)) / total)
        h_bar = float(np.dot(n_bar, position)) - offset - self.params.r_safe
        return n_bar.reshape(1, 2), np.array([h_bar])

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
            return rhs
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
            A_obj, b_obj, b_obj_free, h_obj = np.empty((0, 2)), np.empty(0), np.empty(0), np.empty(0)
        else:
            A_obj, b_obj, b_obj_free, h_obj = self._object_rows(
                position,
                boundary_points,
                boundary_normals if boundary_normals is not None else np.zeros_like(boundary_points),
                object_velocity if object_velocity is not None else np.zeros(2),
            )

        self._agent_row_count = len(A_agent)
        A = np.vstack([A_agent, A_obj]) if len(A_agent) or len(A_obj) else np.empty((0, 2))
        b = np.concatenate([b_agent, b_obj]) if len(b_agent) or len(b_obj) else np.empty(0)
        # Same rows with the ISSf robustness margin removed. Used only when the
        # margin itself is what makes the problem infeasible.
        b_no_margin = np.concatenate([b_agent, b_obj_free]) if len(b) else b

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

        modification = float(np.linalg.norm(u - u_nom))
        self.stats.max_modification = max(self.stats.max_modification, modification)
        agent_active = object_active = 0
        if len(b):
            residual = A @ u - b
            active = np.abs(residual) <= _ACTIVE_ROW_TOLERANCE
            agent_active = int(np.count_nonzero(active[: len(A_agent)]))
            object_active = int(np.count_nonzero(active[len(A_agent) :]))
        return FilterResult(
            velocity=u,
            status=status,
            object_rows=len(A_obj),
            agent_rows=len(A_agent),
            modification=modification,
            zero_input_feasible=zero_feasible,
            agent_rows_active=agent_active,
            object_rows_active=object_active,
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
        """
        backend = self.params.backend
        if backend == "projection":
            return self._project(u_nom, A, b), "projection"

        for rhs, relaxed in ((b, False), (b_no_margin, True)):
            solution = self._attempt(u_nom, A, rhs)
            if solution is None:
                self.stats.fallbacks += 1
                return self._project(u_nom, A, b), "fallback_projection"
            if solution.feasible:
                if relaxed:
                    self.stats.margin_relaxations += 1
                    return solution.u, "relaxed_margin"
                return solution.u, "optimal"
            if self.params.rho <= 0.0:
                break

        scaled = self._scaled_barrier_solve(u_nom, A, b_no_margin)
        if scaled is not None:
            u, scale = scaled
            self.stats.barrier_scalings += 1
            self.stats.min_barrier_scale = min(self.stats.min_barrier_scale, scale)
            return u, "scaled_barrier"

        # ``infeasible`` means no admissible input existed at all, which is what
        # the word has to mean for the gate on it to be worth anything. Steps that
        # needed a relaxation are counted under the relaxation that was used --
        # ``margin_relaxations`` or ``barrier_scalings`` -- and both are gated
        # separately, so nothing gets through by being renamed.
        self.stats.infeasible += 1
        self.stats.fallbacks += 1
        return self._project(u_nom, A, b), "fallback_projection"

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
        split = getattr(self, "_agent_row_count", len(A))
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
