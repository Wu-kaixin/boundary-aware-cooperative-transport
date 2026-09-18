"""Static sampled ``theorem_mode``: execution interface, not a new theorem.

Sampling order at each control instant ``t_k`` with period ``Δ``:

    1. Sense, or inject the configured map (oracle / local).
    2. Form ``M_k`` (no registration motion: the object pose is frozen).
    3. Decide ``u_nom = k_c (c_hat_i - p_i)`` from limited-range CVT only.
    4. QP: ``U_k = argmin ||u - u_nom||`` s.t. robot-robot, wall, object, ``||u|| ≤ u_max``.
    5. Hold: ``P(t_k + τ) = P_k + τ U_k`` for ``0 ≤ τ ≤ Δ``. No coordinate clipping.

Mode switches that this path **excludes** (not proved, not executed):
SEARCH, recall, APPROACH, REDEPLOY, TRANSPORT, BRAKE, HOLD, lead-offset,
gap/explore density terms. The object pose, offset law and reference density
are fixed. Map provenance is an explicit config field.

Safety claims (what *is* guaranteed vs what is only recorded)
-------------------------------------------------------------
Robot–robot. The executed ``U_k`` is checked against the pairwise half-constraint
of Wang, Ames and Egerstedt for every neighbour inside ``comm_range``. Any pair
omitted from the QP must satisfy
``||p_i - p_j|| > d_min + 2 u_max Δ`` (they cannot enter the forbidden ball in
one hold). The minimum distance along the linear hold segment is checked; a
violation aborts the case. This is a discrete sample-and-hold statement, not a
continuous-time CBF certificate for an unsampled ODE.

Wall / domain ``D``. Four linear velocity rows enforce ``P_k + Δ U_k ∈ D``.
``D`` is a closed rectangle, hence convex, so the whole hold segment stays in
``D``. Coordinate clipping is off. An out-of-domain or infeasible step is a
recorded failure, not a projected continuation.

Object. Feature-cover segment-distance rows on ``Φ(p)`` (see
``barrier_invariance_derivation.md``) keep ``h_true ≥ ρ/γ`` on each hold under
P0 and the range-truncation condition ``R_row - u_max Δ ≥ r_safe + ρ/γ``.
The hold-segment clearance sampler remains a fault detector; it is not the
invariance proof. Fallback projection is forbidden: an infeasible QP aborts.
Abort stops the case; it is not a safety certificate of a continued trajectory.
On the proved invariant set the original-ρ QP contains 0, so abort does not
fire. Object-row scaling and margin clamp, if enabled in YAML, are idle on that
set and are not part of the theorem cascade.

These object guarantees apply to oracle mode only. Local mode supplies only
measured boundary samples to the QP. It never uses true vertices or a true
clearance abort to select/withhold a command. Object clearance must be measured
by an external observer; an empirical safe rollout is not a local-map theorem.

Cost. The safety filter may change ``u_nom``. ``H_{k+1} - H_k`` is recorded and
is **not** assumed negative.
"""

from __future__ import annotations

import os
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field

import numpy as np

from .boundary_density import BoundaryAwareDensity
from .cargo import Cargo
from .geometry import polygon_perimeter, signed_distance_to_polygon
from .local_cvt import coverage_cost
from .phase import PhaseSignals
from .types import AgentState, BoundaryView, ControlCommand

# Pairwise pairs farther than this cannot violate ``d_min`` in one hold, even if
# both robots travel directly toward each other at ``u_max``.
def omitted_pair_radius(d_min: float, u_max: float, dt: float) -> float:
    return float(d_min + 2.0 * u_max * dt)


class TheoremModeAbort(Exception):
    """Illegal sampled update: the case stops and keeps the last legal state."""

    def __init__(self, reason: str, frame: int, details: dict | None = None):
        self.reason = str(reason)
        self.frame = int(frame)
        self.details = dict(details or {})
        super().__init__(f"theorem_mode abort at frame {frame}: {reason}")

    def as_dict(self) -> dict:
        return {"reason": self.reason, "frame": self.frame, "details": self.details}


@dataclass
class TheoremStepRecord:
    """One control instant of the sampled static mode."""

    frame: int
    time: float
    positions: np.ndarray
    nominal_velocities: np.ndarray
    u_cvt: np.ndarray
    u_saturated: np.ndarray
    u_command: np.ndarray
    command_velocities: np.ndarray
    modes: list[str]
    map_source: str
    cell_mass: np.ndarray
    cell_centroid: np.ndarray
    solver_status: list[str]
    barrier_scale: np.ndarray
    agent_residual_min: np.ndarray
    wall_residual_min: np.ndarray
    object_residual_min: np.ndarray
    speed: np.ndarray
    hold_min_pair_distance: float
    hold_min_object_clearance: float
    hold_object_clearance_sampled: float
    hold_object_clearance_margin: float
    hold_clearance_n_samples: int
    hold_clearance_h: float
    omitted_pair_min_distance: float
    wall_rows_active: list[int] | None = None
    cost_H: float | None = None
    cost_delta: float | None = None
    reference_mass: np.ndarray | None = None
    reference_centroid: np.ndarray | None = None
    reference_gradient: np.ndarray | None = None
    abort_reason: str | None = None


@dataclass
class TheoremRunLog:
    records: list[TheoremStepRecord] = field(default_factory=list)
    abort: dict | None = None
    map_source: str = "oracle"
    sampling_order: tuple[str, ...] = (
        "sense_or_inject_map",
        "form_M_k",
        "cvt_nominal",
        "qp_with_agent_wall_object_speed",
        "hold_P_plus_tau_U",
    )

    def as_summary(self) -> dict:
        statuses: dict[str, int] = {}
        scales = []
        for rec in self.records:
            for status in rec.solver_status:
                statuses[status] = statuses.get(status, 0) + 1
            scales.extend(float(s) for s in rec.barrier_scale)
        hold_min = [rec.hold_min_pair_distance for rec in self.records]
        return {
            "frames": len(self.records),
            "map_source": self.map_source,
            "sampling_order": list(self.sampling_order),
            "aborted": self.abort is not None,
            "abort": self.abort,
            "solver_statuses": statuses,
            "barrier_scalings": int(sum(1 for s in scales if s < 1.0 - 1e-12)),
            "min_barrier_scale": float(min(scales)) if scales else 1.0,
            "hold_min_pair_distance": float(min(hold_min)) if hold_min else float("inf"),
            "modes_seen": sorted({m for rec in self.records for m in rec.modes}),
        }


def point_in_domain(
    point: np.ndarray,
    domain: tuple[float, float, float, float],
    tol: float = 1e-12,
) -> bool:
    xmin, xmax, ymin, ymax = domain
    p = np.asarray(point, dtype=float).reshape(2)
    return bool(
        p[0] >= xmin - tol
        and p[0] <= xmax + tol
        and p[1] >= ymin - tol
        and p[1] <= ymax + tol
    )


def wall_halfplanes(
    position: np.ndarray,
    domain: tuple[float, float, float, float],
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Linear rows ``A u >= b`` equivalent to ``p + dt u ∈ D``.

    Written as one-step invariance of the closed rectangle, not as a CBF on
    clearance-to-wall. Four rows, some of them slack when the robot is interior.
    """
    if dt <= 0.0:
        raise ValueError("wall rows require dt > 0")
    xmin, xmax, ymin, ymax = domain
    p = np.asarray(position, dtype=float).reshape(2)
    A = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
    b = np.array(
        [
            (xmin - p[0]) / dt,
            (p[0] - xmax) / dt,
            (ymin - p[1]) / dt,
            (p[1] - ymax) / dt,
        ]
    )
    return A, b


def pairwise_halfplanes(
    position: np.ndarray,
    neighbor_positions: np.ndarray,
    d_min: float,
    gamma_agent: float,
) -> tuple[np.ndarray, np.ndarray]:
    if len(neighbor_positions) == 0:
        return np.empty((0, 2)), np.empty(0)
    p = np.asarray(position, dtype=float).reshape(2)
    q = np.asarray(neighbor_positions, dtype=float).reshape(-1, 2)
    diff = p[None, :] - q
    h = np.sum(diff * diff, axis=1) - d_min**2
    return 2.0 * diff, -0.5 * gamma_agent * h


def hold_segment_min_distance(
    p_i: np.ndarray,
    p_j: np.ndarray,
    u_i: np.ndarray,
    u_j: np.ndarray,
    dt: float,
) -> float:
    """Minimum ``||p_i - p_j + τ (u_i - u_j)||`` for ``τ ∈ [0, Δ]``."""
    r0 = np.asarray(p_i, dtype=float).reshape(2) - np.asarray(p_j, dtype=float).reshape(2)
    v = np.asarray(u_i, dtype=float).reshape(2) - np.asarray(u_j, dtype=float).reshape(2)
    v2 = float(np.dot(v, v))
    if v2 <= 1e-18:
        return float(np.linalg.norm(r0))
    tau = float(np.clip(-np.dot(r0, v) / v2, 0.0, dt))
    return float(np.linalg.norm(r0 + tau * v))


def team_hold_min_distance(positions: np.ndarray, velocities: np.ndarray, dt: float) -> float:
    p = np.asarray(positions, dtype=float).reshape(-1, 2)
    u = np.asarray(velocities, dtype=float).reshape(-1, 2)
    n = len(p)
    if n < 2:
        return float("inf")
    best = float("inf")
    for i in range(n):
        for j in range(i + 1, n):
            best = min(best, hold_segment_min_distance(p[i], p[j], u[i], u[j], dt))
    return best


def omitted_neighbor_certificate(
    positions: np.ndarray,
    neighbor_indices: list[list[int]],
    d_min: float,
    u_max: float,
    dt: float,
) -> tuple[bool, float, list[tuple[int, int, float]]]:
    """True iff every pair not in the QP neighbour lists is outside the one-step ball."""
    p = np.asarray(positions, dtype=float).reshape(-1, 2)
    radius = omitted_pair_radius(d_min, u_max, dt)
    n = len(p)
    included = [set(idx) for idx in neighbor_indices]
    violations: list[tuple[int, int, float]] = []
    farthest_omitted = float("inf")
    has_omitted = False
    for i in range(n):
        for j in range(i + 1, n):
            if j in included[i] or i in included[j]:
                continue
            dist = float(np.linalg.norm(p[i] - p[j]))
            has_omitted = True
            farthest_omitted = dist if farthest_omitted == float("inf") else min(farthest_omitted, dist)
            if dist <= radius + 1e-12:
                violations.append((i, j, dist))
    if not has_omitted:
        farthest_omitted = float("inf")
    return (len(violations) == 0), farthest_omitted, violations


def oracle_boundary_view(cargoes: list[Cargo], spacing: float = 0.04) -> BoundaryView:
    """Common accurate map: true polygon edges, known arc length, confidence 1.

    This is the map source ``oracle``. It is not a local perception map and it
    does not go through voxel fusion or registration.
    """
    if not cargoes:
        return BoundaryView.empty()
    points: list[np.ndarray] = []
    normals: list[np.ndarray] = []
    arcs: list[float] = []
    ids: list[str] = []
    for cargo in cargoes:
        vertices = cargo.vertices
        nvert = len(vertices)
        for k in range(nvert):
            a = vertices[k]
            b = vertices[(k + 1) % nvert]
            edge = b - a
            length = float(np.linalg.norm(edge))
            if length <= 1e-12:
                continue
            tangent = edge / length
            normal = np.array([tangent[1], -tangent[0]], dtype=float)
            count = max(1, int(np.ceil(length / max(spacing, 1e-6))))
            ds = length / count
            for s in range(count):
                t = (s + 0.5) / count
                points.append(a + t * edge)
                normals.append(normal)
                arcs.append(ds)
                ids.append(cargo.object_id)
    if not points:
        return BoundaryView.empty()
    return BoundaryView(
        points=np.vstack(points),
        normals=np.vstack(normals),
        confidence=np.ones(len(points)),
        arc_length=np.asarray(arcs, dtype=float),
        object_ids=np.asarray(ids, dtype="<U32"),
    )


def freeze_cargoes(cargoes: list[Cargo]) -> None:
    for cargo in cargoes:
        cargo.movable = False
        cargo.set_twist(np.zeros(2), 0.0)


def assert_theorem_params(params) -> None:
    """Fail closed: theorem_mode does not silently ignore excluded knobs."""
    if not getattr(params, "theorem_mode", False):
        return
    source = str(getattr(params, "theorem_map_source", "oracle"))
    if source not in ("oracle", "local"):
        raise ValueError(f"theorem_map_source must be 'oracle' or 'local', got {source!r}")
    if str(params.density_mode) != "offset":
        raise ValueError("theorem_mode covers only density_mode='offset'")
    if float(params.gap_gain) != 0.0:
        raise ValueError("theorem_mode forbids gap_gain ≠ 0 (search/redeploy density term)")
    if float(params.explore_gain) != 0.0:
        raise ValueError("theorem_mode forbids explore_gain ≠ 0")
    if params.lead_offset is not None:
        raise ValueError("theorem_mode forbids lead_offset; the offset law is the uniform cage_offset")
    if str(params.task_mode) == "transport":
        raise ValueError("theorem_mode excludes transport; set task_mode to caging")
    if float(params.gamma_agent) * float(params.dt) > 1.0 + 1e-9:
        raise ValueError("theorem_mode requires gamma_agent * dt ≤ 1")
    if float(getattr(params, "communication_dropout_prob", 0.0)) != 0.0:
        raise ValueError("theorem_mode forbids communication dropout (neighbour symmetry)")
    from .safety_filter import range_truncation_holds, range_truncation_numbers

    if not range_truncation_holds(
        float(params.object_row_range),
        float(params.max_speed),
        float(params.dt),
        float(params.r_safe),
        float(params.rho),
        float(params.gamma_obj),
    ):
        nums = range_truncation_numbers(
            float(params.object_row_range),
            float(params.max_speed),
            float(params.dt),
            float(params.r_safe),
            float(params.rho),
            float(params.gamma_obj),
        )
        raise ValueError(
            "theorem_mode requires R_row - u_max Δ ≥ r_safe + ρ/γ; "
            f"got {nums['R_row_minus_u_max_Delta']:.6f} < {nums['r_safe_plus_rho_over_gamma']:.6f}"
        )
    if float(params.comm_range) + 1e-12 < 2.0 * float(params.local_radius):
        raise ValueError("theorem_mode requires comm_range ≥ 2 local_radius")


def reference_offset_note(cage_offset: float) -> str:
    return (
        f"uniform offset d_c={cage_offset:.6g}, no lead_offset, no gap/explore terms; "
        f"true perimeter measure uses polygon_perimeter={polygon_perimeter.__name__}"
    )


def hold_segment_object_clearance(
    positions: np.ndarray,
    velocities: np.ndarray,
    cargo_vertices_list: list[np.ndarray],
    dt: float,
    n_samples: int = 21,
) -> tuple[float, float, float, dict]:
    """Certified hold-segment object clearance via dense sampling + Lipschitz bound.

    Signed distance to a fixed polygon is 1-Lipschitz along each robot hold
    segment ``p_i + τ u_i``. With samples at spacing ``h = Δ/(n-1)``, the true
    minimum clearance is at least ``sampled_min_i - ‖u_i‖ h`` per robot; the team
    gate uses ``min_i`` of those per-robot lower bounds.

    Returns ``(clearance_lower_bound, sampled_min_clearance, lipschitz_margin, details)``.
    """
    p = np.asarray(positions, dtype=float).reshape(-1, 2)
    u = np.asarray(velocities, dtype=float).reshape(-1, 2)
    n_pts = max(2, int(n_samples))
    h = float(dt) / (n_pts - 1)
    if len(p) == 0 or not cargo_vertices_list:
        return float("inf"), float("inf"), 0.0, {"n_samples": n_pts, "h": h, "dt": float(dt)}
    taus = np.linspace(0.0, float(dt), n_pts)
    sampled_min = float("inf")
    lower_bounds: list[float] = []
    per_robot_sampled: list[float] = []
    worst: dict = {}
    for i in range(len(p)):
        robot_sampled = float("inf")
        robot_worst: dict = {}
        for tau in taus:
            sample = p[i] + tau * u[i]
            for k, vertices in enumerate(cargo_vertices_list):
                verts = np.asarray(vertices, dtype=float)
                signed = float(signed_distance_to_polygon(sample.reshape(1, 2), verts)[0])
                if signed < robot_sampled:
                    robot_sampled = signed
                    robot_worst = {
                        "robot_index": i,
                        "tau": float(tau),
                        "cargo_index": k,
                        "signed_clearance": signed,
                        "sample": sample.tolist(),
                    }
        speed_i = float(np.linalg.norm(u[i]))
        lb_i = robot_sampled - speed_i * h
        per_robot_sampled.append(robot_sampled)
        lower_bounds.append(lb_i)
        if robot_sampled < sampled_min:
            sampled_min = robot_sampled
            worst = robot_worst
    clearance_lower_bound = float(min(lower_bounds))
    lipschitz_margin = float(np.max(np.linalg.norm(u, axis=1))) * h
    details = {
        "worst": worst,
        "n_samples": n_pts,
        "h": h,
        "dt": float(dt),
        "per_robot_sampled_min": per_robot_sampled,
        "per_robot_lower_bound": lower_bounds,
        "lipschitz_margin_max_speed": lipschitz_margin,
    }
    return clearance_lower_bound, sampled_min, lipschitz_margin, details


def attach_theorem_runtime(controller) -> None:
    """Attach per-run theorem_mode state to a controller instance."""
    controller.theorem_log = TheoremRunLog(
        map_source=str(controller.params.theorem_map_source)
    )
    controller.last_theorem_record = None
    controller._theorem_abort = None
    controller._last_cost_H = None
    controller._oracle_view = None
    controller._theorem_cargoes_frozen = False
    controller._last_cell_mass = np.empty(0)
    controller._last_cell_centroid = np.empty((0, 2))
    controller._last_nominal = np.empty((0, 2))
    controller._last_filter_results = []


def _merge_scans(scans: list[BoundaryView], indices: list[int]) -> tuple[BoundaryView, np.ndarray]:
    live = [(k, scans[k]) for k in indices if len(scans[k])]
    if not live:
        return BoundaryView.empty(), np.empty(0, dtype=np.int64)
    codes = np.concatenate([np.full(len(view), k, dtype=np.int64) for k, view in live])
    merged = BoundaryView(
        points=np.vstack([view.points for _, view in live]),
        normals=np.vstack([view.normals for _, view in live]),
        confidence=np.concatenate([view.confidence for _, view in live]),
        arc_length=np.concatenate([view.arc_length for _, view in live]),
        object_ids=np.concatenate([view.object_ids for _, view in live]),
    )
    return merged, codes


def _abort(controller, reason: str, details: dict | None = None) -> None:
    abort = TheoremModeAbort(reason, controller._frame, details)
    controller._theorem_abort = abort
    controller.theorem_log.abort = abort.as_dict()
    raise abort


def theorem_step(
    controller,
    agents: list[AgentState],
    cargoes: list[Cargo],
    timestamp: float,
    dt: float,
) -> list[ControlCommand]:
    """Static sampled cover: CVT nominal + agent/wall/object QP only."""
    if controller._theorem_abort is not None:
        raise controller._theorem_abort
    controller._time = float(timestamp)
    controller._ensure_state(agents)
    if not controller._theorem_cargoes_frozen:
        freeze_cargoes(cargoes)
        controller._theorem_cargoes_frozen = True

    neighbors = controller._neighbor_indices(agents)
    positions = np.vstack([a.position for a in agents])
    for agent in agents:
        if not point_in_domain(agent.position, controller.domain):
            _abort(
                controller,
                "position_outside_domain_before_step",
                {"agent": agent.agent_id, "position": agent.position.tolist()},
            )

    ok, omitted_min, omitted_violations = omitted_neighbor_certificate(
        positions,
        neighbors,
        controller.params.d_min,
        controller.params.max_speed,
        dt,
    )
    if not ok:
        _abort(
            controller,
            "omitted_neighbor_inside_one_step_ball",
            {
                "radius": controller.params.d_min + 2.0 * controller.params.max_speed * dt,
                "violations": [
                    {"i": i, "j": j, "distance": dist} for i, j, dist in omitted_violations
                ],
            },
        )

    source = str(controller.params.theorem_map_source)
    if source == "oracle":
        view = oracle_boundary_view(cargoes, spacing=controller.params.theorem_oracle_spacing)
        controller._oracle_view = view
        for agent in agents:
            controller._views[agent.agent_id] = view
        scans = [view for _ in agents]
    else:
        scans = [controller.sensor.sense_view(agent, cargoes, timestamp) for agent in agents]
        for i, agent in enumerate(agents):
            local_map = controller.maps[agent.agent_id]
            batch, codes = _merge_scans(scans, [i] + list(neighbors[i]))
            local_map.update(batch, timestamp, agent_codes=codes, dt=dt)
            controller._views[agent.agent_id] = local_map.view(timestamp)
        if controller.trace_enabled:
            controller.last_scans = {agents[i].agent_id: scans[i] for i in range(len(agents))}

    controller.phase_signals = PhaseSignals(agent_count=len(agents))
    controller.diagnostics = []
    commands: list[ControlCommand] = []
    filter_results = []
    nominals = np.zeros((len(agents), 2))
    u_cvt_all = np.zeros((len(agents), 2))
    u_saturated_all = np.zeros((len(agents), 2))
    cell_masses = np.zeros(len(agents))
    cell_centroids = np.zeros((len(agents), 2))
    modes: list[str] = []

    def _nominal_i(i: int):
        agent = agents[i]
        view = controller._views[agent.agent_id]
        return controller._theorem_nominal(i, agents, neighbors[i], view)

    cvt_workers = max(1, int(os.environ.get("DBACT_CVT_WORKERS", "1") or "1"))
    cvt_workers = min(cvt_workers, max(1, len(agents)))
    if cvt_workers <= 1:
        nominal_pack = [_nominal_i(i) for i in range(len(agents))]
    else:
        # Same frozen (P_k, map) snapshot for every agent.  Do not advance time
        # inside this pool; adjacent steps remain sequential.
        with ThreadPoolExecutor(max_workers=cvt_workers) as pool:
            nominal_pack = list(pool.map(_nominal_i, range(len(agents))))

    for i, agent in enumerate(agents):
        u_nom, u_cvt, u_sat, mode, cell_mass, centroid = nominal_pack[i]
        view = controller._views[agent.agent_id]
        nominals[i] = u_nom
        u_cvt_all[i] = u_cvt
        u_saturated_all[i] = u_sat
        cell_masses[i] = cell_mass
        cell_centroids[i] = centroid
        modes.append(mode)
        points, normals, _v_obj, _v_points = controller._object_rows_from_map(
            agent.agent_id, agent.position, view
        )
        result = controller.safety.filter_velocity(
            agent.position,
            u_nom,
            [agents[j].position for j in neighbors[i]],
            boundary_points=points,
            boundary_normals=normals,
            object_velocity=np.zeros(2),
            boundary_point_velocities=None,
            obstacle_vertices=cargoes[0].vertices if source == "oracle" and cargoes else None,
        )
        filter_results.append(result)
        if not result.feasible or result.status == "infeasible":
            _abort(
                controller,
                "qp_infeasible",
                {
                    "agent": agent.agent_id,
                    "status": result.status,
                    "agent_rows": result.agent_rows,
                    "wall_rows": result.wall_rows,
                    "object_rows": result.object_rows,
                },
            )
        if float(np.linalg.norm(result.velocity)) > controller.params.max_speed + 1e-9:
            _abort(
                controller,
                "speed_limit_violated",
                {
                    "agent": agent.agent_id,
                    "speed": float(np.linalg.norm(result.velocity)),
                    "max_speed": controller.params.max_speed,
                },
            )
        if result.agent_residual_min < -1e-6:
            _abort(
                controller,
                "agent_half_constraint_residual_violated",
                {
                    "agent": agent.agent_id,
                    "agent_residual_min": result.agent_residual_min,
                },
            )
        commands.append(ControlCommand(agent.agent_id, result.velocity, mode=mode))

    from .controller import AgentDiagnostics

    for i, agent in enumerate(agents):
        view = controller._views[agent.agent_id]
        result = filter_results[i]
        u_nom = nominals[i]
        cell_mass = float(cell_masses[i])
        centroid = cell_centroids[i]
        mode = modes[i]
        controller.diagnostics.append(
            AgentDiagnostics(
                agent_id=agent.agent_id,
                mode=mode,
                cell_mass=cell_mass,
                object_rows=result.object_rows,
                contact_ready=False,
                push_side=False,
                solver_status=result.status,
                modification=result.modification,
                map_points=len(view),
                agent_rows=result.agent_rows,
                agent_rows_active=result.agent_rows_active,
                object_rows_active=result.object_rows_active,
                zero_input_feasible=result.zero_input_feasible,
                speed_nominal=float(np.linalg.norm(u_nom)),
                speed_command=float(np.linalg.norm(result.velocity)),
                cvt_centroid=(float(centroid[0]), float(centroid[1])),
            )
        )

    velocities = np.vstack([cmd.velocity for cmd in commands])
    hold_min = team_hold_min_distance(positions, velocities, dt)
    if hold_min + 1e-9 < controller.params.d_min:
        _abort(
            controller,
            "hold_segment_pair_distance_below_d_min",
            {"hold_min": hold_min, "d_min": controller.params.d_min},
        )

    for i, agent in enumerate(agents):
        nxt = agent.position + dt * velocities[i]
        if not point_in_domain(nxt, controller.domain):
            _abort(
                controller,
                "unclipped_update_outside_domain",
                {
                    "agent": agent.agent_id,
                    "next_position": nxt.tolist(),
                    "wall_residual_min": filter_results[i].wall_residual_min,
                },
            )

    # Local control cannot consult simulator geometry even as a pre-execution
    # veto. NaN explicitly means unobserved here; the paired experiment measures
    # true hold clearance outside the controller without feeding it back.
    hold_obj_lb, hold_obj_sampled, obj_details = float("nan"), float("nan"), {}
    if source == "oracle":
        cargo_vertices = [c.vertices for c in cargoes]
        hold_obj_lb, hold_obj_sampled, _lipschitz_margin, obj_details = hold_segment_object_clearance(
            positions, velocities, cargo_vertices, dt
        )
    hold_obj_margin = float(hold_obj_lb - controller.params.r_safe)
    if source == "oracle" and hold_obj_lb + 1e-9 < controller.params.r_safe:
        _abort(
            controller,
            "hold_segment_object_clearance",
            {
                "hold_min_object_clearance": hold_obj_lb,
                "hold_object_clearance_sampled": hold_obj_sampled,
                "hold_object_clearance_margin": hold_obj_margin,
                "r_safe": controller.params.r_safe,
                "worst": obj_details.get("worst", {}),
                "sampling": {
                    "n_samples": obj_details.get("n_samples"),
                    "h": obj_details.get("h"),
                },
            },
        )

    cost_H = None
    cost_delta = None
    if len(agents) and source == "oracle" and controller._oracle_view is not None:
        density = BoundaryAwareDensity.from_view(
            controller._oracle_view,
            controller.density_params,
            robot_positions=positions,
            goal_direction=None,
        )
        cost_H = float(
            coverage_cost(positions, density, controller.params.local_radius, controller.domain)
        )
        if controller._last_cost_H is not None:
            cost_delta = float(cost_H - controller._last_cost_H)
        controller._last_cost_H = cost_H

    record = TheoremStepRecord(
        frame=controller._frame,
        time=float(timestamp),
        positions=positions.copy(),
        nominal_velocities=nominals.copy(),
        u_cvt=u_cvt_all.copy(),
        u_saturated=u_saturated_all.copy(),
        u_command=velocities.copy(),
        command_velocities=velocities.copy(),
        modes=list(modes),
        map_source=source,
        cell_mass=cell_masses.copy(),
        cell_centroid=cell_centroids.copy(),
        solver_status=[r.status for r in filter_results],
        barrier_scale=np.asarray([r.barrier_scale for r in filter_results], dtype=float),
        agent_residual_min=np.asarray([r.agent_residual_min for r in filter_results], dtype=float),
        wall_residual_min=np.asarray([r.wall_residual_min for r in filter_results], dtype=float),
        object_residual_min=np.asarray([r.object_residual_min for r in filter_results], dtype=float),
        speed=np.linalg.norm(velocities, axis=1),
        hold_min_pair_distance=float(hold_min),
        hold_min_object_clearance=float(hold_obj_lb),
        hold_object_clearance_sampled=float(hold_obj_sampled),
        hold_object_clearance_margin=float(hold_obj_margin),
        hold_clearance_n_samples=int(obj_details.get("n_samples", 0)),
        hold_clearance_h=float(obj_details.get("h", 0.0)),
        omitted_pair_min_distance=float(omitted_min),
        wall_rows_active=[r.wall_rows_active for r in filter_results],
        cost_H=cost_H,
        cost_delta=cost_delta,
    )
    controller.last_theorem_record = record
    controller.theorem_log.records.append(record)
    controller._last_cell_mass = cell_masses
    controller._last_cell_centroid = cell_centroids
    controller._last_nominal = nominals
    controller._last_filter_results = filter_results
    controller._frame += 1
    return commands


def apply_theorem_commands(
    controller,
    agents: list[AgentState],
    commands: list[ControlCommand],
    dt: float,
) -> None:
    """Apply commands without coordinate clipping; abort if next position leaves D."""
    by_id = {cmd.agent_id: cmd for cmd in commands}
    for agent in agents:
        cmd = by_id[agent.agent_id]
        agent.velocity = cmd.velocity.copy()
        nxt = agent.position + cmd.velocity * dt
        if not point_in_domain(nxt, controller.domain):
            _abort(
                controller,
                "apply_commands_outside_domain",
                {"agent": agent.agent_id, "next_position": nxt.tolist()},
            )
        agent.position = nxt


__all__ = [
    "TheoremModeAbort",
    "TheoremStepRecord",
    "TheoremRunLog",
    "omitted_pair_radius",
    "point_in_domain",
    "wall_halfplanes",
    "pairwise_halfplanes",
    "hold_segment_min_distance",
    "team_hold_min_distance",
    "omitted_neighbor_certificate",
    "oracle_boundary_view",
    "freeze_cargoes",
    "assert_theorem_params",
    "reference_offset_note",
    "hold_segment_object_clearance",
    "attach_theorem_runtime",
    "theorem_step",
    "apply_theorem_commands",
]
