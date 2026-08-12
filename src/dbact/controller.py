"""S7 - the decentralised controller.

Per robot, per step, using only its own observations and its communication
neighbours:

    ray-cast scan -> map registration -> voxel map -> boundary density
                  -> limited-range CVT centroid
                  -> convoy feed-forward + transport effort (gated)
                  -> CBF-QP safety filter

Behaviours that the pre-refactor controller did not have:

**Approach mode.** The density is compactly supported, so a robot far from the
cargo has an almost-empty cell and move-to-centroid returns its own position. It
does not enter exploration either, because communication has filled its map with
neighbours' observations -- its map is non-empty, it just contains nothing
nearby. The result is a robot that is permanently stuck while believing it is
converged; measured over a 12-robot run, 5 robots sat in this state and strict
coverage topped out at 0.069. The test is the cell mass ``m_i``: below
``ratio * phi_0 * pi R_l^2`` the cell carries no boundary information, and the
robot heads for the centroid of the observations it actually holds.

**Push-side allocation.** A transport effort applied by every robot is applied by
the robots in front of the cargo too, and those cancel or reverse the intended
motion. Only robots whose observed outward normal opposes the goal direction --
the ones actually behind the object -- press.

**A gate that is local.** Effort is enabled only once the robot itself is in the
contact band and enough of its neighbours report the same, held for a dwell. One
bit per neighbour, so it stays decentralised.

**A map that moves with the object** (``boundary_map.register``) and **an outer
loop closed on the task** (``transport_control``). Those two are what turn the
quasi-static cage of the A branch into transport; the reasoning is in those
modules rather than repeated here.

**A stopping condition the robots own.** Each robot integrates its own
registration estimate to get the distance the cargo has travelled, agrees with
its neighbours by median, and releases the press when that estimate reaches the
task distance. No simulator pose reaches the control path -- not for the barrier
rows, not for the velocity estimate, not for the stop.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from .boundary_density import BoundaryAwareDensity, DensityParams
from .boundary_map import LocalBoundaryMap
from .cargo import Cargo
from .contracts import ContactSafetyContract, CoverageContract, TransportFeasibilityContract
from .geometry import clip_to_domain, normalize
from .local_cvt import LocalCVT, empty_cell_threshold
from .perception import PerceptionParams, RayCastBoundarySensor
from .phase import Phase, PhaseGates, PhaseMonitor, PhaseSignals
from .safety_filter import SafetyFilter, SafetyFilterParams
from .task import TransportTask
from .transport_control import (
    DirectionalProgressController,
    TransportControlParams,
)
from .types import AgentState, BoundaryView, ControlCommand


@dataclass
class DBACTParams:
    """All controller parameters. Cross-layer relations are checked, not assumed."""

    task_mode: str = "caging"  # "caging" | "transport" | "coverage"

    # --- perception (S2) ---
    sensor_range: float = 1.2
    ray_count: int = 96
    range_noise_std: float = 0.0
    pca_neighbors: int = 5
    residual_tolerance: float = 0.03
    min_confidence: float = 0.15

    # --- communication ---
    comm_range: float = 1.6

    # --- map (S3) ---
    voxel_size: float = 0.06
    age_decay: float = 0.30
    max_voxels_per_object: int = 600
    # Motion compensation. Off reproduces the A-branch world-frame map, which is
    # the A1 ablation: the team cages where the object used to be.
    motion_compensation: bool = True
    registration_gate: float = 0.12
    registration_normal_cosine: float = 0.5
    max_object_speed: float = 0.60
    # Free-space carving removes cells the current scan sees through. Off leaves
    # the ghost trail behind a moving object, which is the A1 ablation.
    carve_enabled: bool = True
    carve_margin: float = 0.06
    # SE(2) registration (T2). Off by default, and the default is a measured
    # decision rather than caution: see docs/CLOSED_LOOP_V2.md. With it off the
    # registration keeps two unknowns, no stored normal is rotated, and the
    # boundary-point velocity reduces exactly to the translational one, so the
    # baseline is bit-identical to v1.
    estimate_object_yaw: bool = False
    max_object_yaw_rate: float = 0.80
    min_yaw_lever_voxels: float = 2.0

    # --- density (S4) ---
    density_mode: str = "offset"
    cage_offset: float = 0.135
    # Offset used on the leading arc. Must exceed robot_radius so that the robots
    # in front of the object guide it without resisting the pushing arc; C1 checks
    # this. Set to None for a uniform cage (enclosure task, or the ablation that
    # shows why a uniform cage cannot be transported).
    lead_offset: float | None = 0.22
    lead_threshold: float = 0.35
    # D10-DWELL. ``_enclosure_geometry`` documents ENCLOSE as "one uniform contact
    # ring" and did not implement it: before TRANSPORT it returned the base
    # geometry, which carries ``lead_offset``. Lifting the leading arc clear is
    # what stops it resisting the press, and during ENCLOSE there is no press to
    # resist -- what it does instead is put those robots' standoff floor *above*
    # the contact band, so a robot that enters the band is immediately driven back
    # out of it by the very loop that band membership switches on. Measured over
    # eight seeds, 96-97% of all band exits were robots whose own floor was above
    # the band. True restores the documented geometry; False is the ablation.
    enclose_uniform_ring: bool = False
    sigma: float = 0.20
    base_density: float = 1e-3
    gap_gain: float = 0.6
    gap_radius: float = 0.35

    # --- coverage (S5) ---
    local_radius: float = 0.80
    grid_resolution: int = 24
    approach_mass_ratio: float = 3.0
    redeploy_gap_ratio: float = 0.15

    # --- unobserved-boundary exploration (D10) --- #
    # The redeploy rule redistributes robots over boundary the map already holds.
    # Measured over eight far-field seeds, 84.5% of its requests returned no
    # candidate at all while 4.34 m of a 7.2 m perimeter was in nobody's map, so
    # what it lacked was not a better choice among candidates but any candidate on
    # the far side. ``explore_gain`` is the weight of a second term in the same
    # density -- demand placed one tangent step past the ends of what has been
    # observed -- and 0.0 leaves the density bit-identical, so the A/B is an
    # ablation of one term rather than a comparison of two controllers.
    explore_gain: float = 0.0
    explore_step: float = 0.25
    explore_window: float = 0.18

    # --- safety (S1) ---
    robot_radius: float = 0.16
    delta_max: float = 0.05
    d_min: float = 0.30
    gamma_agent: float = 6.0
    gamma_obj: float = 4.0
    rho: float = 0.05
    max_speed: float = 0.30
    backend: str = "qp"
    # Ablation switch for the B0 safety baseline: with the object rows removed the
    # filter reduces to the pre-refactor inter-robot-only filter.
    use_object_barrier: bool = True
    max_object_rows: int = 12
    object_row_range: float = 0.60
    object_row_window: float = 0.28
    object_row_face_cosine: float = 0.26
    object_row_mode: str = "aggregate"
    dt: float = 0.05
    object_velocity_bound: float = 0.20
    recovery_fraction: float = 0.6

    # --- search (D9) ---
    # "sweep" is a boustrophedon lawnmower on a static lane partition; "spiral" is
    # the pre-D9 outward drift, kept as the ablation. The sweep is what turns a
    # detection time into a bounded quantity: robot i owns the lane
    # ``x in [xmin + i W/N, xmin + (i+1) W/N]`` and traverses its height, so with
    # ``W/N <= 2 R_s`` every point of the workspace passes within sensor range
    # within ``T <= (H + d_lane) / v_max`` -- a coverage argument rather than a
    # hope. The partition is static and known a priori; it is not negotiated at
    # run time, and saying so is part of the claim.
    search_mode: str = "sweep"
    search_margin: float = 0.45
    search_speed: float = 0.28
    search_lane_tolerance: float = 0.12
    # An object token is one message -- object id, an estimated position, and the
    # time it was seen -- relayed hop by hop. Without it a detection stays inside
    # one communication neighbourhood: robots sweeping lanes on an 8 m workspace
    # are far outside ``comm_range`` of each other, so the finder would enclose the
    # object alone. The token carries no polygon and no shape, only where to look.
    token_ttl: float = 12.0
    token_arrival_radius: float = 0.30

    # --- gains ---
    kp_explore: float = 0.25
    kp_cage: float = 0.9
    kp_transport: float = 0.18  # only used by transport_law="constant" (A-branch law)

    # --- transport gating (S7) ---
    push_side_threshold: float = 0.35
    min_push_agents: int = 3
    contact_band_tolerance: float = 0.08
    object_velocity_filter: float = 0.35

    # --- transport outer loop (D3/D4) ---
    transport_law: str = "progress"  # "progress" | "constant"
    transport_reference_speed: float = 0.055
    transport_kp: float = 2.4
    transport_ki: float = 3.0
    transport_effort_max: float = 0.30
    transport_deadband: float = 0.05
    brake_gain: float = 0.55
    convoy_gain: float = 1.0
    convoy_max: float = 0.25
    # Inner standoff loop: a 0.05 m depth error asks for 0.15 m/s. It regulates
    # the ring offset for every robot whose nearest observed boundary is within
    # ``standoff_range``; further out the coverage law has the robot on its own.
    kp_press: float = 3.0
    standoff_range: float = 0.55
    # Slack the transport press leaves between its deepest command and the object
    # barrier's own boundary. ``None`` derives it from C5 rather than setting it:
    # ``safety_factor * (object_velocity_bound + rho) / gamma_obj``, the width of
    # the band inside which object rows demand active retreat. A literal value is
    # allowed so the ablation can put the press back inside the band and show the
    # scaled-barrier events return.
    press_margin: float | None = None
    press_margin_safety: float = 1.2
    # Standoff used once the team is holding: strictly above robot_radius, so the
    # enclosure is kept without any robot remaining in contact. A cage at the
    # contact offset never stops pushing -- every trailing robot still applies
    # k_p (r_robot - d_c) = 12.5 N -- so "arrived" and "still creeping" would be
    # the same state.
    hold_offset: float = 0.17
    # The leading arc stands off by the distance the object covers while a robot
    # gets out of the way. Without it the enclosure is run over from behind
    # exactly when the transport is working.
    lead_lookahead_time: float = 1.2
    lead_lookahead_max: float = 0.18
    # Lateral steering: how hard the push arc is rotated to cancel cross-track
    # error, and the largest rotation it may ask for.
    # Soft inter-robot separation. The inter-agent barrier is a *last resort*, and
    # C5 makes the same argument for the object barrier: at every scaled-barrier
    # event the agent rows were satisfiable at u = 0 but only barely (h_ij between
    # 0.0007 and 0.006), so a robot asked to retreat from the object had nowhere to
    # go. A repulsion that switches on above d_min keeps that slack available, and
    # costs nothing when the ring is not crowded.
    separation_band: float = 0.06
    kp_separate: float = 1.2
    cross_track_gain: float = 0.8
    cross_track_max_deg: float = 30.0
    cross_track_deadband: float = 0.03
    # Damping on the cross-track loop, in seconds: multiplies the lateral component
    # of the estimated object velocity. Without it the loop is undamped.
    cross_track_damping: float = 12.0
    # Lateral error at which the differential allocation reaches full authority.
    cross_track_reference: float = 0.10
    push_share_floor: float = 0.15

    # --- phase supervisor (D2) ---
    phase_informed_fraction: float = 0.55
    phase_map_coverage_min: float = 0.70
    contact_dwell: int = 20
    brake_fraction: float = 0.80

    # --- legacy 'coverage' task mode (region coverage without a cargo) ---
    target_center: list[float] = field(default_factory=lambda: [4.0, 4.0])
    target_radius: float = 1.0
    target_sensor_range: float = 2.0
    target_samples: int = 36

    @property
    def r_safe(self) -> float:
        return self.robot_radius - self.delta_max

    @classmethod
    def from_dict(cls, data: dict) -> "DBACTParams":
        known = {k: v for k, v in data.items() if k in cls.__dataclass_fields__}
        unknown = sorted(set(data) - set(known))
        if unknown:
            raise ValueError(
                f"unknown controller parameters {unknown}; a silently ignored parameter is a "
                "configuration that does not describe the run"
            )
        return cls(**known)

    def contact_contract(self) -> ContactSafetyContract:
        return ContactSafetyContract(
            robot_radius=self.robot_radius,
            cage_offset=self.cage_offset,
            delta_max=self.delta_max,
            gamma_obj=self.gamma_obj,
            rho=self.rho,
            d_min=self.d_min,
            lead_offset=self.lead_offset if self.task_mode == "transport" else None,
        )

    def coverage_contract(self) -> CoverageContract:
        return CoverageContract(local_radius=self.local_radius, comm_range=self.comm_range)

    def transport_feasibility(
        self, stiffness: float | None = None, breakaway_force: float | None = None
    ) -> TransportFeasibilityContract:
        contract = TransportFeasibilityContract(
            r_safe=self.r_safe,
            robot_radius=self.robot_radius,
            gamma_obj=self.gamma_obj,
            rho=self.rho,
            object_velocity_bound=self.object_velocity_bound,
            press_margin=0.0,
            stiffness=stiffness,
            breakaway_force=breakaway_force,
            min_push_agents=self.min_push_agents,
            safety_factor=self.press_margin_safety,
        )
        margin = self.press_margin if self.press_margin is not None else contract.required_margin
        return replace(contract, press_margin=margin)

    def transport_control(self) -> TransportControlParams:
        return TransportControlParams(
            reference_speed=self.transport_reference_speed,
            kp=self.transport_kp,
            ki=self.transport_ki,
            effort_max=min(self.transport_effort_max, self.max_speed),
            deadband_fraction=self.transport_deadband,
            brake_gain=self.brake_gain,
            convoy_gain=self.convoy_gain,
            convoy_max=self.convoy_max,
        )

    def phase_gates(self) -> PhaseGates:
        return PhaseGates(
            informed_fraction=self.phase_informed_fraction,
            map_coverage_min=self.phase_map_coverage_min,
            contact_quorum=self.min_push_agents,
            contact_dwell=self.contact_dwell,
            transport_quorum=self.min_push_agents,
            brake_fraction=self.brake_fraction,
        )


@dataclass
class AgentDiagnostics:
    agent_id: str
    mode: str
    cell_mass: float
    object_rows: int
    contact_ready: bool
    push_side: bool
    solver_status: str
    modification: float
    effort: float = 0.0
    progress: float = 0.0
    map_points: int = 0

    # --- D10-DIAG trace fields ---------------------------------------- #
    # Populated only when ``DBACTController.trace_enabled`` is set. They exist so
    # that "the team took 591 frames to reach contact-ready" can be resolved into
    # which robot was waiting on what; nothing in the control path reads them, and
    # with tracing off they carry their defaults and cost nothing.
    #
    # Two of them are the whole point of the exercise. ``redeploy_reason`` says
    # which branch of ``_redeploy_step`` a robot took, and
    # ``redeploy_candidates`` says how many unheld cage targets its own map
    # offered at that moment. A robot whose cell is saturated *and* whose
    # candidate count is zero is a robot the redeploy rule cannot move, and the
    # count separates "the boundary is all owned" from "the boundary is not in the
    # map at all".
    direct_visible: bool = False
    own_scan_points: int = 0
    boundary_distance: float = float("inf")
    token_distance: float = float("inf")
    map_angular_coverage: float = 0.0
    cell_held_fraction: float = 0.0
    cell_unheld_mass: float = 0.0
    centroid_distance: float = 0.0
    redeploy_requested: bool = False
    redeploy_active: bool = False
    redeploy_reason: str = ""
    redeploy_candidates: int = 0
    cage_targets: int = 0
    agent_rows: int = 0
    agent_rows_active: int = 0
    object_rows_active: int = 0
    zero_input_feasible: bool = True
    speed_nominal: float = 0.0
    speed_command: float = 0.0
    map_centroid: tuple[float, float] = (float("nan"), float("nan"))
    cvt_centroid: tuple[float, float] = (float("nan"), float("nan"))
    redeploy_target: tuple[float, float] = (float("nan"), float("nan"))


class DBACTController:
    """Decentralised boundary-aware enclosure and cooperative transport."""

    def __init__(
        self,
        params: DBACTParams,
        domain: tuple[float, float, float, float],
        goal_directions: dict[str, np.ndarray] | None = None,
        seed: int = 0,
        tasks: dict[str, TransportTask] | None = None,
    ):
        self.params = params
        self.domain = domain
        self.seed = int(seed)
        self.goal_directions = {
            k: normalize(np.asarray(v, dtype=float)) for k, v in (goal_directions or {}).items()
        }
        self.tasks: dict[str, TransportTask] = dict(tasks or {})
        for object_id, task in self.tasks.items():
            self.goal_directions[object_id] = normalize(task.direction)

        # Contracts first: a controller that cannot satisfy them must not be built.
        params.coverage_contract().assert_valid()
        contract = params.contact_contract() if params.task_mode != "coverage" else None
        if contract is not None:
            contract.assert_valid()

        self.sensor = RayCastBoundarySensor(
            PerceptionParams(
                sensor_range=params.sensor_range,
                ray_count=params.ray_count,
                range_noise_std=params.range_noise_std,
                pca_neighbors=params.pca_neighbors,
                residual_tolerance=params.residual_tolerance,
                min_confidence=params.min_confidence,
                seed=self.seed,
            )
        )
        self.cvt = LocalCVT(
            local_radius=params.local_radius,
            grid_resolution=params.grid_resolution,
            comm_range=params.comm_range,
        )
        self.safety = SafetyFilter(
            SafetyFilterParams(
                d_min=params.d_min,
                gamma_agent=params.gamma_agent,
                gamma_obj=params.gamma_obj,
                rho=params.rho,
                r_safe=params.r_safe,
                max_speed=params.max_speed,
                backend=params.backend,
                enable_object_rows=params.use_object_barrier,
                max_object_rows=params.max_object_rows,
                object_row_range=params.object_row_range,
                object_row_window=params.object_row_window,
                object_row_inner_limit=params.robot_radius,
                object_row_face_cosine=params.object_row_face_cosine,
                object_row_mode=params.object_row_mode,
                dt=params.dt,
                object_velocity_bound=params.object_velocity_bound,
                recovery_fraction=params.recovery_fraction,
            ),
            contract=contract,
        )
        self.density_params = DensityParams(
            mode=params.density_mode,
            cage_offset=params.cage_offset,
            sigma=params.sigma,
            base_density=params.base_density,
            gap_gain=params.gap_gain,
            gap_radius=params.gap_radius,
            lead_offset=params.lead_offset if params.task_mode == "transport" else None,
            lead_threshold=params.lead_threshold,
            explore_gain=params.explore_gain,
            explore_step=params.explore_step,
            explore_window=params.explore_window,
        )
        # C5 fixes where the press may stop. Asserted at construction with the force
        # budget left out -- the contact stiffness lives in the simulator, so the
        # environment re-asserts the full contract once it knows it.
        self.feasibility = params.transport_feasibility()
        if params.task_mode == "transport":
            self.feasibility.assert_structural()
        self._press_floor = self.feasibility.press_floor

        self.empty_cell_mass = empty_cell_threshold(
            params.local_radius, params.base_density, params.approach_mass_ratio
        )

        self.maps: dict[str, LocalBoundaryMap] = {}
        self._redeploy_target: dict[str, np.ndarray | None] = {}
        self._views: dict[str, BoundaryView] = {}
        self._sweep_direction: dict[int, float] = {}
        self._tokens: dict[str, dict[str, tuple[np.ndarray, float]]] = {}
        self._progress: dict[str, dict[str, float]] = {}
        self.transport_loops: dict[str, DirectionalProgressController] = {}
        self.phase_monitor = PhaseMonitor(gates=params.phase_gates())
        self.target_region_points = self._build_target_region_points()
        self.diagnostics: list[AgentDiagnostics] = []
        self.phase_signals = PhaseSignals()
        self.team_progress: dict[str, float] = {}
        self._time = 0.0
        self._frame = 0
        # D10-DIAG. Off by default: the extra quantities cost an unheld-target
        # search per robot per step, which is not free, and a diagnosis must not
        # change the thing it is diagnosing. Set it before the first ``step``.
        self.trace_enabled = False
        self._trace: dict = {}
        self.last_scans: dict[str, BoundaryView] = {}

    # ------------------------------------------------------------------ #
    # main loop
    # ------------------------------------------------------------------ #

    @property
    def phase(self) -> Phase:
        return self.phase_monitor.phase

    def step(
        self, agents: list[AgentState], cargoes: list[Cargo], timestamp: float, dt: float
    ) -> list[ControlCommand]:
        self._time = float(timestamp)
        self._ensure_state(agents)
        neighbors = self._neighbor_indices(agents)

        scans = [self.sensor.sense_view(agent, cargoes, timestamp) for agent in agents]
        if self.trace_enabled:
            # A robot's own scan, before the neighbour relay is merged into it.
            # Attribution needs it: "who first saw the far side" is a question about
            # who pointed a sensor at it, and the fused view cannot answer it
            # because a relayed point looks the same as an observed one.
            self.last_scans = {agents[i].agent_id: scans[i] for i in range(len(agents))}

        # Own observations first, then one hop of neighbour relay. Voxel fusion
        # makes the relay idempotent: hearing the same cell twice adds no mass.
        # Registration runs on the robot's own scan only -- a relayed point is
        # another robot's view of the same surface at the same instant, so it adds
        # no temporal information.
        for i, agent in enumerate(agents):
            local_map = self.maps[agent.agent_id]
            local_map.register(scans[i], dt)
            if local_map.carve_enabled:
                local_map.carve(agent.position, scans[i])
            batch, codes = _merge_scans(scans, [i] + list(neighbors[i]))
            local_map.update(batch, timestamp, agent_codes=codes, dt=dt)
            self._views[agent.agent_id] = local_map.view(timestamp)

        self._update_tokens(agents, neighbors, timestamp)
        contact_ready = [self._contact_ready(agents[i], self._views[agents[i].agent_id]) for i in range(len(agents))]
        self._update_progress(agents, neighbors)
        self.phase_signals = self._phase_signals(agents, contact_ready)
        self.phase_monitor.update(self.phase_signals, self._frame)

        self.diagnostics = []
        commands: list[ControlCommand] = []
        for i, agent in enumerate(agents):
            view = self._views[agent.agent_id]
            self._trace = {}
            u_nom, mode, cell_mass, push_side, effort = self._nominal_command(
                i, agents, neighbors[i], view, contact_ready, dt
            )
            points, normals, v_obj, v_points = self._object_rows_from_map(
                agent.agent_id, agent.position, view
            )
            result = self.safety.filter_velocity(
                agent.position,
                u_nom,
                [agents[j].position for j in neighbors[i]],
                boundary_points=points,
                boundary_normals=normals,
                object_velocity=v_obj,
                boundary_point_velocities=v_points,
            )
            commands.append(ControlCommand(agent.agent_id, result.velocity, mode=mode))
            diagnostic = AgentDiagnostics(
                agent_id=agent.agent_id,
                mode=mode,
                cell_mass=cell_mass,
                object_rows=result.object_rows,
                contact_ready=contact_ready[i],
                push_side=push_side,
                solver_status=result.status,
                modification=result.modification,
                effort=effort,
                progress=self._agent_progress(agent.agent_id),
                map_points=len(view),
            )
            if self.trace_enabled:
                self._fill_trace(diagnostic, agent, view, scans[i], u_nom, result)
            self.diagnostics.append(diagnostic)
        self._frame += 1
        return commands

    # ------------------------------------------------------------------ #
    # D10-DIAG instrumentation
    # ------------------------------------------------------------------ #

    def _fill_trace(self, diagnostic, agent, view, scan, u_nom, result) -> None:
        """Copy this step's per-robot state onto the diagnostic record.

        Everything here is read from quantities the step already computed, plus
        one extra unheld-target search so that the *candidate count* is recorded
        on every step rather than only on the steps where the redeploy rule
        happened to ask for one. That distinction is the measurement: a rule that
        never fires because its gate is closed and a rule that never fires because
        its candidate set is empty are the same silence, and only the count tells
        them apart.
        """
        scratch = self._trace
        diagnostic.direct_visible = len(scan) > 0
        diagnostic.own_scan_points = len(scan)
        if len(view):
            nearest = self._nearest_index(view, agent.position)
            diagnostic.boundary_distance = float(np.linalg.norm(view.points[nearest] - agent.position))
            diagnostic.map_angular_coverage = _angular_coverage(view.points)
            centroid = view.points.mean(axis=0)
            diagnostic.map_centroid = (float(centroid[0]), float(centroid[1]))
        token = self._token_target(agent.agent_id)
        if token is not None:
            diagnostic.token_distance = float(np.linalg.norm(token - agent.position))
        for name in ("cell_held_fraction", "cell_unheld_mass", "centroid_distance",
                     "redeploy_requested", "redeploy_active", "redeploy_reason",
                     "redeploy_candidates", "cage_targets"):
            if name in scratch:
                setattr(diagnostic, name, scratch[name])
        for name in ("cvt_centroid", "redeploy_target"):
            value = scratch.get(name)
            if value is not None:
                setattr(diagnostic, name, (float(value[0]), float(value[1])))
        diagnostic.agent_rows = result.agent_rows
        diagnostic.agent_rows_active = result.agent_rows_active
        diagnostic.object_rows_active = result.object_rows_active
        diagnostic.zero_input_feasible = result.zero_input_feasible
        diagnostic.speed_nominal = float(np.linalg.norm(u_nom))
        diagnostic.speed_command = float(np.linalg.norm(result.velocity))

    # ------------------------------------------------------------------ #
    # progress estimation (decentralised)
    # ------------------------------------------------------------------ #

    def _update_progress(self, agents: list[AgentState], neighbors: list[list[int]]) -> None:
        """Each robot's own estimate of how far the cargo has travelled, then a
        median with its neighbours.

        A robot that has lost sight of the object stops accumulating and its own
        number goes stale, so a plain average would drag the team's estimate down
        as soon as anyone looked away. The median over the robots that are
        currently registering matches is robust to both the stale ones and to a
        single bad registration, and it costs one float per neighbour per step on
        the link that is already carrying the observations.
        """
        if not self.tasks:
            return
        own: dict[str, dict[str, float]] = {}
        informed: dict[str, list[int]] = {}
        for i, agent in enumerate(agents):
            local_map = self.maps[agent.agent_id]
            own[agent.agent_id] = {}
            for object_id, task in self.tasks.items():
                displacement = local_map.object_displacement(object_id)
                own[agent.agent_id][object_id] = float(np.dot(displacement, task.direction))
                registration = local_map.last_registration.get(object_id)
                if registration is not None and registration.matches > 0:
                    informed.setdefault(object_id, []).append(i)

        for i, agent in enumerate(agents):
            group = [i] + list(neighbors[i])
            self._progress[agent.agent_id] = {}
            for object_id in self.tasks:
                eligible = [j for j in group if j in informed.get(object_id, [])] or group
                values = [own[agents[j].agent_id][object_id] for j in eligible]
                self._progress[agent.agent_id][object_id] = float(np.median(values))

        self.team_progress = {}
        for object_id in self.tasks:
            eligible = informed.get(object_id) or list(range(len(agents)))
            values = [own[agents[j].agent_id][object_id] for j in eligible]
            self.team_progress[object_id] = float(np.median(values)) if values else 0.0

    def _agent_progress(self, agent_id: str, object_id: str | None = None) -> float:
        entries = self._progress.get(agent_id, {})
        if not entries:
            return 0.0
        if object_id is not None:
            return entries.get(object_id, 0.0)
        return float(next(iter(entries.values())))

    # ------------------------------------------------------------------ #
    # phase supervisor
    # ------------------------------------------------------------------ #

    def _phase_signals(self, agents: list[AgentState], contact_ready: list[bool]) -> PhaseSignals:
        informed = 0
        coverage = 0.0
        for agent in agents:
            view = self._views[agent.agent_id]
            if len(view) >= 8:
                informed += 1
                coverage = max(coverage, _angular_coverage(view.points))
        object_id = next(iter(self.tasks), None)
        target = self.tasks[object_id].distance if object_id else 1.0
        progress = self.team_progress.get(object_id, 0.0) if object_id else 0.0
        active = sum(1 for d in self.diagnostics if d.push_side) if self.diagnostics else 0
        return PhaseSignals(
            agent_count=len(agents),
            informed_agents=informed,
            map_coverage=coverage,
            contact_ready=sum(1 for flag in contact_ready if flag),
            transport_active=active,
            progress=progress,
            target_distance=float(target),
        )

    # ------------------------------------------------------------------ #
    # nominal control law
    # ------------------------------------------------------------------ #

    def _nominal_command(
        self,
        i: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        view: BoundaryView,
        contact_ready: list[bool],
        dt: float,
    ) -> tuple[np.ndarray, str, float, bool, float]:
        agent = agents[i]
        if self.params.task_mode == "coverage":
            u, mode = self._region_coverage_command(i, agents, neighbor_indices, self._time)
            return u, mode, 0.0, False, 0.0

        if len(view) == 0:
            # Somebody has seen the object and the news reached here: go and look,
            # rather than carry on sweeping a lane that no longer matters.
            target = self._token_target(agent.agent_id)
            if target is not None and float(np.linalg.norm(target - agent.position)) > self.params.token_arrival_radius:
                u = self.params.kp_cage * (target - agent.position)
                speed = float(np.linalg.norm(u))
                if speed > self.params.max_speed:
                    u = u * (self.params.max_speed / speed)
                return u, "recall", 0.0, False, 0.0
            u = self._exploration_velocity(i, agents, neighbor_indices, self._time)
            return u, "search", 0.0, False, 0.0

        crowd = np.vstack([agent.position] + [agents[j].position for j in neighbor_indices])
        goal = self._goal_for(view) if self.params.task_mode == "transport" else None
        shape = self._enclosure_geometry(agent.agent_id, view)
        density = BoundaryAwareDensity.from_view(
            view, shape, robot_positions=crowd, goal_direction=goal
        )
        cell = self.cvt.compute(i, agents, neighbor_indices, density, self.domain)

        if self.trace_enabled:
            self._trace.update(
                cell_held_fraction=float(cell.held_fraction),
                cell_unheld_mass=float(cell.unheld_mass),
                centroid_distance=float(np.linalg.norm(cell.centroid - agent.position)),
                cvt_centroid=cell.centroid,
            )
            self._trace.update(self._candidate_census(view, agent.position, crowd, goal, shape))

        if cell.cell_mass <= self.empty_cell_mass:
            # Non-empty map, empty cell: head for the nearest piece of cage ring
            # that no robot is holding yet. Aiming at the object centroid instead
            # sends the robot radially inward, straight into the robots already on
            # the ring, where the inter-robot barrier stops it -- a robot that has
            # deadlocked while believing it has converged.
            target = self._approach_target(view, agent.position, crowd)
            u = self.params.kp_cage * (target - agent.position)
            return u, "approach", cell.cell_mass, False, 0.0

        target = self._redeploy_step(agent, view, crowd, cell, goal, contact_ready[i], shape)
        if target is not None:
            return self.params.kp_cage * (target - agent.position), "redeploy", cell.cell_mass, False, 0.0

        u = self.params.kp_cage * (cell.centroid - agent.position) + self._separation_velocity(
            agent, [agents[j] for j in neighbor_indices]
        )
        push_side = False
        effort = 0.0
        if self.params.task_mode == "transport":
            bias, push_side, effort, normal = self._transport_command(
                i, agents, neighbor_indices, view, contact_ready, dt, shape
            )
            if push_side and normal is not None:
                # Only along the *outward* half-axis. The coverage law places the
                # robot along the boundary and may legitimately pull it inward; what
                # it must not do is pull a pressing robot back out, because then the
                # press and the coverage term balance at a depth that is a property
                # of the two gains rather than of the task. That balance is the
                # A-branch equilibrium, and it sat outside contact.
                outward = float(np.dot(u, normal))
                if outward > 0.0:
                    u = u - outward * normal
            u = u + bias
        mode = "push" if push_side else "cage"
        return u, mode, cell.cell_mass, push_side, effort

    def _redeploy_step(
        self,
        agent: AgentState,
        view: BoundaryView,
        crowd: np.ndarray,
        cell,
        goal: np.ndarray | None,
        contact_ready: bool,
        shape: DensityParams,
    ) -> np.ndarray | None:
        """Leave a saturated cell for unheld boundary elsewhere in this robot's map.

        Move-to-centroid on a limited-range cell has a local equilibrium that the
        coverage law cannot escape: with every robot arriving from the same side,
        the near-side robots converge onto the arc they can see, no robot's disc
        ever overlaps the far side, and there is no gradient pointing around the
        object. Measured on the L shape this pinned strict coverage at 0.63 with
        nothing on the trailing face, so the transport effort never activated at
        all and the small net displacement that did occur pointed backwards.

        The escape uses only local information. When the boundary inside this
        robot's own cell is already held by neighbours -- unheld mass below
        ``redeploy_gap_ratio`` of the cell mass -- and its own map contains a cage
        target nobody is near, it commits to that target. Commitment is sticky
        until it arrives or somebody else takes the target, which is what stops the
        two-robot exchange oscillation.

        A robot already in the contact band never redeploys. Without that gate the
        rule eats itself: once the boundary is fully covered every cell reads as
        held, so the robots that are doing the work walk away to chase whatever
        fragment still looks unheld, and the contact set never stabilises.
        """
        trace = self.trace_enabled
        if contact_ready:
            self._redeploy_target[agent.agent_id] = None
            if trace:
                self._trace.update(redeploy_reason="contact_ready", redeploy_active=False)
            return None

        held = self._redeploy_target.get(agent.agent_id)
        reason = "committed" if held is not None else ""
        if held is not None:
            others = crowd[1:] if len(crowd) > 1 else np.empty((0, 2))
            arrived = float(np.linalg.norm(held - agent.position)) <= self.params.gap_radius
            taken = len(others) > 0 and bool(
                np.min(np.linalg.norm(others - held[None, :], axis=1)) <= self.params.gap_radius
            )
            if arrived or taken:
                self._redeploy_target[agent.agent_id] = None
                held = None
                reason = "arrived" if arrived else "taken"

        requested = held is None and cell.held_fraction >= 1.0 - self.params.redeploy_gap_ratio
        if requested:
            candidate = self._unheld_target(
                view, agent.position, crowd, self.params.local_radius, goal, shape
            )
            if candidate is not None:
                self._redeploy_target[agent.agent_id] = candidate
                held = candidate
                reason = "committed_new"
            else:
                reason = "no_candidate"
        elif held is None and not reason:
            reason = "cell_not_saturated"
        if trace:
            self._trace.update(
                redeploy_requested=bool(requested),
                redeploy_active=held is not None,
                redeploy_reason=reason,
                redeploy_target=held if held is not None else None,
            )
        return held

    def _enclosure_geometry(self, agent_id: str, view: BoundaryView) -> DensityParams:
        """The cage ring this robot is aiming at, which depends on the phase.

        Three regimes, each with a reason:

        ``ENCLOSE``    one uniform contact ring; the team is forming the cage.
                       Only when ``enclose_uniform_ring`` is set -- without it the
                       leading arc is already lifted to ``lead_offset``, which is
                       above the contact band, so those robots are driven out of
                       the band as soon as they enter it and the contact quorum
                       cannot hold its dwell. See D10-DWELL.
        ``TRANSPORT``  trailing arc at the contact offset so it can press, leading
                       arc lifted clear *and* pushed out by the distance the object
                       covers while a robot steps aside. Without that look-ahead
                       the enclosure is run over from behind precisely when the
                       transport starts working, and a robot pinched between the
                       advancing cargo and a neighbour at ``d_min`` is the one
                       state in which the safety QP has no feasible input.
        ``HOLD``       one uniform ring outside contact. A cage at the contact
                       offset never stops pushing, so without this the cargo keeps
                       creeping after the team has decided it has arrived.
        """
        base = self.density_params
        if self.params.task_mode != "transport":
            return base
        if self.phase_monitor.reached(Phase.HOLD):
            standoff = max(self.params.hold_offset, self.params.robot_radius + 1e-3)
            return replace(base, cage_offset=standoff, lead_offset=standoff)
        if not self.phase_monitor.reached(Phase.TRANSPORT) or base.lead_offset is None:
            return replace(base, lead_offset=None) if self.params.enclose_uniform_ring else base
        speed = 0.0
        for object_id in np.unique(view.object_ids):
            speed = max(speed, float(np.linalg.norm(self.maps[agent_id].object_velocity(str(object_id)))))
        lookahead = min(self.params.lead_lookahead_time * speed, self.params.lead_lookahead_max)
        return replace(base, lead_offset=base.lead_offset + lookahead)

    def _cage_targets(
        self, view: BoundaryView, goal: np.ndarray | None, shape: DensityParams | None = None
    ) -> np.ndarray:
        offsets = (shape or self.density_params).offsets_for(view.normals, goal)
        return view.points + offsets[:, None] * view.normals

    def _unheld_target(
        self,
        view: BoundaryView,
        position: np.ndarray,
        crowd: np.ndarray,
        min_distance: float,
        goal: np.ndarray | None = None,
        shape: DensityParams | None = None,
    ) -> np.ndarray | None:
        """Nearest cage target beyond ``min_distance`` that no robot is holding."""
        targets = self._cage_targets(view, goal, shape)
        occupancy = np.min(np.linalg.norm(targets[:, None, :] - crowd[None, :, :], axis=2), axis=1)
        reach = np.linalg.norm(targets - position[None, :], axis=1)
        free = (occupancy > self.params.gap_radius) & (reach > min_distance)
        if not np.any(free):
            return None
        candidates = targets[free]
        return candidates[int(np.argmin(reach[free]))]

    def _candidate_census(
        self,
        view: BoundaryView,
        position: np.ndarray,
        crowd: np.ndarray,
        goal: np.ndarray | None,
        shape: DensityParams | None,
    ) -> dict:
        """How many redeploy candidates this robot's map offers, whether or not the
        rule asked. Diagnostic; the same predicate as ``_unheld_target``.

        ``cage_targets`` is every ring target the map implies and
        ``redeploy_candidates`` is the subset the rule would accept. Both are
        drawn from ``view.points``, so both are zero on boundary the map does not
        contain -- which is the structural statement the diagnosis has to
        distinguish from "the boundary is contained and already owned".
        """
        if len(view) == 0:
            return {"cage_targets": 0, "redeploy_candidates": 0}
        targets = self._cage_targets(view, goal, shape)
        occupancy = np.min(np.linalg.norm(targets[:, None, :] - crowd[None, :, :], axis=2), axis=1)
        reach = np.linalg.norm(targets - position[None, :], axis=1)
        free = (occupancy > self.params.gap_radius) & (reach > self.params.local_radius)
        return {"cage_targets": int(len(targets)), "redeploy_candidates": int(np.count_nonzero(free))}

    # ------------------------------------------------------------------ #
    # transport
    # ------------------------------------------------------------------ #

    def _transport_command(
        self,
        i: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        view: BoundaryView,
        contact_ready: list[bool],
        dt: float,
        shape: DensityParams,
    ) -> tuple[np.ndarray, bool, float, np.ndarray | None]:
        """Convoy feed-forward, standoff regulation, and the transport press.

        The normal axis belongs to this method and the tangential axis belongs to
        the coverage law. Splitting them is what removes the equilibrium the
        A branch settled into: a velocity press and a move-to-centroid term along
        the same axis balance at a depth set by the two gains, and measured on the
        L shape that balance point sat *outside* contact -- five robots reported
        push-side while the true contact count was one, and the net force along the
        task direction stayed at 20 N against a 31 N breakaway.

        Standoff is regulated for every robot on the ring, not only the pushers,
        because the ring offset is what the enclosure geometry means. The coverage
        centroid cannot deliver it: the density is a sum of Gaussians of width
        ``sigma`` around the offset curve, so its centroid inside a cell sits well
        off the curve, and the ring the team actually forms is not the ring the
        configuration asked for. That is also why the cargo used to keep creeping
        after HOLD -- the "released" ring was still in contact at 12.5 N a robot.
        """
        agent = agents[i]
        object_id = self._object_for(view, agent.position)
        task = self.tasks.get(object_id) if object_id else None
        goal = self.goal_directions.get(object_id) if object_id else None
        if goal is None:
            return np.zeros(2), False, 0.0, None

        velocity = self.maps[agent.agent_id].object_velocity(object_id)
        loop = self.transport_loops[agent.agent_id]

        # Station-keeping feed-forward for every robot that can see the object,
        # pushing or not: the enclosure has to travel with the cargo or it tears.
        # It stops at HOLD, because a ring that keeps chasing the cargo keeps
        # pushing it, and the run would never come to rest.
        travelling = self.phase_monitor.reached(Phase.TRANSPORT) and not self.phase_monitor.reached(Phase.HOLD)
        bias = loop.convoy_velocity(velocity) if travelling else np.zeros(2)

        nearest = self._nearest_index(view, agent.position)
        if nearest is None:
            return bias, False, 0.0, None
        normal = view.normals[nearest]
        measured = float(np.dot(agent.position - view.points[nearest], normal))
        if measured > self.params.standoff_range:
            return bias, False, 0.0, None
        if not contact_ready[i]:
            return bias, False, 0.0, None

        # Steer the push arc to cancel cross-track error rather than aiming every
        # robot at the nominal task direction. Without it the press is symmetric
        # about d_goal, nothing corrects an asymmetric contact set, and the cargo
        # leaves the line and stays off it: measured, up to 0.98 m of cross-track
        # on a 0.56 m task.
        command = self._steered_direction(agent.agent_id, object_id, goal)
        ring = float(shape.offsets_for(normal[None, :], command)[0])

        progress = self._agent_progress(agent.agent_id, object_id)
        distance = task.distance if task is not None else float("inf")
        quorum = 1 + sum(1 for j in neighbor_indices if contact_ready[j])
        # Membership is decided against the *task* direction and the weight against
        # the *steered* one. Deciding both against the steered direction couples
        # them: rotating the command to correct a lateral error also rotates the
        # membership test, robots at the edge of a three-robot arc drop out of the
        # push set entirely, and the correction costs more force than it buys aim.
        # Split like this, steering redistributes effort inside a stable arc.
        alignment = float(np.dot(normal, goal))
        pushing = (
            quorum >= self.params.min_push_agents
            and self.phase_monitor.reached(Phase.CONTACT_READY)
            and alignment <= -self.params.push_side_threshold
            # A robot whose own estimate says the cargo has arrived stops being a
            # pusher and becomes part of the ring, which is what backs it out to
            # the hold standoff. Leaving it in the push set only zeroed its
            # *effort*: it stayed where it was, still in contact at k_p (r_robot -
            # d_c) = 12.5 N, and the cargo went on creeping -- measured, one seed
            # latched HOLD at frame 134 and still ended at J/L = 2.15.
            and progress < distance
        )

        if pushing and self.params.transport_law == "constant":
            # A-branch law, kept as the B1 baseline: a fixed press, no task feedback.
            return bias + self.params.kp_transport * (-alignment) * (-normal), True, 0.0, normal

        if pushing:
            effort = loop.update(
                direction=command,
                object_velocity=velocity,
                progress=progress,
                target_distance=distance,
                dt=dt,
                blocked=self._blocked_along(agent.agent_id),
            )
            # Press inward along the robot's own observed normal at the speed the
            # outer loop asks for. The depth it reaches is set by the object
            # barrier, which stops the robot at ``r_safe`` and therefore at exactly
            # the penetration budget C1 leaves open -- the safety layer is the
            # depth limiter, so the transport loop does not need to be one, and a
            # robot chasing a receding cargo keeps its contact instead of settling
            # at a standoff the object has already left.
            # The allocation is the only steering authority the arc has: the press
            # is always along the robot's own normal -- a press along the commanded
            # direction is inward only at the centre of the trailing face and
            # tangential everywhere else -- so what the steered direction changes is
            # *how much* each robot presses, not where. Weighted against the steered
            # direction inside an arc whose membership is fixed by the task
            # direction, so a correction redistributes effort instead of shrinking
            # the arc. The floor keeps a robot that the steering has turned away
            # from still holding its patch of boundary: dropping it to zero was
            # measured and cost more force than the aim was worth.
            # Weighted against the steered direction, inside an arc whose membership
            # is fixed by the task direction. Weighting each robot directly by how
            # its own press acts on the lateral error -- first-order in the error
            # rather than second -- was measured and is worse, not better: the loop
            # from allocation to force direction to drift carries a delay, and the
            # direct weight drove it into a lateral oscillation that reached 0.68 m
            # against 0.39 m for the rotation. ``_lateral_weight`` is kept for the
            # ablation that shows it.
            share = float(np.clip(-float(np.dot(normal, command)), self.params.push_share_floor, 1.0))
            speed = effort.effort * share

            # The barrier is a limit, not a setpoint. Left alone, the press drives
            # every pushing robot onto h = 0 exactly, because that is where the
            # object row stops it -- and then the barrier state is being held on
            # the edge of an *estimate* that moves in steps. Instrumenting the
            # scaled-barrier events showed every single one is an object row
            # demanding retreat with the inter-robot rows already at h ~ 0, and
            # the demands are the size gamma_obj times one map update, not the
            # size of anything the robot did. Stopping ``press_margin`` short of
            # r_safe keeps that slack: the force is k_p (r_robot - r_safe -
            # margin) instead of k_p delta_max, which is 17.5 N against 25 N here
            # and still several times the per-robot share of the breakaway.
            floor = self._press_floor
            approach = max(0.0, (measured - floor)) / max(dt, 1e-9)
            press = min(speed, approach) * (-normal)
            return bias + press, True, effort.effort, normal

        # One-sided for everyone who is not pressing, and only ever outwards.
        # Regulating a non-pushing robot *onto* its ring makes the enclosure rigid,
        # and a rigid ring resists: with the lateral arc held in contact, its
        # tangential friction cancelled the push arc and progress collapsed from
        # 0.70 m to 0.01 m on the same seed. Standing off is a floor, not a
        # setpoint -- too far out is the coverage law's business; too far in is a
        # safety matter, and that is the only side worth actuating. It is also what
        # makes HOLD mean something: the ring backs out to ``hold_offset`` and the
        # cargo stops instead of creeping on a 12.5 N residual per robot.
        if measured >= ring:
            return bias, False, 0.0, None
        press = self.params.kp_press * (measured - ring) * (-normal)
        speed = float(np.linalg.norm(press))
        if speed > self.params.max_speed:
            press = press * (self.params.max_speed / speed)
        return bias + press, False, 0.0, None

    def _separation_velocity(self, agent: AgentState, neighbours: list[AgentState]) -> np.ndarray:
        """Push apart before the barrier has to. Zero unless a neighbour is inside
        ``d_min + separation_band``, so it never perturbs an uncrowded ring."""
        if not neighbours or self.params.kp_separate <= 0.0:
            return np.zeros(2)
        threshold = self.params.d_min + self.params.separation_band
        others = np.vstack([n.position for n in neighbours])
        delta = agent.position[None, :] - others
        distance = np.linalg.norm(delta, axis=1)
        close = (distance < threshold) & (distance > 1e-9)
        if not np.any(close):
            return np.zeros(2)
        weight = (threshold - distance[close]) / max(threshold, 1e-9)
        direction = delta[close] / distance[close][:, None]
        return self.params.kp_separate * float(np.sum(weight)) * normalize(
            np.sum(direction * weight[:, None], axis=0)
        )

    def _lateral_weight(
        self, agent_id: str, object_id: str, goal: np.ndarray, normal: np.ndarray
    ) -> float:
        """Scale one robot's press by how it acts on the cross-track error.

        A robot presses along ``-n``, so the lateral component of its force is
        ``-(n . e_hat)`` for a unit lateral-error direction ``e_hat``. To pull the
        cargo back towards the line, robots with ``n . e_hat > 0`` are favoured and
        the rest are damped, in proportion to the error and saturating at
        ``cross_track_reference``. Below the noise floor the weight is exactly one,
        so an on-track run is untouched.
        """
        displacement = self.maps[agent_id].object_displacement(object_id)
        lateral = displacement - float(np.dot(displacement, goal)) * goal
        # Derivative term. Cross-track is J * sin(direction error) -- measured
        # correlation 0.968 over twelve seeds -- so the lateral position error is
        # the integral of the direction error, and a proportional law alone closes
        # a second-order loop with no damping around it. That is what oscillated
        # when the allocation was given more authority: the offset grew to 0.68 m
        # rather than settling. The lateral component of the robot's own velocity
        # estimate is the damping signal, and it costs nothing extra -- it is the
        # same registration output the transport loop already runs on.
        velocity = self.maps[agent_id].object_velocity(object_id)
        lateral_rate = velocity - float(np.dot(velocity, goal)) * goal
        offset = float(np.linalg.norm(lateral))
        if offset < self.params.cross_track_deadband:
            return 1.0
        strength = min(offset / max(self.params.cross_track_reference, 1e-9), 1.0)
        return 1.0 + self.params.cross_track_gain * strength * float(np.dot(normal, lateral / offset))

    def _steered_direction(self, agent_id: str, object_id: str, goal: np.ndarray) -> np.ndarray:
        """Task direction rotated to bring the cargo back onto the goal line.

        The robot's own integrated registration gives it the cargo's displacement;
        the component perpendicular to ``d_goal`` is the cross-track error, and
        commanding ``d_goal - k e_perp`` turns the whole push arc towards the line.
        The rotation is capped so that a large lateral error cannot turn the press
        so far that it stops making forward progress, which would trade one gate
        for another.
        """
        displacement = self.maps[agent_id].object_displacement(object_id)
        lateral = displacement - float(np.dot(displacement, goal)) * goal
        # Derivative term. Cross-track is J * sin(direction error) -- measured
        # correlation 0.968 over twelve seeds -- so the lateral position error is
        # the integral of the direction error, and a proportional law alone closes
        # a second-order loop with no damping around it. That is what oscillated
        # when the allocation was given more authority: the offset grew to 0.68 m
        # rather than settling. The lateral component of the robot's own velocity
        # estimate is the damping signal, and it costs nothing extra -- it is the
        # same registration output the transport loop already runs on.
        velocity = self.maps[agent_id].object_velocity(object_id)
        lateral_rate = velocity - float(np.dot(velocity, goal)) * goal
        offset = float(np.linalg.norm(lateral))
        if offset < self.params.cross_track_deadband:
            # Below the estimator's own noise floor the "cross-track error" is
            # registration jitter, and steering on it swings the push arc off the
            # trailing face for no reason: measured, a run that reached 0.51 m with
            # the arc held straight reached 0.06 m once it started chasing a 0.02 m
            # phantom offset. The band is on the lateral component alone rather than
            # on total displacement, so a run that is genuinely leaving the line is
            # corrected from the first centimetre that exceeds the noise instead of
            # after the cargo has travelled far enough to notice.
            return goal
        correction = -(
            self.params.cross_track_gain * lateral + self.params.cross_track_damping * lateral_rate
        )
        limit = np.tan(np.radians(self.params.cross_track_max_deg))
        magnitude = float(np.linalg.norm(correction))
        if magnitude > limit:
            correction = correction * (limit / magnitude)
        return normalize(goal + correction, fallback=goal)

    def _blocked_along(self, agent_id: str) -> bool:
        """True when the safety filter modified this robot's input last step.

        A robot at the barrier is not short of effort, so integrating there only
        stores demand it will have to unwind. The flag is read from the previous
        step's own QP result, which is local.
        """
        for diagnostic in self.diagnostics:
            if diagnostic.agent_id == agent_id:
                return diagnostic.modification > 1e-3
        return False

    def _goal_for(self, view: BoundaryView) -> np.ndarray | None:
        for object_id in np.unique(view.object_ids):
            direction = self.goal_directions.get(str(object_id))
            if direction is not None:
                return direction
        return None

    def _object_for(self, view: BoundaryView, position: np.ndarray) -> str | None:
        index = self._nearest_index(view, position)
        if index is None:
            return None
        return str(view.object_ids[index])

    @staticmethod
    def _nearest_index(view: BoundaryView, position: np.ndarray) -> int | None:
        if len(view) == 0:
            return None
        delta = view.points - np.asarray(position, dtype=float).reshape(1, 2)
        return int(np.argmin(np.einsum("ij,ij->i", delta, delta)))

    def _approach_target(self, view: BoundaryView, position: np.ndarray, crowd: np.ndarray) -> np.ndarray:
        """Nearest unheld cage target in this robot's own map."""
        targets = self._cage_targets(view, None)
        occupied = np.min(np.linalg.norm(targets[:, None, :] - crowd[None, :, :], axis=2), axis=1)
        free = occupied > self.params.gap_radius
        if not np.any(free):
            weights = np.maximum(view.confidence, 1e-6)
            return np.sum(view.points * weights[:, None], axis=0) / float(np.sum(weights))
        candidates = targets[free]
        return candidates[int(np.argmin(np.linalg.norm(candidates - position[None, :], axis=1)))]

    def _exploration_velocity(
        self, i: int, agents: list[AgentState], neighbor_indices: list[int], timestamp: float
    ) -> np.ndarray:
        """Outward spiral sweep with neighbour repulsion.

        A robot with an empty map has no gradient to follow, so the sweep has to
        cover ground rather than mill about: the heading rotates slowly with time
        and with the robot index, which fans the team out instead of sending it
        all the same way, and the outward bias from the team centroid stops the
        sweep from collapsing back onto the start.
        """
        agent = agents[i]
        repel = np.zeros(2, dtype=float)
        for j in neighbor_indices:
            d = agent.position - agents[j].position
            distance = float(np.linalg.norm(d))
            if distance > 1e-6:
                repel += d / (distance * distance)
        if self.params.search_mode == "sweep":
            return self._sweep_velocity(i, len(agents), agent.position) + 0.15 * normalize(repel)
        angle = 0.7 * i + 0.25 * timestamp
        sweep = np.array([np.cos(angle), np.sin(angle)], dtype=float)
        return self.params.kp_explore * (0.7 * normalize(repel) + 0.3 * sweep)

    def _sweep_velocity(self, index: int, count: int, position: np.ndarray) -> np.ndarray:
        """Boustrophedon lane sweep.

        Robot ``i`` of ``N`` owns the vertical lane centred at
        ``xmin + m + (i + 0.5) (W - 2m) / N`` and walks it from end to end,
        reversing at the ends. Two things follow, and both are the point.

        *Coverage is bounded.* Every point of the workspace lies within
        ``max(lane_width/2, R_s)`` of some lane centre, so once each robot has
        traversed its lane height ``H`` the whole workspace has passed within
        sensor range: ``T_cover <= (d_to_lane + H) / v_max``, with no dependence on
        where the object is. That is the statement the spiral could not make.

        *No run-time coordination is needed.* The partition is a function of the
        robot index and the workspace bounds, both static. It is not a frontier
        method and does not claim to be: nothing is re-planned as the map fills,
        which costs efficiency on a partly-explored workspace and buys a bound that
        holds without any assumption about the communication graph.
        """
        xmin, xmax, ymin, ymax = self.domain
        margin = self.params.search_margin
        lanes = max(1, int(count))
        width = max(xmax - xmin - 2.0 * margin, 1e-6)
        lane_x = xmin + margin + (index + 0.5) * width / lanes
        low, high = ymin + margin, ymax - margin

        state = self._sweep_direction.setdefault(index, 1.0 if (index % 2 == 0) else -1.0)
        if position[1] >= high - self.params.search_lane_tolerance and state > 0.0:
            state = -1.0
        elif position[1] <= low + self.params.search_lane_tolerance and state < 0.0:
            state = 1.0
        self._sweep_direction[index] = state

        # Cross-lane error first, then along-lane travel: a robot that is far from
        # its lane closes on it before starting to sweep, so the lanes stay a
        # partition instead of everyone drifting through the middle.
        offset = lane_x - float(position[0])
        along = state * self.params.search_speed
        across = float(np.clip(2.0 * offset, -self.params.search_speed, self.params.search_speed))
        velocity = np.array([across, along], dtype=float)
        if abs(offset) > 4.0 * self.params.search_lane_tolerance:
            velocity[1] *= 0.35
        return velocity

    # ------------------------------------------------------------------ #
    # object token
    # ------------------------------------------------------------------ #

    def _update_tokens(self, agents: list[AgentState], neighbors: list[list[int]], timestamp: float) -> None:
        """One message per known object, relayed hop by hop.

        A robot that sees boundary refreshes its own token at the centroid of what
        it holds. Every step each robot merges its neighbours' tokens and keeps the
        freshest per object; tokens older than ``token_ttl`` are dropped, so a
        stale rumour expires instead of pulling the team to where the object used
        to be. This is the only thing that crosses the workspace: robots sweeping
        8 m of lanes are nowhere near each other's ``comm_range``, so without a
        relay the finder would have to enclose the object by itself.
        """
        for agent in agents:
            view = self._views[agent.agent_id]
            if len(view) == 0:
                continue
            store = self._tokens.setdefault(agent.agent_id, {})
            for object_id in np.unique(view.object_ids):
                picked = view.object_ids == object_id
                store[str(object_id)] = (view.points[picked].mean(axis=0), float(timestamp))

        merged: dict[str, dict[str, tuple[np.ndarray, float]]] = {}
        for i, agent in enumerate(agents):
            best = dict(self._tokens.get(agent.agent_id, {}))
            for j in neighbors[i]:
                for object_id, entry in self._tokens.get(agents[j].agent_id, {}).items():
                    if object_id not in best or entry[1] > best[object_id][1]:
                        best[object_id] = entry
            merged[agent.agent_id] = {
                object_id: entry
                for object_id, entry in best.items()
                if timestamp - entry[1] <= self.params.token_ttl
            }
        self._tokens = merged

    def _token_target(self, agent_id: str) -> np.ndarray | None:
        store = self._tokens.get(agent_id)
        if not store:
            return None
        return min(store.values(), key=lambda entry: -entry[1])[0]

    # ------------------------------------------------------------------ #
    # map-derived quantities
    # ------------------------------------------------------------------ #

    def _object_rows_from_map(
        self, agent_id: str, position: np.ndarray, view: BoundaryView
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None]:
        """Rows for the safety filter, from this robot's own map only.

        T2 adds the fourth return value: the estimated velocity of the material point
        at each map cell. It is ``None`` while ``estimate_yaw`` is off, which makes the
        filter take v1's single-translational-velocity path rather than an equivalent
        but differently-computed one.
        """
        if len(view) == 0:
            return np.empty((0, 2)), np.empty((0, 2)), np.zeros(2), None
        nearest = self._nearest_index(view, position)
        velocity = np.zeros(2)
        point_velocities = None
        if nearest is not None:
            object_id = str(view.object_ids[nearest])
            local_map = self.maps[agent_id]
            velocity = local_map.object_velocity(object_id)
            if local_map.estimate_yaw:
                point_velocities = local_map.object_point_velocities(object_id, view.points)
        return view.points, view.normals, velocity, point_velocities

    def _contact_ready(self, agent: AgentState, view: BoundaryView) -> bool:
        """True when the robot's *own* map says it sits in the contact band."""
        nearest = self._nearest_index(view, agent.position)
        if nearest is None:
            return False
        distance = float(np.linalg.norm(view.points[nearest] - agent.position))
        return distance <= self.params.cage_offset + self.params.contact_band_tolerance

    # ------------------------------------------------------------------ #
    # legacy region-coverage task mode
    # ------------------------------------------------------------------ #

    def _region_coverage_command(
        self, i: int, agents: list[AgentState], neighbor_indices: list[int], timestamp: float
    ) -> tuple[np.ndarray, str]:
        agent = agents[i]
        visible = self._visible_target_points(agent.position)
        if len(visible) == 0:
            return self._exploration_velocity(i, agents, neighbor_indices, timestamp), "search"
        density = BoundaryAwareDensity.from_targets(
            visible, sigma=self.params.sigma, base_density=self.params.base_density
        )
        cell = self.cvt.compute(i, agents, neighbor_indices, density, self.domain)
        return self.params.kp_cage * (cell.centroid - agent.position), "region_coverage"

    def _build_target_region_points(self) -> np.ndarray:
        center = np.asarray(self.params.target_center, dtype=float).reshape(2)
        count = max(1, int(self.params.target_samples))
        if count == 1:
            return center.reshape(1, 2)
        rings = max(1, int(np.ceil(np.sqrt(count))) - 1)
        points = [center]
        for ring in range(1, rings + 1):
            radius = self.params.target_radius * ring / rings
            samples = max(6, int(np.ceil(2.0 * np.pi * ring)))
            for k in range(samples):
                if len(points) >= count:
                    break
                angle = 2.0 * np.pi * k / samples
                points.append(center + radius * np.array([np.cos(angle), np.sin(angle)]))
            if len(points) >= count:
                break
        return np.asarray([clip_to_domain(p, self.domain) for p in points], dtype=float)

    def _visible_target_points(self, position: np.ndarray) -> np.ndarray:
        if len(self.target_region_points) == 0:
            return np.empty((0, 2))
        distances = np.linalg.norm(self.target_region_points - position[None, :], axis=1)
        return self.target_region_points[distances <= self.params.target_sensor_range]

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #

    def _ensure_state(self, agents: list[AgentState]) -> None:
        control = self.params.transport_control()
        for agent in agents:
            if agent.agent_id not in self.maps:
                self.maps[agent.agent_id] = LocalBoundaryMap(
                    voxel_size=self.params.voxel_size,
                    age_decay=self.params.age_decay,
                    max_voxels_per_object=self.params.max_voxels_per_object,
                    motion_compensation=self.params.motion_compensation,
                    registration_gate=self.params.registration_gate,
                    registration_normal_cosine=self.params.registration_normal_cosine,
                    max_object_speed=self.params.max_object_speed,
                    velocity_filter=self.params.object_velocity_filter,
                    estimate_yaw=self.params.estimate_object_yaw,
                    max_object_yaw_rate=self.params.max_object_yaw_rate,
                    min_yaw_lever_voxels=self.params.min_yaw_lever_voxels,
                    carve_enabled=self.params.carve_enabled,
                    carve_margin=self.params.carve_margin,
                )
            if agent.agent_id not in self.transport_loops:
                self.transport_loops[agent.agent_id] = DirectionalProgressController(control)
            self._views.setdefault(agent.agent_id, BoundaryView.empty())

    def _neighbor_indices(self, agents: list[AgentState]) -> list[list[int]]:
        if not agents:
            return []
        positions = np.vstack([a.position for a in agents])
        d = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
        np.fill_diagonal(d, np.inf)
        return [list(np.flatnonzero(row <= self.params.comm_range)) for row in d]

    def apply_commands(self, agents: list[AgentState], commands: list[ControlCommand], dt: float) -> None:
        by_id = {cmd.agent_id: cmd for cmd in commands}
        for agent in agents:
            cmd = by_id[agent.agent_id]
            agent.velocity = cmd.velocity.copy()
            agent.position = clip_to_domain(agent.position + cmd.velocity * dt, self.domain)

    def mode_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for diag in self.diagnostics:
            counts[diag.mode] = counts.get(diag.mode, 0) + 1
        return counts

    def map_snapshot(self, agent_id: str) -> BoundaryView:
        """What one robot believes the boundary is. Used by the renderer."""
        return self._views.get(agent_id, BoundaryView.empty())


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #


def _merge_scans(scans: list[BoundaryView], indices: list[int]) -> tuple[BoundaryView, np.ndarray]:
    """Concatenate a robot's own scan with its neighbours', tagging the source."""
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


def _angular_coverage(points: np.ndarray, bins: int = 36) -> float:
    """Fraction of bearing bins around the map's own centroid that hold boundary.

    A closed enclosure estimate is one whose boundary points surround their own
    centroid; a robot that has only seen one face gets a low score however many
    points it holds. The centroid is the map's, not the cargo's, so this is a
    statement about what the team has observed rather than about the object.
    """
    if len(points) < 3:
        return 0.0
    centroid = points.mean(axis=0)
    delta = points - centroid[None, :]
    radius = np.linalg.norm(delta, axis=1)
    live = radius > 1e-9
    if not np.any(live):
        return 0.0
    angle = np.arctan2(delta[live, 1], delta[live, 0])
    index = ((angle + np.pi) / (2.0 * np.pi) * bins).astype(int) % bins
    return float(len(np.unique(index)) / bins)


__all__ = ["DBACTController", "DBACTParams", "AgentDiagnostics"]
