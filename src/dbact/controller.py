"""S7 - the decentralised controller.

Per robot, per step, using only its own observations and its communication
neighbours:

    ray-cast scan -> voxel map -> boundary density -> limited-range CVT centroid
                                                   -> transport bias (gated)
                                                   -> CBF-QP safety filter

Three behaviours the previous controller did not have:

**Approach mode.** The density is compactly supported, so a robot far from the
cargo has an almost-empty cell and move-to-centroid returns its own position. It
does not enter exploration either, because communication has filled its map with
neighbours' observations -- its map is non-empty, it just contains nothing
nearby. The result is a robot that is permanently stuck while believing it is
converged; measured over a 12-robot run, 5 robots sat in this state and strict
coverage topped out at 0.069. The test is the cell mass ``m_i``: below
``ratio * phi_0 * pi R_l^2`` the cell carries no boundary information, and the
robot heads for the centroid of the observations it actually holds. This is still
a purely local computation. Adding it raised strict coverage from 0.069 to 0.444.

**Push-side allocation.** A transport bias applied by every robot is applied by
the robots in front of the cargo too, and those cancel or reverse the intended
motion. Only robots whose observed outward normal opposes the goal direction --
the ones actually behind the object -- add bias.

**A gate that is local.** Bias is enabled only once the robot itself is in the
contact band and enough of its neighbours report the same. This is one bit per
neighbour, so it stays decentralised.

The object-boundary rows given to the safety filter come from the robot's latest
raw scan, never from the persistent planning map or the simulator.  Keeping
those channels separate prevents stale gossiped geometry from becoming a hard
constraint while retaining it for enclosure planning.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import lsq_linear, nnls

from .boundary_density import BoundaryAwareDensity, DensityParams
from .boundary_map import LocalBoundaryMap
from .cargo import Cargo
from .contracts import ContactSafetyContract, CoverageContract
from .geometry import clip_to_domain, normalize
from .local_cvt import LocalCVT, empty_cell_threshold
from .perception import PerceptionParams, RayCastBoundarySensor
from .progress_control import ProgressPIController, ProgressPIOutput, ProgressPIParams
from .provenance import frame_rng
from .safety_filter import SafetyFilter, SafetyFilterParams
from .types import AgentState, BoundaryObservation, ControlCommand


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
    # Hard CBF rows need a stronger contract than planning-map samples.  Raw
    # observations below this PCA residual confidence remain available to the
    # map/auditor but cannot define a safety half-space.
    safety_min_confidence: float = 0.15

    # Perception and local CVT are the two expensive loops.  They may run at a
    # lower rate than the safety filter, which remains active on every physics
    # step.  A value of one preserves the original behaviour.
    perception_every: int = 1
    planning_every: int = 1
    # Full-rate flooding is retained during the certified rendezvous relay
    # interval.  During independent sweep and subsequent boundary mapping,
    # complete map snapshots can be exchanged less often because local scans and
    # hard safety rows continue at ``perception_every``.
    map_gossip_every: int = 1

    # --- communication ---
    comm_range: float = 1.6
    communication_dropout_prob: float = 0.0

    # --- map (S3) ---
    voxel_size: float = 0.06
    age_decay: float = 0.30
    max_voxels_per_object: int = 600
    map_max_translation_per_update: float = 0.03
    map_max_rotation_per_update: float = 0.04

    # --- density (S4) ---
    density_mode: str = "offset"
    cage_offset: float = 0.135
    # Offset used on the leading arc. Must exceed robot_radius so that the robots
    # in front of the object guide it without resisting the pushing arc; C1 checks
    # this. Set to None for a uniform cage (enclosure task, or the ablation that
    # shows why a uniform cage cannot be transported).
    lead_offset: float | None = 0.22
    lead_threshold: float = 0.35
    sigma: float = 0.20
    base_density: float = 1e-3
    gap_gain: float = 0.6
    gap_radius: float = 0.35

    # --- coverage (S5) ---
    local_radius: float = 0.80
    grid_resolution: int = 24
    approach_mass_ratio: float = 3.0
    redeploy_gap_ratio: float = 0.15

    # --- safety (S1) ---
    robot_radius: float = 0.16
    delta_max: float = 0.05
    boundary_error_bound: float = 0.0
    d_min: float = 0.30
    agent_distance_buffer: float = 0.0
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
    object_row_inner_limit: float | None = None
    object_barrier_geometry: str = "tangent_plane"
    object_active_tolerance: float = 0.02
    object_polyline_max_gap: float = 0.15
    object_polyline_max_normal_angle_deg: float = 30.0
    recovery_fraction: float = 0.6

    # --- gains ---
    kp_explore: float = 0.25
    kp_cage: float = 0.9
    kp_transport: float = 0.18

    # Search without object-position knowledge. ``paired_lanes`` gives a finite
    # rectangular-workspace coverage bound: two lane teams sweep inward from
    # opposite sides, rendezvous, and hold long enough for neighbour-to-neighbour
    # map gossip before enclosure starts.
    search_pattern: str = "legacy"  # "legacy" | "contracting_ring" | "paired_lanes"
    search_center: list[float] | None = None
    search_inner_radius: float = 1.6
    search_inward_speed: float = 0.18
    search_angular_speed: float = 0.04
    search_detection_radius: float = 0.80
    search_speed: float = 0.30
    search_meeting_gap: float = 0.40
    search_gossip_time: float = 0.90
    search_local_gossip_time: float = 0.60
    map_gossip: bool = False
    boundary_mapping_time: float = 0.0
    boundary_mapping_radius: float = 1.0
    boundary_mapping_angular_speed: float = 0.35

    # --- transport gating (S7) ---
    push_side_threshold: float = 0.35
    min_push_agents: int = 3
    contact_band_tolerance: float = 0.08
    object_velocity_filter: float = 0.4
    # Common feed-forward velocity used after a locally observed contact quorum
    # has persisted for ``transport_dwell_steps``.  A non-zero value translates
    # the *whole* enclosure; restricting it to the pushing arc tears the cage
    # apart as soon as the cargo starts to move.  Contact forces still arise only
    # from the physics engine and the push-side robots still provide the inward
    # preload through ``kp_transport``.
    transport_speed: float = 0.0
    transport_dwell_steps: int = 0
    transport_min_steps: int = 0
    # Stop the feed-forward/pushing phase after this locally estimated signed
    # displacement.  Zero disables the bound.  The estimate is the integral of
    # point-to-plane map registrations, so the controller does not read the
    # simulator's cargo pose to decide when to stop.
    transport_distance: float = 0.0
    transport_progress_estimator: str = "centroid"  # "centroid" | "motion_integral"

    # Closed-loop task-progress / contact-pressure regulation. Legacy configs
    # keep this disabled; research configs enable it and no longer depend on a
    # fixed transport feed-forward speed or a scheduled HOLD frame.
    progress_feedback: bool = False
    progress_consensus: bool = True
    progress_consensus_hops: int = 18
    cross_track_gain: float = 2.0
    progress_kp: float = 0.8
    progress_max_speed: float = 0.18
    pressure_position_gain: float = 0.25
    pressure_velocity_kp: float = 0.9
    pressure_velocity_ki: float = 0.35
    pressure_bias: float = 0.025
    pressure_limit: float = 0.20
    pressure_integral_limit: float = 0.50
    pressure_anti_windup_gain: float = 1.0
    brake_position_gain: float = 0.15
    brake_activation_distance: float = 0.035
    brake_position_tolerance: float = 0.025
    brake_speed_tolerance: float = 0.025
    brake_dwell_steps: int = 12
    brake_reengage_error: float = 0.06
    hold_exit_error: float = 0.08
    convoy_feedback_gain: float = 1.0
    wrench_allocation: bool = False
    wrench_torque_weight: float = 1.0
    wrench_residual_tolerance: float = 0.35
    wrench_weight_limit: float = 3.0
    wrench_gossip_hops: int = 18
    wrench_regularization: float = 0.0
    contact_release_gain: float = 3.0
    contact_release_speed: float = 0.20
    contact_release_enabled: bool = True
    safety_pressure_reserve: bool = True

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
            boundary_error_bound=self.boundary_error_bound,
            d_min=self.d_min,
            lead_offset=self.lead_offset if self.task_mode == "transport" else None,
        )

    def coverage_contract(self) -> CoverageContract:
        return CoverageContract(local_radius=self.local_radius, comm_range=self.comm_range)


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
    transport_progress: float = 0.0
    parallel_velocity: float = 0.0
    position_error: float = 0.0
    velocity_error: float = 0.0
    velocity_reference: float = 0.0
    pressure_effort: float = 0.0
    pressure_saturated: bool = False
    wrench_weight: float = 1.0
    wrench_residual: float = 0.0
    wrench_feasible: bool = True
    max_full_margin_deficit: float = 0.0
    max_barrier_deficit: float = 0.0
    max_object_margin_deficit: float = 0.0
    min_object_h: float = float("inf")
    max_object_velocity_projection: float = 0.0


class DBACTController:
    """Decentralised boundary-aware enclosure and cooperative transport."""

    def __init__(
        self,
        params: DBACTParams,
        domain: tuple[float, float, float, float],
        goal_directions: dict[str, np.ndarray] | None = None,
        seed: int = 0,
    ):
        self.params = params
        self.domain = domain
        self.seed = int(seed)
        self.goal_directions = {k: normalize(np.asarray(v, dtype=float)) for k, v in (goal_directions or {}).items()}

        if (
            params.perception_every < 1
            or params.planning_every < 1
            or params.map_gossip_every < 1
        ):
            raise ValueError(
                "perception_every, planning_every and map_gossip_every must be positive integers"
            )
        if not 0.0 <= params.communication_dropout_prob < 1.0:
            raise ValueError("communication_dropout_prob must lie in [0, 1)")
        if not 0.0 <= params.safety_min_confidence <= 1.0:
            raise ValueError("safety_min_confidence must lie in [0, 1]")
        if params.object_barrier_geometry not in {
            "tangent_plane",
            "point_distance",
            "polyline_distance",
        }:
            raise ValueError(
                "object_barrier_geometry must be 'tangent_plane', 'point_distance' or "
                "'polyline_distance'"
            )
        if params.object_active_tolerance < 0.0:
            raise ValueError("object_active_tolerance cannot be negative")
        if params.use_object_barrier and params.rho >= params.max_speed:
            raise ValueError(
                "rho must be strictly below max_speed; otherwise the full ISSf row is "
                "kinematically infeasible at h=0 even for a stationary object"
            )
        if params.agent_distance_buffer < 0.0:
            raise ValueError("agent_distance_buffer cannot be negative")
        if params.map_max_translation_per_update <= 0.0:
            raise ValueError("map_max_translation_per_update must be positive")
        if params.map_max_rotation_per_update <= 0.0:
            raise ValueError("map_max_rotation_per_update must be positive")
        if params.object_polyline_max_gap <= 0.0:
            raise ValueError("object_polyline_max_gap must be positive")
        if not 0.0 <= params.object_polyline_max_normal_angle_deg < 180.0:
            raise ValueError("object_polyline_max_normal_angle_deg must lie in [0, 180)")
        if params.search_pattern not in {"legacy", "contracting_ring", "paired_lanes"}:
            raise ValueError("search_pattern must be 'legacy', 'contracting_ring' or 'paired_lanes'")
        if params.search_pattern == "paired_lanes":
            if params.search_speed <= 0.0:
                raise ValueError("paired_lanes search_speed must be positive")
            if params.search_detection_radius <= 0.0:
                raise ValueError("paired_lanes search_detection_radius must be positive")
            if params.search_meeting_gap < params.d_min:
                raise ValueError("paired_lanes search_meeting_gap must be at least d_min")
            if params.search_gossip_time < 0.0 or params.search_local_gossip_time < 0.0:
                raise ValueError("paired_lanes gossip times cannot be negative")
            if params.boundary_mapping_time < 0.0 or params.boundary_mapping_radius <= 0.0:
                raise ValueError("paired_lanes boundary mapping time/radius are invalid")
        if params.progress_feedback:
            ProgressPIParams(
                target=params.transport_distance,
                progress_kp=params.progress_kp,
                max_reference_speed=params.progress_max_speed,
                position_effort_gain=params.pressure_position_gain,
                velocity_kp=params.pressure_velocity_kp,
                velocity_ki=params.pressure_velocity_ki,
                pressure_bias=params.pressure_bias,
                effort_limit=params.pressure_limit,
                integral_limit=params.pressure_integral_limit,
                anti_windup_gain=params.pressure_anti_windup_gain,
                brake_position_gain=params.brake_position_gain,
            ).assert_valid()
            if params.brake_activation_distance < 0.0 or params.brake_position_tolerance < 0.0:
                raise ValueError("brake distances cannot be negative")
            if params.brake_speed_tolerance < 0.0 or params.brake_dwell_steps < 1:
                raise ValueError("brake speed tolerance/dwell are invalid")
            if params.brake_reengage_error <= params.brake_position_tolerance:
                raise ValueError("brake_reengage_error must exceed brake_position_tolerance")
            if params.hold_exit_error <= params.brake_position_tolerance:
                raise ValueError("hold_exit_error must exceed brake_position_tolerance")
            if params.convoy_feedback_gain < 0.0:
                raise ValueError("convoy_feedback_gain cannot be negative")
            if params.cross_track_gain < 0.0:
                raise ValueError("cross_track_gain cannot be negative")
            if params.wrench_torque_weight < 0.0 or params.wrench_residual_tolerance < 0.0:
                raise ValueError("wrench allocation weights/tolerance cannot be negative")
            if params.wrench_regularization < 0.0:
                raise ValueError("wrench_regularization cannot be negative")
            if params.wrench_weight_limit <= 0.0:
                raise ValueError("wrench_weight_limit must be positive")
            if params.wrench_gossip_hops < 1:
                raise ValueError("wrench_gossip_hops must be positive")
            if params.progress_consensus_hops < 1:
                raise ValueError("progress_consensus_hops must be positive")
            if params.contact_release_gain < 0.0 or params.contact_release_speed < 0.0:
                raise ValueError("contact release gain/speed cannot be negative")
        if params.transport_progress_estimator not in {"centroid", "motion_integral"}:
            raise ValueError("transport_progress_estimator must be 'centroid' or 'motion_integral'")

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
                d_min=params.d_min + params.agent_distance_buffer,
                gamma_agent=params.gamma_agent,
                gamma_obj=params.gamma_obj,
                rho=params.rho,
                r_safe=params.r_safe,
                boundary_error_bound=params.boundary_error_bound,
                max_speed=params.max_speed,
                backend=params.backend,
                enable_object_rows=params.use_object_barrier,
                max_object_rows=params.max_object_rows,
                object_row_range=params.object_row_range,
                object_row_window=params.object_row_window,
                object_row_inner_limit=(
                    params.robot_radius if params.object_row_inner_limit is None else params.object_row_inner_limit
                ),
                object_barrier_geometry=params.object_barrier_geometry,
                object_active_tolerance=params.object_active_tolerance,
                object_polyline_max_gap=params.object_polyline_max_gap,
                object_polyline_max_normal_angle_deg=params.object_polyline_max_normal_angle_deg,
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
        )
        self.empty_cell_mass = empty_cell_threshold(params.local_radius, params.base_density, params.approach_mass_ratio)

        self.maps: dict[str, LocalBoundaryMap] = {}
        self._redeploy_target: dict[str, np.ndarray | None] = {}
        self.object_velocity: dict[str, dict[str, np.ndarray]] = {}
        self.object_angular_velocity: dict[str, dict[str, float]] = {}
        self._object_centroid: dict[str, dict[str, np.ndarray]] = {}
        self._transport_ready_streak: dict[str, int] = {}
        self._transport_progress: dict[str, dict[str, float]] = {}
        self._transport_displacement: dict[str, dict[str, np.ndarray]] = {}
        self._transport_origin_centroid: dict[str, dict[str, np.ndarray]] = {}
        self._transport_complete_latch: dict[str, bool] = {}
        self._transport_start_frame: dict[str, int] = {}
        self._transport_phase: dict[str, str] = {}
        self._transport_brake_streak: dict[str, int] = {}
        self._progress_regulators: dict[tuple[str, str], ProgressPIController] = {}
        self._progress_feedback: dict[str, ProgressPIOutput] = {}
        self._planned_transport_phase: dict[str, str] = {}
        self._wrench_weights: dict[str, float] = {}
        self._wrench_residuals: dict[str, float] = {}
        self._wrench_feasible: dict[str, bool] = {}
        self._search_initial_polar: dict[str, tuple[float, float]] = {}
        self._search_initial_pose: dict[str, np.ndarray] = {}
        self._first_observation_time: dict[str, float] = {}
        self._nominal_cache: dict[str, tuple[np.ndarray, str, float, bool]] = {}
        self._fused_cache: dict[str, list[BoundaryObservation]] = {}
        # Planning needs a persistent, gossiped outline.  Safety does not: old
        # tangent planes become false half-spaces when the cargo rotates.  Keep
        # the most recent one-hop raw scan separately so the CBF only sees
        # locally observable, time-bounded geometry.
        self._safety_observation_cache: dict[str, list[BoundaryObservation]] = {}
        self.target_region_points = self._build_target_region_points()
        self.diagnostics: list[AgentDiagnostics] = []
        self.last_detection_counts: dict[str, int] = {}
        self.last_sensed_observations: dict[str, list[BoundaryObservation]] = {}
        self.last_perception_timestamp: float | None = None
        self.communication_candidate_links = 0
        self.communication_delivered_links = 0
        self._time = 0.0
        self._frame = 0

    # ------------------------------------------------------------------ #
    # main loop
    # ------------------------------------------------------------------ #

    def step(self, agents: list[AgentState], cargoes: list[Cargo], timestamp: float, dt: float) -> list[ControlCommand]:
        self._time = float(timestamp)
        self._ensure_maps(agents)
        physical_neighbors = self._neighbor_indices(agents)
        neighbors = self._communication_neighbor_indices(agents, physical_neighbors)

        refresh_perception = self._frame % int(self.params.perception_every) == 0
        sensed: dict[str, list[BoundaryObservation]] = {agent.agent_id: [] for agent in agents}
        if refresh_perception:
            for agent in agents:
                sensed[agent.agent_id] = self.sensor.sense(agent, cargoes, timestamp)
            self.last_sensed_observations = {
                agent_id: list(observations) for agent_id, observations in sensed.items()
            }
            self.last_perception_timestamp = float(timestamp)
            self.last_detection_counts = {}
            for observations in sensed.values():
                for observation in observations:
                    self.last_detection_counts[observation.object_id] = (
                        self.last_detection_counts.get(observation.object_id, 0) + 1
                    )
        else:
            self.last_detection_counts = {}

        # Own observations first, then one hop of neighbour relay. Voxel fusion
        # makes the relay idempotent: hearing the same cell twice adds no mass.
        # The fused view is read once per agent per step and reused, because
        # rebuilding it for every consumer also re-prunes the map each time.
        fused: dict[str, list[BoundaryObservation]] = {}
        prior_fused = {agent_id: list(items) for agent_id, items in self._fused_cache.items()}
        full_map_gossip = self.params.map_gossip
        if self.params.search_pattern == "paired_lanes":
            sweep, rendezvous, gossip = self._paired_search_durations()
            full_map_gossip = full_map_gossip and self._time <= (
                sweep + rendezvous + gossip + self.params.boundary_mapping_time
            )
        gossip_due = self._map_gossip_due(refresh_perception)
        for i, agent in enumerate(agents):
            if refresh_perception:
                batch = list(sensed[agent.agent_id])
                safety_batch = [
                    obs for obs in batch if obs.confidence >= self.params.safety_min_confidence
                ]
                for j in neighbors[i]:
                    batch.extend(sensed[agents[j].agent_id])
                    # Neighbour scans improve map completeness but their tangent
                    # planes need not be local to this robot (especially across
                    # a concave corner).  They are therefore never admitted as
                    # hard CBF rows for this agent.
                self._safety_observation_cache[agent.agent_id] = safety_batch
                self.maps[agent.agent_id].update(batch, timestamp)
                if full_map_gossip and gossip_due:
                    for j in neighbors[i]:
                        self.maps[agent.agent_id].merge_observations(
                            prior_fused.get(agents[j].agent_id, []),
                            timestamp,
                        )
                self._fused_cache[agent.agent_id] = self.maps[agent.agent_id].all_observations(timestamp)
            fused[agent.agent_id] = self._fused_cache.get(agent.agent_id, [])
        if refresh_perception:
            for agent in agents:
                self._update_object_velocity(
                    agent.agent_id,
                    fused[agent.agent_id],
                    dt * int(self.params.perception_every),
                )

        contact_ready = [self._contact_ready(agents[i], fused[agents[i].agent_id]) for i in range(len(agents))]
        ready_flags: list[bool] = []
        for i, agent in enumerate(agents):
            supporters = int(contact_ready[i]) + sum(int(contact_ready[j]) for j in neighbors[i])
            search_released = (
                self.params.search_pattern != "paired_lanes"
                or self._time >= self._paired_search_durations()[0] + self._paired_search_durations()[1]
                + self._paired_search_durations()[2] - 1e-12
            )
            if (
                self.params.task_mode == "transport"
                and supporters >= self.params.min_push_agents
                and search_released
            ):
                self._transport_ready_streak[agent.agent_id] = self._transport_ready_streak.get(agent.agent_id, 0) + 1
            else:
                self._transport_ready_streak[agent.agent_id] = 0
            ready = self._transport_ready_streak[agent.agent_id] >= max(0, int(self.params.transport_dwell_steps))
            if ready:
                self._transport_start_frame.setdefault(agent.agent_id, self._frame)
                if self.params.progress_feedback:
                    self._transport_phase.setdefault(agent.agent_id, "transport")
            ready_flags.append(ready)

        if self.params.progress_feedback and self.params.progress_consensus:
            ready_flags = self._consensus_ready_flags(agents, neighbors, ready_flags)
            for i, agent in enumerate(agents):
                if ready_flags[i]:
                    self._transport_start_frame.setdefault(agent.agent_id, self._frame)
                    self._transport_phase.setdefault(agent.agent_id, "transport")

        if refresh_perception:
            track_progress = bool(self._transport_phase)
            for i, agent in enumerate(agents):
                phase = self._transport_phase.get(agent.agent_id, "cage")
                if track_progress or ready_flags[i] or phase in {"transport", "brake", "hold"}:
                    self._accumulate_transport_progress(agent.agent_id)
            if self.params.progress_feedback and self.params.progress_consensus:
                self._consensus_progress_and_velocity(agents, neighbors, fused)
            if self.params.progress_feedback:
                for i, agent in enumerate(agents):
                    phase = self._transport_phase.get(agent.agent_id, "cage")
                    if ready_flags[i] or phase in {"transport", "brake", "hold"}:
                        self._update_progress_feedback(
                            agent.agent_id,
                            fused[agent.agent_id],
                            dt * int(self.params.perception_every),
                            braking=phase == "brake",
                        )

        transport_phases: list[str] = []
        transport_complete: list[bool] = []
        transport_active: list[bool] = []
        for i, agent in enumerate(agents):
            if self.params.progress_feedback:
                phase = self._advance_transport_phase(
                    agent.agent_id,
                    fused[agent.agent_id],
                    ready=ready_flags[i],
                )
                planned_phase = phase if ready_flags[i] or phase == "hold" else "cage"
                transport_phases.append(planned_phase)
                transport_complete.append(phase == "hold")
                transport_active.append(planned_phase == "transport")
            else:
                complete = self._transport_complete(agent.agent_id, fused[agent.agent_id])
                transport_complete.append(complete)
                transport_active.append(ready_flags[i] and not complete)
                transport_phases.append("hold" if complete else ("transport" if transport_active[-1] else "cage"))

        if self.params.progress_feedback and self.params.wrench_allocation:
            self._update_wrench_allocations(
                agents,
                neighbors,
                fused,
                contact_ready,
                transport_phases,
            )

        self.diagnostics = []
        commands: list[ControlCommand] = []
        for i, agent in enumerate(agents):
            observations = fused[agent.agent_id]
            refresh_plan = (
                self._frame % int(self.params.planning_every) == 0
                or agent.agent_id not in self._nominal_cache
                or self._planned_transport_phase.get(agent.agent_id) != transport_phases[i]
            )
            if refresh_plan:
                self._nominal_cache[agent.agent_id] = self._nominal_command(
                    i,
                    agents,
                    neighbors[i],
                    observations,
                    contact_ready,
                    transport_active[i],
                    transport_phase=transport_phases[i],
                )
                self._planned_transport_phase[agent.agent_id] = transport_phases[i]
            cached = self._nominal_cache[agent.agent_id]
            u_nom, mode, cell_mass, push_side = cached[0].copy(), cached[1], cached[2], cached[3]
            if transport_complete[i] and not push_side and mode != "hold":
                mode = "hold"
            # Persistent/gossiped geometry remains the planning map.  The
            # safety rows deliberately use only the latest local/one-hop scan.
            # Between perception frames its boundary points are propagated by
            # the locally estimated object velocity.
            safety_observations = self._safety_observation_cache.get(agent.agent_id, [])
            points, normals, v_obj = self._object_rows_from_map(
                agent.agent_id,
                agent.position,
                safety_observations,
                timestamp=self._time,
            )
            result = self.safety.filter_velocity(
                agent.position,
                u_nom,
                [agents[j].position for j in physical_neighbors[i]],
                # Inter-agent safety is based on local relative-position sensing,
                # not a best-effort communication packet.  Dropout therefore
                # affects map/progress/wrench messages but never removes a CBF row.
                boundary_points=points,
                boundary_normals=normals,
                object_velocity=v_obj,
            )
            commands.append(ControlCommand(agent.agent_id, result.velocity, mode=mode))
            feedback = self._progress_feedback.get(agent.agent_id)
            self.diagnostics.append(
                AgentDiagnostics(
                    agent_id=agent.agent_id,
                    mode=mode,
                    cell_mass=cell_mass,
                    object_rows=result.object_rows,
                    contact_ready=contact_ready[i],
                    push_side=push_side,
                    solver_status=result.status,
                    modification=result.modification,
                    transport_progress=self._progress_for(agent.agent_id, observations),
                    parallel_velocity=feedback.parallel_velocity if feedback is not None else 0.0,
                    position_error=feedback.position_error if feedback is not None else 0.0,
                    velocity_error=feedback.velocity_error if feedback is not None else 0.0,
                    velocity_reference=feedback.velocity_reference if feedback is not None else 0.0,
                    pressure_effort=feedback.effort if feedback is not None else 0.0,
                    pressure_saturated=feedback.saturated if feedback is not None else False,
                    wrench_weight=self._wrench_weights.get(agent.agent_id, 1.0),
                    wrench_residual=self._wrench_residuals.get(agent.agent_id, 0.0),
                    wrench_feasible=self._wrench_feasible.get(agent.agent_id, True),
                    max_full_margin_deficit=result.max_full_margin_deficit,
                    max_barrier_deficit=result.max_barrier_deficit,
                    max_object_margin_deficit=result.max_object_margin_deficit,
                    min_object_h=result.min_object_h,
                    max_object_velocity_projection=result.max_object_velocity_projection,
                )
            )
        self._frame += 1
        return commands

    def _accumulate_transport_progress(self, agent_id: str) -> None:
        """Update locally estimated task progress without reading cargo truth.

        Historical v3 uses a boundary-map centroid difference. The feedback
        controller instead integrates the translational component of the SE(2)
        point-to-plane registration. That avoids treating changing visible-map
        support as body motion while the registration explicitly separates
        rotation from translation.
        """
        if self.params.search_pattern == "paired_lanes":
            sweep, rendezvous, gossip = self._paired_search_durations()
            if self._time < sweep + rendezvous + gossip - 1e-12:
                return
        progress = self._transport_progress.setdefault(agent_id, {})
        displacement = self._transport_displacement.setdefault(agent_id, {})
        origins = self._transport_origin_centroid.setdefault(agent_id, {})
        observations = self.maps[agent_id].all_observations(self._time)
        object_ids = {obs.object_id for obs in observations}
        for object_id in object_ids:
            goal = self.goal_directions.get(object_id)
            if goal is None:
                continue
            if self.params.transport_progress_estimator == "motion_integral":
                delta = np.asarray(
                    self.maps[agent_id].last_motion.get(object_id, np.zeros(2)), dtype=float
                )
                displacement[object_id] = np.asarray(
                    displacement.get(object_id, np.zeros(2)), dtype=float
                ) + delta
                progress[object_id] = float(np.dot(displacement[object_id], goal))
                continue
            points = np.asarray(
                [obs.point for obs in observations if obs.object_id == object_id],
                dtype=float,
            )
            centroid = np.mean(points, axis=0)
            origin = origins.setdefault(object_id, centroid.copy())
            displacement[object_id] = centroid - origin
            progress[object_id] = float(np.dot(displacement[object_id], goal))

    def _progress_for(self, agent_id: str, observations: list[BoundaryObservation]) -> float:
        progress = self._transport_progress.get(agent_id, {})
        for obs in observations:
            if obs.object_id in progress:
                return float(progress[obs.object_id])
        return 0.0

    def _progress_object_id(self, observations: list[BoundaryObservation]) -> str | None:
        for obs in observations:
            if obs.object_id in self.goal_directions:
                return obs.object_id
        return None

    def _progress_regulator(self, agent_id: str, object_id: str) -> ProgressPIController:
        key = (agent_id, object_id)
        regulator = self._progress_regulators.get(key)
        if regulator is None:
            regulator = ProgressPIController(
                ProgressPIParams(
                    target=self.params.transport_distance,
                    progress_kp=self.params.progress_kp,
                    max_reference_speed=self.params.progress_max_speed,
                    position_effort_gain=self.params.pressure_position_gain,
                    velocity_kp=self.params.pressure_velocity_kp,
                    velocity_ki=self.params.pressure_velocity_ki,
                    pressure_bias=self.params.pressure_bias,
                    effort_limit=self.params.pressure_limit,
                    integral_limit=self.params.pressure_integral_limit,
                    anti_windup_gain=self.params.pressure_anti_windup_gain,
                    brake_position_gain=self.params.brake_position_gain,
                )
            )
            self._progress_regulators[key] = regulator
        return regulator

    def _consensus_progress_and_velocity(
        self,
        agents: list[AgentState],
        neighbors: list[list[int]],
        fused: dict[str, list[BoundaryObservation]],
    ) -> None:
        """Finite-hop consensus-equivalent robust fusion of task estimates.

        Every perception period each local estimator contributes one motion
        increment.  Component-wise medians reject a single ICP jump and, because
        the previous consensus value is shared, integrate the median increment
        without accessing simulator cargo state.
        """
        unseen = set(range(len(agents)))
        while unseen:
            root = min(unseen)
            component = {root}
            frontier = {root}
            for _ in range(int(self.params.progress_consensus_hops)):
                expanded: set[int] = set()
                for index in frontier:
                    expanded.update(neighbors[index])
                expanded -= component
                if not expanded:
                    break
                component.update(expanded)
                frontier = expanded
            unseen -= component
            object_ids = {
                obs.object_id
                for index in component
                for obs in fused.get(agents[index].agent_id, [])
                if obs.object_id in self.goal_directions
            }
            for object_id in object_ids:
                progress_values = [
                    self._transport_progress.get(agents[index].agent_id, {}).get(object_id)
                    for index in component
                ]
                progress_values = [float(value) for value in progress_values if value is not None]
                if progress_values:
                    consensus_progress = float(np.median(progress_values))
                    for index in component:
                        self._transport_progress.setdefault(agents[index].agent_id, {})[
                            object_id
                        ] = consensus_progress
                displacement_values = [
                    self._transport_displacement.get(agents[index].agent_id, {}).get(object_id)
                    for index in component
                ]
                displacement_values = [
                    np.asarray(value, dtype=float)
                    for value in displacement_values
                    if value is not None
                ]
                if displacement_values:
                    consensus_displacement = np.median(
                        np.vstack(displacement_values),
                        axis=0,
                    )
                    consensus_progress = float(
                        np.dot(consensus_displacement, self.goal_directions[object_id])
                    )
                    for index in component:
                        agent_id = agents[index].agent_id
                        self._transport_displacement.setdefault(agent_id, {})[
                            object_id
                        ] = consensus_displacement.copy()
                        self._transport_progress.setdefault(agent_id, {})[
                            object_id
                        ] = consensus_progress
                velocity_values = [
                    self.object_velocity.get(agents[index].agent_id, {}).get(object_id)
                    for index in component
                ]
                velocity_values = [
                    np.asarray(value, dtype=float) for value in velocity_values if value is not None
                ]
                if velocity_values:
                    consensus_velocity = np.median(np.vstack(velocity_values), axis=0)
                    for index in component:
                        self.object_velocity.setdefault(agents[index].agent_id, {})[
                            object_id
                        ] = consensus_velocity.copy()
                angular_values = [
                    self.object_angular_velocity.get(agents[index].agent_id, {}).get(object_id)
                    for index in component
                ]
                angular_values = [
                    float(value) for value in angular_values if value is not None
                ]
                if angular_values:
                    consensus_angular_velocity = float(np.median(angular_values))
                    for index in component:
                        self.object_angular_velocity.setdefault(
                            agents[index].agent_id,
                            {},
                        )[object_id] = consensus_angular_velocity

    def _consensus_ready_flags(
        self,
        agents: list[AgentState],
        neighbors: list[list[int]],
        ready_flags: list[bool],
    ) -> list[bool]:
        """Flood a persistent local contact quorum through each comm component."""
        consensus = list(ready_flags)
        unseen = set(range(len(agents)))
        while unseen:
            root = min(unseen)
            component = {root}
            frontier = {root}
            for _ in range(int(self.params.progress_consensus_hops)):
                expanded: set[int] = set()
                for index in frontier:
                    expanded.update(neighbors[index])
                expanded -= component
                if not expanded:
                    break
                component.update(expanded)
                frontier = expanded
            unseen -= component
            component_ready = any(ready_flags[index] for index in component)
            if component_ready:
                for index in component:
                    consensus[index] = True
        return consensus

    def _update_progress_feedback(
        self,
        agent_id: str,
        observations: list[BoundaryObservation],
        dt: float,
        *,
        braking: bool,
    ) -> ProgressPIOutput | None:
        """Update one agent's regulator from local map motion only."""
        object_id = self._progress_object_id(observations)
        if object_id is None:
            return None
        progress = float(self._transport_progress.get(agent_id, {}).get(object_id, 0.0))
        goal = self.goal_directions[object_id]
        velocity = np.asarray(
            self.object_velocity.get(agent_id, {}).get(object_id, np.zeros(2)), dtype=float
        )
        output = self._progress_regulator(agent_id, object_id).update(
            progress,
            float(np.dot(velocity, goal)),
            dt,
            braking=braking,
        )
        self._progress_feedback[agent_id] = output
        return output

    def _advance_transport_phase(
        self,
        agent_id: str,
        observations: list[BoundaryObservation],
        *,
        ready: bool,
    ) -> str:
        """Distributed TRANSPORT -> BRAKE -> HOLD supervisor.

        Transitions use only the agent's progress/velocity feedback. Loss of the
        local contact quorum pauses pressure application but does not erase the
        phase or integrator, so a transient communication gap can recover.
        """
        phase = self._transport_phase.get(agent_id, "cage")
        if phase == "cage" and ready:
            phase = "transport"
            self._transport_phase[agent_id] = phase
        feedback = self._progress_feedback.get(agent_id)
        if feedback is None:
            return phase

        if phase == "transport" and feedback.progress >= (
            self.params.transport_distance - self.params.brake_activation_distance
        ):
            phase = "brake"
            self._transport_phase[agent_id] = phase
            self._transport_brake_streak[agent_id] = 0
            object_id = self._progress_object_id(observations)
            if object_id is not None:
                # The positive transport-pressure integral must not survive the
                # mode switch and overpower a negative BRAKE position error.
                self._progress_regulator(agent_id, object_id).reset()
            self._update_progress_feedback(agent_id, observations, 0.0, braking=True)
            feedback = self._progress_feedback.get(agent_id, feedback)

        if phase == "brake":
            position_ok = abs(feedback.position_error) <= self.params.brake_position_tolerance
            speed_ok = abs(feedback.parallel_velocity) <= self.params.brake_speed_tolerance
            if position_ok and speed_ok:
                streak = self._transport_brake_streak.get(agent_id, 0) + 1
                self._transport_brake_streak[agent_id] = streak
                if streak >= int(self.params.brake_dwell_steps):
                    phase = "hold"
                    self._transport_phase[agent_id] = phase
            else:
                self._transport_brake_streak[agent_id] = 0
                if abs(feedback.position_error) >= self.params.brake_reengage_error:
                    phase = "transport"
                    self._transport_phase[agent_id] = phase
                    self._update_progress_feedback(agent_id, observations, 0.0, braking=False)

        if phase == "hold" and abs(feedback.position_error) >= self.params.hold_exit_error:
            phase = "transport"
            self._transport_phase[agent_id] = phase
            self._transport_brake_streak[agent_id] = 0
            self._update_progress_feedback(agent_id, observations, 0.0, braking=False)
        return phase

    def _update_wrench_allocations(
        self,
        agents: list[AgentState],
        neighbors: list[list[int]],
        fused: dict[str, list[BoundaryObservation]],
        contact_ready: list[bool],
        phases: list[str],
    ) -> None:
        """Finite-hop consensus allocation of nonnegative contact pressure.

        Repeated neighbour flooding gives every robot in a connected component
        the same contact descriptors and map-centroid estimates.  This routine
        computes that converged result directly (bounded by
        ``wrench_gossip_hops``), then solves one common force/zero-torque NNLS.
        No simulator pose or measured contact wrench enters the calculation.
        """
        for agent in agents:
            self._wrench_weights[agent.agent_id] = 0.0
            self._wrench_residuals[agent.agent_id] = 0.0
            self._wrench_feasible[agent.agent_id] = True

        unseen = set(range(len(agents)))
        while unseen:
            root = min(unseen)
            component = {root}
            frontier = {root}
            for _ in range(int(self.params.wrench_gossip_hops)):
                expanded = set(frontier)
                for index in frontier:
                    expanded.update(neighbors[index])
                expanded -= component
                if not expanded:
                    break
                component.update(expanded)
                frontier = expanded
            unseen -= component

            active = sorted(
                index
                for index in component
                if contact_ready[index] and phases[index] in {"transport", "brake"}
            )
            if not active:
                continue
            object_ids = [
                self._progress_object_id(fused.get(agents[index].agent_id, [])) for index in active
            ]
            object_id = next((value for value in object_ids if value is not None), None)
            if object_id is None:
                for index in active:
                    self._wrench_feasible[agents[index].agent_id] = False
                    self._wrench_residuals[agents[index].agent_id] = float("inf")
                continue

            signed_efforts = [
                self._progress_feedback[agents[index].agent_id].effort
                for index in active
                if agents[index].agent_id in self._progress_feedback
            ]
            sign = 1.0 if not signed_efforts or float(np.median(signed_efforts)) >= 0.0 else -1.0
            representative_id = agents[active[0]].agent_id
            drive = self._transport_drive_direction(
                representative_id,
                object_id,
                longitudinal_sign=sign,
            )
            component_points = [
                obs.point
                for index in component
                for obs in fused.get(agents[index].agent_id, [])
                if obs.object_id == object_id
            ]
            if not component_points:
                continue
            points = np.vstack(component_points)
            center = np.mean(points, axis=0)
            scale = max(float(np.max(np.linalg.norm(points - center[None, :], axis=1))), 1e-6)

            candidate_indices: list[int] = []
            columns: list[np.ndarray] = []
            alignments: list[float] = []
            for index in active:
                # A contact whose full ISSf reserve is nearly exhausted has no
                # safe inward wrench capacity. Exclude it before solving the
                # allocation so its zero weight activates local contact release;
                # rho and d_min remain unchanged in the hard QP.
                release_guard = self.params.contact_release_speed / max(
                    self.params.gamma_obj,
                    1e-9,
                )
                if self._contact_robust_margin(agents[index], object_id) <= release_guard:
                    continue
                local = [
                    obs
                    for obs in fused.get(agents[index].agent_id, [])
                    if obs.object_id == object_id
                ]
                nearest = self._nearest_observation(local, agents[index].position)
                if nearest is None:
                    continue
                force = -np.asarray(nearest.normal, dtype=float)
                alignment = float(np.dot(force, drive))
                if alignment <= 1e-6:
                    continue
                arm = np.asarray(nearest.point, dtype=float) - center
                torque = float(arm[0] * force[1] - arm[1] * force[0]) / scale
                candidate_indices.append(index)
                alignments.append(alignment)
                columns.append(
                    np.array(
                        [force[0], force[1], self.params.wrench_torque_weight * torque],
                        dtype=float,
                    )
                )

            if not columns:
                for index in active:
                    self._wrench_feasible[agents[index].agent_id] = False
                    self._wrench_residuals[agents[index].agent_id] = float("inf")
                continue
            matrix = np.column_stack(columns)
            target = np.array([drive[0], drive[1], 0.0], dtype=float)
            try:
                if self.params.wrench_regularization > 0.0:
                    root_regularization = float(np.sqrt(self.params.wrench_regularization))
                    prior = np.asarray(alignments, dtype=float)
                    augmented = np.vstack(
                        [matrix, root_regularization * np.eye(matrix.shape[1])]
                    )
                    augmented_target = np.concatenate([target, root_regularization * prior])
                    result = lsq_linear(augmented, augmented_target, bounds=(0.0, np.inf))
                    if not result.success:
                        raise RuntimeError(str(result.message))
                    allocation = np.asarray(result.x, dtype=float)
                    residual = float(np.linalg.norm(matrix @ allocation - target))
                else:
                    allocation, residual = nnls(matrix, target)
            except (RuntimeError, ValueError, np.linalg.LinAlgError):
                allocation = np.asarray(alignments, dtype=float)
                residual = float(np.linalg.norm(matrix @ allocation - target))
            positive = allocation[allocation > 1e-9]
            if len(positive):
                allocation = allocation / float(np.mean(positive))
            allocation = np.clip(allocation, 0.0, self.params.wrench_weight_limit)
            feasible = bool(
                np.isfinite(residual) and residual <= self.params.wrench_residual_tolerance
            )
            for index in active:
                agent_id = agents[index].agent_id
                self._wrench_residuals[agent_id] = float(residual)
                self._wrench_feasible[agent_id] = feasible
                if index in candidate_indices:
                    self._wrench_weights[agent_id] = float(allocation[candidate_indices.index(index)])

    def _transport_complete(self, agent_id: str, observations: list[BoundaryObservation]) -> bool:
        if self.params.transport_distance <= 0.0:
            return False
        start_frame = self._transport_start_frame.get(agent_id)
        if start_frame is None or self._frame - start_frame < max(0, int(self.params.transport_min_steps)):
            return False
        if self.params.search_pattern == "paired_lanes":
            sweep, rendezvous, gossip = self._paired_search_durations()
            if self._time < sweep + rendezvous + gossip - 1e-12:
                return False
        if self._transport_complete_latch.get(agent_id, False):
            return True
        complete = self._progress_for(agent_id, observations) >= self.params.transport_distance
        if complete:
            self._transport_complete_latch[agent_id] = True
        return complete

    def transport_progress_summary(self) -> dict[str, dict[str, float]]:
        """Per-object range of local progress estimates for provenance/diagnosis."""
        by_object: dict[str, list[float]] = {}
        for progress in self._transport_progress.values():
            for object_id, value in progress.items():
                by_object.setdefault(object_id, []).append(float(value))
        return {
            object_id: {
                "min": float(np.min(values)),
                "mean": float(np.mean(values)),
                "max": float(np.max(values)),
            }
            for object_id, values in by_object.items()
            if values
        }

    # ------------------------------------------------------------------ #
    # nominal control law
    # ------------------------------------------------------------------ #

    def _nominal_command(
        self,
        i: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        observations: list[BoundaryObservation],
        contact_ready: list[bool],
        transport_active: bool = False,
        transport_phase: str = "cage",
    ) -> tuple[np.ndarray, str, float, bool]:
        agent = agents[i]
        if self.params.task_mode == "coverage":
            u, mode = self._region_coverage_command(i, agents, neighbor_indices, self._time)
            return u, mode, 0.0, False

        if observations:
            self._first_observation_time.setdefault(agent.agent_id, float(self._time))
        forced_search_phase = self._forced_search_phase(self._time)
        if self.params.search_pattern == "paired_lanes" and observations:
            seen_for = float(self._time) - self._first_observation_time[agent.agent_id]
            mapping_start = float(self.params.search_local_gossip_time)
            mapping_end = mapping_start + float(self.params.boundary_mapping_time)
            relay = self._is_search_relay(i, len(agents))
            # One courier on each side completes the sweep and carries the map to
            # rendezvous. Other informed robots remain near the cargo, first
            # gossiping locally and then scanning occluded boundary arcs.
            if (forced_search_phase is None or not relay) and mapping_start <= seen_for < mapping_end:
                return (
                    self._boundary_mapping_velocity(i, agents, observations, self._time),
                    "map_boundary",
                    0.0,
                    False,
                )
            if not relay and seen_for >= mapping_end:
                forced_search_phase = None
        if forced_search_phase is not None:
            return (
                self._exploration_velocity(i, agents, neighbor_indices, self._time),
                forced_search_phase,
                0.0,
                False,
            )

        if not observations:
            return self._exploration_velocity(i, agents, neighbor_indices, self._time), "explore", 0.0, False

        crowd = np.asarray(
            [agent.position] + [agents[j].position for j in neighbor_indices],
            dtype=float,
        )
        goal = self._goal_for(observations) if self.params.task_mode == "transport" else None
        density = BoundaryAwareDensity.from_observations(
            observations, self.density_params, robot_positions=crowd, goal_direction=goal
        )
        cell = self.cvt.compute(i, agents, neighbor_indices, density, self.domain)

        if cell.cell_mass <= self.empty_cell_mass:
            # Non-empty map, empty cell: head for the nearest piece of cage ring
            # that no robot is holding yet. Aiming at the object centroid instead
            # sends the robot radially inward, straight into the robots already on
            # the ring, where the inter-robot barrier stops it -- a robot that has
            # deadlocked while believing it has converged.
            target = self._approach_target(observations, agent.position, crowd)
            u = self.params.kp_cage * (target - agent.position)
            return u, "approach", cell.cell_mass, False

        target = self._redeploy_step(agent, observations, crowd, cell, goal, contact_ready[i])
        if target is not None:
            return self.params.kp_cage * (target - agent.position), "redeploy", cell.cell_mass, False

        u = self.params.kp_cage * (cell.centroid - agent.position)
        push_side = False
        if self.params.task_mode == "transport":
            feedback = self._progress_feedback.get(agent.agent_id) if self.params.progress_feedback else None
            if (
                self.params.progress_feedback
                and transport_phase in {"transport", "brake"}
                and feedback is not None
                and goal is not None
            ):
                # A position loop generates v_ref; the whole enclosure follows
                # that reference while the selected contact arc applies the PI
                # pressure effort. BRAKE sets v_ref=0 and may reverse the contact
                # side according to the measured object velocity.
                u = u + self.params.convoy_feedback_gain * feedback.velocity_reference * goal
                allocation_weight = self._wrench_weights.get(agent.agent_id, 1.0)
                effort = float(feedback.effort) * allocation_weight
                if self.params.safety_pressure_reserve:
                    effort *= self._pressure_reserve_scale(agent)
                object_id = self._progress_object_id(observations)
                drive = (
                    self._transport_drive_direction(
                        agent.agent_id,
                        object_id,
                        longitudinal_sign=1.0 if feedback.effort >= 0.0 else -1.0,
                    )
                    if object_id is not None
                    else (goal if feedback.effort >= 0.0 else -goal)
                )
                bias, push_side = self._transport_bias(
                    i,
                    agents,
                    neighbor_indices,
                    observations,
                    contact_ready,
                    transport_active=True,
                    effort=abs(effort),
                    drive_direction=drive,
                )
                u = u + bias
                if self.params.contact_release_enabled:
                    u = self._release_unallocated_contact(
                        agent,
                        observations,
                        u,
                        drive,
                        contact_ready[i],
                        allocation_weight,
                    )
            elif transport_active and goal is not None and self.params.transport_speed > 0.0:
                # Translate the entire locally informed enclosure.  This is a
                # task-space feed-forward term, not an object-motion shortcut:
                # the cargo still moves only through measured contacts.
                u = u + self.params.transport_speed * goal
                bias, push_side = self._transport_bias(
                    i, agents, neighbor_indices, observations, contact_ready, transport_active
                )
                u = u + bias
            elif not self.params.progress_feedback:
                bias, push_side = self._transport_bias(
                    i, agents, neighbor_indices, observations, contact_ready, transport_active
                )
                u = u + bias
        if transport_phase == "hold":
            mode = "hold"
        elif transport_phase == "brake":
            mode = "brake"
        else:
            mode = "push" if push_side else ("convoy" if transport_active else "cage")
        return u, mode, cell.cell_mass, push_side

    def _pressure_reserve_scale(self, agent: AgentState) -> float:
        """Taper inward PI effort before the full ISSf row becomes active.

        At zero robot velocity, the full object row is feasible whenever
        ``gamma*h >= max(0, n.T v_hat) + rho``.  The cage controller already
        targets ``cage_offset``; this scale only suppresses the additional
        contact pressure as the measured clearance enters that reserve band.
        """
        observations = self._safety_observation_cache.get(agent.agent_id, [])
        nearest = self._nearest_observation(observations, agent.position)
        if nearest is None:
            return 0.0
        velocity = np.asarray(
            self.object_velocity.get(agent.agent_id, {}).get(nearest.object_id, np.zeros(2)),
            dtype=float,
        )
        point = np.asarray(nearest.point, dtype=float) + max(
            0.0,
            self._time - float(nearest.timestamp),
        ) * velocity
        relative = np.asarray(agent.position, dtype=float) - point
        distance = float(np.linalg.norm(relative))
        if distance <= 1e-12:
            return 0.0
        radial = relative / distance
        effective_r_safe = self.params.r_safe + self.params.boundary_error_bound
        h = distance - effective_r_safe
        reserve = (
            max(0.0, float(np.dot(radial, velocity))) + self.params.rho
        ) / max(self.params.gamma_obj, 1e-9)
        nominal_band = max(self.params.cage_offset - effective_r_safe, 1e-9)
        if reserve >= nominal_band:
            return 0.0 if h <= reserve else 1.0
        return float(np.clip((h - reserve) / (nominal_band - reserve), 0.0, 1.0))

    def _redeploy_step(
        self,
        agent: AgentState,
        observations: list[BoundaryObservation],
        crowd: np.ndarray,
        cell,
        goal: np.ndarray | None,
        contact_ready: bool,
    ) -> np.ndarray | None:
        """Leave a saturated cell for unheld boundary elsewhere in this robot's map.

        Move-to-centroid on a limited-range cell has a local equilibrium that the
        coverage law cannot escape: with every robot arriving from the same side,
        the near-side robots converge onto the arc they can see, no robot's disc
        ever overlaps the far side, and there is no gradient pointing around the
        object. Measured on the L shape this pinned strict coverage at 0.63 with
        nothing on the trailing face, so the transport bias never activated at all
        and the small net displacement that did occur pointed backwards.

        The escape uses only local information. When the boundary inside this
        robot's own cell is already held by neighbours -- unheld mass below
        ``redeploy_gap_ratio`` of the cell mass -- and its own map contains a cage
        target nobody is near, it commits to that target. Commitment is sticky
        until it arrives or somebody else takes the target, which is what stops the
        two-robot exchange oscillation.

        This is also the mechanism the recruitment argument needs: standard CVT
        keeps all N robots engaged forever and only redistributes them, so
        "boundary mass is proportional to estimated perimeter, therefore robots are
        recruited in proportion to perimeter" has nothing behind it. A cell-mass
        test that moves robots off saturated boundary is that something.

        A robot already in the contact band never redeploys. Without that gate the
        rule eats itself: once the boundary is fully covered every cell reads as
        held, so the robots that are doing the work walk away to chase whatever
        fragment still looks unheld, and the contact set never stabilises. Measured
        on the L shape, 5 of 12 robots were permanently in redeploy including both
        that were actually touching the cargo.
        """
        if contact_ready:
            self._redeploy_target[agent.agent_id] = None
            return None

        held = self._redeploy_target.get(agent.agent_id)
        if held is not None:
            others = crowd[1:] if len(crowd) > 1 else np.empty((0, 2))
            arrived = float(np.linalg.norm(held - agent.position)) <= self.params.gap_radius
            taken = len(others) > 0 and bool(
                np.min(np.linalg.norm(others - held[None, :], axis=1)) <= self.params.gap_radius
            )
            if arrived or taken:
                self._redeploy_target[agent.agent_id] = None
                held = None

        if held is None and cell.held_fraction >= 1.0 - self.params.redeploy_gap_ratio:
            candidate = self._unheld_target(observations, agent.position, crowd, self.params.local_radius, goal)
            if candidate is not None:
                self._redeploy_target[agent.agent_id] = candidate
                held = candidate
        return held

    def _cage_targets(
        self, observations: list[BoundaryObservation], goal: np.ndarray | None
    ) -> np.ndarray:
        points = np.asarray([obs.point for obs in observations], dtype=float)
        normals = np.asarray([obs.normal for obs in observations], dtype=float)
        offsets = self.density_params.offsets_for(normals, goal)
        return points + offsets[:, None] * normals

    def _unheld_target(
        self,
        observations: list[BoundaryObservation],
        position: np.ndarray,
        crowd: np.ndarray,
        min_distance: float,
        goal: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """Nearest cage target beyond ``min_distance`` that no robot is holding."""
        targets = self._cage_targets(observations, goal)
        occupancy = np.min(np.linalg.norm(targets[:, None, :] - crowd[None, :, :], axis=2), axis=1)
        reach = np.linalg.norm(targets - position[None, :], axis=1)
        free = (occupancy > self.params.gap_radius) & (reach > min_distance)
        if not np.any(free):
            return None
        candidates = targets[free]
        return candidates[int(np.argmin(reach[free]))]

    def _transport_bias(
        self,
        i: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        observations: list[BoundaryObservation],
        contact_ready: list[bool],
        transport_active: bool = True,
        effort: float | None = None,
        drive_direction: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool]:
        agent = agents[i]
        if not transport_active or not contact_ready[i]:
            return np.zeros(2), False
        supporters = 1 + sum(1 for j in neighbor_indices if contact_ready[j])
        if supporters < self.params.min_push_agents:
            return np.zeros(2), False

        goal = self._goal_for(observations) if drive_direction is None else normalize(drive_direction)
        if goal is None:
            return np.zeros(2), False

        nearest = self._nearest_observation(observations, agent.position)
        if nearest is None:
            return np.zeros(2), False
        # The outward normal of the trailing face points against the goal.
        alignment = float(np.dot(nearest.normal, goal))
        if alignment > -self.params.push_side_threshold:
            return np.zeros(2), False

        # Press inward along the locally observed normal, not along the goal
        # direction. A bias along u_goal is inward only at the very centre of the
        # trailing face and tangential everywhere else, so it slides robots around
        # the object and off the arc they were assigned: raising the gain from 0.18
        # to 1.50 shrank the pushing arc from 5 robots to 1 and turned the net goal
        # force from +3.55 N to -3.75 N. Pressing along -n keeps each robot on its
        # own patch of boundary, and scaling by (-n . u_goal) makes the press
        # strongest exactly where it does the most good for the task.
        magnitude = self.params.kp_transport if effort is None else max(0.0, float(effort))
        return magnitude * (-alignment) * (-nearest.normal), True

    def _release_unallocated_contact(
        self,
        agent: AgentState,
        observations: list[BoundaryObservation],
        nominal: np.ndarray,
        drive_direction: np.ndarray,
        contact_ready: bool,
        allocation_weight: float,
    ) -> np.ndarray:
        """Remove passive contacts that oppose the allocated wrench.

        A zero allocation is ineffective if Local-CVT continues pressing the
        robot into the leading face.  This local radial override moves such a
        robot to the configured leading offset while preserving tangential CVT
        motion and leaving the final command to the hard-QP filter.
        """
        if not contact_ready:
            return nominal
        # Contact release is a local geometric decision. The persistent
        # planning map can lag a translating/rotating cargo, so prefer the
        # latest raw local scan whenever it contains an object return.
        safety_observations = self._safety_observation_cache.get(agent.agent_id, [])
        release_observations = safety_observations or observations
        nearest = self._nearest_observation(release_observations, agent.position)
        if nearest is None:
            return nominal
        normal = np.asarray(nearest.normal, dtype=float)
        leading = float(np.dot(normal, normalize(drive_direction))) >= -self.params.push_side_threshold
        unallocated = allocation_weight <= 1e-8
        if not (leading or unallocated):
            return nominal
        desired = float(self.params.lead_offset or (self.params.robot_radius + self.params.delta_max))
        distance = float(np.linalg.norm(agent.position - nearest.point))
        outward = float(
            np.clip(
                self.params.contact_release_gain * max(0.0, desired - distance),
                0.0,
                self.params.contact_release_speed,
            )
        )
        current = float(np.dot(nominal, normal))
        if current >= outward:
            return nominal
        return nominal + (outward - current) * normal

    def _contact_robust_margin(self, agent: AgentState, object_id: str) -> float:
        """Remaining local clearance beyond the full moving-boundary ISSf reserve."""
        observations = [
            obs
            for obs in self._safety_observation_cache.get(agent.agent_id, [])
            if obs.object_id == object_id
        ]
        nearest = self._nearest_observation(observations, agent.position)
        if nearest is None:
            return float("inf")
        velocity = np.asarray(
            self.object_velocity.get(agent.agent_id, {}).get(object_id, np.zeros(2)),
            dtype=float,
        )
        point = np.asarray(nearest.point, dtype=float) + max(
            0.0,
            self._time - float(nearest.timestamp),
        ) * velocity
        relative = np.asarray(agent.position, dtype=float) - point
        distance = float(np.linalg.norm(relative))
        radial = relative / max(distance, 1e-12)
        h = distance - self.params.r_safe - self.params.boundary_error_bound
        reserve = (
            max(0.0, float(np.dot(radial, velocity))) + self.params.rho
        ) / max(self.params.gamma_obj, 1e-9)
        return float(h - reserve)

    def _goal_for(self, observations: list[BoundaryObservation]) -> np.ndarray | None:
        for obs in observations:
            direction = self.goal_directions.get(obs.object_id)
            if direction is not None:
                return direction
        return None

    @staticmethod
    def _nearest_observation(observations: list[BoundaryObservation], position: np.ndarray) -> BoundaryObservation | None:
        if not observations:
            return None
        points = np.asarray([obs.point for obs in observations], dtype=float)
        return observations[int(np.argmin(np.linalg.norm(points - position[None, :], axis=1)))]

    def _approach_target(
        self,
        observations: list[BoundaryObservation],
        position: np.ndarray,
        crowd: np.ndarray,
    ) -> np.ndarray:
        """Nearest unheld cage target in this robot's own map.

        Computed from the robot's own observations and the positions of its
        communication neighbours only, so it is as local as the coverage law it
        stands in for.
        """
        targets = self._cage_targets(observations, None)
        occupied = np.min(np.linalg.norm(targets[:, None, :] - crowd[None, :, :], axis=2), axis=1)
        free = occupied > self.params.gap_radius
        if not np.any(free):
            return self._map_centroid(observations)
        candidates = targets[free]
        return candidates[int(np.argmin(np.linalg.norm(candidates - position[None, :], axis=1)))]

    @staticmethod
    def _map_centroid(observations: list[BoundaryObservation]) -> np.ndarray:
        points = np.asarray([obs.point for obs in observations], dtype=float)
        weights = np.asarray([max(obs.confidence, 1e-6) for obs in observations])
        return np.sum(points * weights[:, None], axis=0) / float(np.sum(weights))

    def _exploration_velocity(
        self, i: int, agents: list[AgentState], neighbor_indices: list[int], timestamp: float
    ) -> np.ndarray:
        agent = agents[i]
        if self.params.search_pattern == "contracting_ring":
            return self._contracting_ring_velocity(agent, timestamp)
        if self.params.search_pattern == "paired_lanes":
            return self._paired_lane_velocity(agent, timestamp)

        repel = np.zeros(2, dtype=float)
        for j in neighbor_indices:
            d = agent.position - agents[j].position
            dist = float(np.linalg.norm(d))
            if dist > 1e-6:
                repel += d / (dist * dist)
        angle = 0.7 * i + 0.25 * timestamp
        sweep = np.array([np.cos(angle), np.sin(angle)], dtype=float)
        return self.params.kp_explore * (0.7 * normalize(repel) + 0.3 * sweep)

    def _contracting_ring_velocity(self, agent: AgentState, timestamp: float) -> np.ndarray:
        """Sweep a bounded region while keeping the initial ring connected.

        The target depends only on the domain, time, and the robot's initial
        polar slot.  It does not use cargo position, outline, or simulator state.
        A radial feed-forward term makes discovery time predictable; a small
        angular term prevents the same boundary rays from being revisited.
        """
        if self.params.search_center is None:
            xmin, xmax, ymin, ymax = self.domain
            center = np.array([(xmin + xmax) / 2.0, (ymin + ymax) / 2.0], dtype=float)
        else:
            center = np.asarray(self.params.search_center, dtype=float).reshape(2)

        if agent.agent_id not in self._search_initial_polar:
            offset = agent.position - center
            self._search_initial_polar[agent.agent_id] = (
                float(np.linalg.norm(offset)),
                float(np.arctan2(offset[1], offset[0])),
            )
        initial_radius, initial_angle = self._search_initial_polar[agent.agent_id]
        radius = max(
            float(self.params.search_inner_radius),
            initial_radius - float(self.params.search_inward_speed) * float(timestamp),
        )
        angle = initial_angle + float(self.params.search_angular_speed) * float(timestamp)
        radial = np.array([np.cos(angle), np.sin(angle)], dtype=float)
        tangent = np.array([-radial[1], radial[0]], dtype=float)
        target = center + radius * radial
        feed_forward = (
            -float(self.params.search_inward_speed) * radial
            + radius * float(self.params.search_angular_speed) * tangent
        )
        return feed_forward + self.params.kp_explore * (target - agent.position)

    def _map_gossip_due(self, refresh_perception: bool) -> bool:
        """Schedule planning-map flooding without decimating hard safety.

        The paired-lane discovery proof needs one relay opportunity per
        perception update during its explicit rendezvous/gossip interval.  The
        independent sweep before it and boundary mapping after it may exchange
        the much larger snapshots at a lower rate.  Local perception, raw CBF
        rows, and the QP are unaffected.
        """
        if not refresh_perception or not self.params.map_gossip:
            return False
        if self.params.search_pattern == "paired_lanes":
            sweep, rendezvous, gossip = self._paired_search_durations()
            relay_start = sweep + rendezvous
            relay_end = relay_start + gossip
            if relay_start - 1e-12 <= self._time <= relay_end + 1e-12:
                return True
        perception_index = self._frame // int(self.params.perception_every)
        return perception_index % int(self.params.map_gossip_every) == 0

    def _paired_search_durations(self) -> tuple[float, float, float]:
        """Return sweep, rendezvous, and gossip durations in seconds."""
        xmin, xmax, _, _ = self.domain
        half_width = 0.5 * (xmax - xmin)
        initial = next(iter(self._search_initial_pose.values()), None)
        if initial is None:
            # Before the first control call the configured layout convention is
            # the only information available.  The robot radius is a conservative
            # edge padding and matches the default paired_sweep layout.
            edge_padding = self.params.robot_radius
        else:
            edge_padding = min(float(initial[0] - xmin), float(xmax - initial[0]))
        sweep_distance = max(0.0, half_width - edge_padding - self.params.search_detection_radius)
        rendezvous_distance = max(
            0.0, self.params.search_detection_radius - 0.5 * self.params.search_meeting_gap
        )
        speed = float(self.params.search_speed)
        return sweep_distance / speed, rendezvous_distance / speed, float(self.params.search_gossip_time)

    def _forced_search_phase(self, timestamp: float) -> str | None:
        if self.params.search_pattern != "paired_lanes":
            return None
        sweep, rendezvous, gossip = self._paired_search_durations()
        t = float(timestamp)
        if t < sweep - 1e-12:
            return "search_sweep"
        if t < sweep + rendezvous - 1e-12:
            return "search_rendezvous"
        release = sweep + rendezvous + gossip
        if t < release - 1e-12:
            return "search_gossip"
        return None

    def _transport_drive_direction(
        self,
        agent_id: str,
        object_id: str,
        longitudinal_sign: float = 1.0,
    ) -> np.ndarray:
        """Goal direction with feedback that rejects estimated cross-track drift.

        The controller uses only the distributed motion estimate accumulated
        from boundary-map registration.  Cargo truth is never consulted.  The
        lateral correction keeps the commanded wrench pointed back toward the
        task line while ``longitudinal_sign`` independently supports forward
        transport and reverse braking.
        """
        goal = np.asarray(self.goal_directions[object_id], dtype=float)
        displacement = np.asarray(
            self._transport_displacement.get(agent_id, {}).get(object_id, np.zeros(2)),
            dtype=float,
        )
        cross_track = displacement - float(np.dot(displacement, goal)) * goal
        command = float(longitudinal_sign) * goal - self.params.cross_track_gain * cross_track
        if float(np.linalg.norm(command)) <= 1e-12:
            return normalize(float(longitudinal_sign) * goal)
        return normalize(command)

    @staticmethod
    def _is_search_relay(index: int, count: int) -> bool:
        if count < 2 or count % 2:
            return False
        per_side = count // 2
        return index in {per_side // 2, per_side + per_side // 2}

    def paired_search_bound(self) -> dict[str, float]:
        """Controller-side timing record used by the independent certificate."""
        sweep, rendezvous, gossip = self._paired_search_durations()
        return {
            "sweep_seconds": sweep,
            "rendezvous_seconds": rendezvous,
            "gossip_seconds": gossip,
            "release_seconds": sweep + rendezvous + gossip,
            "mapping_seconds": float(self.params.boundary_mapping_time),
        }

    def transport_feedback_summary(self) -> dict[str, dict[str, object]]:
        return {
            agent_id: {
                "progress": output.progress,
                "parallel_velocity": output.parallel_velocity,
                "position_error": output.position_error,
                "velocity_reference": output.velocity_reference,
                "velocity_error": output.velocity_error,
                "pressure_effort": output.effort,
                "integral": output.integral,
                "saturated": output.saturated,
                "braking": output.braking,
                "phase": self._transport_phase.get(agent_id, "cage"),
                "wrench_weight": self._wrench_weights.get(agent_id, 1.0),
                "wrench_residual": self._wrench_residuals.get(agent_id, 0.0),
                "wrench_feasible": self._wrench_feasible.get(agent_id, True),
            }
            for agent_id, output in self._progress_feedback.items()
        }

    def _paired_lane_velocity(self, agent: AgentState, timestamp: float) -> np.ndarray:
        """Execute the predetermined lane path without cargo-pose information."""
        xmin, xmax, _, _ = self.domain
        center_x = 0.5 * (xmin + xmax)
        initial = self._search_initial_pose.setdefault(agent.agent_id, agent.position.copy())
        side = -1.0 if initial[0] < center_x else 1.0
        toward_center = -side
        sweep, rendezvous, _ = self._paired_search_durations()
        speed = float(self.params.search_speed)

        if timestamp < sweep:
            distance = min(speed * float(timestamp), speed * sweep)
            target = initial + np.array([toward_center * distance, 0.0])
            feed_forward = np.array([toward_center * speed, 0.0])
        elif timestamp < sweep + rendezvous:
            sweep_end = np.array([center_x + side * self.params.search_detection_radius, initial[1]])
            distance = min(speed * (float(timestamp) - sweep), speed * rendezvous)
            target = sweep_end + np.array([toward_center * distance, 0.0])
            feed_forward = np.array([toward_center * speed, 0.0])
        else:
            target = np.array([center_x + side * 0.5 * self.params.search_meeting_gap, initial[1]])
            feed_forward = np.zeros(2)
        return feed_forward + self.params.kp_explore * (target - agent.position)

    def _boundary_mapping_velocity(
        self,
        i: int,
        agents: list[AgentState],
        observations: list[BoundaryObservation],
        timestamp: float,
    ) -> np.ndarray:
        """Distribute observers around the discovered outline before caging.

        The centre is computed from the locally gossiped boundary map.  No cargo
        pose or simulator vertex is used.  Equally spaced deterministic slots
        expose occluded sides to different robots; a slow common rotation scans
        around vertices and narrow concavities instead of freezing one set of
        sight lines.
        """
        center = self._map_centroid(observations)
        first_seen = self._first_observation_time.get(agents[i].agent_id, float(timestamp))
        elapsed = max(0.0, float(timestamp) - first_seen - float(self.params.search_local_gossip_time))
        # Golden-angle slots make either half-team alone cover the full circle;
        # contiguous agent ids would otherwise occupy only one semicircle.
        angle = 2.0 * np.pi * ((i * 0.6180339887498949) % 1.0) + float(
            self.params.boundary_mapping_angular_speed
        ) * elapsed
        radial = np.array([np.cos(angle), np.sin(angle)], dtype=float)
        tangent = np.array([-radial[1], radial[0]], dtype=float)
        radius = float(self.params.boundary_mapping_radius)
        target = center + radius * radial
        feed_forward = radius * float(self.params.boundary_mapping_angular_speed) * tangent
        return feed_forward + self.params.kp_explore * (target - agents[i].position)

    # ------------------------------------------------------------------ #
    # map-derived quantities
    # ------------------------------------------------------------------ #

    def _object_rows_from_map(
        self,
        agent_id: str,
        position: np.ndarray,
        observations: list[BoundaryObservation],
        timestamp: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not observations:
            return np.empty((0, 2)), np.empty((0, 2)), np.zeros(2)
        velocities = self.object_velocity.get(agent_id, {})
        angular_velocities = self.object_angular_velocity.get(agent_id, {})
        nearest = self._nearest_observation(observations, position)
        if nearest is None:
            return np.empty((0, 2)), np.empty((0, 2)), np.zeros(2)
        # One QP call carries one object velocity.  Restrict the rows to the
        # nearest observed object rather than silently applying its velocity to
        # every cargo in a multi-object scene.
        selected = [obs for obs in observations if obs.object_id == nearest.object_id]
        v_obj = np.asarray(velocities.get(nearest.object_id, np.zeros(2)), dtype=float)
        omega = float(angular_velocities.get(nearest.object_id, 0.0))
        centroid = np.asarray(
            self._object_centroid.get(agent_id, {}).get(
                nearest.object_id,
                np.mean(np.asarray([obs.point for obs in selected], dtype=float), axis=0),
            ),
            dtype=float,
        )
        now = self._time if timestamp is None else float(timestamp)
        points = []
        rotated_relatives = []
        for obs in selected:
            age = max(0.0, now - float(obs.timestamp))
            relative = np.asarray(obs.point, dtype=float) - centroid
            angle = omega * age
            c, s = np.cos(angle), np.sin(angle)
            rotated = np.array(
                [c * relative[0] - s * relative[1], s * relative[0] + c * relative[1]],
                dtype=float,
            )
            points.append(centroid + age * v_obj + rotated)
            rotated_relatives.append(rotated)
        points = np.asarray(points, dtype=float)
        normals = np.asarray([obs.normal for obs in selected], dtype=float)
        relative_points = np.asarray(rotated_relatives, dtype=float)
        rotational_velocity = omega * np.column_stack(
            [-relative_points[:, 1], relative_points[:, 0]]
        )
        point_velocities = v_obj[None, :] + rotational_velocity
        return points, normals, point_velocities

    def _update_object_velocity(
        self, agent_id: str, observations: list[BoundaryObservation], dt: float
    ) -> None:
        """Estimate object velocity from point-to-plane map registration.

        Differencing the centroid of a *visible arc* does not estimate rigid-body
        motion: the centroid can jump when a corner enters or leaves the scan even
        while the object is static.  In the 500-frame scenario that produced a
        fictitious velocity ``[0.182, 0.372] m/s`` and made otherwise safe object
        rows mutually infeasible.  ``LocalBoundaryMap.last_motion`` is instead the
        point-to-plane rigid translation estimated between consecutive scans.
        Its remaining error is what the ISSf margin ``rho`` covers.
        """
        object_ids = {obs.object_id for obs in observations}
        filtered = self.object_velocity.setdefault(agent_id, {})
        filtered_angular = self.object_angular_velocity.setdefault(agent_id, {})
        centroids = self._object_centroid.setdefault(agent_id, {})
        alpha = float(np.clip(self.params.object_velocity_filter, 0.0, 1.0))
        motions = self.maps[agent_id].last_motion
        rotations = self.maps[agent_id].last_rotation
        for object_id in object_ids:
            raw = motions.get(object_id, np.zeros(2)) / max(float(dt), 1e-9)
            prior = filtered.get(object_id, np.zeros(2))
            filtered[object_id] = (1.0 - alpha) * prior + alpha * raw
            raw_angular = float(rotations.get(object_id, 0.0)) / max(float(dt), 1e-9)
            prior_angular = float(filtered_angular.get(object_id, 0.0))
            filtered_angular[object_id] = (1.0 - alpha) * prior_angular + alpha * raw_angular
            points = np.asarray(
                [obs.point for obs in observations if obs.object_id == object_id],
                dtype=float,
            )
            if len(points):
                centroids[object_id] = np.mean(points, axis=0)
        for object_id in list(filtered):
            if object_id not in object_ids:
                del filtered[object_id]
                filtered_angular.pop(object_id, None)
                centroids.pop(object_id, None)

    def _contact_ready(self, agent: AgentState, observations: list[BoundaryObservation]) -> bool:
        """True when the robot's *own* map says it sits in the contact band."""
        nearest = self._nearest_observation(observations, agent.position)
        if nearest is None:
            return False
        distance = float(np.linalg.norm(nearest.point - agent.position))
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
        density = BoundaryAwareDensity.from_targets(visible, sigma=self.params.sigma, base_density=self.params.base_density)
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

    def _ensure_maps(self, agents: list[AgentState]) -> None:
        for agent in agents:
            self.maps.setdefault(
                agent.agent_id,
                LocalBoundaryMap(
                    voxel_size=self.params.voxel_size,
                    age_decay=self.params.age_decay,
                    max_voxels_per_object=self.params.max_voxels_per_object,
                    max_translation_per_update=self.params.map_max_translation_per_update,
                    max_rotation_per_update=self.params.map_max_rotation_per_update,
                ),
            )

    def _neighbor_indices(self, agents: list[AgentState]) -> list[list[int]]:
        if not agents:
            return []
        positions = np.asarray([a.position for a in agents], dtype=float)
        d = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
        np.fill_diagonal(d, np.inf)
        return [list(np.where(row <= self.params.comm_range)[0]) for row in d]

    def _communication_neighbor_indices(
        self,
        agents: list[AgentState],
        physical_neighbors: list[list[int]],
    ) -> list[list[int]]:
        """Apply deterministic symmetric packet dropout to communication links."""
        if self.params.communication_dropout_prob <= 0.0:
            candidates = sum(len(row) for row in physical_neighbors) // 2
            self.communication_candidate_links += candidates
            self.communication_delivered_links += candidates
            return [list(row) for row in physical_neighbors]
        delivered = [[] for _ in agents]
        for i, row in enumerate(physical_neighbors):
            for j in row:
                if j <= i:
                    continue
                self.communication_candidate_links += 1
                rng = frame_rng(
                    "communication_link",
                    agents[i].agent_id,
                    agents[j].agent_id,
                    self._frame,
                    base=self.seed,
                )
                if float(rng.random()) < self.params.communication_dropout_prob:
                    continue
                delivered[i].append(j)
                delivered[j].append(i)
                self.communication_delivered_links += 1
        return delivered

    @property
    def communication_delivery_rate(self) -> float:
        if self.communication_candidate_links <= 0:
            return 1.0
        return self.communication_delivered_links / self.communication_candidate_links

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


__all__ = ["DBACTController", "DBACTParams", "AgentDiagnostics"]
