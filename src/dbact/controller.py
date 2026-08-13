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

The object-boundary rows given to the safety filter come from the robot's own
map, never from the simulator. That is what makes the normal-estimate error a
quantity with consequences rather than a number in a table.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .boundary_density import BoundaryAwareDensity, DensityParams
from .boundary_map import LocalBoundaryMap
from .cargo import Cargo
from .contracts import ContactSafetyContract, CoverageContract
from .geometry import clip_to_domain, normalize
from .local_cvt import LocalCVT, empty_cell_threshold
from .perception import PerceptionParams, RayCastBoundarySensor
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

    # Perception and local CVT are the two expensive loops.  They may run at a
    # lower rate than the safety filter, which remains active on every physics
    # step.  A value of one preserves the original behaviour.
    perception_every: int = 1
    planning_every: int = 1

    # --- communication ---
    comm_range: float = 1.6

    # --- map (S3) ---
    voxel_size: float = 0.06
    age_decay: float = 0.30
    max_voxels_per_object: int = 600

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
    object_row_inner_limit: float | None = None
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

        if params.perception_every < 1 or params.planning_every < 1:
            raise ValueError("perception_every and planning_every must both be positive integers")
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
                object_row_inner_limit=(
                    params.robot_radius if params.object_row_inner_limit is None else params.object_row_inner_limit
                ),
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
        self._object_centroid: dict[str, dict[str, np.ndarray]] = {}
        self._transport_ready_streak: dict[str, int] = {}
        self._transport_progress: dict[str, dict[str, float]] = {}
        self._transport_origin_centroid: dict[str, dict[str, np.ndarray]] = {}
        self._transport_complete_latch: dict[str, bool] = {}
        self._transport_start_frame: dict[str, int] = {}
        self._search_initial_polar: dict[str, tuple[float, float]] = {}
        self._search_initial_pose: dict[str, np.ndarray] = {}
        self._first_observation_time: dict[str, float] = {}
        self._nominal_cache: dict[str, tuple[np.ndarray, str, float, bool]] = {}
        self._fused_cache: dict[str, list[BoundaryObservation]] = {}
        self.target_region_points = self._build_target_region_points()
        self.diagnostics: list[AgentDiagnostics] = []
        self.last_detection_counts: dict[str, int] = {}
        self._time = 0.0
        self._frame = 0

    # ------------------------------------------------------------------ #
    # main loop
    # ------------------------------------------------------------------ #

    def step(self, agents: list[AgentState], cargoes: list[Cargo], timestamp: float, dt: float) -> list[ControlCommand]:
        self._time = float(timestamp)
        self._ensure_maps(agents)
        neighbors = self._neighbor_indices(agents)

        refresh_perception = self._frame % int(self.params.perception_every) == 0
        sensed: dict[str, list[BoundaryObservation]] = {agent.agent_id: [] for agent in agents}
        if refresh_perception:
            for agent in agents:
                sensed[agent.agent_id] = self.sensor.sense(agent, cargoes, timestamp)
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
        for i, agent in enumerate(agents):
            if refresh_perception:
                batch = list(sensed[agent.agent_id])
                for j in neighbors[i]:
                    batch.extend(sensed[agents[j].agent_id])
                    if self.params.map_gossip:
                        batch.extend(prior_fused.get(agents[j].agent_id, []))
                self.maps[agent.agent_id].update(batch, timestamp)
                self._fused_cache[agent.agent_id] = self.maps[agent.agent_id].all_observations(timestamp)
            fused[agent.agent_id] = self._fused_cache.get(agent.agent_id, [])
            if refresh_perception:
                self._update_object_velocity(
                    agent.agent_id,
                    fused[agent.agent_id],
                    dt * int(self.params.perception_every),
                )

        contact_ready = [self._contact_ready(agents[i], fused[agents[i].agent_id]) for i in range(len(agents))]
        transport_active: list[bool] = []
        transport_complete: list[bool] = []
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
            complete = self._transport_complete(agent.agent_id, fused[agent.agent_id])
            transport_complete.append(complete)
            transport_active.append(ready and not complete)

        if refresh_perception:
            for i, agent in enumerate(agents):
                if transport_active[i]:
                    self._accumulate_transport_progress(agent.agent_id)

        self.diagnostics = []
        commands: list[ControlCommand] = []
        for i, agent in enumerate(agents):
            observations = fused[agent.agent_id]
            refresh_plan = (
                self._frame % int(self.params.planning_every) == 0
                or agent.agent_id not in self._nominal_cache
            )
            if refresh_plan:
                self._nominal_cache[agent.agent_id] = self._nominal_command(
                    i, agents, neighbors[i], observations, contact_ready, transport_active[i]
                )
            cached = self._nominal_cache[agent.agent_id]
            u_nom, mode, cell_mass, push_side = cached[0].copy(), cached[1], cached[2], cached[3]
            if transport_complete[i] and not push_side:
                mode = "hold"
            points, normals, v_obj = self._object_rows_from_map(agent.agent_id, agent.position, observations)
            result = self.safety.filter_velocity(
                agent.position,
                u_nom,
                [agents[j].position for j in neighbors[i]],
                boundary_points=points,
                boundary_normals=normals,
                object_velocity=v_obj,
            )
            commands.append(ControlCommand(agent.agent_id, result.velocity, mode=mode))
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
                )
            )
        self._frame += 1
        return commands

    def _accumulate_transport_progress(self, agent_id: str) -> None:
        """Measure displacement of the locally gossiped boundary-map centroid.

        Incrementally summing scan registrations is fragile under rotation: a
        small per-scan bias accumulates for hundreds of frames and can prevent
        HOLD even after the cargo has visibly passed the target.  At transport
        activation the boundary map is already required to be epsilon-dense, so
        its voxel centroid is a bounded local proxy for body translation.  The
        baseline/difference form does not integrate drift and still reads no
        simulator pose.
        """
        if self.params.search_pattern == "paired_lanes":
            sweep, rendezvous, gossip = self._paired_search_durations()
            if self._time < sweep + rendezvous + gossip - 1e-12:
                return
        progress = self._transport_progress.setdefault(agent_id, {})
        origins = self._transport_origin_centroid.setdefault(agent_id, {})
        observations = self.maps[agent_id].all_observations(self._time)
        object_ids = {obs.object_id for obs in observations}
        for object_id in object_ids:
            goal = self.goal_directions.get(object_id)
            if goal is None:
                continue
            points = np.vstack([obs.point for obs in observations if obs.object_id == object_id])
            centroid = np.mean(points, axis=0)
            origin = origins.setdefault(object_id, centroid.copy())
            progress[object_id] = float(np.dot(centroid - origin, goal))

    def _progress_for(self, agent_id: str, observations: list[BoundaryObservation]) -> float:
        progress = self._transport_progress.get(agent_id, {})
        for obs in observations:
            if obs.object_id in progress:
                return float(progress[obs.object_id])
        return 0.0

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

        crowd = np.vstack([agent.position] + [agents[j].position for j in neighbor_indices])
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
            if transport_active and goal is not None and self.params.transport_speed > 0.0:
                # Translate the entire locally informed enclosure.  This is a
                # task-space feed-forward term, not an object-motion shortcut:
                # the cargo still moves only through measured contacts.
                u = u + self.params.transport_speed * goal
            bias, push_side = self._transport_bias(
                i, agents, neighbor_indices, observations, contact_ready, transport_active
            )
            u = u + bias
        mode = "push" if push_side else ("convoy" if transport_active else "cage")
        return u, mode, cell.cell_mass, push_side

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
        points = np.vstack([obs.point for obs in observations])
        normals = np.vstack([obs.normal for obs in observations])
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
    ) -> tuple[np.ndarray, bool]:
        agent = agents[i]
        if not transport_active or not contact_ready[i]:
            return np.zeros(2), False
        supporters = 1 + sum(1 for j in neighbor_indices if contact_ready[j])
        if supporters < self.params.min_push_agents:
            return np.zeros(2), False

        goal = self._goal_for(observations)
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
        return self.params.kp_transport * (-alignment) * (-nearest.normal), True

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
        points = np.vstack([obs.point for obs in observations])
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
        points = np.vstack([obs.point for obs in observations])
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
        self, agent_id: str, position: np.ndarray, observations: list[BoundaryObservation]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not observations:
            return np.empty((0, 2)), np.empty((0, 2)), np.zeros(2)
        points = np.vstack([obs.point for obs in observations])
        normals = np.vstack([obs.normal for obs in observations])
        velocities = self.object_velocity.get(agent_id, {})
        v_obj = np.zeros(2)
        if velocities:
            nearest = self._nearest_observation(observations, position)
            if nearest is not None:
                v_obj = velocities.get(nearest.object_id, np.zeros(2))
        return points, normals, v_obj

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
        alpha = float(np.clip(self.params.object_velocity_filter, 0.0, 1.0))
        motions = self.maps[agent_id].last_motion
        for object_id in object_ids:
            raw = motions.get(object_id, np.zeros(2)) / max(float(dt), 1e-9)
            prior = filtered.get(object_id, np.zeros(2))
            filtered[object_id] = (1.0 - alpha) * prior + alpha * raw
        for object_id in list(filtered):
            if object_id not in object_ids:
                del filtered[object_id]

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
                ),
            )

    def _neighbor_indices(self, agents: list[AgentState]) -> list[list[int]]:
        if not agents:
            return []
        positions = np.vstack([a.position for a in agents])
        d = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
        np.fill_diagonal(d, np.inf)
        return [list(np.where(row <= self.params.comm_range)[0]) for row in d]

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
