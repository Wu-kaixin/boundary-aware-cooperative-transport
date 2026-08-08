from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np

from .boundary_density import BoundaryAwareDensity
from .boundary_map import LocalBoundaryMap
from .cargo import Cargo
from .distributed_cbf import DistributedCBFQP
from .geometry import clip_to_domain, normalize
from .local_cvt import LocalCVT
from .local_sensing import LocalBoundarySensor
from .types import AgentState, BoundaryObservation, ControlCommand


@dataclass
class DBACTParams:
    task_mode: str = "caging"
    # Paper methods: dbact (B3), arm (B0), oracle (B1), no_cbf (B2)
    method: str = "dbact"
    sensor_range: float = 1.1
    comm_range: float = 1.6
    cage_offset: float = 0.32
    sigma: float = 0.35
    d_min: float = 0.28
    max_speed: float = 0.35
    kp_explore: float = 0.25
    kp_cage: float = 0.9
    kp_transport: float = 0.18
    grid_spacing: float = 0.08
    map_ttl: float = 4.0
    map_voxel_size: float = 0.08
    map_decay_lambda: float = 0.35
    density_gap_gain: float = 1.0
    gap_cover_radius: float = 0.45
    cbf_gamma: float = 6.0
    cbf_use_qp: bool = True
    robot_radius: float = 0.12
    cbf_alpha_object: float = 4.0
    boundary_position_error: float = 0.0
    object_speed_bound: float = 0.0
    cbf_contact_allowance: float = 0.0
    object_cbf_points: int = 3
    enable_transport_bias: bool = False
    transport_activation_time: float = 0.0
    task_velocity: list[float] = field(default_factory=lambda: [0.0, 0.0])
    transport_contact_margin: float = 0.18
    packet_dropout: float = 0.0
    sensor_noise_std: float = 0.0
    normal_error_deg: float = 0.0
    random_seed: int = 1
    gap_weighting: bool = True
    map_sharing: bool = True
    target_center: list[float] = field(default_factory=lambda: [4.0, 4.0])
    target_radius: float = 1.0
    target_sensor_range: float = 2.0
    target_samples: int = 36

    @classmethod
    def from_dict(cls, data: dict) -> "DBACTParams":
        fields = {k: v for k, v in data.items() if k in cls.__dataclass_fields__}
        return cls(**fields)


@dataclass
class ControllerStats:
    cbf_calls: int = 0
    cbf_interventions: int = 0
    solve_time_s: float = 0.0
    infeasible_calls: int = 0

    @property
    def intervention_rate(self) -> float:
        if self.cbf_calls <= 0:
            return 0.0
        return float(self.cbf_interventions) / float(self.cbf_calls)

    @property
    def mean_solve_time_s(self) -> float:
        if self.cbf_calls <= 0:
            return 0.0
        return float(self.solve_time_s) / float(self.cbf_calls)


class DBACTController:
    """Decentralized boundary-aware cooperative transportation controller.

    Paper pipeline:
        local boundary measurements → local map → boundary-measure density
        → limited local CVT → nominal velocity → distributed CBF-QP

    Method switch (paper baselines):
        - dbact / B3: full proposed stack
        - arm / B0: agent-centered Gaussian density (ARM-style)
        - oracle / B1: global geometry-aware density from true cargo boundary
        - no_cbf / B2: boundary density + CVT without CBF filtering
    """

    def __init__(self, params: DBACTParams, domain: tuple[float, float, float, float]):
        self.params = params
        self.domain = domain
        self.sensor = LocalBoundarySensor(
            sensor_range=params.sensor_range,
            noise_std=params.sensor_noise_std,
            random_seed=params.random_seed,
        )
        self.cvt = LocalCVT(
            grid_spacing=params.grid_spacing,
            local_radius=params.comm_range,
        )
        estimation_margin = float(params.boundary_position_error) + 2.0 * float(
            params.cage_offset
        ) * np.sin(0.5 * np.deg2rad(float(params.normal_error_deg)))
        self.cbf = DistributedCBFQP(
            d_min=params.d_min,
            gamma=params.cbf_gamma,
            max_speed=params.max_speed,
            use_qp=params.cbf_use_qp,
            robot_radius=params.robot_radius,
            alpha_object=params.cbf_alpha_object,
            boundary_error_margin=estimation_margin,
            object_speed_bound=params.object_speed_bound,
            contact_allowance=params.cbf_contact_allowance,
        )
        self.maps: dict[str, LocalBoundaryMap] = {}
        self.target_region_points = self._build_target_region_points()
        self.stats = ControllerStats()
        self._rng = np.random.default_rng(int(params.random_seed))

    def reset_stats(self) -> None:
        self.stats = ControllerStats()

    def step(self, agents: list[AgentState], cargoes: list[Cargo], timestamp: float, dt: float) -> list[ControlCommand]:
        del dt
        self._ensure_maps(agents)
        method = str(self.params.method).lower()
        sensed_by_agent: dict[str, list[BoundaryObservation]] = {}
        detecting_agents: set[str] = set()
        for agent in agents:
            observations = self.sensor.sense(agent, cargoes, timestamp)
            observations = self._perturb_normals(observations)
            sensed_by_agent[agent.agent_id] = observations
            if observations:
                detecting_agents.add(agent.agent_id)
            self.maps[agent.agent_id].update(observations, timestamp)

        if self.params.map_sharing:
            for i, agent in enumerate(agents):
                for j, other in enumerate(agents):
                    if i == j:
                        continue
                    if np.linalg.norm(agent.position - other.position) > self.params.comm_range:
                        continue
                    payload = sensed_by_agent.get(other.agent_id, [])
                    if self.params.packet_dropout > 0.0 and payload:
                        keep = [
                            obs
                            for obs in payload
                            if self._rng.random() > self.params.packet_dropout
                        ]
                        payload = keep
                    self.maps[agent.agent_id].update(payload, timestamp)

        commands: list[ControlCommand] = []
        for i, agent in enumerate(agents):
            neighbor_indices = [
                j
                for j, other in enumerate(agents)
                if j != i and np.linalg.norm(agent.position - other.position) <= self.params.comm_range
            ]
            if self.params.task_mode == "coverage":
                u_nom, mode = self._coverage_command(i, agents, neighbor_indices)
                boundary_points: list[np.ndarray] = []
                boundary_normals: list[np.ndarray] = []
            else:
                observations = self.maps[agent.agent_id].all_observations(timestamp)
                if self.params.gap_weighting:
                    observations = self._annotate_gap_scores(observations, agents)
                else:
                    for obs in observations:
                        obs.gap_score = 0.0

                if method in {"arm", "b0"}:
                    u_nom, mode = self._arm_command(i, agents, neighbor_indices, detecting_agents, timestamp)
                elif method in {"oracle", "b1"}:
                    u_nom, mode = self._oracle_command(i, agents, neighbor_indices, cargoes, timestamp)
                elif observations:
                    age_weights = [
                        self.maps[agent.agent_id].age_weight(obs, timestamp) for obs in observations
                    ]
                    density = BoundaryAwareDensity.from_observations(
                        observations,
                        cage_offset=self.params.cage_offset,
                        sigma=self.params.sigma,
                        timestamp=timestamp,
                        decay_lambda=self.params.map_decay_lambda,
                        gap_gain=self.params.density_gap_gain if self.params.gap_weighting else 0.0,
                        age_weights=age_weights,
                    )
                    centroid = self.cvt.compute_centroid(i, agents, neighbor_indices, density, self.domain)
                    u_nom = self.params.kp_cage * (centroid - agent.position)
                    u_nom = u_nom + self._observation_transport_bias(
                        agent, observations, timestamp
                    )
                    mode = "dbact_cage"
                else:
                    u_nom = self._exploration_velocity(i, agents, neighbor_indices, timestamp)
                    mode = "dbact_explore"
                boundary_points, boundary_normals = self._nearest_boundary_constraints(agent, observations)

            neighbor_positions = [agents[j].position for j in neighbor_indices]
            if method in {"no_cbf", "b2"}:
                u_safe = self._cap_speed(np.asarray(u_nom, dtype=float).reshape(2))
                mode = f"{mode}_nocbf"
            else:
                u_safe = self._filter_with_stats(
                    agent.position,
                    u_nom,
                    neighbor_positions,
                    boundary_points,
                    boundary_normals,
                )
            commands.append(ControlCommand(agent.agent_id, u_safe, mode=mode))
        return commands

    def _filter_with_stats(
        self,
        position: np.ndarray,
        u_nom: np.ndarray,
        neighbor_positions: list[np.ndarray],
        boundary_points: list[np.ndarray],
        boundary_normals: list[np.ndarray],
    ) -> np.ndarray:
        t0 = time.perf_counter()
        u_safe = self.cbf.filter_velocity(
            position,
            u_nom,
            neighbor_positions,
            boundary_points=boundary_points,
            boundary_normals=boundary_normals,
        )
        elapsed = time.perf_counter() - t0
        self.stats.cbf_calls += 1
        self.stats.solve_time_s += elapsed
        if float(np.linalg.norm(np.asarray(u_safe) - np.asarray(u_nom))) > 1e-4:
            self.stats.cbf_interventions += 1
        if not bool(getattr(self.cbf, "last_feasible", True)):
            self.stats.infeasible_calls += 1
        return u_safe

    def _cap_speed(self, velocity: np.ndarray) -> np.ndarray:
        speed = float(np.linalg.norm(velocity))
        if speed <= self.params.max_speed:
            return velocity
        return velocity / speed * self.params.max_speed

    def _ensure_maps(self, agents: list[AgentState]) -> None:
        for agent in agents:
            self.maps.setdefault(
                agent.agent_id,
                LocalBoundaryMap(
                    ttl=self.params.map_ttl,
                    voxel_size=self.params.map_voxel_size,
                    decay_lambda=self.params.map_decay_lambda,
                ),
            )

    def _perturb_normals(self, observations: list[BoundaryObservation]) -> list[BoundaryObservation]:
        err_deg = float(self.params.normal_error_deg)
        if err_deg <= 0.0 or not observations:
            return observations
        out: list[BoundaryObservation] = []
        for obs in observations:
            angle = np.deg2rad(self._rng.uniform(-err_deg, err_deg))
            c, s = np.cos(angle), np.sin(angle)
            rot = np.array([[c, -s], [s, c]], dtype=float)
            normal = rot @ obs.normal
            out.append(
                BoundaryObservation(
                    object_id=obs.object_id,
                    agent_id=obs.agent_id,
                    point=obs.point,
                    normal=normal,
                    timestamp=obs.timestamp,
                    confidence=obs.confidence,
                    arc_length=obs.arc_length,
                    gap_score=obs.gap_score,
                )
            )
        return out

    def _annotate_gap_scores(
        self,
        observations: list[BoundaryObservation],
        agents: list[AgentState],
    ) -> list[BoundaryObservation]:
        if not observations:
            return observations
        positions = np.vstack([a.position for a in agents])
        radius = self.params.gap_cover_radius
        annotated: list[BoundaryObservation] = []
        for obs in observations:
            target = obs.point + self.params.cage_offset * obs.normal
            dists = np.linalg.norm(positions - target[None, :], axis=1)
            covered = float(np.min(dists)) <= radius if len(dists) else False
            gap = 0.0 if covered else 1.0
            annotated.append(
                BoundaryObservation(
                    object_id=obs.object_id,
                    agent_id=obs.agent_id,
                    point=obs.point,
                    normal=obs.normal,
                    timestamp=obs.timestamp,
                    confidence=obs.confidence,
                    arc_length=obs.arc_length,
                    gap_score=gap,
                )
            )
        return annotated

    def _nearest_boundary_constraints(
        self,
        agent: AgentState,
        observations: list[BoundaryObservation],
    ) -> tuple[list[np.ndarray], list[np.ndarray]]:
        if not observations:
            return [], []
        dists = [float(np.linalg.norm(obs.point - agent.position)) for obs in observations]
        order = np.argsort(dists)[: max(1, int(self.params.object_cbf_points))]
        points = [observations[int(i)].point for i in order]
        normals = [observations[int(i)].normal for i in order]
        return points, normals

    def _observation_transport_bias(
        self,
        agent: AgentState,
        observations: list[BoundaryObservation],
        timestamp: float,
    ) -> np.ndarray:
        """Task velocity bias using local measurements only (no cargo geometry)."""
        if (
            not self.params.enable_transport_bias
            or not observations
            or float(timestamp) < float(self.params.transport_activation_time)
        ):
            return np.zeros(2, dtype=float)
        vd = np.asarray(self.params.task_velocity, dtype=float).reshape(2)
        if float(np.linalg.norm(vd)) < 1e-9:
            return np.zeros(2, dtype=float)
        # A common task velocity translates the entire locally informed
        # enclosure. Restricting this term to contact agents tears the formation
        # apart as the cargo moves and was the main cause of coverage collapse.
        # Contact forces still arise only where the physics engine detects them.
        del agent
        return self.params.kp_transport * normalize(vd)

    def _arm_command(
        self,
        i: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        detecting_agents: set[str],
        timestamp: float,
    ) -> tuple[np.ndarray, str]:
        """B0: ARM-style agent-centered Gaussian density peaks."""
        agent = agents[i]
        local_ids = [agent.agent_id] + [agents[j].agent_id for j in neighbor_indices]
        peaks = [agents[k].position for k, a in enumerate(agents) if a.agent_id in detecting_agents and a.agent_id in local_ids]
        # Also include self if this agent currently detects via its map.
        if agent.agent_id in detecting_agents and not any(
            np.allclose(agent.position, p) for p in peaks
        ):
            peaks.append(agent.position)
        if not peaks:
            # Fall back to any local map evidence: treat own position as weak peak once map nonempty.
            if self.maps[agent.agent_id].all_observations(timestamp):
                peaks = [agent.position]
            else:
                return self._exploration_velocity(i, agents, neighbor_indices, timestamp), "arm_explore"
        density = BoundaryAwareDensity.from_targets(peaks, sigma=self.params.sigma, object_id="arm_peaks")
        centroid = self.cvt.compute_centroid(i, agents, neighbor_indices, density, self.domain)
        u_nom = self.params.kp_cage * (centroid - agent.position)
        observations = self.maps[agent.agent_id].all_observations(timestamp)
        u_nom = u_nom + self._observation_transport_bias(agent, observations, timestamp)
        return u_nom, "arm_cage"

    def _oracle_command(
        self,
        i: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        cargoes: list[Cargo],
        timestamp: float,
    ) -> tuple[np.ndarray, str]:
        """B1: geometry-aware oracle using true cargo boundary samples."""
        agent = agents[i]
        if not cargoes:
            return self._exploration_velocity(i, agents, neighbor_indices, 0.0), "oracle_explore"
        targets: list[np.ndarray] = []
        weights: list[float] = []
        for cargo in cargoes:
            points, normals = cargo.boundary_samples(96)
            cage = points + self.params.cage_offset * normals
            # Limit to local radius for fair limited-CVT comparison.
            mask = np.linalg.norm(cage - agent.position[None, :], axis=1) <= self.params.comm_range * 1.5
            local = cage[mask]
            if len(local) == 0:
                continue
            targets.extend(list(local))
            weights.extend([1.0] * len(local))
        if not targets:
            return self._exploration_velocity(i, agents, neighbor_indices, 0.0), "oracle_search"
        density = BoundaryAwareDensity.from_targets(targets, sigma=self.params.sigma, weights=weights, object_id="oracle")
        centroid = self.cvt.compute_centroid(i, agents, neighbor_indices, density, self.domain)
        u_nom = self.params.kp_cage * (centroid - agent.position)
        # Oracle may use task velocity when near true boundary.
        if (
            self.params.enable_transport_bias
            and float(timestamp) >= float(self.params.transport_activation_time)
        ):
            for cargo in cargoes:
                _, _, dist = cargo.closest_boundary(agent.position)
                if dist <= self.params.cage_offset + self.params.transport_contact_margin:
                    vd = np.asarray(self.params.task_velocity, dtype=float).reshape(2)
                    if float(np.linalg.norm(vd)) > 1e-9:
                        u_nom = u_nom + self.params.kp_transport * normalize(vd)
                    break
        return u_nom, "oracle_cage"

    def _coverage_command(
        self,
        i: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
    ) -> tuple[np.ndarray, str]:
        agent = agents[i]
        visible_targets = self._visible_target_points(agent.position)
        if len(visible_targets) == 0:
            return self._exploration_velocity(i, agents, neighbor_indices, 0.0), "dbact_search"
        density = BoundaryAwareDensity.from_targets(
            visible_targets,
            sigma=self.params.sigma,
            object_id="coverage_region",
        )
        centroid = self.cvt.compute_centroid(i, agents, neighbor_indices, density, self.domain)
        return self.params.kp_cage * (centroid - agent.position), "dbact_coverage"

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

    def _exploration_velocity(self, i: int, agents: list[AgentState], neighbor_indices: list[int], timestamp: float) -> np.ndarray:
        agent = agents[i]
        repel = np.zeros(2, dtype=float)
        for j in neighbor_indices:
            d = agent.position - agents[j].position
            dist = float(np.linalg.norm(d))
            if dist > 1e-6:
                repel += d / (dist * dist)
        angle = 0.7 * i + 0.25 * timestamp
        sweep = np.array([np.cos(angle), np.sin(angle)], dtype=float)
        u = self.params.kp_explore * (0.7 * normalize(repel) + 0.3 * sweep)
        return u

    def apply_commands(
        self,
        agents: list[AgentState],
        commands: list[ControlCommand],
        dt: float,
        *,
        advance_positions: bool = True,
    ) -> None:
        by_id = {cmd.agent_id: cmd for cmd in commands}
        for agent in agents:
            cmd = by_id[agent.agent_id]
            agent.velocity = cmd.velocity.copy()
            if advance_positions:
                agent.position = clip_to_domain(agent.position + cmd.velocity * dt, self.domain)
