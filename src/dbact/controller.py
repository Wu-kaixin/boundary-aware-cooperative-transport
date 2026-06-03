from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .boundary_density import BoundaryAwareDensity
from .boundary_map import LocalBoundaryMap
from .cargo import Cargo
from .geometry import clip_to_domain, normalize
from .local_cbf_qp import LocalCBFQP
from .local_cvt import LocalCVT
from .local_sensing import LocalBoundarySensor
from .types import AgentState, ControlCommand


@dataclass
class DBACTParams:
    task_mode: str = "caging"
    sensor_range: float = 1.1
    comm_range: float = 1.6
    cage_offset: float = 0.32
    sigma: float = 0.35
    d_min: float = 0.28
    max_speed: float = 0.35
    kp_explore: float = 0.25
    kp_cage: float = 0.9
    kp_transport: float = 0.18
    grid_resolution: int = 34
    map_ttl: float = 4.0
    cbf_gamma: float = 6.0
    cbf_use_qp: bool = True
    cbf_slack_weight: float = 1000.0
    target_center: list[float] = field(default_factory=lambda: [4.0, 4.0])
    target_radius: float = 1.0
    target_sensor_range: float = 2.0
    target_samples: int = 36

    @classmethod
    def from_dict(cls, data: dict) -> "DBACTParams":
        fields = {k: v for k, v in data.items() if k in cls.__dataclass_fields__}
        return cls(**fields)


class DBACTController:
    """Decentralized boundary-aware cooperative transportation controller.

    In the simulator this object manages per-agent maps to emulate decentralized
    local memory and neighbor communication. Each robot command is computed only
    from its local observations and communication-range neighbors.
    """

    def __init__(self, params: DBACTParams, domain: tuple[float, float, float, float]):
        self.params = params
        self.domain = domain
        self.sensor = LocalBoundarySensor(sensor_range=params.sensor_range)
        self.cvt = LocalCVT(grid_resolution=params.grid_resolution, local_radius=params.comm_range)
        self.cbf = LocalCBFQP(
            d_min=params.d_min,
            gamma=params.cbf_gamma,
            max_speed=params.max_speed,
            use_qp=params.cbf_use_qp,
            slack_weight=params.cbf_slack_weight,
        )
        self.maps: dict[str, LocalBoundaryMap] = {}
        self.target_region_points = self._build_target_region_points()

    def step(self, agents: list[AgentState], cargoes: list[Cargo], timestamp: float, dt: float) -> list[ControlCommand]:
        self._ensure_maps(agents)
        sensed_by_agent: dict[str, list] = {}
        for agent in agents:
            observations = self.sensor.sense(agent, cargoes, timestamp)
            sensed_by_agent[agent.agent_id] = observations
            self.maps[agent.agent_id].update(observations, timestamp)

        # Local communication: exchange recent observations with neighbors only.
        positions = np.vstack([a.position for a in agents])
        for i, agent in enumerate(agents):
            for j, other in enumerate(agents):
                if i == j:
                    continue
                if np.linalg.norm(agent.position - other.position) <= self.params.comm_range:
                    self.maps[agent.agent_id].update(sensed_by_agent.get(other.agent_id, []), timestamp)

        commands: list[ControlCommand] = []
        for i, agent in enumerate(agents):
            neighbor_indices = [j for j, other in enumerate(agents) if j != i and np.linalg.norm(agent.position - other.position) <= self.params.comm_range]
            if self.params.task_mode == "coverage":
                u_nom, mode = self._coverage_command(i, agents, neighbor_indices)
            else:
                observations = self.maps[agent.agent_id].all_observations(timestamp)
                if observations:
                    density = BoundaryAwareDensity.from_observations(
                        observations,
                        cage_offset=self.params.cage_offset,
                        sigma=self.params.sigma,
                    )
                    centroid = self.cvt.compute_centroid(i, agents, neighbor_indices, density, self.domain)
                    u_nom = self.params.kp_cage * (centroid - agent.position)
                    # Caging-only stage:
                    # Do NOT use cargo geometry prior here.
                    # The controller must not call cargo.closest_boundary(), cargo.center,
                    # cargo.radius, or cargo.vertices.
                    # Cargo geometry is only used inside the simulated local sensor to generate
                    # local BoundaryObservation.
                    # Transport bias is disabled because cargo is assumed unknown.
                    # u_nom += self._transport_bias(agent, observations, cargoes)
                    mode = "dbact_cage"
                else:
                    u_nom = self._exploration_velocity(i, agents, neighbor_indices, timestamp)
                    mode = "dbact_explore"

            neighbor_positions = [agents[j].position for j in neighbor_indices]
            neighbor_velocities = [agents[j].velocity for j in neighbor_indices]
            u_safe = self.cbf.filter_velocity(agent.position, u_nom, neighbor_positions, neighbor_velocities)
            commands.append(ControlCommand(agent.agent_id, u_safe, mode=mode))
        return commands

    def _ensure_maps(self, agents: list[AgentState]) -> None:
        for agent in agents:
            self.maps.setdefault(agent.agent_id, LocalBoundaryMap(ttl=self.params.map_ttl))

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

    # NOTE:
    # This function uses simulator-side cargo geometry and is NOT used in the
    # unknown-cargo caging experiments. Keep it disabled unless a future transport
    # module explicitly provides object motion information through local sensing.
    def _transport_bias(self, agent: AgentState, observations: list, cargoes: list[Cargo]) -> np.ndarray:
        # Task-level transport direction is assumed available after object discovery in the simulator.
        # For physical robots this should come from task planning or operator command.
        object_ids = {obs.object_id for obs in observations}
        bias = np.zeros(2, dtype=float)
        for cargo in cargoes:
            if cargo.object_id in object_ids:
                q, _, distance = cargo.closest_boundary(agent.position)
                if distance <= self.params.cage_offset + 0.18:
                    bias += self.params.kp_transport * cargo.transport_direction
        return bias

    def _exploration_velocity(self, i: int, agents: list[AgentState], neighbor_indices: list[int], timestamp: float) -> np.ndarray:
        agent = agents[i]
        repel = np.zeros(2, dtype=float)
        for j in neighbor_indices:
            d = agent.position - agents[j].position
            dist = float(np.linalg.norm(d))
            if dist > 1e-6:
                repel += d / (dist * dist)
        # A small deterministic sweeping term helps robots leave compact initial states.
        angle = 0.7 * i + 0.25 * timestamp
        sweep = np.array([np.cos(angle), np.sin(angle)], dtype=float)
        u = self.params.kp_explore * (0.7 * normalize(repel) + 0.3 * sweep)
        return u

    def apply_commands(self, agents: list[AgentState], commands: list[ControlCommand], dt: float) -> None:
        by_id = {cmd.agent_id: cmd for cmd in commands}
        for agent in agents:
            cmd = by_id[agent.agent_id]
            agent.velocity = cmd.velocity.copy()
            agent.position = clip_to_domain(agent.position + cmd.velocity * dt, self.domain)
