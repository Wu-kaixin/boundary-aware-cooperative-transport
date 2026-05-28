from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .cargo import Cargo
from .types import AgentState


@dataclass
class TransportParams:
    contact_radius: float = 0.42
    coverage_threshold: float = 0.42
    min_contact_agents: int = 3
    speed: float = 0.16
    boundary_samples: int = 120


@dataclass
class CargoTransportStatus:
    object_id: str
    coverage: float
    contact_agents: int
    moved: bool


class SimpleCagingTransportDynamics:
    """Simplified object dynamics for algorithm-level validation.

    A cargo moves only when enough boundary samples are covered and enough agents
    are close to its boundary. This is not a full rigid-body contact solver.
    """

    def __init__(self, params: TransportParams):
        self.params = params

    def step(self, cargoes: list[Cargo], agents: list[AgentState], dt: float) -> list[CargoTransportStatus]:
        statuses: list[CargoTransportStatus] = []
        positions = np.vstack([a.position for a in agents]) if agents else np.empty((0, 2))
        for cargo in cargoes:
            coverage, contact_agents = self._coverage_and_contacts(cargo, positions)
            moved = False
            if cargo.movable and coverage >= self.params.coverage_threshold and contact_agents >= self.params.min_contact_agents:
                delta = cargo.transport_direction * self.params.speed * coverage * dt
                cargo.translate(delta)
                moved = True
            statuses.append(CargoTransportStatus(cargo.object_id, coverage, contact_agents, moved))
        return statuses

    def _coverage_and_contacts(self, cargo: Cargo, positions: np.ndarray) -> tuple[float, int]:
        if len(positions) == 0:
            return 0.0, 0
        boundary, _ = cargo.boundary_samples(self.params.boundary_samples)
        dists = np.linalg.norm(boundary[:, None, :] - positions[None, :, :], axis=2)
        covered = np.any(dists <= self.params.contact_radius, axis=1)
        coverage = float(np.mean(covered))
        min_agent_to_boundary = np.min(dists, axis=0)
        contact_agents = int(np.sum(min_agent_to_boundary <= self.params.contact_radius))
        return coverage, contact_agents
