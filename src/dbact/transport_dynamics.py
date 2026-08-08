from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .cargo import Cargo
from .types import AgentState


@dataclass
class TransportParams:
    backend: str = "scripted"  # scripted | pymunk
    contact_radius: float = 0.42
    coverage_threshold: float = 0.42
    min_contact_agents: int = 3
    speed: float = 0.16
    boundary_samples: int = 120
    robot_radius: float = 0.12
    cargo_mass: float = 8.0
    cargo_friction: float = 0.85
    cargo_elasticity: float = 0.05
    agent_friction: float = 0.9
    linear_damping: float = 0.65
    angular_damping: float = 0.75
    substeps: int = 2


@dataclass
class CargoTransportStatus:
    object_id: str
    coverage: float
    contact_agents: int
    moved: bool


class TransportDynamics(Protocol):
    def step(self, cargoes: list[Cargo], agents: list[AgentState], dt: float) -> list[CargoTransportStatus]:
        ...


class SimpleCagingTransportDynamics:
    """Scripted object dynamics for algorithm-level validation.

    A cargo moves only when enough boundary samples are covered and enough agents
    are close to its boundary. Retained as the ``scripted`` backend for ablations
    and fast smoke tests. Not a full rigid-body contact solver.
    """

    def __init__(self, params: TransportParams):
        self.params = params
        self.advances_agents = False

    def step(self, cargoes: list[Cargo], agents: list[AgentState], dt: float) -> list[CargoTransportStatus]:
        statuses: list[CargoTransportStatus] = []
        positions = np.vstack([a.position for a in agents]) if agents else np.empty((0, 2))
        for cargo in cargoes:
            coverage, contact_agents = self._coverage_and_contacts(cargo, positions)
            moved = False
            if (
                cargo.movable
                and coverage >= self.params.coverage_threshold
                and contact_agents >= self.params.min_contact_agents
            ):
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


class PymunkTransportDynamics:
    """Physics-based transport: robots push cargo through PyMunk contacts."""

    def __init__(self, params: TransportParams, cargoes: list[Cargo], agents: list[AgentState]):
        from dbact_sim.rigid_body_world import RigidBodyParams, RigidBodyWorld

        self.params = params
        self.advances_agents = True
        self._coverage_helper = SimpleCagingTransportDynamics(params)
        self.world = RigidBodyWorld(
            cargoes,
            agents,
            RigidBodyParams(
                robot_radius=params.robot_radius,
                cargo_mass=params.cargo_mass,
                cargo_friction=params.cargo_friction,
                cargo_elasticity=params.cargo_elasticity,
                agent_friction=params.agent_friction,
                linear_damping=params.linear_damping,
                angular_damping=params.angular_damping,
                substeps=params.substeps,
            ),
        )

    def step(self, cargoes: list[Cargo], agents: list[AgentState], dt: float) -> list[CargoTransportStatus]:
        self.world.sync_agents_from_state(agents)
        self.world.step(dt)
        self.world.write_agents(agents)
        moved_flags = self.world.write_cargoes(cargoes)
        positions = np.vstack([a.position for a in agents]) if agents else np.empty((0, 2))
        statuses: list[CargoTransportStatus] = []
        for cargo in cargoes:
            coverage, contact_agents = self._coverage_helper._coverage_and_contacts(cargo, positions)
            statuses.append(
                CargoTransportStatus(
                    cargo.object_id,
                    coverage,
                    contact_agents,
                    bool(moved_flags.get(cargo.object_id, False)),
                )
            )
        return statuses


def build_transport(
    params: TransportParams,
    cargoes: list[Cargo] | None = None,
    agents: list[AgentState] | None = None,
) -> SimpleCagingTransportDynamics | PymunkTransportDynamics:
    backend = str(params.backend).lower()
    if backend == "pymunk":
        if cargoes is None or agents is None:
            raise ValueError("pymunk transport requires cargoes and agents at construction time")
        return PymunkTransportDynamics(params, cargoes, agents)
    return SimpleCagingTransportDynamics(params)
