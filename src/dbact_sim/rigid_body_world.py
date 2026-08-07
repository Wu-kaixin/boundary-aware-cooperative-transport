"""Planar rigid-body contact world for physics-based cooperative transport.

Uses PyMunk so object displacement comes from contact forces, not scripted
evaluator motion. Agents are kinematic circles; cargoes are dynamic polygons.
Agent–agent collisions are disabled (handled by the distributed CBF filter).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dbact.cargo import Cargo
from dbact.geometry import ensure_ccw, polygon_centroid
from dbact.types import AgentState


@dataclass
class RigidBodyParams:
    robot_radius: float = 0.12
    cargo_mass: float = 8.0
    cargo_friction: float = 0.85
    cargo_elasticity: float = 0.05
    agent_friction: float = 0.9
    linear_damping: float = 0.65
    angular_damping: float = 0.75
    substeps: int = 2


class RigidBodyWorld:
    """Thin wrapper around a PyMunk space synchronized with DBACT cargo/agents."""

    def __init__(
        self,
        cargoes: list[Cargo],
        agents: list[AgentState],
        params: RigidBodyParams | None = None,
    ):
        try:
            import pymunk
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "PyMunk is required for physics transport. Install with: pip install 'dbact[sim]'"
            ) from exc

        self.pymunk = pymunk
        self.params = params or RigidBodyParams()
        self.space = pymunk.Space()
        self.space.gravity = (0.0, 0.0)
        self.space.damping = float(np.clip(1.0 - self.params.linear_damping, 0.05, 1.0))

        self._cargo_bodies: dict[str, object] = {}
        self._cargo_local_vertices: dict[str, np.ndarray] = {}
        self._agent_bodies: dict[str, object] = {}

        # ShapeFilter categories/masks: agents (0b01) collide only with cargo (0b10).
        # Agent–agent collisions are disabled; inter-agent safety is handled by CBF.

        for cargo in cargoes:
            self._add_cargo(cargo)
        for agent in agents:
            self._add_agent(agent)

    def _add_cargo(self, cargo: Cargo) -> None:
        pymunk = self.pymunk
        vertices = ensure_ccw(cargo.vertices)
        center = polygon_centroid(vertices)
        local = vertices - center
        moment = pymunk.moment_for_poly(self.params.cargo_mass, local.tolist(), (0, 0))
        body = pymunk.Body(self.params.cargo_mass, max(moment, 1e-3))
        body.position = (float(center[0]), float(center[1]))
        body.angle = 0.0
        shape = pymunk.Poly(body, [(float(x), float(y)) for x, y in local])
        shape.friction = self.params.cargo_friction
        shape.elasticity = self.params.cargo_elasticity
        shape.collision_type = 2
        shape.filter = pymunk.ShapeFilter(categories=0b10, mask=0b01)
        self.space.add(body, shape)
        self._cargo_bodies[cargo.object_id] = body
        self._cargo_local_vertices[cargo.object_id] = local.copy()
        # Angular damping approximated each step via velocity scaling.

    def _add_agent(self, agent: AgentState) -> None:
        pymunk = self.pymunk
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        body.position = (float(agent.position[0]), float(agent.position[1]))
        shape = pymunk.Circle(body, self.params.robot_radius)
        shape.friction = self.params.agent_friction
        shape.elasticity = 0.0
        shape.collision_type = 1
        shape.filter = pymunk.ShapeFilter(categories=0b01, mask=0b10)
        self.space.add(body, shape)
        self._agent_bodies[agent.agent_id] = body

    def sync_agents_from_state(self, agents: list[AgentState]) -> None:
        for agent in agents:
            body = self._agent_bodies.get(agent.agent_id)
            if body is None:
                continue
            body.position = (float(agent.position[0]), float(agent.position[1]))
            body.velocity = (float(agent.velocity[0]), float(agent.velocity[1]))

    def step(self, dt: float) -> None:
        sub = max(1, int(self.params.substeps))
        h = float(dt) / sub
        damp = float(np.clip(1.0 - self.params.angular_damping * h, 0.0, 1.0))
        for _ in range(sub):
            self.space.step(h)
            for body in self._cargo_bodies.values():
                body.angular_velocity *= damp

    def write_cargoes(self, cargoes: list[Cargo]) -> dict[str, bool]:
        """Update cargo polygons from rigid bodies. Returns moved flags."""
        moved: dict[str, bool] = {}
        for cargo in cargoes:
            body = self._cargo_bodies.get(cargo.object_id)
            local = self._cargo_local_vertices.get(cargo.object_id)
            if body is None or local is None or not cargo.movable:
                moved[cargo.object_id] = False
                continue
            angle = float(body.angle)
            c, s = np.cos(angle), np.sin(angle)
            rot = np.array([[c, -s], [s, c]], dtype=float)
            center = np.array([float(body.position.x), float(body.position.y)], dtype=float)
            new_vertices = local @ rot.T + center
            delta = float(np.linalg.norm(new_vertices.mean(axis=0) - cargo.vertices.mean(axis=0)))
            cargo.vertices = ensure_ccw(new_vertices)
            moved[cargo.object_id] = delta > 1e-6
        return moved
