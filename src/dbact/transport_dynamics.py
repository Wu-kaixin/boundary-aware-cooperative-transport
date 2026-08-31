"""S6 - transport engines.

Three engines, selected by an explicit ``transport.engine`` field with no default:

``penalty``  compliant contact against the rigid-body cargo (default for
             experiments; dependency-free and deterministic)
``pymunk``   the same scenario in an external rigid-body engine, as an
             independent check that the results are not an artefact of the
             contact model
``scripted`` the legacy behaviour: the cargo slides along a configured direction
             once a coverage threshold is met

``scripted`` is kept on purpose. It is the pre-refactor baseline, and the fact
that under it the net displacement direction agrees with the configured
direction to 0.000000 degrees is the cleanest available evidence that the earlier
"transport" results were assumed rather than produced. It must be requested by
name, and ``scenarios.py`` refuses to load it from a paper configuration.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .cargo import Cargo
from .contact_dynamics import ContactParams, ContactReport, PenaltyContactModel
from .types import AgentState


@dataclass
class TransportStatus:
    object_id: str
    contact_count: int
    net_force: np.ndarray
    net_torque: float
    max_penetration: float
    engine: str


@dataclass
class ScriptedParams:
    """Legacy engine parameters. Only meaningful for ``engine: scripted``."""

    contact_radius: float = 0.42
    coverage_threshold: float = 0.42
    min_contact_agents: int = 3
    speed: float = 0.16
    boundary_samples: int = 120
    goal_directions: dict[str, np.ndarray] = field(default_factory=dict)


class PenaltyTransportEngine:
    """Contact-only transport. The cargo moves iff robots push it."""

    name = "penalty"

    def __init__(self, params: ContactParams):
        self.params = params
        self.model = PenaltyContactModel(params)
        self.last_reports: dict[str, ContactReport] = {}

    def step(self, cargoes: list[Cargo], agents: list[AgentState], dt: float) -> list[TransportStatus]:
        statuses: list[TransportStatus] = []
        for cargo in cargoes:
            report = self.model.step(cargo, agents, dt)
            self.last_reports[cargo.object_id] = report
            statuses.append(
                TransportStatus(
                    object_id=cargo.object_id,
                    contact_count=report.contact_count,
                    net_force=report.net_force.copy(),
                    net_torque=report.net_torque,
                    max_penetration=report.max_penetration,
                    engine=self.name,
                )
            )
        return statuses


class PymunkTransportEngine:
    """Same scenario inside pymunk (Chipmunk2D).

    Robots are kinematic circle bodies driven from ``AgentState``; each cargo is
    one dynamic body carrying an ear-clipped triangle decomposition of its
    outline, since the solver only accepts convex shapes. Gravity is zero, so
    again the only thing that can move the cargo is a contact impulse.
    """

    name = "pymunk"

    def __init__(self, params: ContactParams, friction: float | None = None):
        import pymunk  # imported here so the dependency is only needed on request

        from .geometry import triangulate_simple_polygon

        self._pymunk = pymunk
        self._triangulate = triangulate_simple_polygon
        self.params = params
        self.friction = params.friction if friction is None else friction
        self.space = pymunk.Space()
        # Planar top-down view: no gravity in the plane. Ground friction is applied
        # as an explicit Coulomb velocity clamp after each step, matching the
        # penalty engine, because ``space.damping`` is viscous and would reintroduce
        # the runaway-cargo artefact this model exists to avoid.
        self.space.gravity = (0.0, 0.0)
        self.space.damping = 1.0
        self._bodies: dict[str, object] = {}
        self._agent_bodies: dict[str, object] = {}
        self._built = False

    def _build(self, cargoes: list[Cargo], agents: list[AgentState]) -> None:
        pymunk = self._pymunk
        for cargo in cargoes:
            body = pymunk.Body(
                mass=float(cargo.mass),
                moment=float(cargo.inertia),
                body_type=pymunk.Body.DYNAMIC if cargo.movable else pymunk.Body.STATIC,
            )
            body.position = tuple(cargo.position)
            body.angle = float(cargo.angle)
            self.space.add(body)
            for triangle in self._triangulate(cargo.local_vertices):
                shape = pymunk.Poly(body, [tuple(p) for p in triangle])
                shape.friction = self.friction
                shape.elasticity = 0.0
                self.space.add(shape)
            self._bodies[cargo.object_id] = body

        for agent in agents:
            body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
            body.position = tuple(agent.position)
            shape = pymunk.Circle(body, float(self.params.robot_radius))
            shape.friction = self.friction
            shape.elasticity = 0.0
            self.space.add(body, shape)
            self._agent_bodies[agent.agent_id] = body
        self._built = True

    def _apply_ground_friction(self, body, cargo: Cargo, delta_v: float) -> None:
        speed = float(np.hypot(body.velocity.x, body.velocity.y))
        if speed <= delta_v:
            body.velocity = (0.0, 0.0)
        else:
            scale = 1.0 - delta_v / speed
            body.velocity = (body.velocity.x * scale, body.velocity.y * scale)
        radius_of_gyration = max(float(np.sqrt(cargo.inertia / cargo.mass)), 1e-9)
        delta_omega = delta_v / radius_of_gyration
        omega = float(body.angular_velocity)
        body.angular_velocity = 0.0 if abs(omega) <= delta_omega else omega - np.copysign(delta_omega, omega)

    def step(self, cargoes: list[Cargo], agents: list[AgentState], dt: float) -> list[TransportStatus]:
        if not self._built:
            self._build(cargoes, agents)

        for agent in agents:
            body = self._agent_bodies.get(agent.agent_id)
            if body is None:
                continue
            body.position = tuple(agent.position)
            body.velocity = tuple(agent.velocity)

        n_sub = max(1, int(self.params.substeps))
        h = dt / n_sub
        delta_v = h * self.params.ground_friction * self.params.gravity
        for _ in range(n_sub):
            self.space.step(h)
            for cargo in cargoes:
                self._apply_ground_friction(self._bodies[cargo.object_id], cargo, delta_v)

        statuses: list[TransportStatus] = []
        for cargo in cargoes:
            body = self._bodies[cargo.object_id]
            cargo.set_pose(np.array([body.position.x, body.position.y]), float(body.angle))
            cargo.set_twist(np.array([body.velocity.x, body.velocity.y]), float(body.angular_velocity))
            # Net wrench read back from the solver's own impulses, so it is the
            # engine's answer rather than a recomputation of it.
            accumulated = {"force": np.zeros(2), "count": 0, "penetration": 0.0}

            def collect(arbiter, store=accumulated):
                impulse = arbiter.total_impulse
                store["force"] = store["force"] + np.array([impulse.x, impulse.y])
                store["count"] += 1
                for point in arbiter.contact_point_set.points:
                    store["penetration"] = max(store["penetration"], -float(point.distance))

            body.each_arbiter(collect)
            net_force = accumulated["force"] / max(dt, 1e-9)
            net_torque = 0.0
            count = accumulated["count"]
            max_penetration = accumulated["penetration"]
            statuses.append(
                TransportStatus(
                    object_id=cargo.object_id,
                    contact_count=count,
                    net_force=net_force,
                    net_torque=net_torque,
                    max_penetration=max_penetration,
                    engine=self.name,
                )
            )
        return statuses


class ScriptedTransportEngine:
    """Legacy engine: the cargo slides along a configured direction.

    Retained only as the pre-refactor baseline (B0). It reads a direction that no
    robot ever measured, so its output is a restatement of its configuration.
    """

    name = "scripted"

    def __init__(self, params: ScriptedParams):
        self.params = params

    def step(self, cargoes: list[Cargo], agents: list[AgentState], dt: float) -> list[TransportStatus]:
        statuses: list[TransportStatus] = []
        positions = np.vstack([a.position for a in agents]) if agents else np.empty((0, 2))
        for cargo in cargoes:
            coverage, contacts = self._coverage_and_contacts(cargo, positions)
            direction = self.params.goal_directions.get(cargo.object_id)
            if (
                cargo.movable
                and direction is not None
                and coverage >= self.params.coverage_threshold
                and contacts >= self.params.min_contact_agents
            ):
                cargo.translate(np.asarray(direction, dtype=float) * self.params.speed * coverage * dt)
            statuses.append(
                TransportStatus(
                    object_id=cargo.object_id,
                    contact_count=contacts,
                    net_force=np.zeros(2),
                    net_torque=0.0,
                    max_penetration=0.0,
                    engine=self.name,
                )
            )
        return statuses

    def _coverage_and_contacts(self, cargo: Cargo, positions: np.ndarray) -> tuple[float, int]:
        if len(positions) == 0:
            return 0.0, 0
        boundary, _ = cargo.boundary_samples(self.params.boundary_samples)
        dists = np.linalg.norm(boundary[:, None, :] - positions[None, :, :], axis=2)
        coverage = float(np.mean(np.any(dists <= self.params.contact_radius, axis=1)))
        contacts = int(np.sum(np.min(dists, axis=0) <= self.params.contact_radius))
        return coverage, contacts


def build_engine(
    engine: str,
    contact_params: ContactParams,
    scripted_params: ScriptedParams | None = None,
):
    if engine == "penalty":
        return PenaltyTransportEngine(contact_params)
    if engine == "pymunk":
        return PymunkTransportEngine(contact_params)
    if engine == "scripted":
        return ScriptedTransportEngine(scripted_params or ScriptedParams())
    raise ValueError(f"unknown transport engine {engine!r}; expected 'penalty', 'pymunk' or 'scripted'")


__all__ = [
    "TransportStatus",
    "ScriptedParams",
    "PenaltyTransportEngine",
    "PymunkTransportEngine",
    "ScriptedTransportEngine",
    "build_engine",
]
