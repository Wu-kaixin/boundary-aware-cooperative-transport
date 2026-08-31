"""S6 - contact model: the only channel through which the cargo can move.

Robots are kinematic discs of radius ``r_robot``; the cargo is a dynamic rigid
body. Contact is a penalty (compliant) model:

    delta   = r_robot - s                      penetration of the disc
    f_n     = max(0, k_p delta - k_d n.v_rel)  normal magnitude
    F       = -f_n n  +  f_t                   force on the cargo
    tau     = (q - x) x F                       torque about the cargo centroid

with ``s`` the signed distance from the robot centre to the cargo boundary, ``n``
the outward unit normal, ``q`` the footpoint and ``v_rel`` the robot velocity
relative to the material point of the cargo at ``q``. Tangential force is
Coulomb friction regularised by a viscous branch so that it is continuous through
zero sliding speed.

The invariants this buys are checkable rather than asserted in prose:

* no contact  =>  zero net wrench  =>  the cargo does not move;
* the motion direction is a function of contact geometry only. No field of the
  configuration reaches this module, so a scripted direction cannot leak in.

``k_p`` is tied to C1: the largest force the team can produce at the cage ring is
``k_p * delta_max``, where ``delta_max = r_robot - r_safe`` is exactly the
penetration budget the object-boundary CBF leaves open.

Ground friction is Coulomb, not viscous
---------------------------------------
The object rests on a floor, so it resists motion with a breakaway force
``mu_g m g`` that is independent of speed, and it does not move at all while the
net contact force stays below that value.

Modelling the floor as linear drag instead is not a harmless simplification: it
gives the cargo a terminal speed ``||F||/(m c)`` with no lower threshold, and for
any reasonable stiffness that speed exceeds the robots' own speed limit. The
cargo is then kicked away by whichever robot touches it first, the penetration
relaxes to exactly zero, and the cage can never close. Measured across a
12-configuration parameter sweep, the minimum signed clearance came out at
0.160 m -- exactly ``r_robot``, i.e. grazing contact -- in every single
configuration, with mean contact counts of 0.005 to 0.43 out of 12 robots. That
is a dynamics artefact and no amount of controller tuning removes it.

Coulomb friction also makes the task cooperative in the first place: with
``mu_g m g > k_p delta_max`` a single robot cannot move the object at all, so a
result showing transport is a result about the team.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .cargo import Cargo
from .types import AgentState


@dataclass
class ContactParams:
    robot_radius: float = 0.16
    stiffness: float = 220.0
    damping: float = 12.0
    friction: float = 0.6
    tangential_stiffness: float = 60.0
    ground_friction: float = 0.35
    gravity: float = 9.81
    substeps: int = 4
    max_speed: float = 2.0
    max_angular_speed: float = 6.0

    def breakaway_force(self, mass: float) -> float:
        """Net contact force below which the cargo does not move at all."""
        return self.ground_friction * mass * self.gravity

    def min_cooperating_robots(self, mass: float, cage_offset: float) -> float:
        """Aligned robots needed to break the object loose at the cage ring.

        Reported so that "cooperative" is a measured property of the parameters
        rather than an adjective. Uses the nominal cage-ring penetration
        ``r_robot - d_c``, i.e. the force one robot applies once the coverage law
        has converged.
        """
        per_robot = self.stiffness * max(self.robot_radius - cage_offset, 1e-9)
        return self.breakaway_force(mass) / per_robot


@dataclass
class Contact:
    agent_id: str
    point: np.ndarray
    normal: np.ndarray
    penetration: float
    force: np.ndarray
    torque: float


@dataclass
class ContactReport:
    object_id: str
    contacts: list[Contact] = field(default_factory=list)
    net_force: np.ndarray = field(default_factory=lambda: np.zeros(2))
    net_torque: float = 0.0

    @property
    def contact_count(self) -> int:
        return len(self.contacts)

    @property
    def max_penetration(self) -> float:
        return max((c.penetration for c in self.contacts), default=0.0)


class PenaltyContactModel:
    """Compliant contact between kinematic robot discs and a rigid cargo."""

    def __init__(self, params: ContactParams):
        self.params = params

    def compute_contacts(self, cargo: Cargo, agents: list[AgentState]) -> ContactReport:
        report = ContactReport(object_id=cargo.object_id)
        if not agents:
            return report

        positions = np.vstack([a.position for a in agents])
        signed, normals, footpoints = cargo.signed_distance(positions)
        penetration = self.params.robot_radius - signed
        touching = np.where(penetration > 0.0)[0]

        net_force = np.zeros(2)
        net_torque = 0.0
        for idx in touching:
            n = normals[idx]
            q = footpoints[idx]
            v_rel = np.asarray(agents[idx].velocity, dtype=float).reshape(2) - cargo.point_velocity(q)
            normal_rate = float(np.dot(n, v_rel))

            f_n = self.params.stiffness * float(penetration[idx]) - self.params.damping * normal_rate
            if f_n <= 0.0:
                continue

            v_t = v_rel - normal_rate * n
            slip = float(np.linalg.norm(v_t))
            if slip > 1e-9:
                magnitude = min(self.params.friction * f_n, self.params.tangential_stiffness * slip)
                f_t = magnitude * (v_t / slip)
            else:
                f_t = np.zeros(2)

            force = -f_n * n + f_t
            r = q - cargo.position
            torque = float(r[0] * force[1] - r[1] * force[0])

            report.contacts.append(
                Contact(
                    agent_id=agents[idx].agent_id,
                    point=q.copy(),
                    normal=n.copy(),
                    penetration=float(penetration[idx]),
                    force=force,
                    torque=torque,
                )
            )
            net_force += force
            net_torque += torque

        report.net_force = net_force
        report.net_torque = net_torque
        return report

    def step(self, cargo: Cargo, agents: list[AgentState], dt: float) -> ContactReport:
        """Advance one cargo by ``dt`` under contact forces alone."""
        report = self.compute_contacts(cargo, agents)
        if not cargo.movable:
            cargo.set_twist(np.zeros(2), 0.0)
            return report

        # Substepping keeps the stiff penalty spring stable without asking the
        # whole simulation to run at the contact timescale.
        n_sub = max(1, int(self.params.substeps))
        h = dt / n_sub
        radius_of_gyration = float(np.sqrt(cargo.inertia / cargo.mass))
        for k in range(n_sub):
            sub = report if k == 0 else self.compute_contacts(cargo, agents)

            free_velocity = cargo.linear_velocity + h * sub.net_force / cargo.mass
            free_omega = cargo.angular_velocity + h * sub.net_torque / cargo.inertia

            # Coulomb ground friction, applied as a velocity clamp. Below the
            # breakaway impulse the motion is removed entirely rather than merely
            # damped, which is what gives the object genuine stiction.
            delta_v = h * self.params.ground_friction * self.params.gravity
            speed = float(np.linalg.norm(free_velocity))
            if speed <= delta_v:
                free_velocity = np.zeros(2)
            else:
                free_velocity = free_velocity * (1.0 - delta_v / speed)

            delta_omega = delta_v / max(radius_of_gyration, 1e-9)
            if abs(free_omega) <= delta_omega:
                free_omega = 0.0
            else:
                free_omega -= math.copysign(delta_omega, free_omega)

            cargo.linear_velocity = _clip_norm(free_velocity, self.params.max_speed)
            cargo.angular_velocity = float(
                np.clip(free_omega, -self.params.max_angular_speed, self.params.max_angular_speed)
            )
            cargo.translate(cargo.linear_velocity * h)
            cargo.rotate_by(cargo.angular_velocity * h)
        return report


def _clip_norm(v: np.ndarray, limit: float) -> np.ndarray:
    speed = float(np.linalg.norm(v))
    if speed <= limit:
        return v
    return v * (limit / speed)


__all__ = ["ContactParams", "Contact", "ContactReport", "PenaltyContactModel"]
