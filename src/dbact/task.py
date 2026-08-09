"""D1 - the transport task: a random direction that the workspace admits.

A fixed goal direction in a configuration file is not a task, it is a setting,
and a controller tuned against one direction says nothing about the next. Each
episode therefore samples its own:

    theta ~ U(0, 2pi)          direction
    L     ~ U(L_min, L_max)    distance along it

and rejects the draw unless the object, together with the ring of robots around
it, still fits inside the workspace at the end of the run. That rejection is what
makes "a random direction, but within the controllable range" a definition rather
than a hope: the admissible set is exactly the set of accepted draws, it can be
enumerated, and its acceptance rate is a number the sampler reports.

The clearance a goal has to leave is

    c = R_obj + d_c + r_robot + margin

-- object reach, cage ring, robot body, and a margin for the transient. Because
the workspace is a rectangle and the inflated workspace is therefore convex, a
segment whose endpoints both clear the walls by ``c`` clears them everywhere
along its length, so the corridor test is exact rather than sampled.

Where the direction is allowed to go
------------------------------------
Into the controller, and into the success criterion. Not into the contact engine:
``dbact.contact_dynamics`` has no field that could carry it and no argument
through which it could arrive, so "the cargo moved the way the configuration
said" is not an outcome this simulator is able to produce.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .contracts import ContractViolation


@dataclass(frozen=True)
class TransportTask:
    """One episode's sampled transport task."""

    object_id: str
    direction: np.ndarray
    distance: float
    start: np.ndarray
    goal_point: np.ndarray
    tolerance: float
    clearance: float
    attempts: int

    @property
    def angle_deg(self) -> float:
        return float(np.degrees(math.atan2(self.direction[1], self.direction[0])) % 360.0)

    def progress(self, position: np.ndarray) -> float:
        """Signed directional progress ``J`` of a cargo centre."""
        return float(np.dot(np.asarray(position, dtype=float).reshape(2) - self.start, self.direction))

    def cross_track(self, position: np.ndarray) -> float:
        """Distance from the straight line through ``start`` along ``direction``."""
        delta = np.asarray(position, dtype=float).reshape(2) - self.start
        return float(abs(delta[0] * self.direction[1] - delta[1] * self.direction[0]))

    def as_dict(self) -> dict:
        return {
            "object_id": self.object_id,
            "direction": self.direction.tolist(),
            "angle_deg": self.angle_deg,
            "target_distance": self.distance,
            "start": self.start.tolist(),
            "goal_point": self.goal_point.tolist(),
            "tolerance": self.tolerance,
            "clearance": self.clearance,
            "sampling_attempts": self.attempts,
        }


@dataclass
class TaskSampler:
    """Rejection sampler for workspace-admissible transport tasks."""

    distance_min: float = 0.35
    distance_max: float = 0.70
    wall_margin: float = 0.20
    tolerance: float = 0.12
    angle_min_deg: float = 0.0
    angle_max_deg: float = 360.0
    max_attempts: int = 512

    def clearance_for(self, object_radius: float, cage_offset: float, robot_radius: float) -> float:
        return float(object_radius + cage_offset + robot_radius + self.wall_margin)

    def sample(
        self,
        rng: np.random.Generator,
        object_id: str,
        start: np.ndarray,
        object_radius: float,
        cage_offset: float,
        robot_radius: float,
        domain: tuple[float, float, float, float],
    ) -> TransportTask:
        x0 = np.asarray(start, dtype=float).reshape(2)
        clearance = self.clearance_for(object_radius, cage_offset, robot_radius)
        xmin, xmax, ymin, ymax = domain

        if not self._inside(x0, domain, clearance):
            raise ContractViolation(
                f"cargo {object_id!r} starts {clearance:.3f} m or less from a workspace wall "
                f"(centre {x0.tolist()}, domain {domain}); no transport direction can be admissible, "
                "so the scenario is rejected rather than silently sampled from an empty set"
            )

        lo = math.radians(self.angle_min_deg)
        hi = math.radians(self.angle_max_deg)
        for attempt in range(1, self.max_attempts + 1):
            theta = rng.uniform(lo, hi)
            direction = np.array([math.cos(theta), math.sin(theta)], dtype=float)
            distance = float(rng.uniform(self.distance_min, self.distance_max))
            goal = x0 + distance * direction
            if not self._inside(goal, domain, clearance):
                continue
            return TransportTask(
                object_id=str(object_id),
                direction=direction,
                distance=distance,
                start=x0.copy(),
                goal_point=goal,
                tolerance=self.tolerance,
                clearance=clearance,
                attempts=attempt,
            )

        raise ContractViolation(
            f"no admissible transport direction for cargo {object_id!r} after {self.max_attempts} draws "
            f"(clearance {clearance:.3f} m, distance up to {self.distance_max:.3f} m, "
            f"domain {domain}); shrink the distance or enlarge the workspace"
        )

    @staticmethod
    def _inside(point: np.ndarray, domain: tuple[float, float, float, float], clearance: float) -> bool:
        xmin, xmax, ymin, ymax = domain
        return bool(
            xmin + clearance <= point[0] <= xmax - clearance
            and ymin + clearance <= point[1] <= ymax - clearance
        )

    def acceptance_rate(
        self,
        rng: np.random.Generator,
        start: np.ndarray,
        object_radius: float,
        cage_offset: float,
        robot_radius: float,
        domain: tuple[float, float, float, float],
        draws: int = 2000,
    ) -> float:
        """Fraction of raw draws the workspace admits. Reported, not assumed."""
        x0 = np.asarray(start, dtype=float).reshape(2)
        clearance = self.clearance_for(object_radius, cage_offset, robot_radius)
        theta = rng.uniform(math.radians(self.angle_min_deg), math.radians(self.angle_max_deg), size=draws)
        distance = rng.uniform(self.distance_min, self.distance_max, size=draws)
        goals = x0[None, :] + distance[:, None] * np.column_stack([np.cos(theta), np.sin(theta)])
        xmin, xmax, ymin, ymax = domain
        ok = (
            (goals[:, 0] >= xmin + clearance)
            & (goals[:, 0] <= xmax - clearance)
            & (goals[:, 1] >= ymin + clearance)
            & (goals[:, 1] <= ymax - clearance)
        )
        return float(np.mean(ok))


__all__ = ["TransportTask", "TaskSampler"]
