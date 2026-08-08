"""Evaluation metrics.

These functions are allowed to read the true cargo geometry because none of them
is part of the controller. They exist to make three things measurable that the
previous metric set could not distinguish:

* a robot standing *inside* the cargo used to count as covering the boundary, so
  a run in which 8 of 14 robots had passed through the object still reported
  0.988 coverage;
* "the cargo moved" said nothing about *which way*;
* penetration was never recorded at all, so there was no way to notice it.
"""

from __future__ import annotations

import numpy as np

from .cargo import Cargo
from .geometry import signed_distance_to_polygon
from .types import AgentState


def min_inter_agent_distance(agents: list[AgentState]) -> float:
    if len(agents) < 2:
        return float("inf")
    pts = np.vstack([a.position for a in agents])
    diff = pts[:, None, :] - pts[None, :, :]
    dist = np.linalg.norm(diff, axis=2)
    np.fill_diagonal(dist, np.inf)
    return float(np.min(dist))


def signed_clearances(cargo: Cargo, agents: list[AgentState]) -> np.ndarray:
    """Signed distance from each robot centre to the cargo boundary."""
    if not agents:
        return np.empty(0)
    positions = np.vstack([a.position for a in agents])
    return signed_distance_to_polygon(positions, cargo.vertices)


def boundary_coverage(cargo: Cargo, agents: list[AgentState], contact_radius: float = 0.42, samples: int = 160) -> float:
    """Legacy coverage: any robot within ``contact_radius`` counts.

    Kept only so that pre-refactor numbers can be reproduced and compared against
    the strict definition. Do not report this on its own.
    """
    if not agents:
        return 0.0
    boundary, _ = cargo.boundary_samples(samples)
    positions = np.vstack([a.position for a in agents])
    dists = np.linalg.norm(boundary[:, None, :] - positions[None, :, :], axis=2)
    return float(np.mean(np.any(dists <= contact_radius, axis=1)))


def strict_boundary_coverage(
    cargo: Cargo,
    agents: list[AgentState],
    contact_radius: float = 0.42,
    samples: int = 160,
) -> float:
    """Coverage counting only robots whose centre is outside the cargo.

    This is the difference that made the legacy 0.988 meaningless: a robot that
    has passed through the boundary is close to many boundary samples at once and
    therefore inflates the score precisely when the run has failed.
    """
    if not agents:
        return 0.0
    positions = np.vstack([a.position for a in agents])
    outside = signed_distance_to_polygon(positions, cargo.vertices) >= 0.0
    if not np.any(outside):
        return 0.0
    boundary, _ = cargo.boundary_samples(samples)
    dists = np.linalg.norm(boundary[:, None, :] - positions[None, outside, :], axis=2)
    return float(np.mean(np.any(dists <= contact_radius, axis=1)))


def penetration_report(cargo: Cargo, agents: list[AgentState], robot_radius: float) -> dict:
    """Penetration statistics for one cargo at one instant.

    ``penetration = max(0, r_robot - s)`` is the overlap the penalty contact model
    turns into normal force. ``agents_inside`` counts centres that have crossed
    the boundary outright, which is the failure the old metric set rewarded.
    """
    clearances = signed_clearances(cargo, agents)
    if len(clearances) == 0:
        return {
            "min_signed_clearance": float("inf"),
            "max_penetration": 0.0,
            "agents_inside": 0,
            "deepest_inside": 0.0,
            "agents_in_contact": 0,
        }
    penetration = np.maximum(0.0, robot_radius - clearances)
    inside = clearances < 0.0
    return {
        "min_signed_clearance": float(np.min(clearances)),
        "max_penetration": float(np.max(penetration)),
        "agents_inside": int(np.sum(inside)),
        "deepest_inside": float(-np.min(clearances)) if np.any(inside) else 0.0,
        "agents_in_contact": int(np.sum(penetration > 0.0)),
    }


def clearance_margin(cargo: Cargo, agents: list[AgentState], r_safe: float) -> float:
    """Worst-case slack of the object-boundary barrier, ``min_i s_i - r_safe``."""
    clearances = signed_clearances(cargo, agents)
    if len(clearances) == 0:
        return float("inf")
    return float(np.min(clearances) - r_safe)


def recruited_agents_count(cargo: Cargo, agents: list[AgentState], contact_radius: float = 0.42) -> int:
    """Robots close enough to the cargo boundary to be considered engaged."""
    clearances = signed_clearances(cargo, agents)
    if len(clearances) == 0:
        return 0
    return int(np.sum(np.abs(clearances) <= contact_radius))


def directional_progress(start: np.ndarray, end: np.ndarray, goal_direction: np.ndarray) -> dict:
    """Signed progress ``J`` along the goal direction plus its efficiency."""
    x0 = np.asarray(start, dtype=float).reshape(2)
    xt = np.asarray(end, dtype=float).reshape(2)
    u = np.asarray(goal_direction, dtype=float).reshape(2)
    norm = float(np.linalg.norm(u))
    if norm < 1e-9:
        return {"J": 0.0, "displacement": 0.0, "efficiency": 0.0, "angle_deg": float("nan")}
    u = u / norm
    dx = xt - x0
    displacement = float(np.linalg.norm(dx))
    j = float(np.dot(dx, u))
    efficiency = j / displacement if displacement > 1e-12 else 0.0
    if displacement > 1e-12:
        angle = float(np.degrees(np.arccos(np.clip(j / displacement, -1.0, 1.0))))
    else:
        angle = float("nan")
    return {"J": j, "displacement": displacement, "efficiency": efficiency, "angle_deg": angle}


def path_lengths(history: dict[str, list[np.ndarray]]) -> dict[str, float]:
    out: dict[str, float] = {}
    for agent_id, points in history.items():
        if len(points) < 2:
            out[agent_id] = 0.0
        else:
            arr = np.vstack(points)
            out[agent_id] = float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))
    return out


__all__ = [
    "min_inter_agent_distance",
    "signed_clearances",
    "boundary_coverage",
    "strict_boundary_coverage",
    "penetration_report",
    "clearance_margin",
    "recruited_agents_count",
    "directional_progress",
    "path_lengths",
]
