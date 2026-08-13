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
from .guarantees import minimum_facing_cage_clearance
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


def maximum_uncovered_boundary_arc(
    covered: np.ndarray,
    perimeter: float,
) -> dict[str, float | int]:
    """Conservative sampled upper bound on the longest cyclic uncovered arc."""
    mask = np.asarray(covered, dtype=bool).reshape(-1)
    count = len(mask)
    if count == 0:
        return {
            "sample_count": 0,
            "sample_resolution_m": float("inf"),
            "longest_uncovered_samples": 0,
            "max_uncovered_arc_upper_m": float("inf"),
        }
    resolution = float(perimeter) / count
    if np.all(mask):
        longest = 0
        upper = 0.0
    elif not np.any(mask):
        longest = count
        upper = float(perimeter)
    else:
        doubled = np.concatenate([~mask, ~mask])
        longest = 0
        run = 0
        for value in doubled:
            run = run + 1 if value else 0
            longest = min(count, max(longest, run))
        # The two sample-adjacent half intervals add at most one sampling
        # interval to the observed consecutive-uncovered run.
        upper = min(float(perimeter), (longest + 1) * resolution)
    return {
        "sample_count": count,
        "sample_resolution_m": resolution,
        "longest_uncovered_samples": int(longest),
        "max_uncovered_arc_upper_m": float(upper),
    }


def operational_enclosure_certificate(
    cargo: Cargo,
    agents: list[AgentState],
    *,
    contact_radius: float,
    strict_coverage_min: float,
    max_uncovered_arc_m: float,
    d_min: float,
    cage_offset: float,
    min_engaged_agents: int,
    engaged_radius: float | None = None,
    facing_clearance_m: float | None = None,
    samples: int = 360,
) -> dict:
    """Truth-audit certificate for operational boundary enclosure.

    This is deliberately not a configuration-space escape proof and therefore
    never claims formal caging.  It certifies measurable boundary occupancy,
    exterior robot centres, pairwise safety, offset-curve compatibility and an
    engaged quorum at one simulation frame.
    """
    positions = np.vstack([agent.position for agent in agents]) if agents else np.empty((0, 2))
    clearances = signed_distance_to_polygon(positions, cargo.vertices) if agents else np.empty(0)
    outside = clearances >= 0.0
    boundary, _ = cargo.boundary_samples(max(3, int(samples)))
    if len(positions) and np.any(outside):
        distances = np.linalg.norm(boundary[:, None, :] - positions[None, outside, :], axis=2)
        covered = np.any(distances <= float(contact_radius), axis=1)
    else:
        covered = np.zeros(len(boundary), dtype=bool)
    coverage = float(np.mean(covered)) if len(covered) else 0.0
    arc = maximum_uncovered_boundary_arc(covered, cargo.perimeter)
    engagement_limit = float(contact_radius if engaged_radius is None else engaged_radius)
    engaged = int(np.sum(outside & (clearances <= engagement_limit))) if len(clearances) else 0
    minimum_distance = min_inter_agent_distance(agents)
    facing_clearance = (
        minimum_facing_cage_clearance(cargo.vertices, float(cage_offset))
        if facing_clearance_m is None
        else float(facing_clearance_m)
    )
    checks = {
        "strict_boundary_coverage": bool(coverage + 1e-12 >= float(strict_coverage_min)),
        "maximum_uncovered_boundary_arc": bool(
            arc["max_uncovered_arc_upper_m"] <= float(max_uncovered_arc_m) + 1e-12
        ),
        "all_robot_centres_outside": bool(len(clearances) == len(agents) and np.all(outside)),
        "inter_agent_safety": bool(minimum_distance + 1e-12 >= float(d_min)),
        "cage_offset_feasible": bool(facing_clearance + 1e-12 >= float(d_min)),
        "engaged_quorum": bool(engaged >= int(min_engaged_agents)),
    }
    return {
        "certificate_type": "operational_boundary_enclosure",
        "formal_caging": False,
        "formal_caging_nonclaim": "no configuration-space escape proof is implemented",
        "passed": bool(all(checks.values())),
        "strict_boundary_coverage": coverage,
        **arc,
        "all_robot_centres_outside": checks["all_robot_centres_outside"],
        "min_signed_clearance_m": float(np.min(clearances)) if len(clearances) else float("inf"),
        "min_inter_agent_distance_m": minimum_distance,
        "facing_cage_clearance_m": (
            float(facing_clearance) if np.isfinite(facing_clearance) else None
        ),
        "engaged_agents": engaged,
        "thresholds": {
            "strict_coverage_min": float(strict_coverage_min),
            "max_uncovered_arc_m": float(max_uncovered_arc_m),
            "d_min_m": float(d_min),
            "cage_offset_m": float(cage_offset),
            "min_engaged_agents": int(min_engaged_agents),
            "engaged_radius_m": engagement_limit,
        },
        "checks": checks,
        "failure_reasons": [name for name, passed in checks.items() if not passed],
    }


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
    "maximum_uncovered_boundary_arc",
    "operational_enclosure_certificate",
    "penetration_report",
    "clearance_margin",
    "recruited_agents_count",
    "directional_progress",
    "path_lengths",
]
