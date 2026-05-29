from __future__ import annotations

import numpy as np

from .cargo import Cargo
from .types import AgentState


def min_inter_agent_distance(agents: list[AgentState]) -> float:
    if len(agents) < 2:
        return float("inf")
    pts = np.vstack([a.position for a in agents])
    best = float("inf")
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            best = min(best, float(np.linalg.norm(pts[i] - pts[j])))
    return best


def boundary_coverage(cargo: Cargo, agents: list[AgentState], contact_radius: float = 0.42, samples: int = 160) -> float:
    if not agents:
        return 0.0
    boundary, _ = cargo.boundary_samples(samples)
    positions = np.vstack([a.position for a in agents])
    dists = np.linalg.norm(boundary[:, None, :] - positions[None, :, :], axis=2)
    return float(np.mean(np.any(dists <= contact_radius, axis=1)))

def recruited_agents_count(
    cargo: Cargo,
    agents: list[AgentState],
    contact_radius: float = 0.42,
) -> int:
    """Count agents close enough to the cargo boundary.

    This is an offline evaluation metric. It is allowed to use the true cargo
    geometry here because this function is not part of the controller.
    """
    count = 0
    for agent in agents:
        _, _, distance = cargo.closest_boundary(agent.position)
        if distance <= contact_radius:
            count += 1
    return count

def path_lengths(history: dict[str, list[np.ndarray]]) -> dict[str, float]:
    out: dict[str, float] = {}
    for agent_id, points in history.items():
        if len(points) < 2:
            out[agent_id] = 0.0
        else:
            arr = np.vstack(points)
            out[agent_id] = float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))
    return out
