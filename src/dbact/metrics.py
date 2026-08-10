from __future__ import annotations

import numpy as np

from .cargo import Cargo
from .geometry import closest_boundary_distances
from .types import AgentState


def min_inter_agent_distance(agents: list[AgentState]) -> float:
    if len(agents) < 2:
        return float("inf")
    pts = np.vstack([a.position for a in agents])
    diff = pts[:, None, :] - pts[None, :, :]
    dist = np.linalg.norm(diff, axis=2)
    # Ignore the diagonal (self-distance = 0).
    np.fill_diagonal(dist, np.inf)
    return float(np.min(dist))


def min_agent_boundary_distance(cargo: Cargo, agents: list[AgentState]) -> float:
    """Minimum observed agent-to-true-boundary distance (evaluation only)."""
    if not agents:
        return float("inf")
    positions = np.vstack([a.position for a in agents])
    return float(np.min(closest_boundary_distances(cargo.vertices, positions)))


def boundary_and_min_distance(
    cargo: Cargo,
    agents: list[AgentState],
    contact_radius: float = 0.42,
    samples: int = 160,
) -> tuple[float, float]:
    """Coverage from samples plus exact min agent–boundary distance."""
    if not agents:
        return 0.0, float("inf")
    boundary, _ = cargo.boundary_samples(samples)
    positions = np.vstack([a.position for a in agents])
    dists = np.linalg.norm(boundary[:, None, :] - positions[None, :, :], axis=2)
    coverage = float(np.mean(np.any(dists <= contact_radius, axis=1)))
    min_dist = float(np.min(closest_boundary_distances(cargo.vertices, positions)))
    return coverage, min_dist


def boundary_coverage(cargo: Cargo, agents: list[AgentState], contact_radius: float = 0.42, samples: int = 160) -> float:
    if not agents:
        return 0.0
    coverage, _ = boundary_and_min_distance(cargo, agents, contact_radius=contact_radius, samples=samples)
    return coverage


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


def enclosure_time(
    coverage_history: list[float],
    times: list[float],
    threshold: float = 0.5,
) -> float | None:
    """First time coverage reaches threshold; None if never enclosed."""
    for cov, t in zip(coverage_history, times):
        if cov >= threshold:
            return float(t)
    return None


def success_flag(
    final_coverage: float,
    cargo_displacement: float,
    coverage_threshold: float = 0.5,
    min_displacement: float = 0.2,
    require_transport: bool = False,
) -> bool:
    if final_coverage < coverage_threshold:
        return False
    if require_transport and cargo_displacement < min_displacement:
        return False
    return True


def summarize_seeds(rows: list[dict]) -> dict:
    """Aggregate multi-seed scalar metrics as mean ± std."""
    if not rows:
        return {}
    keys = sorted({k for row in rows for k, v in row.items() if isinstance(v, (int, float, bool))})
    summary: dict = {"n_seeds": len(rows)}
    for key in keys:
        vals = [float(row[key]) for row in rows if key in row and row[key] is not None]
        if not vals:
            continue
        summary[key] = {
            "mean": float(np.mean(vals)),
            "std": float(np.std(vals)),
            "min": float(np.min(vals)),
            "max": float(np.max(vals)),
        }
    return summary
