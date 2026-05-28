from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .boundary_density import BoundaryAwareDensity
from .geometry import clip_to_domain
from .types import AgentState


@dataclass
class LocalCVT:
    """Grid-approximated local weighted Voronoi centroid."""

    grid_resolution: int = 32
    local_radius: float = 1.6

    def compute_centroid(
        self,
        agent_index: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        density: BoundaryAwareDensity,
        domain: tuple[float, float, float, float],
    ) -> np.ndarray:
        agent = agents[agent_index]
        local_indices = [agent_index] + [idx for idx in neighbor_indices if idx != agent_index]
        local_positions = np.vstack([agents[idx].position for idx in local_indices])

        samples = self._sample_local_region(agent.position, density, domain)
        if len(samples) == 0:
            return agent.position.copy()

        diff = samples[:, None, :] - local_positions[None, :, :]
        dist2 = np.sum(diff * diff, axis=2)
        owners = np.argmin(dist2, axis=1)
        own_mask = owners == 0
        if not np.any(own_mask):
            return agent.position.copy()
        own_samples = samples[own_mask]
        centroid = density.weighted_centroid(own_samples)
        if centroid is None:
            return agent.position.copy()
        return clip_to_domain(centroid, domain)

    def _sample_local_region(self, position: np.ndarray, density: BoundaryAwareDensity, domain: tuple[float, float, float, float]) -> np.ndarray:
        xmin, xmax, ymin, ymax = domain
        targets = density.targets
        if len(targets) > 0:
            lo = np.minimum(np.min(targets, axis=0), position - self.local_radius)
            hi = np.maximum(np.max(targets, axis=0), position + self.local_radius)
            lo -= 0.4
            hi += 0.4
        else:
            lo = position - self.local_radius
            hi = position + self.local_radius
        lo[0] = max(lo[0], xmin)
        lo[1] = max(lo[1], ymin)
        hi[0] = min(hi[0], xmax)
        hi[1] = min(hi[1], ymax)
        if hi[0] <= lo[0] or hi[1] <= lo[1]:
            return np.empty((0, 2))
        xs = np.linspace(lo[0], hi[0], self.grid_resolution)
        ys = np.linspace(lo[1], hi[1], self.grid_resolution)
        xx, yy = np.meshgrid(xs, ys)
        return np.column_stack([xx.ravel(), yy.ravel()])
