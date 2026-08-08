from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .boundary_density import BoundaryAwareDensity
from .geometry import clip_to_domain
from .types import AgentState


@dataclass
class LocalCVT:
    """Grid-approximated limited local weighted Voronoi centroid.

    Integration domain is strictly Ω_i = D ∩ B(p_i, R_ℓ). Local Voronoi cell:
        V_i^ℓ = {q ∈ Ω_i : ‖q − p_i‖ ≤ ‖q − p_j‖, ∀ j ∈ N_i}
    so computation / communication complexity depend on neighborhood size,
    not total robot population.
    """

    grid_spacing: float = 0.08
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

        samples = self._sample_local_region(agent.position, domain)
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

    def _sample_local_region(
        self,
        position: np.ndarray,
        domain: tuple[float, float, float, float],
    ) -> np.ndarray:
        xmin, xmax, ymin, ymax = domain
        r = float(self.local_radius)
        lo = np.array([max(position[0] - r, xmin), max(position[1] - r, ymin)], dtype=float)
        hi = np.array([min(position[0] + r, xmax), min(position[1] + r, ymax)], dtype=float)
        if hi[0] <= lo[0] or hi[1] <= lo[1]:
            return np.empty((0, 2), dtype=float)
        spacing = float(self.grid_spacing)
        spacing = max(spacing, 1e-4)
        # Anchor samples to the world grid instead of each agent's bounding box;
        # the effective spatial resolution is therefore comparable across runs.
        x0 = np.ceil(lo[0] / spacing) * spacing
        y0 = np.ceil(lo[1] / spacing) * spacing
        xs = np.arange(x0, hi[0] + 0.5 * spacing, spacing)
        ys = np.arange(y0, hi[1] + 0.5 * spacing, spacing)
        xx, yy = np.meshgrid(xs, ys)
        samples = np.column_stack([xx.ravel(), yy.ravel()])
        # Hard constraint: only integrate inside B(p_i, R_ℓ).
        mask = np.sum((samples - position[None, :]) ** 2, axis=1) <= r * r + 1e-12
        return samples[mask]
