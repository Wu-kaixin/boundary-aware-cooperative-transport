from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .types import BoundaryObservation


@dataclass
class DensityPoint:
    target: np.ndarray
    weight: float
    object_id: str


class BoundaryAwareDensity:
    """Gaussian density field centered at boundary-offset cage targets."""

    def __init__(self, density_points: list[DensityPoint], sigma: float = 0.35, base_density: float = 1e-3):
        self.density_points = density_points
        self.sigma = float(sigma)
        self.base_density = float(base_density)

    @classmethod
    def from_observations(
        cls,
        observations: list[BoundaryObservation],
        cage_offset: float,
        sigma: float,
        base_density: float = 1e-3,
    ) -> "BoundaryAwareDensity":
        points: list[DensityPoint] = []
        for obs in observations:
            target = obs.point + cage_offset * obs.normal
            points.append(DensityPoint(target=target, weight=float(obs.confidence), object_id=obs.object_id))
        return cls(points, sigma=sigma, base_density=base_density)

    @property
    def targets(self) -> np.ndarray:
        if not self.density_points:
            return np.empty((0, 2), dtype=float)
        return np.vstack([p.target for p in self.density_points])

    def __call__(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        single = False
        if q.ndim == 1:
            q = q[None, :]
            single = True
        rho = np.full(q.shape[0], self.base_density, dtype=float)
        if self.density_points:
            targets = self.targets
            weights = np.asarray([p.weight for p in self.density_points], dtype=float)
            diff = q[:, None, :] - targets[None, :, :]
            dist2 = np.sum(diff * diff, axis=2)
            rho += np.sum(weights[None, :] * np.exp(-dist2 / (2.0 * self.sigma * self.sigma)), axis=1)
        return rho[0] if single else rho

    def weighted_centroid(self, samples: np.ndarray) -> np.ndarray | None:
        if len(samples) == 0:
            return None
        weights = self(samples)
        total = float(np.sum(weights))
        if total <= 1e-12:
            return None
        return np.sum(samples * weights[:, None], axis=0) / total
