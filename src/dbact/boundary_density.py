from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .accel import gaussian_density
from .types import BoundaryObservation


@dataclass
class DensityPoint:
    target: np.ndarray
    weight: float
    object_id: str


class BoundaryAwareDensity:
    """Boundary-measure-induced Gaussian density field.

    Continuous form (paper):
        φ_i(q,t) = φ_0 + ∫_{Γ̂_i(t)} w_i(b,t) K_σ(q - [b + d_c n(b)]) ds

    Discrete approximation:
        φ_i(q,t) = φ_0 + Σ_k Δs_k c_k e^{-λ(t-t_k)} (1 + κ g_k) K_σ(q - ξ_k)
        ξ_k = b_k + d_c n_k

    Density total mass grows with estimated boundary length, so larger / more
    complex objects naturally request more allocation without knowing radius,
    perimeter, or a predefined team size.
    """

    def __init__(self, density_points: list[DensityPoint], sigma: float = 0.35, base_density: float = 1e-3):
        self.density_points = density_points
        self.sigma = float(sigma)
        self.base_density = float(base_density)
        if density_points:
            self._targets = np.vstack([p.target for p in density_points]).astype(float, copy=False)
            self._weights = np.asarray([p.weight for p in density_points], dtype=float)
        else:
            self._targets = np.empty((0, 2), dtype=float)
            self._weights = np.empty(0, dtype=float)

    @classmethod
    def from_observations(
        cls,
        observations: list[BoundaryObservation],
        cage_offset: float,
        sigma: float,
        base_density: float = 1e-3,
        timestamp: float | None = None,
        decay_lambda: float = 0.35,
        gap_gain: float = 1.0,
        age_weights: dict[int, float] | list[float] | None = None,
    ) -> "BoundaryAwareDensity":
        points: list[DensityPoint] = []
        t_now = float(timestamp) if timestamp is not None else None
        for idx, obs in enumerate(observations):
            target = obs.point + cage_offset * obs.normal
            if age_weights is not None:
                if isinstance(age_weights, dict):
                    age = float(age_weights.get(idx, 1.0))
                else:
                    age = float(age_weights[idx]) if idx < len(age_weights) else 1.0
            elif t_now is not None:
                age = float(np.exp(-decay_lambda * max(0.0, t_now - obs.timestamp)))
            else:
                age = 1.0
            gap = float(max(0.0, obs.gap_score))
            weight = float(obs.arc_length) * float(obs.confidence) * age * (1.0 + gap_gain * gap)
            points.append(DensityPoint(target=target, weight=max(weight, 0.0), object_id=obs.object_id))
        return cls(points, sigma=sigma, base_density=base_density)

    @classmethod
    def from_targets(
        cls,
        targets: list[np.ndarray] | np.ndarray,
        sigma: float,
        weights: list[float] | np.ndarray | None = None,
        object_id: str = "target_region",
        base_density: float = 1e-3,
    ) -> "BoundaryAwareDensity":
        arr = np.asarray(targets, dtype=float).reshape(-1, 2)
        if weights is None:
            weight_arr = np.ones(len(arr), dtype=float)
        else:
            weight_arr = np.asarray(weights, dtype=float).reshape(len(arr))
        points = [
            DensityPoint(target=target.copy(), weight=float(weight), object_id=object_id)
            for target, weight in zip(arr, weight_arr)
        ]
        return cls(points, sigma=sigma, base_density=base_density)

    @property
    def targets(self) -> np.ndarray:
        return self._targets

    def total_mass(self) -> float:
        if self._weights.size == 0:
            return 0.0
        return float(np.sum(self._weights))

    def __call__(self, q: np.ndarray) -> np.ndarray:
        return gaussian_density(q, self._targets, self._weights, self.sigma, self.base_density)

    def weighted_centroid(self, samples: np.ndarray) -> np.ndarray | None:
        if len(samples) == 0:
            return None
        samples_arr = np.asarray(samples, dtype=float).reshape(-1, 2)
        weights = np.atleast_1d(self(samples_arr))
        total = float(np.sum(weights))
        if total <= 1e-12:
            return None
        return np.sum(samples_arr * weights[:, None], axis=0) / total
