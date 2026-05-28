from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class LocalCBFQP:
    """Lightweight local CBF safety filter.

    This module solves a small 2D projection problem using iterative half-plane
    projections. It follows the CBF-QP idea but avoids requiring cvxpy. Replace
    this class with a true QP solver if formal optimality is required.
    """

    d_min: float = 0.28
    gamma: float = 6.0
    max_speed: float = 0.35
    iterations: int = 8

    def filter_velocity(
        self,
        position: np.ndarray,
        nominal_velocity: np.ndarray,
        neighbor_positions: list[np.ndarray],
        neighbor_velocities: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        u = np.asarray(nominal_velocity, dtype=float).reshape(2).copy()
        if neighbor_velocities is None:
            neighbor_velocities = [np.zeros(2, dtype=float) for _ in neighbor_positions]

        for _ in range(self.iterations):
            for p_j, u_j in zip(neighbor_positions, neighbor_velocities):
                d = position - np.asarray(p_j, dtype=float).reshape(2)
                h = float(np.dot(d, d) - self.d_min * self.d_min)
                a = 2.0 * d
                b = -self.gamma * h + float(np.dot(a, u_j))
                denom = float(np.dot(a, a))
                if denom < 1e-12:
                    continue
                violation = b - float(np.dot(a, u))
                if violation > 0.0:
                    u = u + (violation / denom) * a
            speed = float(np.linalg.norm(u))
            if speed > self.max_speed:
                u = u / speed * self.max_speed
        return u
