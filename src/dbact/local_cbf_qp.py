from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class LocalCBFQP:
    """Local CBF safety filter with an optional small optimization solve.

    When cvxpy is available this solves the local CBF safety problem directly.
    If cvxpy or a compatible solver is unavailable, it falls back to iterative
    half-plane projections so the simulator remains dependency-light.
    """

    d_min: float = 0.28
    gamma: float = 6.0
    max_speed: float = 0.35
    iterations: int = 8
    use_qp: bool = True
    slack_weight: float = 1000.0

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

        if self.use_qp and neighbor_positions:
            solved = self._filter_velocity_qp(u, position, neighbor_positions, neighbor_velocities)
            if solved is not None:
                return solved

        return self._filter_velocity_projection(u, position, neighbor_positions, neighbor_velocities)

    def _filter_velocity_qp(
        self,
        nominal_velocity: np.ndarray,
        position: np.ndarray,
        neighbor_positions: list[np.ndarray],
        neighbor_velocities: list[np.ndarray],
    ) -> np.ndarray | None:
        try:
            import cvxpy as cp  # type: ignore
        except Exception:
            return None

        u_var = cp.Variable(2)
        slack = cp.Variable(len(neighbor_positions), nonneg=True)
        constraints = [cp.norm(u_var, 2) <= self.max_speed]
        for k, (p_j, u_j) in enumerate(zip(neighbor_positions, neighbor_velocities)):
            d = position - np.asarray(p_j, dtype=float).reshape(2)
            h = float(np.dot(d, d) - self.d_min * self.d_min)
            a = 2.0 * d
            b = -self.gamma * h + float(np.dot(a, u_j))
            constraints.append(a @ u_var + slack[k] >= b)
        objective = cp.Minimize(
            cp.sum_squares(u_var - nominal_velocity)
            + self.slack_weight * cp.sum_squares(slack)
        )
        problem = cp.Problem(objective, constraints)
        try:
            problem.solve(warm_start=True)
        except Exception:
            return None
        if problem.status not in {"optimal", "optimal_inaccurate"} or u_var.value is None:
            return None
        return self._cap_speed(np.asarray(u_var.value, dtype=float).reshape(2))

    def _filter_velocity_projection(
        self,
        nominal_velocity: np.ndarray,
        position: np.ndarray,
        neighbor_positions: list[np.ndarray],
        neighbor_velocities: list[np.ndarray],
    ) -> np.ndarray:
        u = nominal_velocity.copy()
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
            u = self._cap_speed(u)
        return u

    def _cap_speed(self, velocity: np.ndarray) -> np.ndarray:
        speed = float(np.linalg.norm(velocity))
        if speed <= self.max_speed:
            return velocity
        return velocity / speed * self.max_speed
