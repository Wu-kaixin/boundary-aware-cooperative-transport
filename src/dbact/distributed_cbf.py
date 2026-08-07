from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class DistributedCBFQP:
    """Distributed responsibility-splitting CBF safety filter.

    Robot–robot (pairwise half-responsibility):
        h_ij = ‖p_i − p_j‖² − d_min²
        2(p_i − p_j)ᵀ u_i + (γ/2) h_ij ≥ 0

    Each robot needs only neighbor positions, not neighbor control inputs.
    Summing i and j constraints recovers ḣ_ij + γ h_ij ≥ 0.

    Object–boundary (allow contact, forbid penetration):
        h_iO = n̂ᵀ(p_i − b̂) − r_r
        n̂ᵀ u_i + α(h_iO) ≥ 0
    """

    d_min: float = 0.28
    gamma: float = 6.0
    max_speed: float = 0.35
    iterations: int = 8
    use_qp: bool = True
    slack_weight: float = 1000.0
    robot_radius: float = 0.12
    alpha_object: float = 4.0

    def filter_velocity(
        self,
        position: np.ndarray,
        nominal_velocity: np.ndarray,
        neighbor_positions: list[np.ndarray],
        neighbor_velocities: list[np.ndarray] | None = None,
        boundary_points: list[np.ndarray] | None = None,
        boundary_normals: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        del neighbor_velocities  # intentionally unused: fully decentralized
        u = np.asarray(nominal_velocity, dtype=float).reshape(2).copy()
        boundary_points = boundary_points or []
        boundary_normals = boundary_normals or []

        if self.use_qp and (neighbor_positions or boundary_points):
            solved = self._filter_velocity_qp(u, position, neighbor_positions, boundary_points, boundary_normals)
            if solved is not None:
                return solved

        return self._filter_velocity_projection(u, position, neighbor_positions, boundary_points, boundary_normals)

    def _robot_constraint(self, position: np.ndarray, p_j: np.ndarray) -> tuple[np.ndarray, float]:
        d = position - np.asarray(p_j, dtype=float).reshape(2)
        h = float(np.dot(d, d) - self.d_min * self.d_min)
        a = 2.0 * d
        # 2(p_i-p_j)ᵀ u_i >= -(γ/2) h  ⇔  aᵀ u >= b with b = -(γ/2)h
        b = -0.5 * self.gamma * h
        return a, b

    def _object_constraint(
        self,
        position: np.ndarray,
        boundary_point: np.ndarray,
        boundary_normal: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        n = np.asarray(boundary_normal, dtype=float).reshape(2)
        b_hat = np.asarray(boundary_point, dtype=float).reshape(2)
        nrm = float(np.linalg.norm(n))
        if nrm < 1e-9:
            return np.zeros(2, dtype=float), 0.0
        n = n / nrm
        h = float(np.dot(n, position - b_hat) - self.robot_radius)
        # nᵀ u + α(h) >= 0  ⇔  nᵀ u >= -α h
        return n, -self.alpha_object * h

    def _filter_velocity_qp(
        self,
        nominal_velocity: np.ndarray,
        position: np.ndarray,
        neighbor_positions: list[np.ndarray],
        boundary_points: list[np.ndarray],
        boundary_normals: list[np.ndarray],
    ) -> np.ndarray | None:
        try:
            import cvxpy as cp  # type: ignore
        except Exception:
            return None

        constraints_meta: list[tuple[np.ndarray, float]] = []
        for p_j in neighbor_positions:
            constraints_meta.append(self._robot_constraint(position, p_j))
        for b_pt, b_n in zip(boundary_points, boundary_normals):
            constraints_meta.append(self._object_constraint(position, b_pt, b_n))
        if not constraints_meta:
            return self._cap_speed(nominal_velocity)

        u_var = cp.Variable(2)
        slack = cp.Variable(len(constraints_meta), nonneg=True)
        constraints = [cp.norm(u_var, 2) <= self.max_speed]
        for k, (a, b) in enumerate(constraints_meta):
            constraints.append(a @ u_var + slack[k] >= b)
        objective = cp.Minimize(
            cp.sum_squares(u_var - nominal_velocity) + self.slack_weight * cp.sum_squares(slack)
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
        boundary_points: list[np.ndarray],
        boundary_normals: list[np.ndarray],
    ) -> np.ndarray:
        u = nominal_velocity.copy()
        constraints_meta: list[tuple[np.ndarray, float]] = []
        for p_j in neighbor_positions:
            constraints_meta.append(self._robot_constraint(position, p_j))
        for b_pt, b_n in zip(boundary_points, boundary_normals):
            constraints_meta.append(self._object_constraint(position, b_pt, b_n))

        for _ in range(self.iterations):
            for a, b in constraints_meta:
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


# Backward-compatible alias used by older imports / MAS adapter.
LocalCBFQP = DistributedCBFQP
