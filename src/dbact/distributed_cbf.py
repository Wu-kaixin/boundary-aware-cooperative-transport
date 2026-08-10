from __future__ import annotations

from dataclasses import dataclass, field

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
    iterations: int = 32
    use_qp: bool = True
    robot_radius: float = 0.12
    alpha_object: float = 4.0
    boundary_error_margin: float = 0.0
    object_speed_bound: float = 0.0
    contact_allowance: float = 0.0
    feasibility_tolerance: float = 1e-7
    _qp_cache: dict = field(default_factory=dict, init=False, repr=False)

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
                self.last_feasible = True
                self.last_solver = "qp"
                return solved

        projected = self._filter_velocity_projection(
            u, position, neighbor_positions, boundary_points, boundary_normals
        )
        constraints_meta = self._build_constraints(
            position, neighbor_positions, boundary_points, boundary_normals
        )
        self.last_feasible = self._is_feasible(projected, constraints_meta)
        self.last_solver = "projection"
        if not self.last_feasible and self._is_feasible(np.zeros(2), constraints_meta):
            # For the static safe-set assumptions used in Proposition 2, u=0
            # is a constructive feasible point. Never return a constraint-
            # violating numerical projection when that certificate is available.
            self.last_feasible = True
            self.last_solver = "certified_zero"
            return np.zeros(2, dtype=float)
        return projected

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
        h = float(
            np.dot(n, position - b_hat)
            - self.robot_radius
            - self.boundary_error_margin
            + self.contact_allowance
        )
        # Robust moving-boundary form:
        #   nᵀu - v_O,max + αh >= 0
        # where boundary_error_margin links estimation error to the safe set.
        return n, self.object_speed_bound - self.alpha_object * h

    def _build_constraints(
        self,
        position: np.ndarray,
        neighbor_positions: list[np.ndarray],
        boundary_points: list[np.ndarray],
        boundary_normals: list[np.ndarray],
    ) -> list[tuple[np.ndarray, float]]:
        constraints_meta = [
            self._robot_constraint(position, p_j) for p_j in neighbor_positions
        ]
        constraints_meta.extend(
            self._object_constraint(position, b_pt, b_n)
            for b_pt, b_n in zip(boundary_points, boundary_normals)
        )
        return constraints_meta

    @property
    def component_limit(self) -> float:
        """Axis-aligned input bound whose box lies inside the old speed ball."""
        return float(self.max_speed) / np.sqrt(2.0)

    def _is_feasible(
        self,
        velocity: np.ndarray,
        constraints_meta: list[tuple[np.ndarray, float]],
    ) -> bool:
        u = np.asarray(velocity, dtype=float).reshape(2)
        if np.any(np.abs(u) > self.component_limit + self.feasibility_tolerance):
            return False
        return all(
            float(np.dot(a, u)) + self.feasibility_tolerance >= float(b)
            for a, b in constraints_meta
        )

    def _get_qp_bundle(self, n_constraints: int):
        """Reuse a Parameterized cvxpy Problem for a fixed constraint count."""
        cached = self._qp_cache.get(n_constraints)
        if cached is not None:
            return cached
        try:
            import cvxpy as cp  # type: ignore
        except Exception:
            return None

        u_var = cp.Variable(2)
        u_nom = cp.Parameter(2)
        a_mat = cp.Parameter((n_constraints, 2))
        b_vec = cp.Parameter(n_constraints)
        limit = self.component_limit
        constraints = [u_var >= -limit, u_var <= limit, a_mat @ u_var >= b_vec]
        objective = cp.Minimize(cp.sum_squares(u_var - u_nom))
        problem = cp.Problem(objective, constraints)
        bundle = {
            "cp": cp,
            "u_var": u_var,
            "u_nom": u_nom,
            "a_mat": a_mat,
            "b_vec": b_vec,
            "problem": problem,
        }
        self._qp_cache[n_constraints] = bundle
        return bundle

    def _filter_velocity_qp(
        self,
        nominal_velocity: np.ndarray,
        position: np.ndarray,
        neighbor_positions: list[np.ndarray],
        boundary_points: list[np.ndarray],
        boundary_normals: list[np.ndarray],
    ) -> np.ndarray | None:
        constraints_meta = self._build_constraints(
            position, neighbor_positions, boundary_points, boundary_normals
        )
        if not constraints_meta:
            return self._clip_box(nominal_velocity)

        bundle = self._get_qp_bundle(len(constraints_meta))
        if bundle is None:
            return None

        a_rows = np.vstack([a for a, _ in constraints_meta]).astype(float, copy=False)
        b_vals = np.asarray([b for _, b in constraints_meta], dtype=float)
        bundle["u_nom"].value = np.asarray(nominal_velocity, dtype=float).reshape(2)
        bundle["a_mat"].value = a_rows
        bundle["b_vec"].value = b_vals
        problem = bundle["problem"]
        try:
            problem.solve(solver="OSQP", warm_start=True)
        except Exception:
            try:
                problem.solve(warm_start=True)
            except Exception:
                return None
        u_var = bundle["u_var"]
        if problem.status not in {"optimal", "optimal_inaccurate"} or u_var.value is None:
            return None
        candidate = self._clip_box(np.asarray(u_var.value, dtype=float).reshape(2))
        if not self._is_feasible(candidate, constraints_meta):
            return None
        return candidate

    def _filter_velocity_projection(
        self,
        nominal_velocity: np.ndarray,
        position: np.ndarray,
        neighbor_positions: list[np.ndarray],
        boundary_points: list[np.ndarray],
        boundary_normals: list[np.ndarray],
    ) -> np.ndarray:
        u = nominal_velocity.copy()
        constraints_meta = self._build_constraints(
            position, neighbor_positions, boundary_points, boundary_normals
        )

        for _ in range(self.iterations):
            for a, b in constraints_meta:
                denom = float(np.dot(a, a))
                if denom < 1e-12:
                    continue
                violation = b - float(np.dot(a, u))
                if violation > 0.0:
                    u = u + (violation / denom) * a
            u = self._clip_box(u)
        return self._clip_box(u)

    def _clip_box(self, velocity: np.ndarray) -> np.ndarray:
        limit = self.component_limit
        return np.clip(np.asarray(velocity, dtype=float).reshape(2), -limit, limit)

    def _cap_speed(self, velocity: np.ndarray) -> np.ndarray:
        """Backward-compatible helper; hard-QP code uses the box projection."""
        return self._clip_box(velocity)


# Backward-compatible alias used by older imports / MAS adapter.
LocalCBFQP = DistributedCBFQP
