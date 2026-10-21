"""Exact minimum-norm solver for the planar safety QP.

    minimise    ||u - u_nom||^2
    subject to  A u >= b            (half-planes)
                ||u|| <= v_max      (speed cap)

with ``u`` in R^2. Because the decision variable is two-dimensional the
projection onto the feasible set is attained at one of a small, enumerable set of
points: the unconstrained optimum, the perpendicular projection onto a single
constraint boundary, or the intersection of two active boundaries. Enumerating
those candidates and keeping the feasible one of least cost gives the exact
optimum -- there is no iteration count to tune and no tolerance at which the
answer silently becomes approximate.

Keeping this exact matters for the paper claim: the safety filter is a hard QP
with no slack variables, so the reported constraint satisfaction is a property of
the solution rather than an artefact of a penalty weight.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

_EPS = 1e-12


@dataclass
class QPSolution:
    u: np.ndarray
    feasible: bool
    active_rows: int
    detail: str = ""


def solve_min_norm_2d(
    u_nom: np.ndarray,
    A: np.ndarray,
    b: np.ndarray,
    v_max: float,
    tolerance: float = 1e-7,
) -> QPSolution:
    """Exact projection of ``u_nom`` onto ``{A u >= b} ∩ {||u|| <= v_max}``."""
    u0 = np.asarray(u_nom, dtype=float).reshape(2)
    A = np.asarray(A, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1)
    m = len(A)

    def feasible(points: np.ndarray) -> np.ndarray:
        ok = np.linalg.norm(points, axis=1) <= v_max + tolerance
        if m:
            ok &= np.all(points @ A.T >= b[None, :] - tolerance, axis=1)
        return ok

    candidates = [u0[None, :]]

    speed = float(np.linalg.norm(u0))
    if speed > _EPS:
        candidates.append((v_max * u0 / speed)[None, :])

    if m:
        norms2 = np.sum(A * A, axis=1)
        live = norms2 > _EPS
        if np.any(live):
            Al, bl, nl = A[live], b[live], norms2[live]
            # Perpendicular projections onto each constraint boundary.
            candidates.append(u0[None, :] + ((bl - Al @ u0) / nl)[:, None] * Al)

            # Pairwise intersections of constraint boundaries.
            idx_i, idx_j = np.triu_indices(len(Al), k=1)
            if len(idx_i):
                mats = np.stack([Al[idx_i], Al[idx_j]], axis=1)
                rhs = np.stack([bl[idx_i], bl[idx_j]], axis=1)
                dets = mats[:, 0, 0] * mats[:, 1, 1] - mats[:, 0, 1] * mats[:, 1, 0]
                nondegenerate = np.abs(dets) > 1e-10
                if np.any(nondegenerate):
                    mm, rr, dd = mats[nondegenerate], rhs[nondegenerate], dets[nondegenerate]
                    x = (rr[:, 0] * mm[:, 1, 1] - rr[:, 1] * mm[:, 0, 1]) / dd
                    y = (mm[:, 0, 0] * rr[:, 1] - mm[:, 1, 0] * rr[:, 0]) / dd
                    candidates.append(np.column_stack([x, y]))

            # Intersections of each constraint boundary with the speed circle.
            foot = ((bl / nl)[:, None]) * Al
            gap = v_max * v_max - np.sum(foot * foot, axis=1)
            cutting = gap > 0.0
            if np.any(cutting):
                tangent = np.column_stack([-Al[cutting, 1], Al[cutting, 0]])
                tangent = tangent / np.sqrt(nl[cutting])[:, None]
                offset = np.sqrt(gap[cutting])[:, None] * tangent
                candidates.append(foot[cutting] + offset)
                candidates.append(foot[cutting] - offset)

    points = np.vstack(candidates)
    ok = feasible(points)
    if not np.any(ok):
        return QPSolution(u=np.zeros(2), feasible=False, active_rows=m, detail="infeasible")

    viable = points[ok]
    cost = np.sum((viable - u0[None, :]) ** 2, axis=1)
    best = viable[int(np.argmin(cost))]
    return QPSolution(u=best, feasible=True, active_rows=m, detail="optimal")


def solve_min_norm_2d_cvxpy(
    u_nom: np.ndarray,
    A: np.ndarray,
    b: np.ndarray,
    v_max: float,
) -> QPSolution:
    """Same problem solved through cvxpy. Used to cross-check the exact solver."""
    import cvxpy as cp

    u0 = np.asarray(u_nom, dtype=float).reshape(2)
    A = np.asarray(A, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1)

    u = cp.Variable(2)
    constraints = [cp.norm(u, 2) <= v_max]
    if len(A):
        constraints.append(A @ u >= b)
    problem = cp.Problem(cp.Minimize(cp.sum_squares(u - u0)), constraints)
    problem.solve(solver=cp.CLARABEL)
    if problem.status not in {"optimal", "optimal_inaccurate"} or u.value is None:
        return QPSolution(u=np.zeros(2), feasible=False, active_rows=len(A), detail=str(problem.status))
    return QPSolution(u=np.asarray(u.value, dtype=float).reshape(2), feasible=True, active_rows=len(A), detail="optimal")


__all__ = ["QPSolution", "solve_min_norm_2d", "solve_min_norm_2d_cvxpy"]
