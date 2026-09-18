"""Independent static-deployment diagnostics, extracted from the archived audit.

Numerical observer outputs are experimental metrics, not exact theorem quantities.
"""

from __future__ import annotations

import math

import numpy as np

from numpy.polynomial.legendre import leggauss

from scipy.integrate import cumulative_trapezoid, quad

from scipy.special import erf

class UniformOffsetObserver:
    """Paper-style truncated cell integrals for a uniform-offset Gaussian edge density.

    Oracle map used by the controller is a discrete arc-length sample of the same
    offset curve; this observer integrates the continuous edge measure analytically
    (erf line integrals) and does not reuse the controller 20x20 grid.
    """

    def __init__(self, domain, radius, sigma, floor, cage_offset, ntheta=256, nradial=16):
        self.domain = np.asarray(domain, dtype=float)
        self.R = float(radius)
        self.sigma = float(sigma)
        self.floor = float(floor)
        self.cage = float(cage_offset)
        theta = 2 * np.pi * (np.arange(ntheta) + 0.5) / ntheta
        self.unit = np.column_stack((np.cos(theta), np.sin(theta)))
        self.dtheta = 2 * np.pi / ntheta
        z, w = leggauss(nradial)
        self.radial_nodes, self.radial_weights = (z + 1) / 2, w / 2

    def edges(self, vertices):
        edges = np.roll(vertices, -1, axis=0) - vertices
        lengths = np.linalg.norm(edges, axis=1)
        tangent = edges / lengths[:, None]
        normal = np.column_stack((tangent[:, 1], -tangent[:, 0]))
        start = vertices + self.cage * normal
        return start, tangent, lengths

    def density(self, queries, vertices):
        queries = np.asarray(queries, dtype=float).reshape(-1, 2)
        start, tangent, lengths = self.edges(vertices)
        result = np.full(len(queries), self.floor)
        scale = math.sqrt(2) * self.sigma
        for p, u, length in zip(start, tangent, lengths):
            delta = queries - p
            along = delta @ u
            perpendicular2 = np.maximum(np.sum(delta * delta, axis=1) - along * along, 0.0)
            result += self.sigma * math.sqrt(np.pi / 2) * np.exp(
                -perpendicular2 / (2 * self.sigma**2)
            ) * (erf((length - along) / scale) + erf(along / scale))
        return result

    def total_mass(self, vertices):
        xmin, xmax, ymin, ymax = self.domain
        answer = self.floor * (xmax - xmin) * (ymax - ymin)
        scale = math.sqrt(2) * self.sigma
        start, tangent, lengths = self.edges(vertices)
        for p, u, length in zip(start, tangent, lengths):

            def integrand(s, p=p, u=u):
                x, y = p + s * u
                return (
                    np.pi
                    * self.sigma**2
                    / 2
                    * (erf((xmax - x) / scale) - erf((xmin - x) / scale))
                    * (erf((ymax - y) / scale) - erf((ymin - y) / scale))
                )

            answer += quad(integrand, 0, length, epsabs=1e-11, epsrel=1e-11)[0]
        return float(answer)

    def evaluate(self, positions, vertices):
        p = np.asarray(positions, dtype=float).reshape(-1, 2)
        n, nt = len(p), len(self.unit)
        reach = np.full((n, nt), self.R)
        xmin, xmax, ymin, ymax = self.domain
        if (
            np.any(p[:, 0] < xmin - 1e-10)
            or np.any(p[:, 0] > xmax + 1e-10)
            or np.any(p[:, 1] < ymin - 1e-10)
            or np.any(p[:, 1] > ymax + 1e-10)
        ):
            raise ValueError("Observer requires sites inside D")
        for axis, low, high in [(0, xmin, xmax), (1, ymin, ymax)]:
            v = self.unit[:, axis]
            wall = np.where(
                v[None, :] > 0,
                (high - p[:, axis, None]) / np.maximum(v[None, :], 1e-30),
                (p[:, axis, None] - low) / np.maximum(-v[None, :], 1e-30),
            )
            reach = np.minimum(reach, wall)
        for i in range(n):
            difference = p - p[i]
            squared = np.sum(difference * difference, axis=1)
            dot = difference @ self.unit.T
            denominator = 2 * np.maximum(dot, 1e-30)
            cap = squared[:, None] / denominator
            cap[dot <= 1e-14] = np.inf
            cap[i] = np.inf
            reach[i] = np.minimum(reach[i], np.min(cap, axis=0))
        reach = np.maximum(reach, 0.0)
        r = reach[:, :, None] * self.radial_nodes
        q = p[:, None, None, :] + r[:, :, :, None] * self.unit[None, :, None, :]
        phi = self.density(q.reshape(-1, 2), vertices).reshape(r.shape)
        w = self.dtheta * reach[:, :, None] * self.radial_weights * r
        weighted = phi * w
        mass = np.sum(weighted, axis=(1, 2))
        shift = np.sum(
            weighted[:, :, :, None] * r[:, :, :, None] * self.unit[None, :, None, :],
            axis=(1, 2),
        )
        # Guard empty-mass cells (should be rare with phi0 > 0).
        safe = np.maximum(mass, 1e-30)
        centroid = p + shift / safe[:, None]
        gradient = -2 * shift
        H = self.R**2 * self.total_mass(vertices) + np.sum((r * r - self.R**2) * weighted)
        return dict(
            mass=mass,
            centroid=centroid,
            gradient=gradient,
            gradient2=float(np.sum(gradient * gradient)),
            centroid2=float(np.sum((centroid - p) ** 2)),
            H=float(H),
        )

DISSIPATION_TOL_ABS = 1e-6

DISSIPATION_TOL_REL = 1e-6

OBSERVER_SLACK_DUMP = 1e-3

PROJECTION_RESIDUAL_TOL = 1e-6

def dissipation_rhs(dt: float, gradient: np.ndarray, mass: np.ndarray, U: np.ndarray) -> float:
    g_dot = float(np.sum(gradient * U))
    mass_term = float(dt * dt * np.sum(mass * np.linalg.norm(U, axis=1) ** 2))
    return float(dt * g_dot + mass_term)

def check_projection_step(
    rec,
    filter_results,
    umax: float,
    rho: float,
) -> dict:
    """Offline projection diagnostics for one control instant."""
    statuses = list(rec.solver_status)
    n_agents = len(statuses)
    speed_ok = bool(np.all(rec.speed <= umax + 1e-9))
    agent_ok = bool(np.all(rec.agent_residual_min >= -PROJECTION_RESIDUAL_TOL))
    wall_ok = bool(np.all(rec.wall_residual_min >= -PROJECTION_RESIDUAL_TOL))
    object_ok = bool(np.all(rec.object_residual_min >= -PROJECTION_RESIDUAL_TOL))
    zero_flags = [bool(r.zero_input_feasible) for r in filter_results]
    zero_with_rho = [bool(getattr(r, "zero_input_feasible_with_rho", True)) for r in filter_results]
    inside_band = [bool(getattr(r, "inside_margin_band", False)) for r in filter_results]
    mods = [float(r.modification) for r in filter_results]
    return {
        "statuses": statuses,
        "speed_within_umax": speed_ok,
        "agent_residuals_ok": agent_ok,
        "wall_residuals_ok": wall_ok,
        "object_residuals_ok": object_ok,
        "zero_input_feasible": zero_flags,
        "zero_input_all_feasible": all(zero_flags),
        "zero_input_feasible_with_rho": zero_with_rho,
        "zero_input_with_rho_all_feasible": all(zero_with_rho),
        "inside_margin_band": inside_band,
        "inside_margin_band_count": int(sum(1 for z in inside_band if z)),
        "modification_max": float(max(mods)) if mods else 0.0,
        "optimal_count": int(sum(1 for s in statuses if s == "optimal")),
        "relaxed_margin_count": int(sum(1 for s in statuses if s == "relaxed_margin")),
        "scaled_barrier_count": int(sum(1 for s in statuses if s == "scaled_barrier")),
        "n_agents": n_agents,
        "rho": rho,
    }
