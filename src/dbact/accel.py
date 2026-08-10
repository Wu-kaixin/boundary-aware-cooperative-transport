"""Optional Numba-accelerated kernels with NumPy fallbacks.

Public helpers keep the same math as the pure-NumPy paths so callers can switch
backends without changing control logic. When numba is unavailable,
``HAS_NUMBA`` is False and the NumPy implementations are used.
"""

from __future__ import annotations

import os
from functools import lru_cache

import numpy as np

try:
    from numba import njit, prange

    HAS_NUMBA = True
except Exception:  # pragma: no cover - optional dependency
    HAS_NUMBA = False

    def njit(*args, **kwargs):  # type: ignore[misc]
        def wrap(fn):
            return fn

        if args and callable(args[0]) and not kwargs:
            return args[0]
        return wrap

    def prange(*args):  # type: ignore[misc]
        return range(*args)


def force_numpy_backend() -> bool:
    """Return True when DBACT_FORCE_NUMPY is set to a truthy value."""
    return os.environ.get("DBACT_FORCE_NUMPY", "").strip().lower() in {"1", "true", "yes", "on"}


def using_numba() -> bool:
    return HAS_NUMBA and not force_numpy_backend()


@njit(cache=True, nogil=True, parallel=True)
def _gaussian_density_numba(
    q: np.ndarray,
    targets: np.ndarray,
    weights: np.ndarray,
    sigma: float,
    base_density: float,
) -> np.ndarray:
    n_q = q.shape[0]
    n_t = targets.shape[0]
    inv_two_sigma2 = 1.0 / (2.0 * sigma * sigma)
    out = np.empty(n_q, dtype=np.float64)
    for i in prange(n_q):
        acc = base_density
        qx = q[i, 0]
        qy = q[i, 1]
        for j in range(n_t):
            dx = qx - targets[j, 0]
            dy = qy - targets[j, 1]
            acc += weights[j] * np.exp(-(dx * dx + dy * dy) * inv_two_sigma2)
        out[i] = acc
    return out


@njit(cache=True, nogil=True, parallel=True)
def _voronoi_owners_numba(samples: np.ndarray, local_positions: np.ndarray) -> np.ndarray:
    n_s = samples.shape[0]
    n_p = local_positions.shape[0]
    owners = np.empty(n_s, dtype=np.int64)
    for i in prange(n_s):
        best = 0
        sx = samples[i, 0]
        sy = samples[i, 1]
        dx0 = sx - local_positions[0, 0]
        dy0 = sy - local_positions[0, 1]
        best_d = dx0 * dx0 + dy0 * dy0
        for j in range(1, n_p):
            dx = sx - local_positions[j, 0]
            dy = sy - local_positions[j, 1]
            d2 = dx * dx + dy * dy
            if d2 < best_d:
                best_d = d2
                best = j
        owners[i] = best
    return owners


def _gaussian_density_numpy(
    q: np.ndarray,
    targets: np.ndarray,
    weights: np.ndarray,
    sigma: float,
    base_density: float,
) -> np.ndarray:
    rho = np.full(q.shape[0], base_density, dtype=float)
    if targets.size == 0:
        return rho
    diff = q[:, None, :] - targets[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    rho += np.sum(weights[None, :] * np.exp(-dist2 / (2.0 * sigma * sigma)), axis=1)
    return rho


def _voronoi_owners_numpy(samples: np.ndarray, local_positions: np.ndarray) -> np.ndarray:
    diff = samples[:, None, :] - local_positions[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    return np.argmin(dist2, axis=1).astype(np.int64, copy=False)


def gaussian_density(
    q: np.ndarray,
    targets: np.ndarray,
    weights: np.ndarray,
    sigma: float,
    base_density: float = 1e-3,
) -> np.ndarray:
    """Evaluate φ(q) = base + Σ w_k exp(-‖q-t_k‖² / (2σ²))."""
    q_arr = np.asarray(q, dtype=np.float64)
    single = False
    if q_arr.ndim == 1:
        q_arr = q_arr[None, :]
        single = True
    targets_arr = np.asarray(targets, dtype=np.float64).reshape(-1, 2)
    weights_arr = np.asarray(weights, dtype=np.float64).reshape(-1)
    if using_numba() and targets_arr.size > 0:
        rho = _gaussian_density_numba(q_arr, targets_arr, weights_arr, float(sigma), float(base_density))
    else:
        rho = _gaussian_density_numpy(q_arr, targets_arr, weights_arr, float(sigma), float(base_density))
    return float(rho[0]) if single else rho


def voronoi_owners(samples: np.ndarray, local_positions: np.ndarray) -> np.ndarray:
    """Return nearest local-agent index for each sample (index 0 = self)."""
    samples_arr = np.asarray(samples, dtype=np.float64).reshape(-1, 2)
    local_arr = np.asarray(local_positions, dtype=np.float64).reshape(-1, 2)
    if len(samples_arr) == 0:
        return np.empty(0, dtype=np.int64)
    if using_numba():
        return _voronoi_owners_numba(samples_arr, local_arr)
    return _voronoi_owners_numpy(samples_arr, local_arr)


@njit(cache=True, nogil=True)
def _nearest_ray_hits_numba(
    origin: np.ndarray,
    directions: np.ndarray,
    edge_a: np.ndarray,
    edge_b: np.ndarray,
    edge_object: np.ndarray,
    max_range: float,
    eps: float,
) -> tuple:
    """Nearest ray–segment hits across all object edges.

    Returns (best_t, best_object_index) with -1 object index on miss.
    """
    n_rays = directions.shape[0]
    n_edges = edge_a.shape[0]
    best_t = np.full(n_rays, np.inf, dtype=np.float64)
    best_obj = np.full(n_rays, -1, dtype=np.int64)
    ox = origin[0]
    oy = origin[1]
    for i in range(n_rays):
        dx = directions[i, 0]
        dy = directions[i, 1]
        for e in range(n_edges):
            ax = edge_a[e, 0]
            ay = edge_a[e, 1]
            bx = edge_b[e, 0]
            by = edge_b[e, 1]
            abx = bx - ax
            aby = by - ay
            det = dx * (-aby) - dy * (-abx)
            if det > -eps and det < eps:
                continue
            rhx = ax - ox
            rhy = ay - oy
            t = (rhx * (-aby) - rhy * (-abx)) / det
            s = (dx * rhy - dy * rhx) / det
            if t < eps or t > max_range or s < -eps or s > 1.0 + eps:
                continue
            if t < best_t[i]:
                best_t[i] = t
                best_obj[i] = edge_object[e]
    return best_t, best_obj


def _nearest_ray_hits_numpy(
    origin: np.ndarray,
    directions: np.ndarray,
    edge_a: np.ndarray,
    edge_b: np.ndarray,
    edge_object: np.ndarray,
    max_range: float,
    eps: float,
) -> tuple[np.ndarray, np.ndarray]:
    n_rays = directions.shape[0]
    best_t = np.full(n_rays, np.inf, dtype=np.float64)
    best_obj = np.full(n_rays, -1, dtype=np.int64)
    ox, oy = float(origin[0]), float(origin[1])
    for i in range(n_rays):
        dx, dy = float(directions[i, 0]), float(directions[i, 1])
        for e in range(edge_a.shape[0]):
            ax, ay = float(edge_a[e, 0]), float(edge_a[e, 1])
            bx, by = float(edge_b[e, 0]), float(edge_b[e, 1])
            abx, aby = bx - ax, by - ay
            det = dx * (-aby) - dy * (-abx)
            if abs(det) < eps:
                continue
            rhx, rhy = ax - ox, ay - oy
            t = (rhx * (-aby) - rhy * (-abx)) / det
            s = (dx * rhy - dy * rhx) / det
            if t < eps or t > max_range or s < -eps or s > 1.0 + eps:
                continue
            if t < best_t[i]:
                best_t[i] = t
                best_obj[i] = int(edge_object[e])
    return best_t, best_obj


def nearest_ray_hits(
    origin: np.ndarray,
    directions: np.ndarray,
    edge_a: np.ndarray,
    edge_b: np.ndarray,
    edge_object: np.ndarray,
    max_range: float,
    eps: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray]:
    """Return nearest hit distance and object index for each ray."""
    origin_arr = np.asarray(origin, dtype=np.float64).reshape(2)
    dirs = np.asarray(directions, dtype=np.float64).reshape(-1, 2)
    a = np.asarray(edge_a, dtype=np.float64).reshape(-1, 2)
    b = np.asarray(edge_b, dtype=np.float64).reshape(-1, 2)
    obj = np.asarray(edge_object, dtype=np.int64).reshape(-1)
    if len(dirs) == 0:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.int64)
    if len(a) == 0:
        return np.full(len(dirs), np.inf, dtype=np.float64), np.full(len(dirs), -1, dtype=np.int64)
    if using_numba():
        return _nearest_ray_hits_numba(origin_arr, dirs, a, b, obj, float(max_range), float(eps))
    return _nearest_ray_hits_numpy(origin_arr, dirs, a, b, obj, float(max_range), float(eps))


@lru_cache(maxsize=1)
def warmup() -> bool:
    """Compile hot kernels once (no-op when numba is unavailable)."""
    if not using_numba():
        return False
    q = np.zeros((4, 2), dtype=np.float64)
    targets = np.array([[0.0, 0.0], [1.0, 0.0]], dtype=np.float64)
    weights = np.ones(2, dtype=np.float64)
    _ = _gaussian_density_numba(q, targets, weights, 0.35, 1e-3)
    _ = _voronoi_owners_numba(q, targets)
    dirs = np.array([[1.0, 0.0], [0.0, 1.0]], dtype=np.float64)
    a = np.array([[0.5, -1.0], [-1.0, 0.5]], dtype=np.float64)
    b = np.array([[0.5, 1.0], [1.0, 0.5]], dtype=np.float64)
    obj = np.array([0, 1], dtype=np.int64)
    _ = _nearest_ray_hits_numba(np.zeros(2, dtype=np.float64), dirs, a, b, obj, 2.0, 1e-9)
    return True


__all__ = [
    "HAS_NUMBA",
    "force_numpy_backend",
    "using_numba",
    "gaussian_density",
    "voronoi_owners",
    "nearest_ray_hits",
    "warmup",
]
