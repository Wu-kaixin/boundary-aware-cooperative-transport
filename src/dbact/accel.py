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
    return True


__all__ = [
    "HAS_NUMBA",
    "force_numpy_backend",
    "using_numba",
    "gaussian_density",
    "voronoi_owners",
    "warmup",
]
