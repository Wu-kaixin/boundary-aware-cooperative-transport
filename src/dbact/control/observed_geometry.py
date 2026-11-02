"""Experimental geometry readouts using permitted observed points only.

These are empirical model checks, not contact or center-of-mass certificates.
No shape identifier, object pose, contact force or evaluator input is accepted.
"""
from __future__ import annotations

import numpy as np


def radial_pivot(points, noise_scale):
    """Fit a circular reference pivot, checking residual and split stability.

    A learned circular model is optional: reject noncircular/ill-conditioned
    observations and retain the original reference point. Three-noise-scale
    split stability is an engineering gate, not a statistical confidence bound.
    """
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 16:
        return None, "insufficient_points", float("nan")
    if not np.isfinite(p).all() or not np.isfinite(noise_scale) or noise_scale <= 0:
        raise ValueError("Finite points and positive noise scale required")
    anchor = p.mean(axis=0)
    x = p - anchor

    def fit(rows):
        design = np.column_stack((2 * rows, np.ones(len(rows))))
        sol, _, rank, _ = np.linalg.lstsq(design, np.sum(rows**2, axis=1), rcond=1e-3)
        return sol[:2] if rank == 3 else None

    center = fit(x)
    if center is None:
        return None, "rank_rejected", float("nan")
    radii = np.linalg.norm(x-center, axis=1)
    rms = float(np.sqrt(np.mean((radii-np.mean(radii))**2)))
    if rms > noise_scale:
        return None, "radial_residual_rejected", rms
    halves = [fit(x[j::2]) for j in (0, 1)]
    if any(h is None for h in halves) or np.linalg.norm(halves[0]-halves[1]) > 3*noise_scale:
        return None, "split_rejected", rms
    # A tiny observed arc cannot locate a center reliably, even with low RMS.
    directions = (x-center) / np.maximum(radii[:, None], 1e-12)
    eig = np.linalg.eigvalsh(directions.T @ directions)
    if eig[0] / max(eig[-1], 1e-12) <= 1e-3:
        return None, "direction_rank_rejected", rms
    return anchor+center, "accepted", rms


def patch_distance(scan, position, point, normal, object_id, radius, tolerance):
    """Estimate distance from own fresh patch without selecting its closest outlier.

    Fit a quadratic in the mapped tangent frame. Require interpolation (points
    on both sides), five points, consistent observed normals and an old sensor
    residual tolerance. Retain the mapped normal; neither QP nor map is changed.
    """
    tangent = np.array([-normal[1], normal[0]])
    delta = scan.points - position
    use = ((scan.object_ids == object_id)
           & (np.linalg.norm(scan.points-point, axis=1) <= radius)
           & (scan.normals @ normal >= .5))
    if np.count_nonzero(use) < 5:
        return None, "insufficient_patch", 0
    x, y = delta[use] @ tangent, delta[use] @ normal
    if x.min() >= 0 or x.max() <= 0:
        return None, "no_interpolation", len(x)
    scale = max(float(np.max(np.abs(x))), 1e-12)
    z = x / scale
    design = np.column_stack((np.ones(len(x)), z, z*z))
    fit, _, rank, _ = np.linalg.lstsq(design, y, rcond=1e-3)
    rms = float(np.sqrt(np.mean((design @ fit-y)**2)))
    if rank < 3 or rms > tolerance:
        return None, "patch_fit_rejected", len(x)
    distance = -float(fit[0])
    if not np.isfinite(distance) or distance <= 0:
        return None, "invalid_distance", len(x)
    return distance, "accepted", len(x)
