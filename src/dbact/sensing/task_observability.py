"""Experimental translation-only task certificate; not wired into live control.

Rows are observed fixed correspondences/normals. No truth geometry is accepted.
Rotation, association errors and accumulated map drift are outside this model.
"""

import numpy as np


def directional_certificate(rows, direction, rank_rtol=1e-10):
    a = np.asarray(rows, dtype=float)
    d = np.asarray(direction, dtype=float)
    if a.ndim != 2 or a.shape[1] != 2 or d.shape != (2,):
        raise ValueError("Expected an m-by-2 observation matrix and a 2-vector")
    if not np.isfinite(a).all() or not np.isfinite(d).all() or not 0 < rank_rtol < 1:
        raise ValueError("Finite observations and 0 < rank_rtol < 1 required")
    if not np.isclose(np.linalg.norm(d), 1.0, atol=1e-12, rtol=0):
        raise ValueError("Task direction must be a unit vector")
    u, s, vt = np.linalg.svd(a, full_matrices=False)
    keep = s > rank_rtol * (s[0] if len(s) and s[0] > 0 else 1.0)
    basis = vt[keep]
    residual = d - basis.T @ (basis @ d)
    missing = float(np.linalg.norm(residual))
    identifiable = missing <= rank_rtol
    weights = (u[:, keep] @ ((basis @ d) / s[keep])) if identifiable else None
    return {
        "identifiable": identifiable,
        "rank": int(keep.sum()),
        "null_component": missing,
        "amplification": float(np.linalg.norm(weights)) if identifiable else None,
        "weights": weights,
        "rank_rtol": rank_rtol,
    }


def select_additional_normal(rows, candidates, direction):
    """Choose one already available sensing opportunity; no motion/path claim."""
    candidates = np.asarray(candidates, dtype=float)
    if candidates.ndim != 2 or candidates.shape[1] != 2 or len(candidates) == 0:
        raise ValueError("At least one 2D candidate is required")
    reports = [
        directional_certificate(np.vstack((rows, c)), direction) for c in candidates
    ]
    scores = [
        (
            not r["identifiable"],
            r["null_component"] if not r["identifiable"] else r["amplification"],
            i,
        )
        for i, r in enumerate(reports)
    ]
    index = min(scores)[2]
    return index, reports[index]
