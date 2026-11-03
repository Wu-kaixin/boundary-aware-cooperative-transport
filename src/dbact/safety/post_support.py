"""Empirical current-scan geometry with explicit received-motion assumptions."""
from __future__ import annotations

import numpy as np

from dbact.sensing.finite_support import support_rows


def build_rows(position, packets, params, timestamp, object_velocity, point_velocities=None):
    """Use the configured speed for coverage and received speed for row demand.

    Neither the three-sigma range allowance nor the received-speed norm is a
    deterministic bound on an unknown world. This path is explicitly empirical.
    """
    velocity = np.asarray(object_velocity, dtype=float)
    if velocity.shape != (2,) or not np.isfinite(velocity).all():
        raise ValueError('Current finite support requires a finite two-dimensional motion input')
    if point_velocities is None:
        received = velocity.reshape(1, 2)
    else:
        received = np.asarray(point_velocities, dtype=float)
        if received.ndim != 2 or received.shape[1] != 2 or not len(received) or not np.isfinite(received).all():
            raise ValueError('Current finite support requires nonempty finite point velocities')
    speed = float(min(params.object_velocity_bound, np.max(np.linalg.norm(received, axis=1))))
    rows = support_rows(position, packets, params, timestamp, params.object_velocity_bound,
        isolated_point_guards=True, noise_in_radius=False)
    return rows, speed
