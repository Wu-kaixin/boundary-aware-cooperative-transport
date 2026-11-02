"""Initial observed SE(2) reference and a conditional unknown-center stop test.

No cargo pose, shape, task start, or contact truth enters these interfaces.
The workspace bound is exact for the *estimated* rigid transform when the
initial area centroid lies in the workspace. It is not a registration-error
certificate. In particular partial views and rotational symmetry can leave
the fitted pose ambiguous. Acquisition stationarity must be checked separately.
"""
from __future__ import annotations

import numpy as np

from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from dbact.types import BoundaryView


def workspace_vertices(domain):
    xmin, xmax, ymin, ymax = domain
    if not np.isfinite(domain).all() or xmin >= xmax or ymin >= ymax:
        raise ValueError("A finite nonempty workspace is required")
    return np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])


def center_error_bound(displacement, angle, point, center_vertices, goal):
    """Maximum norm of an affine error over a convex polygon, at its vertices."""
    error = np.asarray(displacement)-np.asarray(goal)
    errors = error+(np.asarray(center_vertices)-np.asarray(point))@(rotation(angle)-np.eye(2)).T
    return float(np.linalg.norm(errors, axis=1).max())


def geometric_median(values):
    """Euclidean geometric median; no preferred coordinate axes."""
    values = np.asarray(values, dtype=float).reshape(-1, 2)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError("Finite nonempty messages required")
    point = values.mean(axis=0)
    for _ in range(100):
        weights = 1/np.maximum(np.linalg.norm(values-point, axis=1), 1e-10)
        updated = np.sum(values*weights[:, None], axis=0)/weights.sum()
        if np.linalg.norm(updated-point) < 1e-10:
            return updated
        point = updated
    return point


class TaskReferenceTracker(ObservedReferenceTracker):
    """A zero-epoch transform read at a declared common point, never at a centroid."""

    def __init__(self, *, reference_point, **kwargs):
        super().__init__(**kwargs)
        self.reference_point = np.asarray(reference_point, dtype=float).reshape(2).copy()
        if not np.isfinite(self.reference_point).all():
            raise ValueError("Finite task reference point required")

    def initialize_initial(self, scan, timestamp):
        super().initialize(scan, np.zeros(2), timestamp)
        if self.initialized:
            # Reuse the exact reference-point conversion and velocity difference
            # in the base tracker; no change to its registration fit.
            self.pivot_shift = self.reference_point-self.anchor
            self.pivot_status = "accepted"

    def initialization_record(self):
        record = super().initialization_record()
        record.update(reference_kind="common_workspace_point_not_object_center",
                      common_reference_point=self.reference_point.tolist(),
                      zero_epoch_assumption="stationary acquisition before robot contact",
                      registration_certified=False)
        return record

    def _geometry_rejection(self, normals, matched_points):
        # Remove the part of rotation that can be explained by translation.
        # An ideal circular patch supplies zero independent yaw information.
        # This three-noise-scale test is empirical, not a pose-error bound.
        turn = normals[:, 1]*matched_points[:, 0]-normals[:, 0]*matched_points[:, 1]
        independent = turn-normals@np.linalg.lstsq(normals, turn, rcond=1e-3)[0]
        if np.sqrt(np.mean(independent**2)) <= 3*self.range_noise_std:
            return 'task_rotation_unobservable'
        return None


class InitialReferenceCapture:
    """Bounded own/one-hop static packets; never grow a template after release."""

    def __init__(self, *, noise, max_points=4096):
        if not np.isfinite(noise) or noise <= 0:
            raise ValueError("Positive observation noise scale required")
        self.noise, self.max_points = float(noise), int(max_points)
        self.cells = {}
        self.frozen = False

    def add(self, agent_id, scan):
        if self.frozen:
            raise RuntimeError("Initial reference already frozen")
        cells = self.cells.setdefault(agent_id, {})
        for point, normal, oid in zip(scan.points, scan.normals, scan.object_ids):
            if len(cells) >= self.max_points:
                break
            key = tuple(np.floor(point/(2*self.noise)).astype(int))
            cells.setdefault(key, (point.copy(), normal.copy(), str(oid)))

    def view(self, agent_id):
        rows = list(self.cells.get(agent_id, {}).values())
        if not rows:
            return BoundaryView.empty()
        return BoundaryView(np.array([r[0] for r in rows]), np.array([r[1] for r in rows]),
                            np.ones(len(rows)), np.full(len(rows), 2*self.noise),
                            np.array([r[2] for r in rows], dtype='<U32'))

    def supported(self, agent_id):
        scan = self.view(agent_id)
        if len(scan) < 32 or ObservedReferenceTracker._conditioning(scan.normals) <= 1e-3:
            return False
        centered = scan.points-scan.points.mean(axis=0)
        minor = np.sqrt(max(0., np.linalg.eigvalsh(centered.T@centered/len(scan))[0]))
        angles = np.sort(np.mod(np.arctan2(scan.normals[:, 1], scan.normals[:, 0]), 2*np.pi))
        gap = np.max(np.diff(np.r_[angles, angles[0]+2*np.pi]))
        return bool(minor > 3*self.noise and gap < np.pi)

    def freeze(self):
        self.frozen = True
