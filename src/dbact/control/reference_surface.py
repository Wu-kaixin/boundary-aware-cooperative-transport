"""Opt-in immutable local line fits for an observed reference template.

Only initial own/one-hop returns are used. All thresholds are inherited from
the existing perception/registration parameters. This empirical representation
does not certify correspondences, curvature, or the location of an object center.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from dbact.control.observed_reference import ObservedReferenceTracker


def refine_reference_surface(points, normals, *, gate, normal_cosine, residual_tolerance):
    """Fit spatial, normal-compatible neighborhoods without editing the inputs."""
    fitted, fitted_normals = points.copy(), normals.copy()
    accepted = np.zeros(len(points), dtype=bool)
    tree = cKDTree(points)
    for i, neighbors in enumerate(tree.query_ball_point(points, gate)):
        idx = np.asarray(neighbors, dtype=int)
        idx = idx[normals[idx] @ normals[i] >= normal_cosine]
        if len(idx) < 8:  # Same minimum support as the original reference match.
            continue
        center = points[idx].mean(axis=0)
        local = points[idx] - center
        values, vectors = np.linalg.eigh(local.T @ local / len(local))
        # Coincident returns do not define a line, regardless of their count.
        if values[-1] <= 0:
            continue
        normal = vectors[:, 0]
        if normal @ normals[i] < 0:
            normal = -normal
        if (np.sqrt(max(values[0], 0.)) > residual_tolerance
                or normal @ normals[i] < normal_cosine):
            continue
        fitted[i] = points[i] - ((points[i]-center) @ normal)*normal
        fitted_normals[i] = normal
        accepted[i] = True
    return fitted, fitted_normals, accepted


class SurfaceReferenceTracker(ObservedReferenceTracker):
    """Change only the initial surface geometry; retain the original estimator."""

    def __init__(self, *, surface_residual_tolerance, **kwargs):
        if not np.isfinite(surface_residual_tolerance) or surface_residual_tolerance <= 0:
            raise ValueError('Surface residual tolerance must be finite and positive')
        super().__init__(**kwargs)
        self.surface_residual_tolerance = surface_residual_tolerance
        self.surface_diagnostics = {}

    def initialize(self, scan, bootstrap_displacement, timestamp):
        # Preserve original bootstrap admission, epoch, anchor, offset and pivot.
        # Refinement never creates a fresh stopping vote during initialization.
        super().initialize(scan, bootstrap_displacement, timestamp)
        if not self.initialized:
            return None
        self.raw_reference = self.reference.copy()
        self.raw_normals = self.normals.copy()
        self.reference, self.normals, accepted = refine_reference_surface(
            self.raw_reference, self.raw_normals, gate=self.gate,
            normal_cosine=self.normal_cosine,
            residual_tolerance=self.surface_residual_tolerance)
        self.tree = cKDTree(self.reference)
        projection = np.linalg.norm(self.reference-self.raw_reference, axis=1)
        self.surface_diagnostics = dict(
            total_points=len(scan), refined_points=int(accepted.sum()),
            projection_rms_m=float(np.sqrt(np.mean(projection**2))),
            projection_max_m=float(projection.max()),
            original_normal_conditioning=self._conditioning(self.raw_normals),
            fitted_normal_conditioning=self._conditioning(self.normals))
        return None

    def initialization_record(self):
        record = super().initialization_record()
        if self.initialized:
            record.update(surface_refinement=dict(self.surface_diagnostics),
                          raw_reference_points_relative=self.raw_reference.tolist(),
                          raw_reference_normals=self.raw_normals.tolist())
        return record
