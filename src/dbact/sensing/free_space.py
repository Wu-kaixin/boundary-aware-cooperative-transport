"""Conditional centroid enclosure from measured free segments, without ray fans.

If a connected object lies inside the workspace, no supplied free segment
intersects it, and each target return is within ``margin`` of its true point,
its positive-mass centroid lies in the returned convex set. These assumptions
are NOT certified for Gaussian range noise. No fit, yaw or task success is
certified here. Inconsistent or unavailable evidence retains the workspace.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from shapely import distance
from shapely.errors import GEOSException
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import polygonize, unary_union


@dataclass(frozen=True)
class FreeRayEnclosure:
    vertices: np.ndarray
    status: str
    faces: int
    feasible_faces: int
    workspace_area: float
    component_area: float
    center_area: float


def _points(values, name):
    points = np.asarray(values, dtype=float)
    if points.size == 0:
        return np.empty((0, 2))
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError(f'{name} must be finite (N, 2) points')
    return points


def measured_free_ray_packet(origin, returns, *, ray_count, sensor_range, margin, target_id):
    """Sanitize one COMPLETE raw scan from the calibrated global-angle scanner.

    A missing ray means no hit within sensor range, so filtered point clouds
    must never be passed here. Only ray index, measured range and object label
    are read; directions are calibrated, and truth normals/geometry are ignored.
    Shortening by ``margin`` is conditional on bounded range error, not a
    Gaussian-noise guarantee. Other-object hits also terminate free segments.
    """
    origin = np.asarray(origin, dtype=float)
    if (origin.shape != (2,) or not np.isfinite(origin).all()
            or not isinstance(ray_count, (int, np.integer)) or ray_count <= 0
            or not np.isfinite(sensor_range) or sensor_range <= 0
            or not np.isfinite(margin) or margin < 0):
        raise ValueError('Finite origin, positive ray count/range and nonnegative margin required')
    angles = np.linspace(0., 2.*np.pi, ray_count, endpoint=False)
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    lengths = np.full(ray_count, sensor_range, dtype=float)
    targets, seen = [], set()
    for item in returns:
        index, measured = item['ray_index'], item['range']
        if (not isinstance(index, (int, np.integer)) or not 0 <= index < ray_count
                or index in seen or not np.isfinite(measured) or measured <= 0):
            raise ValueError('Raw returns need distinct valid ray indices and positive finite ranges')
        seen.add(index)
        lengths[index] = min(measured, sensor_range)
        if item['object_id'] == target_id:
            targets.append(origin+measured*directions[index])
    starts = np.repeat(origin[None, :], ray_count, axis=0)
    ends = starts+np.maximum(0., lengths-margin)[:, None]*directions
    return starts, ends, np.asarray(targets, dtype=float).reshape(-1, 2)


def free_ray_enclosure(starts, ends, target_points, workspace_vertices, *, margin):
    """Keep every arrangement face consistent with every observed target hit.

    ``starts`` and ``ends`` are already-shortened measured free segments, not
    boundary chords or interpolated sensor sectors. ``margin`` bounds target
    return error only conditionally. The function has no object-truth input.
    """
    starts, ends = _points(starts, 'starts'), _points(ends, 'ends')
    points = _points(target_points, 'target_points')
    vertices = _points(workspace_vertices, 'workspace_vertices')
    if len(starts) != len(ends) or not np.isfinite(margin) or margin < 0:
        raise ValueError('Matching segment arrays and a finite nonnegative margin are required')
    if len(vertices) < 3:
        raise ValueError('At least three workspace vertices are required')
    workspace = Polygon(vertices)
    if (not workspace.is_valid or not np.isfinite(workspace.area) or workspace.area <= 0
            or not workspace.equals(workspace.convex_hull)):
        raise ValueError('Workspace must be a valid finite convex polygon')

    def result(geometry, status, faces=0, feasible=0, component_area=None):
        return FreeRayEnclosure(np.asarray(geometry.exterior.coords[:-1]).copy(), status,
            faces, feasible, float(workspace.area),
            float(workspace.area if component_area is None else component_area), float(geometry.area))

    if not len(points):
        return result(workspace, 'NO_TARGET_RETURNS')
    try:
        segments = [workspace.boundary]
        for start, end in zip(starts, ends):
            if np.array_equal(start, end):
                continue
            clipped = LineString([start, end]).intersection(workspace)
            if not clipped.is_empty and clipped.length > 0:
                segments.append(clipped)
        faces = [face for face in polygonize(unary_union(segments))
                 if workspace.covers(face.representative_point())]
        # Geometry roundoff tolerance, not an observation or control parameter.
        area_roundoff = 1e-10*max(1., workspace.area)
        if (not faces or unary_union(faces).symmetric_difference(workspace).area > area_roundoff):
            return result(workspace, 'PARTITION_UNAVAILABLE', len(faces))
        feasible = np.asarray(faces, dtype=object)
        for point in points:
            feasible = feasible[distance(feasible, Point(point)) <= margin+1e-12]
            if not len(feasible):
                return result(workspace, 'INCONSISTENT_RETURNS', len(faces))
        support = unary_union(feasible)
        hull = support.convex_hull
        if hull.is_empty or hull.geom_type != 'Polygon':
            return result(workspace, 'PARTITION_UNAVAILABLE', len(faces), len(feasible))
        status = ('WORKSPACE_ONLY' if abs(hull.area-workspace.area) <= area_roundoff
                  else 'BOUNDED_BY_FREE_RAYS')
        return result(hull, status, len(faces), len(feasible), support.area)
    except GEOSException:
        return result(workspace, 'PARTITION_UNAVAILABLE')
