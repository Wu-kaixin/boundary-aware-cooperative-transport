"""Optional display overlays built exclusively from saved trace data."""

from __future__ import annotations

import numpy as np

from dbact_sim.trace import SimulationTrace


def sensor_segments(trace: SimulationTrace, frame: int) -> np.ndarray:
    snapshot = trace.visual_snapshot(frame)
    if not len(snapshot.detected_points):
        return np.empty((0, 2, 2), dtype=float)
    count = min(len(snapshot.sensor_origins), len(snapshot.detected_points))
    return np.stack(
        [snapshot.sensor_origins[:count], snapshot.detected_points[:count]],
        axis=1,
    )


def fused_boundary_polyline(trace: SimulationTrace, frame: int) -> np.ndarray:
    """Order mapped points around their centroid for a display-only estimate."""
    points = trace.visual_snapshot(frame).mapped_points
    if len(points) < 2:
        return points
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    order = np.argsort(angles, kind="stable")
    ordered = points[order]
    return np.vstack([ordered, ordered[0]])


def voronoi_segments(
    points: np.ndarray,
    domain: tuple[float, float, float, float],
) -> np.ndarray:
    """Finite display-only Voronoi segments clipped by a mirrored workspace."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < 2:
        return np.empty((0, 2, 2), dtype=float)
    try:
        from scipy.spatial import Voronoi
    except Exception:  # pragma: no cover - scipy is a project dependency
        return np.empty((0, 2, 2), dtype=float)
    xmin, xmax, ymin, ymax = domain
    span = max(xmax - xmin, ymax - ymin)
    mirrored = np.vstack(
        [
            points,
            points + (-span, 0.0),
            points + (span, 0.0),
            points + (0.0, -span),
            points + (0.0, span),
        ]
    )
    try:
        voronoi = Voronoi(mirrored)
    except Exception:
        return np.empty((0, 2, 2), dtype=float)
    segments = []
    for first, second in voronoi.ridge_vertices:
        if first < 0 or second < 0:
            continue
        p0, p1 = voronoi.vertices[first], voronoi.vertices[second]
        lower, upper = np.minimum(p0, p1), np.maximum(p0, p1)
        if upper[0] < xmin or lower[0] > xmax or upper[1] < ymin or lower[1] > ymax:
            continue
        segments.append(np.stack([p0, p1]))
    return np.stack(segments) if segments else np.empty((0, 2, 2), dtype=float)


__all__ = ["fused_boundary_polyline", "sensor_segments", "voronoi_segments"]
