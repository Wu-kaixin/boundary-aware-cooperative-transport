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


__all__ = ["fused_boundary_polyline", "sensor_segments"]
