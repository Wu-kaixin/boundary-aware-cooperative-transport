from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CVTResult:
    points: np.ndarray
    centroids: np.ndarray
    labels: np.ndarray
    grid_x: np.ndarray
    grid_y: np.ndarray
    label_grid: np.ndarray


def world_bounds_xy(world_config: dict) -> tuple[float, float, float, float]:
    return (
        float(world_config["x_min"]),
        float(world_config["x_max"]),
        float(world_config["y_min"]),
        float(world_config["y_max"]),
    )


def compute_grid_cvt(points: np.ndarray, world_config: dict, grid_resolution: int) -> CVTResult:
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape (n, 2)")
    if len(points) == 0:
        raise ValueError("points must contain at least one robot position")

    x_min, x_max, y_min, y_max = world_bounds_xy(world_config)
    resolution = max(int(grid_resolution), 2)
    xs = np.linspace(x_min, x_max, resolution)
    ys = np.linspace(y_min, y_max, resolution)
    grid_x, grid_y = np.meshgrid(xs, ys)
    samples = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    distances = np.linalg.norm(samples[:, None, :] - points[None, :, :], axis=2)
    labels = np.argmin(distances, axis=1)
    centroids = np.empty_like(points, dtype=float)
    for index, point in enumerate(points):
        cell_points = samples[labels == index]
        centroids[index] = cell_points.mean(axis=0) if len(cell_points) else point

    return CVTResult(
        points=points,
        centroids=centroids,
        labels=labels,
        grid_x=grid_x,
        grid_y=grid_y,
        label_grid=labels.reshape(grid_x.shape),
    )
