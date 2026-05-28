from __future__ import annotations

import math
from typing import Iterable

import numpy as np


EPS = 1e-9


def normalize(v: np.ndarray, fallback: np.ndarray | None = None) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < EPS:
        if fallback is None:
            return np.zeros_like(v)
        return normalize(np.asarray(fallback, dtype=float))
    return v / n


def rotate(points: np.ndarray, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s], [s, c]], dtype=float)
    return points @ r.T


def polygon_area(vertices: np.ndarray) -> float:
    v = np.asarray(vertices, dtype=float)
    x, y = v[:, 0], v[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def ensure_ccw(vertices: np.ndarray) -> np.ndarray:
    v = np.asarray(vertices, dtype=float)
    if polygon_area(v) < 0:
        return v[::-1].copy()
    return v.copy()


def polygon_centroid(vertices: np.ndarray) -> np.ndarray:
    v = ensure_ccw(vertices)
    area = polygon_area(v)
    if abs(area) < EPS:
        return np.mean(v, axis=0)
    x, y = v[:, 0], v[:, 1]
    x_next, y_next = np.roll(x, -1), np.roll(y, -1)
    cross = x * y_next - x_next * y
    cx = np.sum((x + x_next) * cross) / (6.0 * area)
    cy = np.sum((y + y_next) * cross) / (6.0 * area)
    return np.array([cx, cy], dtype=float)


def point_in_polygon(point: np.ndarray, vertices: np.ndarray) -> bool:
    x, y = np.asarray(point, dtype=float)
    v = np.asarray(vertices, dtype=float)
    inside = False
    j = len(v) - 1
    for i in range(len(v)):
        xi, yi = v[i]
        xj, yj = v[j]
        intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) + EPS) + xi)
        if intersect:
            inside = not inside
        j = i
    return inside


def closest_point_on_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
    p = np.asarray(point, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < EPS:
        return a.copy(), 0.0
    t = float(np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0))
    return a + t * ab, t


def closest_boundary_point_and_normal(vertices: np.ndarray, point: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Return closest boundary point, outward normal and distance.

    Vertices are treated as a CCW polygon. For a CCW edge e=(b-a), the right-hand
    normal [e_y, -e_x] points outside the polygon.
    """
    v = ensure_ccw(vertices)
    p = np.asarray(point, dtype=float)
    best_q = v[0]
    best_n = np.array([1.0, 0.0])
    best_d = float("inf")
    for i in range(len(v)):
        a, b = v[i], v[(i + 1) % len(v)]
        q, _ = closest_point_on_segment(p, a, b)
        d = float(np.linalg.norm(p - q))
        edge = b - a
        n_out = normalize(np.array([edge[1], -edge[0]], dtype=float), fallback=q - polygon_centroid(v))
        if d < best_d:
            best_q, best_n, best_d = q, n_out, d
    return best_q.copy(), best_n.copy(), best_d


def sample_polygon_boundary(vertices: np.ndarray, count: int = 128) -> tuple[np.ndarray, np.ndarray]:
    """Uniformly sample a polygon boundary and return points plus outward normals."""
    v = ensure_ccw(vertices)
    edges = np.roll(v, -1, axis=0) - v
    lengths = np.linalg.norm(edges, axis=1)
    perimeter = float(np.sum(lengths))
    if perimeter < EPS:
        return v.copy(), np.zeros_like(v)
    cumulative = np.cumsum(lengths)
    distances = np.linspace(0.0, perimeter, count, endpoint=False)
    points = []
    normals = []
    start = 0.0
    edge_idx = 0
    for d in distances:
        while d >= cumulative[edge_idx] and edge_idx < len(v) - 1:
            edge_idx += 1
            start = cumulative[edge_idx - 1]
        a = v[edge_idx]
        e = edges[edge_idx]
        length = lengths[edge_idx]
        t = 0.0 if length < EPS else (d - start) / length
        points.append(a + t * e)
        normals.append(normalize(np.array([e[1], -e[0]], dtype=float)))
    return np.asarray(points), np.asarray(normals)


def make_circle(center: Iterable[float], radius: float, count: int = 64) -> np.ndarray:
    center = np.asarray(center, dtype=float)
    theta = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    return np.column_stack([center[0] + radius * np.cos(theta), center[1] + radius * np.sin(theta)])


def make_rectangle(center: Iterable[float], width: float, height: float, yaw: float = 0.0) -> np.ndarray:
    w, h = width / 2.0, height / 2.0
    pts = np.array([[-w, -h], [w, -h], [w, h], [-w, h]], dtype=float)
    return ensure_ccw(rotate(pts, yaw) + np.asarray(center, dtype=float))


def make_l_shape(center: Iterable[float], scale: float = 1.0, yaw: float = 0.0) -> np.ndarray:
    # CCW non-convex L shape around origin.
    pts = np.array([
        [-0.60, -0.60], [0.60, -0.60], [0.60, -0.15],
        [-0.10, -0.15], [-0.10, 0.60], [-0.60, 0.60],
    ], dtype=float) * scale
    return ensure_ccw(rotate(pts, yaw) + np.asarray(center, dtype=float))


def make_nonconvex(center: Iterable[float], scale: float = 1.0, yaw: float = 0.0) -> np.ndarray:
    pts = np.array([
        [-0.75, -0.35], [-0.20, -0.70], [0.65, -0.45],
        [0.30, -0.05], [0.75, 0.45], [0.05, 0.35],
        [-0.45, 0.70], [-0.35, 0.10],
    ], dtype=float) * scale
    return ensure_ccw(rotate(pts, yaw) + np.asarray(center, dtype=float))


def clip_to_domain(point: np.ndarray, domain: tuple[float, float, float, float]) -> np.ndarray:
    xmin, xmax, ymin, ymax = domain
    p = np.asarray(point, dtype=float).copy()
    p[0] = np.clip(p[0], xmin, xmax)
    p[1] = np.clip(p[1], ymin, ymax)
    return p
