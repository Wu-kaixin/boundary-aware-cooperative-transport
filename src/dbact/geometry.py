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


def polygon_perimeter(vertices: np.ndarray) -> float:
    v = np.asarray(vertices, dtype=float)
    edges = np.roll(v, -1, axis=0) - v
    return float(np.sum(np.linalg.norm(edges, axis=1)))


def polygon_diameter(vertices: np.ndarray) -> float:
    """Largest vertex-to-vertex distance of a polygon.

    A polygon edge is a convex segment, so the maximum Euclidean distance over
    the whole closed polygon is attained at vertices.  This makes the value an
    exact (not sampled) footprint diameter for the admissibility certificate.
    """
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    if len(v) < 2:
        return 0.0
    return float(np.max(np.linalg.norm(v[:, None, :] - v[None, :, :], axis=2)))


def is_simple_polygon(vertices: np.ndarray) -> bool:
    """Return whether ``vertices`` form a non-degenerate simple polygon.

    Adjacent edges are allowed to meet at their shared endpoint; every other
    intersection, a repeated non-adjacent vertex, or a zero-length edge rejects
    the outline.  The predicate is intentionally exact up to ``EPS`` because a
    self-intersecting outline has no unambiguous inside/outside or cage offset.
    """
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    n = len(v)
    if n < 3 or abs(polygon_area(v)) <= EPS:
        return False
    edges = np.roll(v, -1, axis=0) - v
    if np.any(np.linalg.norm(edges, axis=1) <= EPS):
        return False
    for i in range(n):
        a, b = v[i], v[(i + 1) % n]
        for j in range(i + 1, n):
            # Consecutive edges and the first/last pair share one legal vertex.
            if j == i or j == (i + 1) % n or i == (j + 1) % n:
                continue
            c, d = v[j], v[(j + 1) % n]
            if _closed_segments_intersect(a, b, c, d):
                return False
    return True


def _closed_segments_intersect(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    """Closed-segment intersection used by :func:`is_simple_polygon`."""

    def orient(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> float:
        return float((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]))

    def on_segment(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> bool:
        return bool(
            min(p[0], r[0]) - EPS <= q[0] <= max(p[0], r[0]) + EPS
            and min(p[1], r[1]) - EPS <= q[1] <= max(p[1], r[1]) + EPS
        )

    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    if ((o1 > EPS and o2 < -EPS) or (o1 < -EPS and o2 > EPS)) and (
        (o3 > EPS and o4 < -EPS) or (o3 < -EPS and o4 > EPS)
    ):
        return True
    return bool(
        (abs(o1) <= EPS and on_segment(a, c, b))
        or (abs(o2) <= EPS and on_segment(a, d, b))
        or (abs(o3) <= EPS and on_segment(c, a, d))
        or (abs(o4) <= EPS and on_segment(c, b, d))
    )


def certified_inscribed_radius(vertices: np.ndarray) -> float:
    """Certified lower bound on the radius of a disk contained in a polygon.

    Ear clipping decomposes a simple polygon into interior triangles.  The
    incircle of every such triangle is contained in the polygon, hence the
    largest triangle inradius is a constructive witness rather than an
    optimistic grid estimate.  A return value of zero means no witness exists.
    """
    v = ensure_ccw(np.asarray(vertices, dtype=float))
    triangles = triangulate_simple_polygon(v)
    radii: list[float] = []
    for tri in triangles:
        lengths = np.linalg.norm(np.roll(tri, -1, axis=0) - tri, axis=1)
        perimeter = float(np.sum(lengths))
        area = abs(polygon_area(tri))
        if perimeter > EPS and area > EPS:
            radii.append(2.0 * area / perimeter)
    # The area centroid of a concave polygon need not be inside. When it is,
    # however, its exact distance to the closed boundary is also a constructive
    # inscribed disk. This avoids the needlessly tiny ear-triangle witness for a
    # many-sided convex outline such as the polygonal circle factory.
    centroid = polygon_centroid(v)
    if point_in_polygon(centroid, v):
        radii.append(float(-signed_distance_to_polygon(centroid[None, :], v)[0]))
    return max(radii, default=0.0)


def polygon_second_moment(vertices: np.ndarray, center: np.ndarray | None = None) -> float:
    """Area second moment of a polygon about ``center`` (unit density)."""
    v = ensure_ccw(vertices)
    c = polygon_centroid(v) if center is None else np.asarray(center, dtype=float)
    p = v - c
    q = np.roll(p, -1, axis=0)
    cross = p[:, 0] * q[:, 1] - q[:, 0] * p[:, 1]
    terms = np.sum(p * p, axis=1) + np.sum(p * q, axis=1) + np.sum(q * q, axis=1)
    return float(np.sum(cross * terms) / 12.0)


def points_in_polygon(points: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """Vectorised even-odd containment test. Returns a boolean array."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    v = np.asarray(vertices, dtype=float)
    if len(p) == 0:
        return np.zeros(0, dtype=bool)
    x, y = p[:, 0], p[:, 1]
    xi, yi = v[:, 0], v[:, 1]
    xj, yj = np.roll(xi, 1), np.roll(yi, 1)
    straddles = (yi[None, :] > y[:, None]) != (yj[None, :] > y[:, None])
    dy = (yj - yi)[None, :]
    dy = np.where(np.abs(dy) < EPS, EPS, dy)
    x_cross = (xj - xi)[None, :] * (y[:, None] - yi[None, :]) / dy + xi[None, :]
    crossings = np.sum(straddles & (x[:, None] < x_cross), axis=1)
    return crossings % 2 == 1


def closest_boundary_points(points: np.ndarray, vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorised closest boundary point, unsigned distance and edge index."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    v = ensure_ccw(vertices)
    if len(p) == 0:
        return np.empty((0, 2)), np.empty(0), np.empty(0, dtype=int)
    a = v
    e = np.roll(v, -1, axis=0) - v
    denom = np.sum(e * e, axis=1)
    denom = np.where(denom < EPS, EPS, denom)
    rel = p[:, None, :] - a[None, :, :]
    t = np.clip(np.sum(rel * e[None, :, :], axis=2) / denom[None, :], 0.0, 1.0)
    q = a[None, :, :] + t[:, :, None] * e[None, :, :]
    d = np.linalg.norm(p[:, None, :] - q, axis=2)
    k = np.argmin(d, axis=1)
    rows = np.arange(len(p))
    return q[rows, k], d[rows, k], k


def signed_distance_to_polygon(points: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """Signed distance to a polygon boundary: positive outside, negative inside."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(p) == 0:
        return np.empty(0)
    _, d, _ = closest_boundary_points(p, vertices)
    inside = points_in_polygon(p, vertices)
    return np.where(inside, -d, d)


def signed_distance_and_gradient(points: np.ndarray, vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Signed distance, its gradient (outward unit normal) and the footpoint.

    The gradient of the signed distance is ``(p - q)/||p - q||`` outside and
    ``(q - p)/||q - p||`` inside, which stays correct at convex corners where the
    incident edge normals disagree. Degenerate points sitting exactly on the
    boundary fall back to the edge normal.
    """
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    v = ensure_ccw(vertices)
    if len(p) == 0:
        return np.empty(0), np.empty((0, 2)), np.empty((0, 2))
    q, d, k = closest_boundary_points(p, v)
    inside = points_in_polygon(p, v)
    signed = np.where(inside, -d, d)

    direction = p - q
    scale = np.where(inside, -1.0, 1.0)[:, None]
    grad = direction * scale
    norms = np.linalg.norm(grad, axis=1)
    degenerate = norms < 1e-9
    if np.any(degenerate):
        e = np.roll(v, -1, axis=0) - v
        edge_normals = np.column_stack([e[:, 1], -e[:, 0]])
        edge_norms = np.linalg.norm(edge_normals, axis=1)
        edge_norms = np.where(edge_norms < EPS, EPS, edge_norms)
        edge_normals = edge_normals / edge_norms[:, None]
        grad[degenerate] = edge_normals[k[degenerate]]
        norms = np.linalg.norm(grad, axis=1)
    norms = np.where(norms < EPS, EPS, norms)
    return signed, grad / norms[:, None], q


def ray_polygon_first_hit(
    origin: np.ndarray,
    direction: np.ndarray,
    vertices: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """First intersection of a ray with a polygon boundary.

    Returns ``(point, outward_edge_normal, range)`` for the nearest hit within
    ``max_range``, or ``None`` when the ray misses. Taking the nearest hit -- not
    every sampled boundary point within sensor range -- is what makes the sensor
    respect occlusion.
    """
    o = np.asarray(origin, dtype=float).reshape(2)
    d = normalize(np.asarray(direction, dtype=float).reshape(2))
    v = ensure_ccw(vertices)
    a = v
    e = np.roll(v, -1, axis=0) - v

    denom = d[0] * e[:, 1] - d[1] * e[:, 0]
    diff = a - o[None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (diff[:, 0] * e[:, 1] - diff[:, 1] * e[:, 0]) / denom
        s = (diff[:, 0] * d[1] - diff[:, 1] * d[0]) / denom
    valid = (np.abs(denom) > EPS) & (s >= -1e-12) & (s <= 1.0 + 1e-12) & (t > 1e-9) & (t <= max_range)
    if not np.any(valid):
        return None
    idx = int(np.argmin(np.where(valid, t, np.inf)))
    hit = o + t[idx] * d
    edge = e[idx]
    normal = normalize(np.array([edge[1], -edge[0]], dtype=float))
    return hit, normal, float(t[idx])


def ray_batch_first_hits(
    origin: np.ndarray,
    directions: np.ndarray,
    vertices: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorised :func:`ray_polygon_first_hit` over a whole scan.

    Returns ``(ranges, edge_indices, hit_mask)`` with ``ranges[k] = inf`` where the
    ray missed. A full scan is one array operation instead of one Python call per
    ray, which matters because the sensor is the hot loop of the simulation.
    """
    o = np.asarray(origin, dtype=float).reshape(2)
    d = np.asarray(directions, dtype=float).reshape(-1, 2)
    v = ensure_ccw(vertices)
    a = v
    e = np.roll(v, -1, axis=0) - v

    denom = d[:, 0:1] * e[None, :, 1] - d[:, 1:2] * e[None, :, 0]
    diff = a - o[None, :]
    numer_t = diff[:, 0] * e[:, 1] - diff[:, 1] * e[:, 0]
    numer_s = diff[None, :, 0] * d[:, 1:2] - diff[None, :, 1] * d[:, 0:1]

    with np.errstate(divide="ignore", invalid="ignore"):
        t = numer_t[None, :] / denom
        s = numer_s / denom
    valid = (
        (np.abs(denom) > EPS)
        & (s >= -1e-12)
        & (s <= 1.0 + 1e-12)
        & (t > 1e-9)
        & (t <= max_range)
    )
    t_masked = np.where(valid, t, np.inf)
    edge_index = np.argmin(t_masked, axis=1)
    ranges = t_masked[np.arange(len(d)), edge_index]
    return ranges, edge_index, np.isfinite(ranges)


def outward_edge_normals(vertices: np.ndarray) -> np.ndarray:
    """Unit outward normal of every edge of a CCW polygon."""
    v = ensure_ccw(vertices)
    e = np.roll(v, -1, axis=0) - v
    normals = np.column_stack([e[:, 1], -e[:, 0]])
    norms = np.linalg.norm(normals, axis=1)
    return normals / np.where(norms < EPS, EPS, norms)[:, None]


def segment_hits_polygon(a: np.ndarray, b: np.ndarray, vertices: np.ndarray, tolerance: float = 0.0) -> bool:
    """True when the open segment ``a``--``b`` crosses the polygon boundary.

    ``tolerance`` shortens the segment at both ends so that a segment which only
    touches the boundary at its own endpoint is not reported as occluded.
    """
    p0 = np.asarray(a, dtype=float).reshape(2)
    p1 = np.asarray(b, dtype=float).reshape(2)
    seg = p1 - p0
    length = float(np.linalg.norm(seg))
    if length < EPS:
        return False
    trim = min(tolerance, 0.49 * length)
    hit = ray_polygon_first_hit(p0 + (trim / length) * seg, seg, vertices, length - 2.0 * trim)
    return hit is not None


def triangulate_simple_polygon(vertices: np.ndarray) -> list[np.ndarray]:
    """Ear-clipping decomposition of a simple polygon into triangles.

    Rigid-body engines accept convex shapes only, so a concave cargo has to be
    attached to its body as several convex pieces. Triangles are the safe choice:
    they are always convex and the decomposition never depends on the concavity
    pattern of the outline.
    """
    v = ensure_ccw(np.asarray(vertices, dtype=float))
    indices = list(range(len(v)))
    triangles: list[np.ndarray] = []
    guard = 0
    while len(indices) > 3 and guard < 10 * len(v):
        guard += 1
        clipped = False
        for k in range(len(indices)):
            i_prev = indices[k - 1]
            i_cur = indices[k]
            i_next = indices[(k + 1) % len(indices)]
            a, b, c = v[i_prev], v[i_cur], v[i_next]
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if cross <= EPS:  # reflex or degenerate corner
                continue
            others = [v[i] for i in indices if i not in (i_prev, i_cur, i_next)]
            if others and np.any(_points_in_triangle(np.asarray(others), a, b, c)):
                continue
            triangles.append(np.vstack([a, b, c]))
            indices.pop(k)
            clipped = True
            break
        if not clipped:
            break
    if len(indices) == 3:
        triangles.append(np.vstack([v[indices[0]], v[indices[1]], v[indices[2]]]))
    return triangles


def _points_in_triangle(points: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    d1 = (p[:, 0] - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (p[:, 1] - b[1])
    d2 = (p[:, 0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (p[:, 1] - c[1])
    d3 = (p[:, 0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (p[:, 1] - a[1])
    has_neg = (d1 < -EPS) | (d2 < -EPS) | (d3 < -EPS)
    has_pos = (d1 > EPS) | (d2 > EPS) | (d3 > EPS)
    return ~(has_neg & has_pos)


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
