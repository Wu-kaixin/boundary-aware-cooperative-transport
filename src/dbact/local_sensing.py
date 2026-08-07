from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .cargo import Cargo
from .geometry import EPS, ensure_ccw, normalize
from .types import AgentState, BoundaryObservation


def _ray_segment_hit(
    origin: np.ndarray,
    direction: np.ndarray,
    a: np.ndarray,
    b: np.ndarray,
    max_range: float,
) -> float | None:
    """Return travel distance t along the ray if it hits segment ab within range."""
    # origin + t d = a + s (b-a), t >= 0, s in [0, 1]
    ab = b - a
    mat = np.array([[direction[0], -ab[0]], [direction[1], -ab[1]]], dtype=float)
    det = float(np.linalg.det(mat))
    if abs(det) < EPS:
        return None
    rhs = a - origin
    t = float((rhs[0] * (-ab[1]) - rhs[1] * (-ab[0])) / det)
    s = float((direction[0] * rhs[1] - direction[1] * rhs[0]) / det)
    if t < EPS or t > max_range or s < -EPS or s > 1.0 + EPS:
        return None
    return t


def _estimate_normals_pca(
    points: np.ndarray,
    agent_position: np.ndarray,
    neighbor_count: int = 4,
) -> np.ndarray:
    """Estimate outward normals from local PCA tangents.

    Sign convention: robot is outside the object, so n̂ᵀ(pᵢ − b̂) > 0.
    """
    n_pts = len(points)
    normals = np.zeros((n_pts, 2), dtype=float)
    if n_pts == 0:
        return normals
    if n_pts == 1:
        normals[0] = normalize(agent_position - points[0], fallback=np.array([1.0, 0.0]))
        return normals

    k = min(neighbor_count, n_pts - 1)
    for i in range(n_pts):
        diffs = points - points[i]
        dist2 = np.sum(diffs * diffs, axis=1)
        dist2[i] = np.inf
        nn = np.argpartition(dist2, k)[:k]
        local = points[np.concatenate(([i], nn))]
        centered = local - np.mean(local, axis=0)
        if len(local) < 2:
            tangent = np.array([1.0, 0.0])
        else:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
            tangent = vh[0]
        normal = normalize(np.array([tangent[1], -tangent[0]], dtype=float), fallback=np.array([1.0, 0.0]))
        if float(np.dot(normal, agent_position - points[i])) < 0.0:
            normal = -normal
        normals[i] = normal
    return normals


def _arc_lengths_from_hits(points: np.ndarray, agent_position: np.ndarray) -> np.ndarray:
    """Approximate Δs_k from angular neighbor spacing of ray hits."""
    n_pts = len(points)
    if n_pts == 0:
        return np.empty(0, dtype=float)
    if n_pts == 1:
        return np.array([1.0], dtype=float)

    angles = np.arctan2(points[:, 1] - agent_position[1], points[:, 0] - agent_position[0])
    order = np.argsort(angles)
    ordered = points[order]
    prev_pts = np.roll(ordered, 1, axis=0)
    next_pts = np.roll(ordered, -1, axis=0)
    ds = 0.5 * (np.linalg.norm(ordered - prev_pts, axis=1) + np.linalg.norm(next_pts - ordered, axis=1))
    ds = np.maximum(ds, 1e-3)
    out = np.empty(n_pts, dtype=float)
    out[order] = ds
    return out


@dataclass
class LocalBoundarySensor:
    """Simulator-side local boundary sensor via ray casting.

    Paper model: the controller never observes complete object geometry. The
    simulator polygon is used only here to produce locally visible measurements
    z_ik = (b̂, n̂, c, t) through ray casting + PCA normal estimation. Occlusion
    is handled by keeping the nearest hit along each ray.
    """

    sensor_range: float
    num_rays: int = 72
    max_points_per_object: int = 24
    noise_std: float = 0.0
    normal_neighbors: int = 4
    base_confidence: float = 1.0

    def sense(self, agent: AgentState, cargoes: list[Cargo], timestamp: float) -> list[BoundaryObservation]:
        observations: list[BoundaryObservation] = []
        rng = np.random.default_rng(abs(hash((agent.agent_id, round(timestamp, 2)))) % (2**32))
        origin = agent.position
        angles = np.linspace(0.0, 2.0 * np.pi, self.num_rays, endpoint=False)

        for cargo in cargoes:
            vertices = ensure_ccw(cargo.vertices)
            hits: list[np.ndarray] = []
            for angle in angles:
                direction = np.array([np.cos(angle), np.sin(angle)], dtype=float)
                best_t: float | None = None
                best_point: np.ndarray | None = None
                for i in range(len(vertices)):
                    a = vertices[i]
                    b = vertices[(i + 1) % len(vertices)]
                    t = _ray_segment_hit(origin, direction, a, b, self.sensor_range)
                    if t is None:
                        continue
                    if best_t is None or t < best_t:
                        best_t = t
                        best_point = origin + t * direction
                if best_point is not None:
                    hits.append(best_point)

            if not hits:
                continue

            points = np.asarray(hits, dtype=float)
            if self.noise_std > 0:
                points = points + rng.normal(scale=self.noise_std, size=points.shape)

            if len(points) > self.max_points_per_object:
                pick = np.linspace(0, len(points) - 1, self.max_points_per_object).astype(int)
                points = points[pick]

            normals = _estimate_normals_pca(points, origin, neighbor_count=self.normal_neighbors)
            arc_lengths = _arc_lengths_from_hits(points, origin)
            for point, normal, ds in zip(points, normals, arc_lengths):
                # Closer hits are slightly more confident.
                dist = float(np.linalg.norm(point - origin))
                confidence = self.base_confidence * float(np.clip(1.0 - 0.25 * dist / max(self.sensor_range, EPS), 0.2, 1.0))
                observations.append(
                    BoundaryObservation(
                        object_id=cargo.object_id,
                        agent_id=agent.agent_id,
                        point=point,
                        normal=normal,
                        timestamp=timestamp,
                        confidence=confidence,
                        arc_length=float(ds),
                    )
                )
        return observations
