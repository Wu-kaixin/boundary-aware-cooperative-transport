from __future__ import annotations

from dataclasses import dataclass, field
import hashlib

import numpy as np

from .accel import nearest_ray_hits
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


def _stable_rng_seed(base_seed: int, agent_id: str, timestamp: float) -> int:
    """Return a process-independent seed for one sensing frame.

    Python's built-in ``hash`` is salted per process, so it cannot be used for
    experiments that report statistics across independent runs.
    """
    payload = f"{int(base_seed)}|{agent_id}|{float(timestamp):.9f}".encode("utf-8")
    digest = hashlib.blake2b(payload, digest_size=8).digest()
    return int.from_bytes(digest, byteorder="little", signed=False)


def _estimate_normals_pca(
    points: np.ndarray,
    agent_position: np.ndarray,
    neighbor_count: int = 4,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate outward normals from local PCA tangents.

    Sign convention: robot is outside the object, so n̂ᵀ(pᵢ − b̂) > 0.
    """
    n_pts = len(points)
    normals = np.zeros((n_pts, 2), dtype=float)
    planarity = np.zeros(n_pts, dtype=float)
    if n_pts == 0:
        return normals, planarity
    if n_pts == 1:
        normals[0] = normalize(agent_position - points[0], fallback=np.array([1.0, 0.0]))
        planarity[0] = 0.2
        return normals, planarity

    k = min(neighbor_count, n_pts - 1)
    # One all-pairs distance matrix replaces the previous per-point rebuild.
    diff = points[:, None, :] - points[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    np.fill_diagonal(dist2, np.inf)
    for i in range(n_pts):
        nn = np.argpartition(dist2[i], k)[:k]
        local = points[np.concatenate(([i], nn))]
        centered = local - np.mean(local, axis=0)
        if len(local) < 2:
            tangent = np.array([1.0, 0.0])
        else:
            _, singular, vh = np.linalg.svd(centered, full_matrices=False)
            tangent = vh[0]
            if len(singular) >= 2:
                planarity[i] = float(np.clip(1.0 - singular[1] / max(singular[0], EPS), 0.0, 1.0))
            else:
                planarity[i] = 0.5
        normal = normalize(np.array([tangent[1], -tangent[0]], dtype=float), fallback=np.array([1.0, 0.0]))
        if float(np.dot(normal, agent_position - points[i])) < 0.0:
            normal = -normal
        normals[i] = normal
    return normals, planarity


def _arc_lengths_from_hits(points: np.ndarray) -> np.ndarray:
    """Approximate Δs_k from adjacent ray hits without closing visibility gaps."""
    n_pts = len(points)
    if n_pts == 0:
        return np.empty(0, dtype=float)
    if n_pts == 1:
        return np.array([1.0], dtype=float)

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    ds = np.empty(n_pts, dtype=float)
    ds[0] = segment_lengths[0]
    ds[-1] = segment_lengths[-1]
    if n_pts > 2:
        ds[1:-1] = 0.5 * (segment_lengths[:-1] + segment_lengths[1:])
    finite = segment_lengths[segment_lengths > 1e-6]
    upper = 3.0 * float(np.median(finite)) if len(finite) else 1.0
    return np.clip(ds, 1e-3, max(upper, 1e-3))


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
    random_seed: int = 0
    _ray_directions: np.ndarray | None = field(default=None, init=False, repr=False)
    _edge_cache_token: int = field(default=-1, init=False, repr=False)
    _edge_a: np.ndarray = field(default_factory=lambda: np.empty((0, 2)), init=False, repr=False)
    _edge_b: np.ndarray = field(default_factory=lambda: np.empty((0, 2)), init=False, repr=False)
    _edge_object: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.int64), init=False, repr=False)
    _object_ids: list[str] = field(default_factory=list, init=False, repr=False)

    def _directions(self) -> np.ndarray:
        if self._ray_directions is None or len(self._ray_directions) != int(self.num_rays):
            angles = np.linspace(0.0, 2.0 * np.pi, int(self.num_rays), endpoint=False)
            self._ray_directions = np.column_stack([np.cos(angles), np.sin(angles)]).astype(float)
        return self._ray_directions

    def _prepare_edges(self, cargoes: list[Cargo]) -> None:
        token = hash(tuple((c.object_id, id(c.vertices), len(c.vertices)) for c in cargoes))
        if token == self._edge_cache_token:
            return
        edge_a: list[np.ndarray] = []
        edge_b: list[np.ndarray] = []
        edge_object: list[int] = []
        object_ids: list[str] = []
        for obj_idx, cargo in enumerate(cargoes):
            object_ids.append(cargo.object_id)
            vertices = ensure_ccw(cargo.vertices)
            n = len(vertices)
            if n < 2:
                continue
            a = vertices
            b = np.roll(vertices, -1, axis=0)
            edge_a.append(a)
            edge_b.append(b)
            edge_object.extend([obj_idx] * n)
        if edge_a:
            self._edge_a = np.vstack(edge_a).astype(float, copy=False)
            self._edge_b = np.vstack(edge_b).astype(float, copy=False)
            self._edge_object = np.asarray(edge_object, dtype=np.int64)
        else:
            self._edge_a = np.empty((0, 2), dtype=float)
            self._edge_b = np.empty((0, 2), dtype=float)
            self._edge_object = np.empty(0, dtype=np.int64)
        self._object_ids = object_ids
        self._edge_cache_token = token

    def sense(self, agent: AgentState, cargoes: list[Cargo], timestamp: float) -> list[BoundaryObservation]:
        observations: list[BoundaryObservation] = []
        if not cargoes:
            return observations
        rng = np.random.default_rng(_stable_rng_seed(self.random_seed, agent.agent_id, timestamp))
        origin = np.asarray(agent.position, dtype=float).reshape(2)
        directions = self._directions()
        self._prepare_edges(cargoes)

        best_t, best_obj = nearest_ray_hits(
            origin,
            directions,
            self._edge_a,
            self._edge_b,
            self._edge_object,
            float(self.sensor_range),
            float(EPS),
        )

        hits_by_object: dict[str, list[tuple[int, np.ndarray]]] = {}
        for ray_index, (t, obj_idx) in enumerate(zip(best_t, best_obj)):
            if obj_idx < 0 or not np.isfinite(t):
                continue
            object_id = self._object_ids[int(obj_idx)]
            hits_by_object.setdefault(object_id, []).append(
                (ray_index, origin + float(t) * directions[ray_index])
            )

        for object_id, indexed_hits in hits_by_object.items():
            indexed_hits.sort(key=lambda item: item[0])
            points = np.asarray([point for _, point in indexed_hits], dtype=float)
            if self.noise_std > 0:
                points = points + rng.normal(scale=self.noise_std, size=points.shape)

            if len(points) > self.max_points_per_object:
                pick = np.linspace(0, len(points) - 1, self.max_points_per_object).astype(int)
                points = points[pick]

            normals, planarities = _estimate_normals_pca(
                points, origin, neighbor_count=self.normal_neighbors
            )
            arc_lengths = _arc_lengths_from_hits(points)
            support = float(np.clip(len(points) / max(self.normal_neighbors + 1, 1), 0.2, 1.0))
            for point, normal, ds, planar in zip(points, normals, arc_lengths, planarities):
                # Confidence combines range, local line fit, and neighborhood support.
                dist = float(np.linalg.norm(point - origin))
                range_score = float(np.clip(1.0 - dist / max(self.sensor_range, EPS), 0.0, 1.0))
                confidence = self.base_confidence * (
                    0.20 + 0.35 * range_score + 0.35 * float(planar) + 0.10 * support
                )
                observations.append(
                    BoundaryObservation(
                        object_id=object_id,
                        agent_id=agent.agent_id,
                        point=point,
                        normal=normal,
                        timestamp=timestamp,
                        confidence=float(np.clip(confidence, 0.05, 1.0)),
                        arc_length=float(ds),
                    )
                )
        return observations
