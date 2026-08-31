"""S2 - perception layer: ray casting with local PCA normals.

Replaces the previous sampler, which returned every boundary sample within
sensor range. That sampler was see-through: a robot standing on one side of an
L-shaped object received points from the far side it could not possibly observe,
and it received the simulator's exact normals, so the normal-error term of the
robustness analysis was identically zero by construction.

Here each ray keeps only its nearest hit across all objects, so self-occlusion
and inter-object occlusion both fall out for free, and normals are estimated from
the returned points alone.

Confidence is a fit residual, not an eigenvalue ratio
----------------------------------------------------
The obvious linearity score ``c = 1 - lambda_min / lambda_max`` does not work
here. Ray-scan point spacing is strongly anisotropic, so a corner neighbourhood
still looks "linear" by ratio: measured on returns whose normal error exceeded
30 degrees, that score still reported 0.83-0.92. The residual form

    sigma_r = sqrt(lambda_min)                     # offline fit RMS
    c       = clip(1 - sigma_r / residual_tolerance, 0, 1)

carries physical units and rejects those returns, because a corner
neighbourhood has a genuinely large fit residual regardless of how the points are
spaced. ``residual_tolerance`` should be set to roughly three times the range
noise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .cargo import Cargo
from .geometry import outward_edge_normals, ray_batch_first_hits, segment_hits_polygon
from .provenance import frame_rng
from .types import AgentState, BoundaryObservation, BoundaryView


@dataclass
class PerceptionParams:
    sensor_range: float = 1.2
    ray_count: int = 96
    range_noise_std: float = 0.0
    pca_neighbors: int = 5
    residual_tolerance: float = 0.03
    min_confidence: float = 0.15
    max_arc_gap: float = 0.25
    seed: int = 0


@dataclass
class RayCastBoundarySensor:
    """Simulated 2-D range scanner over the true cargo geometry.

    In simulation the polygons are the ground truth the rays are cast against.
    On hardware this class is replaced by a LiDAR/depth driver producing the same
    ``BoundaryObservation`` records; nothing downstream reads cargo geometry.
    """

    params: PerceptionParams = field(default_factory=PerceptionParams)

    def sense(
        self,
        agent: AgentState,
        cargoes: list[Cargo],
        timestamp: float,
        apply_gate: bool = True,
    ) -> list[BoundaryObservation]:
        view = self.sense_view(agent, cargoes, timestamp, apply_gate=apply_gate)
        return view.to_observations(timestamp=timestamp, agent_id=agent.agent_id)

    def sense_view(
        self,
        agent: AgentState,
        cargoes: list[Cargo],
        timestamp: float,
        apply_gate: bool = True,
    ) -> BoundaryView:
        """The same scan as ``sense``, as arrays. This is the inner-loop form."""
        returns = self.raw_returns(agent, cargoes, timestamp)
        view = self._estimate_normals_view(agent, returns)
        if apply_gate and len(view):
            view = view.select(view.confidence >= self.params.min_confidence)
        return view

    # ------------------------------------------------------------------ #
    # ray casting
    # ------------------------------------------------------------------ #

    def raw_returns(self, agent: AgentState, cargoes: list[Cargo], timestamp: float) -> list[dict]:
        """Nearest hit per ray across all objects, with range noise along the ray.

        Taking the minimum over objects as well as over edges is what makes one
        object able to occlude another, not just itself.
        """
        if not cargoes:
            return []
        origin = np.asarray(agent.position, dtype=float).reshape(2)
        rng = frame_rng(agent.agent_id, round(float(timestamp), 6), base=self.params.seed)
        count = int(self.params.ray_count)
        angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
        directions = np.column_stack([np.cos(angles), np.sin(angles)])
        noise = (
            rng.normal(scale=self.params.range_noise_std, size=count)
            if self.params.range_noise_std > 0.0
            else np.zeros(count)
        )

        best_range = np.full(count, np.inf)
        best_object = np.full(count, -1, dtype=int)
        best_edge = np.zeros(count, dtype=int)
        for index, cargo in enumerate(cargoes):
            ranges, edges, _ = ray_batch_first_hits(origin, directions, cargo.vertices, self.params.sensor_range)
            closer = ranges < best_range
            best_range = np.where(closer, ranges, best_range)
            best_object = np.where(closer, index, best_object)
            best_edge = np.where(closer, edges, best_edge)

        hit_rays = np.where(best_object >= 0)[0]
        if len(hit_rays) == 0:
            return []
        normals_by_object = [outward_edge_normals(cargo.vertices) for cargo in cargoes]
        measured = np.maximum(1e-6, best_range[hit_rays] + noise[hit_rays])
        points = origin[None, :] + measured[:, None] * directions[hit_rays]

        return [
            {
                "object_id": cargoes[best_object[k]].object_id,
                "point": points[n],
                "true_normal": normals_by_object[best_object[k]][best_edge[k]],
                "range": float(measured[n]),
                "ray_index": int(k),
                "direction": directions[k],
            }
            for n, k in enumerate(hit_rays)
        ]

    # ------------------------------------------------------------------ #
    # normal estimation
    # ------------------------------------------------------------------ #

    def _estimate_normals(
        self,
        agent: AgentState,
        returns: list[dict],
        timestamp: float,
    ) -> list[BoundaryObservation]:
        view = self._estimate_normals_view(agent, returns)
        return view.to_observations(timestamp=timestamp, agent_id=agent.agent_id)

    def _estimate_normals_view(self, agent: AgentState, returns: list[dict]) -> BoundaryView:
        """Batched local plane fit: one ``eigh`` call for the whole scan.

        The fit is per return over its ``k`` nearest neighbours in the same scan,
        exactly as the per-point version, but the neighbourhoods are gathered into
        one ``(N, k, 2)`` array so the covariances and their eigen-decompositions
        are computed together. One scan is at most ``ray_count`` points, so the
        dense pairwise distance is cheaper than any index would be.
        """
        if not returns:
            return BoundaryView.empty()
        by_object: dict[str, list[dict]] = {}
        for item in returns:
            by_object.setdefault(item["object_id"], []).append(item)

        origin = np.asarray(agent.position, dtype=float).reshape(2)
        chunks: list[BoundaryView] = []
        for object_id, items in by_object.items():
            points = np.vstack([item["point"] for item in items])
            arc = self._arc_lengths(points, [item["ray_index"] for item in items])
            k = min(self.params.pca_neighbors, len(points))

            if k < 3:
                outward = points - origin[None, :]
                norm = np.linalg.norm(outward, axis=1, keepdims=True)
                normals = np.where(norm > 1e-9, -outward / np.maximum(norm, 1e-9), np.array([1.0, 0.0]))
                residual = np.full(len(points), np.inf)
            else:
                d2 = np.sum((points[:, None, :] - points[None, :, :]) ** 2, axis=2)
                neighbours = points[np.argsort(d2, axis=1, kind="stable")[:, :k]]
                centered = neighbours - neighbours.mean(axis=1, keepdims=True)
                cov = np.einsum("nki,nkj->nij", centered, centered) / max(1, k - 1)
                eigenvalues, eigenvectors = np.linalg.eigh(cov)
                normals = eigenvectors[:, :, 0]
                residual = np.sqrt(np.maximum(eigenvalues[:, 0], 0.0))
                # Orient outward: the observer stands outside, so the outward normal
                # has a positive component along (observer - boundary point).
                flip = np.einsum("ij,ij->i", normals, origin[None, :] - points) < 0.0
                normals = np.where(flip[:, None], -normals, normals)

            confidence = np.clip(1.0 - residual / max(self.params.residual_tolerance, 1e-9), 0.0, 1.0)
            chunks.append(
                BoundaryView(
                    points=points,
                    normals=normals,
                    confidence=confidence,
                    arc_length=arc,
                    object_ids=np.full(len(points), object_id, dtype="<U32"),
                )
            )

        if len(chunks) == 1:
            return chunks[0]
        return BoundaryView(
            points=np.vstack([c.points for c in chunks]),
            normals=np.vstack([c.normals for c in chunks]),
            confidence=np.concatenate([c.confidence for c in chunks]),
            arc_length=np.concatenate([c.arc_length for c in chunks]),
            object_ids=np.concatenate([c.object_ids for c in chunks]),
        )

    def _pca_normal(self, points: np.ndarray, index: int, k: int, origin: np.ndarray) -> tuple[np.ndarray, float]:
        target = points[index]
        if k < 3:
            fallback = target - origin
            norm = float(np.linalg.norm(fallback))
            direction = -fallback / norm if norm > 1e-9 else np.array([1.0, 0.0])
            return direction, float("inf")

        distances = np.linalg.norm(points - target[None, :], axis=1)
        neighborhood = points[np.argsort(distances)[:k]]
        centered = neighborhood - neighborhood.mean(axis=0, keepdims=True)
        cov = centered.T @ centered / max(1, len(neighborhood) - 1)
        eigvals, eigvecs = np.linalg.eigh(cov)
        normal = eigvecs[:, 0]
        residual = math.sqrt(max(0.0, float(eigvals[0])))

        # Orient outward: the observer stands outside, so the outward normal has a
        # positive component along (observer - boundary point).
        if float(np.dot(normal, origin - target)) < 0.0:
            normal = -normal
        return normal, residual

    def _arc_lengths(self, points: np.ndarray, ray_indices: list[int]) -> np.ndarray:
        """Boundary length each return stands for.

        Rays sweep monotonically in angle, so consecutive returns lie on the same
        visible arc unless there is a jump. Gaps larger than ``max_arc_gap`` mark
        an occlusion boundary or a different face and are not bridged.
        """
        order = np.argsort(np.asarray(ray_indices))
        ordered = points[order]
        n = len(ordered)
        arc = np.zeros(n)
        if n == 1:
            arc[0] = 0.0
            return arc
        gaps = np.linalg.norm(np.diff(ordered, axis=0), axis=1)
        gaps = np.where(gaps > self.params.max_arc_gap, 0.0, gaps)
        arc_sorted = np.zeros(n)
        arc_sorted[:-1] += 0.5 * gaps
        arc_sorted[1:] += 0.5 * gaps
        arc[order] = arc_sorted
        return arc


@dataclass
class LegacyProximitySampler:
    """Pre-refactor sensor, kept as the perception baseline (B0).

    Returns every boundary sample within ``sensor_range`` together with the
    simulator's exact outward normal. Two consequences, both of which the audit
    scripts quantify rather than assert: the sensor sees through the object, and
    the normal-estimate error is identically zero, so any robustness margin
    derived against it is vacuous.
    """

    sensor_range: float = 1.2
    boundary_samples_per_object: int = 160
    max_points_per_object: int = 24

    def sense(self, agent: AgentState, cargoes: list[Cargo], timestamp: float) -> list[BoundaryObservation]:
        observations: list[BoundaryObservation] = []
        for cargo in cargoes:
            points, normals = cargo.boundary_samples(self.boundary_samples_per_object)
            distances = np.linalg.norm(points - agent.position[None, :], axis=1)
            visible = np.where(distances <= self.sensor_range)[0]
            if len(visible) > self.max_points_per_object:
                pick = np.linspace(0, len(visible) - 1, self.max_points_per_object).astype(int)
                visible = visible[pick]
            for idx in visible:
                observations.append(
                    BoundaryObservation(
                        object_id=cargo.object_id,
                        agent_id=agent.agent_id,
                        point=points[idx].copy(),
                        normal=normals[idx].copy(),
                        timestamp=timestamp,
                        confidence=1.0,
                        arc_length=cargo.perimeter / self.boundary_samples_per_object,
                    )
                )
        return observations


# --------------------------------------------------------------------------- #
# perception audits
# --------------------------------------------------------------------------- #


def occlusion_rate(
    observations: list[BoundaryObservation],
    observer: np.ndarray,
    cargoes: list[Cargo],
    tolerance: float = 0.0,
) -> tuple[int, int]:
    """Count returns whose line of sight is cut by a cargo. Returns (bad, total).

    ``tolerance`` must be about three times the range noise. Without it the audit
    reports its own noise: roughly half of the noisy returns land just inside the
    boundary, and a zero-tolerance line-of-sight test then calls every one of them
    occluded -- which is how a clean sensor can be made to look 45-54% occluded.

    A geometric epsilon is always added on top. A noiseless return lies exactly on
    the boundary, so the segment from the observer to it touches the polygon at its
    own endpoint; without the epsilon every such return is reported as blocking its
    own line of sight and a perfect sensor audits at 100% occlusion.
    """
    origin = np.asarray(observer, dtype=float).reshape(2)
    trim = max(float(tolerance), 1e-6)
    blocked = 0
    for obs in observations:
        for cargo in cargoes:
            if segment_hits_polygon(origin, obs.point, cargo.vertices, tolerance=trim):
                blocked += 1
                break
    return blocked, len(observations)


def normal_errors_deg(observations: list[BoundaryObservation], cargoes: list[Cargo]) -> np.ndarray:
    """Angle between each estimated normal and the true outward normal."""
    from .geometry import signed_distance_and_gradient

    by_id = {cargo.object_id: cargo for cargo in cargoes}
    errors: list[float] = []
    for obs in observations:
        cargo = by_id.get(obs.object_id)
        if cargo is None:
            continue
        _, grad, _ = signed_distance_and_gradient(obs.point[None, :], cargo.vertices)
        cosine = float(np.clip(np.dot(obs.normal, grad[0]), -1.0, 1.0))
        errors.append(math.degrees(math.acos(cosine)))
    return np.asarray(errors, dtype=float)


__all__ = [
    "PerceptionParams",
    "RayCastBoundarySensor",
    "LegacyProximitySampler",
    "occlusion_rate",
    "normal_errors_deg",
]
