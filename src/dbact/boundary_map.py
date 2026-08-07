from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .types import BoundaryObservation


@dataclass
class LocalBoundaryMap:
    """Per-agent local memory of boundary observations.

    Paper-grade map features:
    - spatial voxel deduplication (avoids density mass inflation from comm relay)
    - confidence fusion / highest-confidence retention
    - TTL prune with optional age-decay weights exposed to density
    """

    ttl: float = 4.0
    max_points_per_object: int = 160
    voxel_size: float = 0.08
    decay_lambda: float = 0.35
    fusion: str = "confidence_priority"  # confidence_priority | latest | average
    observations: dict[str, list[BoundaryObservation]] = field(default_factory=dict)

    def update(self, new_observations: list[BoundaryObservation], timestamp: float) -> None:
        for obs in new_observations:
            self._upsert(obs)
        self.prune(timestamp)

    def _voxel_key(self, obs: BoundaryObservation) -> tuple[str, int, int]:
        vs = max(self.voxel_size, 1e-6)
        return (
            obs.object_id,
            int(np.round(float(obs.point[0]) / vs)),
            int(np.round(float(obs.point[1]) / vs)),
        )

    def _upsert(self, obs: BoundaryObservation) -> None:
        key = self._voxel_key(obs)
        bucket = self.observations.setdefault(obs.object_id, [])
        for idx, existing in enumerate(bucket):
            if self._voxel_key(existing) != key:
                continue
            bucket[idx] = self._fuse(existing, obs)
            return
        bucket.append(obs)

    def _fuse(self, old: BoundaryObservation, new: BoundaryObservation) -> BoundaryObservation:
        mode = self.fusion
        if mode == "latest":
            return new
        if mode == "average":
            w_old = max(old.confidence, 1e-6)
            w_new = max(new.confidence, 1e-6)
            w_sum = w_old + w_new
            point = (w_old * old.point + w_new * new.point) / w_sum
            normal = (w_old * old.normal + w_new * new.normal) / w_sum
            nrm = float(np.linalg.norm(normal))
            if nrm > 1e-9:
                normal = normal / nrm
            return BoundaryObservation(
                object_id=new.object_id,
                agent_id=new.agent_id,
                point=point,
                normal=normal,
                timestamp=max(old.timestamp, new.timestamp),
                confidence=min(1.0, 0.5 * (old.confidence + new.confidence) + 0.25),
                arc_length=0.5 * (old.arc_length + new.arc_length),
                gap_score=max(old.gap_score, new.gap_score),
            )
        # confidence_priority (default): keep higher confidence; break ties by recency
        if new.confidence > old.confidence or (
            abs(new.confidence - old.confidence) < 1e-9 and new.timestamp >= old.timestamp
        ):
            return new
        return old

    def prune(self, timestamp: float) -> None:
        for object_id in list(self.observations):
            fresh = [obs for obs in self.observations[object_id] if timestamp - obs.timestamp <= self.ttl]
            if len(fresh) > self.max_points_per_object:
                fresh = sorted(fresh, key=lambda o: (o.confidence, o.timestamp), reverse=True)[
                    : self.max_points_per_object
                ]
            if fresh:
                self.observations[object_id] = fresh
            else:
                del self.observations[object_id]

    def age_weight(self, obs: BoundaryObservation, timestamp: float) -> float:
        """Temporal decay e^{-λ(t - t_k)} for boundary-measure density."""
        age = max(0.0, float(timestamp) - float(obs.timestamp))
        return float(np.exp(-self.decay_lambda * age))

    def all_observations(self, timestamp: float | None = None) -> list[BoundaryObservation]:
        if timestamp is not None:
            self.prune(timestamp)
        out: list[BoundaryObservation] = []
        for obs_list in self.observations.values():
            out.extend(obs_list)
        return out

    def object_ids(self) -> list[str]:
        return list(self.observations.keys())
