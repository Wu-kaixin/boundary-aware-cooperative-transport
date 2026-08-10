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
    motion_compensation: bool = True
    motion_match_radius: float = 0.20
    motion_min_matches: int = 3
    max_translation_per_update: float = 0.08
    observations: dict[str, list[BoundaryObservation]] = field(default_factory=dict)
    _voxel_index: dict[tuple[str, int, int], int] = field(default_factory=dict, init=False, repr=False)
    _last_prune_timestamp: float | None = field(default=None, init=False, repr=False)

    def update(self, new_observations: list[BoundaryObservation], timestamp: float) -> None:
        if self.motion_compensation and new_observations:
            grouped: dict[str, list[BoundaryObservation]] = {}
            for obs in new_observations:
                grouped.setdefault(obs.object_id, []).append(obs)
            for object_id, batch in grouped.items():
                self._compensate_translation(object_id, batch)
        for obs in new_observations:
            self._upsert(obs)
        self.prune(timestamp)

    @staticmethod
    def _clone(obs: BoundaryObservation, *, point: np.ndarray | None = None) -> BoundaryObservation:
        return BoundaryObservation(
            object_id=obs.object_id,
            agent_id=obs.agent_id,
            point=obs.point.copy() if point is None else np.asarray(point, dtype=float),
            normal=obs.normal.copy(),
            timestamp=obs.timestamp,
            confidence=obs.confidence,
            arc_length=obs.arc_length,
            gap_score=obs.gap_score,
        )

    def _rebuild_index_for_object(self, object_id: str) -> None:
        for key in [k for k in self._voxel_index if k[0] == object_id]:
            del self._voxel_index[key]
        for idx, obs in enumerate(self.observations.get(object_id, [])):
            self._voxel_index[self._voxel_key(obs)] = idx

    def _compensate_translation(
        self,
        object_id: str,
        new_observations: list[BoundaryObservation],
    ) -> None:
        """Shift a world-frame map using robust nearest-neighbor translation.

        Compensation runs at most once per newer timestamp. Same-frame relayed
        observations therefore cannot move the map repeatedly.
        """
        old = self.observations.get(object_id, [])
        if len(old) < self.motion_min_matches or len(new_observations) < self.motion_min_matches:
            return
        latest_old = max(float(obs.timestamp) for obs in old)
        latest_new = max(float(obs.timestamp) for obs in new_observations)
        if latest_new <= latest_old + 1e-12:
            return
        old_points = np.vstack([obs.point for obs in old])
        old_normals = np.vstack([obs.normal for obs in old])
        new_points = np.vstack([obs.point for obs in new_observations])
        new_normals = np.vstack([obs.normal for obs in new_observations])
        # Vectorized nearest-neighbor candidates among old map points.
        dist2 = np.sum((new_points[:, None, :] - old_points[None, :, :]) ** 2, axis=2)
        deltas: list[np.ndarray] = []
        k = min(5, len(old))
        for i, obs in enumerate(new_observations):
            order = np.argpartition(dist2[i], kth=k - 1)[:k]
            order = order[np.argsort(dist2[i, order])]
            for idx in order:
                distance = float(np.sqrt(dist2[i, int(idx)]))
                alignment = float(np.dot(old_normals[int(idx)], new_normals[i]))
                if distance <= self.motion_match_radius and alignment >= 0.7:
                    deltas.append(obs.point - old_points[int(idx)])
                    break
        if len(deltas) < self.motion_min_matches:
            return
        translation = np.median(np.vstack(deltas), axis=0)
        magnitude = float(np.linalg.norm(translation))
        if magnitude <= 1e-6 or magnitude > self.max_translation_per_update:
            return
        self.observations[object_id] = [
            self._clone(obs, point=obs.point + translation) for obs in old
        ]
        self._rebuild_index_for_object(object_id)

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
        idx = self._voxel_index.get(key)
        if idx is not None and 0 <= idx < len(bucket) and self._voxel_key(bucket[idx]) == key:
            bucket[idx] = self._fuse(bucket[idx], obs)
            return
        # Fallback linear scan keeps correctness if the index is stale.
        for existing_idx, existing in enumerate(bucket):
            if self._voxel_key(existing) != key:
                continue
            bucket[existing_idx] = self._fuse(existing, obs)
            self._voxel_index[key] = existing_idx
            return
        self._voxel_index[key] = len(bucket)
        bucket.append(self._clone(obs))

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
        # confidence_priority (default): retain the best geometry estimate, but
        # always refresh its timestamp when the same voxel is re-observed. This
        # prevents a high-confidence old sample from expiring despite continuous
        # local sensing. Relayed duplicates with an identical timestamp still
        # occupy exactly one voxel and therefore cannot inflate density mass.
        best = new if new.confidence >= old.confidence else old
        latest_timestamp = max(float(old.timestamp), float(new.timestamp))
        return BoundaryObservation(
            object_id=best.object_id,
            agent_id=best.agent_id,
            point=best.point.copy(),
            normal=best.normal.copy(),
            timestamp=latest_timestamp,
            confidence=max(float(old.confidence), float(new.confidence)),
            arc_length=max(float(old.arc_length), float(new.arc_length)),
            gap_score=max(float(old.gap_score), float(new.gap_score)),
        )

    def prune(self, timestamp: float) -> None:
        if self._last_prune_timestamp is not None and abs(timestamp - self._last_prune_timestamp) < 1e-15:
            return
        changed = False
        for object_id in list(self.observations):
            fresh = [obs for obs in self.observations[object_id] if timestamp - obs.timestamp <= self.ttl]
            if len(fresh) > self.max_points_per_object:
                fresh = sorted(fresh, key=lambda o: (o.confidence, o.timestamp), reverse=True)[
                    : self.max_points_per_object
                ]
            if fresh:
                if fresh is not self.observations[object_id] and (
                    len(fresh) != len(self.observations[object_id]) or fresh != self.observations[object_id]
                ):
                    self.observations[object_id] = fresh
                    self._rebuild_index_for_object(object_id)
                    changed = True
            else:
                del self.observations[object_id]
                for key in [k for k in self._voxel_index if k[0] == object_id]:
                    del self._voxel_index[key]
                changed = True
        self._last_prune_timestamp = float(timestamp)
        del changed

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

    def voxel_count(self, object_id: str | None = None) -> int:
        """Return the number of unique spatial memory cells."""
        if object_id is not None:
            return len(self.observations.get(object_id, []))
        return sum(len(bucket) for bucket in self.observations.values())

    def timestamp_span(self, object_id: str) -> float:
        """Observed time span retained for diagnostics (seconds)."""
        bucket = self.observations.get(object_id, [])
        if len(bucket) < 2:
            return 0.0
        stamps = [float(obs.timestamp) for obs in bucket]
        return max(stamps) - min(stamps)
