"""S3 - map layer: per-agent voxel map of observed boundary.

The previous map was a list of raw observation packets with a TTL and a capacity
cap. Two things went wrong with that. Capacity applied to raw packets, so at
realistic scan rates the buffer filled in a fraction of a second and started
discarding *fresh* geometry. And because every relay of the same boundary point
was stored again, a point observed by several neighbours contributed several
times to the density, so the coverage law was pulled towards whichever piece of
boundary happened to be talked about most.

The fix is to key storage by a spatial cell:

    key = (object_id, round(point / voxel_size))

One fused record per cell, capacity applied to *cells* rather than packets, and
the fusion rules chosen so that repetition cannot inflate density mass:

* ``point`` and ``normal``  confidence-weighted averages
* ``confidence``           maximum, not sum
* ``arc_length``           maximum over sources, each source's contribution
  clamped to the cell diagonal -- a cell of side ``v`` cannot represent more than
  ``v*sqrt(2)`` of boundary no matter how many robots report it

Age enters at read time as ``exp(-lambda_age (t - t_k))`` rather than as a hard
cut, so an observation fades instead of vanishing between two consecutive steps.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .types import BoundaryObservation


@dataclass
class VoxelRecord:
    object_id: str
    point: np.ndarray
    normal: np.ndarray
    confidence: float
    arc_length: float
    timestamp: float
    weight_sum: float = 0.0
    observations: int = 0


@dataclass
class LocalBoundaryMap:
    """Per-agent local memory of observed boundary, fused on a voxel grid."""

    voxel_size: float = 0.06
    age_decay: float = 0.30
    max_voxels_per_object: int = 600
    min_weight: float = 1e-3
    records: dict[tuple[str, int, int], VoxelRecord] = field(default_factory=dict)

    # ------------------------------------------------------------------ #

    def _key(self, object_id: str, point: np.ndarray) -> tuple[str, int, int]:
        cell = np.round(np.asarray(point, dtype=float) / self.voxel_size).astype(int)
        return (object_id, int(cell[0]), int(cell[1]))

    @property
    def voxel_diagonal(self) -> float:
        return self.voxel_size * math.sqrt(2.0)

    def update(self, new_observations: list[BoundaryObservation], timestamp: float) -> None:
        if not new_observations:
            self.prune(timestamp)
            return

        # A relayed observation is an exact duplicate of one already in the batch,
        # and it must count once. Deduplicating on (source, time, point) does that
        # while still letting two genuinely different returns from the same scan
        # both contribute to the same cell.
        unique: dict[tuple[str, float, bytes], BoundaryObservation] = {}
        for obs in new_observations:
            unique.setdefault(
                (obs.agent_id, float(obs.timestamp), np.asarray(obs.point, dtype=float).tobytes()), obs
            )
        observations = list(unique.values())

        # Arc length is summed only within one scan of one robot; across robots
        # and across time it is a maximum. Summing everywhere would let a relayed
        # observation add mass that no extra boundary corresponds to.
        batch: dict[tuple[tuple[str, int, int], str], float] = {}
        for obs in observations:
            key = (self._key(obs.object_id, obs.point), obs.agent_id)
            batch[key] = batch.get(key, 0.0) + float(obs.arc_length)

        for obs in observations:
            key = self._key(obs.object_id, obs.point)
            scan_arc = min(batch[(key, obs.agent_id)], self.voxel_diagonal)
            weight = max(float(obs.confidence), 1e-6)
            record = self.records.get(key)
            if record is None:
                self.records[key] = VoxelRecord(
                    object_id=obs.object_id,
                    point=np.asarray(obs.point, dtype=float).copy(),
                    normal=np.asarray(obs.normal, dtype=float).copy(),
                    confidence=float(obs.confidence),
                    arc_length=scan_arc,
                    timestamp=float(obs.timestamp),
                    weight_sum=weight,
                    observations=1,
                )
                continue

            total = record.weight_sum + weight
            record.point = (record.point * record.weight_sum + np.asarray(obs.point, dtype=float) * weight) / total
            fused_normal = record.normal * record.weight_sum + np.asarray(obs.normal, dtype=float) * weight
            norm = float(np.linalg.norm(fused_normal))
            if norm > 1e-9:
                record.normal = fused_normal / norm
            record.weight_sum = total
            record.confidence = max(record.confidence, float(obs.confidence))
            record.arc_length = max(record.arc_length, scan_arc)
            record.timestamp = max(record.timestamp, float(obs.timestamp))
            record.observations += 1

        self.prune(timestamp)

    def prune(self, timestamp: float) -> None:
        """Drop faded cells, then apply the capacity cap per object.

        The cap is on *cells*, so it bounds the map's spatial extent rather than
        its update rate. When it binds, the cells kept are the ones with the
        largest current (decayed) weight.
        """
        stale = [key for key, rec in self.records.items() if self._decay(rec, timestamp) < self.min_weight]
        for key in stale:
            del self.records[key]

        by_object: dict[str, list[tuple[str, int, int]]] = {}
        for key, rec in self.records.items():
            by_object.setdefault(rec.object_id, []).append(key)
        for keys in by_object.values():
            if len(keys) <= self.max_voxels_per_object:
                continue
            ranked = sorted(
                keys,
                key=lambda k: self._decay(self.records[k], timestamp) * max(self.records[k].arc_length, 1e-6),
                reverse=True,
            )
            for key in ranked[self.max_voxels_per_object :]:
                del self.records[key]

    def _decay(self, record: VoxelRecord, timestamp: float) -> float:
        age = max(0.0, float(timestamp) - record.timestamp)
        return float(record.confidence * math.exp(-self.age_decay * age))

    # ------------------------------------------------------------------ #

    def all_observations(self, timestamp: float | None = None) -> list[BoundaryObservation]:
        """Fused cells as observations, with age decay folded into confidence."""
        if timestamp is not None:
            self.prune(timestamp)
        t = timestamp if timestamp is not None else max((r.timestamp for r in self.records.values()), default=0.0)
        out: list[BoundaryObservation] = []
        for record in self.records.values():
            out.append(
                BoundaryObservation(
                    object_id=record.object_id,
                    agent_id="map",
                    point=record.point.copy(),
                    normal=record.normal.copy(),
                    timestamp=record.timestamp,
                    confidence=self._decay(record, t),
                    arc_length=record.arc_length,
                )
            )
        return out

    def object_ids(self) -> list[str]:
        return sorted({record.object_id for record in self.records.values()})

    def total_arc_length(self, object_id: str | None = None) -> float:
        """Estimated observed perimeter -- the total mass the density carries."""
        return float(
            sum(
                rec.arc_length
                for rec in self.records.values()
                if object_id is None or rec.object_id == object_id
            )
        )

    def __len__(self) -> int:
        return len(self.records)


__all__ = ["LocalBoundaryMap", "VoxelRecord"]
