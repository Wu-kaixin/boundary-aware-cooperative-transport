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
    motion_compensation: bool = True
    motion_match_radius: float = 0.18
    motion_min_matches: int = 5
    max_translation_per_update: float = 0.04
    max_rotation_per_update: float = 0.12
    records: dict[tuple[str, int, int], VoxelRecord] = field(default_factory=dict)
    last_motion: dict[str, np.ndarray] = field(default_factory=dict)
    last_rotation: dict[str, float] = field(default_factory=dict)

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

        if self.motion_compensation:
            grouped: dict[str, list[BoundaryObservation]] = {}
            for obs in new_observations:
                grouped.setdefault(obs.object_id, []).append(obs)
            for object_id, batch in grouped.items():
                self._compensate_translation(object_id, batch)

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

        # Quantise a scan in one NumPy operation.  Calling ``np.round`` once per
        # packet dominated long closed-loop runs even though every packet is
        # headed for the same fixed voxel grid.
        cells = np.rint(
            np.vstack([obs.point for obs in observations]) / float(self.voxel_size)
        ).astype(int)
        voxel_keys = [
            (obs.object_id, int(cell[0]), int(cell[1]))
            for obs, cell in zip(observations, cells)
        ]

        # Arc length is summed only within one scan of one robot; across robots
        # and across time it is a maximum. Summing everywhere would let a relayed
        # observation add mass that no extra boundary corresponds to.
        batch: dict[tuple[tuple[str, int, int], str], float] = {}
        for obs, voxel_key in zip(observations, voxel_keys):
            key = (voxel_key, obs.agent_id)
            batch[key] = batch.get(key, 0.0) + float(obs.arc_length)

        for obs, key in zip(observations, voxel_keys):
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

    def merge_observations(
        self,
        observations: list[BoundaryObservation],
        timestamp: float,
    ) -> None:
        """Idempotently merge a relayed voxel map without re-running ICP.

        Relayed records are historical state, not a new scan.  Feeding them to
        :meth:`update` made point-to-plane registration interpret gossip traffic
        as object motion and rebuilt arrays containing several complete neighbour
        maps every perception frame.  This merge keeps at most one record per
        target voxel and selects the freshest geometry, so replaying the same
        packet cannot add mass or change the motion estimate.
        """
        for obs in observations:
            key = self._key(obs.object_id, obs.point)
            incumbent = self.records.get(key)
            if incumbent is None:
                weight = max(float(obs.confidence), 1e-6)
                self.records[key] = VoxelRecord(
                    object_id=obs.object_id,
                    point=np.asarray(obs.point, dtype=float).copy(),
                    normal=np.asarray(obs.normal, dtype=float).copy(),
                    confidence=float(obs.confidence),
                    arc_length=min(float(obs.arc_length), self.voxel_diagonal),
                    timestamp=float(obs.timestamp),
                    weight_sum=weight,
                    observations=1,
                )
                continue
            if float(obs.timestamp) > incumbent.timestamp + 1e-12:
                incumbent.point = np.asarray(obs.point, dtype=float).copy()
                incumbent.normal = np.asarray(obs.normal, dtype=float).copy()
                incumbent.timestamp = float(obs.timestamp)
            incumbent.confidence = max(incumbent.confidence, float(obs.confidence))
            incumbent.arc_length = max(
                incumbent.arc_length,
                min(float(obs.arc_length), self.voxel_diagonal),
            )
            incumbent.weight_sum = max(incumbent.weight_sum, float(obs.confidence), 1e-6)
        self.prune(timestamp)

    def _compensate_translation(
        self,
        object_id: str,
        observations: list[BoundaryObservation],
    ) -> None:
        """Move the world-frame map with a translating rigid body.

        Point-to-point nearest-neighbour motion is biased along a straight edge:
        a newly sampled point can slide along that edge even when the object is
        static.  Point-to-plane ICP removes that ambiguity.  For every
        normal-compatible match it contributes

            n_k.T t = n_k.T (b_new - b_old),

        and the least-squares translation ``t`` is applied once per newer scan.
        With two non-parallel visible faces the estimate is full rank; with one
        face the minimum-norm solution moves only in the observable normal
        direction.  Same-frame relays cannot move the map repeatedly because the
        stored records already carry that timestamp after the first update.
        """
        self.last_motion[object_id] = np.zeros(2, dtype=float)
        self.last_rotation[object_id] = 0.0
        old = [rec for rec in self.records.values() if rec.object_id == object_id]
        if len(old) < self.motion_min_matches or len(observations) < self.motion_min_matches:
            return
        latest_old = max(float(rec.timestamp) for rec in old)
        latest_new = max(float(obs.timestamp) for obs in observations)
        if latest_new <= latest_old + 1e-12:
            return

        old_points = np.vstack([rec.point for rec in old])
        old_normals = np.vstack([rec.normal for rec in old])
        new_points = np.vstack([obs.point for obs in observations])
        new_normals = np.vstack([obs.normal for obs in observations])
        dist2 = np.sum((new_points[:, None, :] - old_points[None, :, :]) ** 2, axis=2)
        alignment = new_normals @ old_normals.T
        admissible = alignment >= 0.70
        masked = np.where(admissible, dist2, np.inf)
        match = np.argmin(masked, axis=1)
        best = masked[np.arange(len(new_points)), match]
        valid = np.isfinite(best) & (best <= self.motion_match_radius ** 2)
        if int(np.sum(valid)) < self.motion_min_matches:
            return

        q_old = old_points[match[valid]]
        n_old = old_normals[match[valid]]
        q_new = new_points[valid]
        rhs = np.sum(n_old * (q_new - q_old), axis=1)
        center = np.mean(old_points, axis=0)
        lever = q_old - center[None, :]
        rotational_column = np.sum(n_old * np.column_stack([-lever[:, 1], lever[:, 0]]), axis=1)
        system = np.column_stack([n_old, rotational_column])
        try:
            twist, _, _, _ = np.linalg.lstsq(system, rhs, rcond=None)
        except np.linalg.LinAlgError:
            return

        residual = np.abs(system @ twist - rhs)
        tolerance = max(0.5 * self.voxel_size, 3.0 * float(np.median(residual)) + 1e-9)
        inliers = residual <= tolerance
        if int(np.sum(inliers)) >= self.motion_min_matches and not np.all(inliers):
            try:
                twist, _, _, _ = np.linalg.lstsq(system[inliers], rhs[inliers], rcond=None)
            except np.linalg.LinAlgError:
                return

        translation = np.asarray(twist[:2], dtype=float)
        rotation = float(twist[2])
        magnitude = float(np.linalg.norm(translation))
        # Reject implausible components independently.  A corner can make the
        # angular column ill-conditioned while the two translational columns are
        # still well observed; discarding the complete twist then freezes task
        # progress under range noise.  Clipping is deliberately avoided because
        # it would turn an outlier into a plausible-looking motion increment.
        if magnitude > self.max_translation_per_update:
            translation = np.zeros(2, dtype=float)
        if abs(rotation) > self.max_rotation_per_update:
            rotation = 0.0
        magnitude = float(np.linalg.norm(translation))
        if magnitude <= 1e-6 and abs(rotation) <= 1e-6:
            return
        # Apply the complete estimated rigid increment to the world-frame map.
        # Historical code discarded rotation, so a perfectly rigid outline
        # accumulated 0.2--0.3 m point error after only a few degrees of cargo
        # yaw.  The translation component remains separately exposed through
        # ``last_motion`` and therefore does not pollute task progress with spin.
        self._shift_object_records(
            object_id,
            translation,
            rotation=rotation,
            center=center,
        )
        self.last_motion[object_id] = np.asarray(translation, dtype=float).copy()
        self.last_rotation[object_id] = rotation

    def _shift_object_records(
        self,
        object_id: str,
        translation: np.ndarray,
        rotation: float = 0.0,
        center: np.ndarray | None = None,
    ) -> None:
        """Apply one planar rigid increment and rebuild voxel keys."""
        pivot = np.zeros(2) if center is None else np.asarray(center, dtype=float).reshape(2)
        c, s = math.cos(rotation), math.sin(rotation)
        matrix = np.array([[c, -s], [s, c]], dtype=float)
        rebuilt: dict[tuple[str, int, int], VoxelRecord] = {}
        for key, record in self.records.items():
            if record.object_id == object_id:
                record.point = (record.point - pivot) @ matrix.T + pivot + translation
                record.normal = record.normal @ matrix.T
                key = self._key(record.object_id, record.point)
            incumbent = rebuilt.get(key)
            if incumbent is None:
                rebuilt[key] = record
                continue
            # A re-key collision represents one spatial cell, not two pieces of
            # boundary.  Keep the stronger geometry and the maximum arc measure.
            if record.weight_sum > incumbent.weight_sum:
                record.arc_length = max(record.arc_length, incumbent.arc_length)
                record.confidence = max(record.confidence, incumbent.confidence)
                rebuilt[key] = record
            else:
                incumbent.arc_length = max(incumbent.arc_length, record.arc_length)
                incumbent.confidence = max(incumbent.confidence, record.confidence)
        self.records = rebuilt

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
