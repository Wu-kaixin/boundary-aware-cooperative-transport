"""S3 - map layer: per-agent voxel map of observed boundary, on a moving object.

Storage is keyed by a spatial cell,

    key = (object_id, round(point / voxel_size))

with one fused record per cell, capacity applied to *cells* rather than to
packets, and fusion rules chosen so that repetition cannot inflate density mass:

* ``point`` and ``normal``  confidence-weighted averages
* ``confidence``            maximum, not sum
* ``arc_length``            maximum over sources, each source's contribution
  clamped to the cell diagonal -- a cell of side ``v`` cannot represent more than
  ``v*sqrt(2)`` of boundary no matter how many robots report it

Age enters at read time as ``exp(-lambda_age (t - t_k))`` rather than as a hard
cut, so an observation fades instead of vanishing between two consecutive steps.

Why the map has to move
-----------------------
A world-frame map of a *moving* body is wrong the moment the body moves, and it
is wrong in a way that looks like success. Measured on the L shape: once the cargo
had travelled 0.06 m, fifteen of sixteen robots still reported "contact-ready"
from their own maps while the true contact count was zero. The team was caging
where the object had been. The little contact that remained was on the leading
arc, so the net force along the task direction read -5.4 N -- the enclosure was
pushing the cargo backwards -- and ``J`` stopped at 0.0561 m for the remaining
400 frames.

The correction is a translation estimated from the robot's own consecutive scans,
by point-to-plane least squares:

    minimise_t  sum_k ( n_k^T (p_k - b_k) - n_k^T t )^2
    =>          ( sum_k n_k n_k^T ) t = sum_k n_k ( n_k^T (p_k - b_k) )

``n_k`` is the map's stored normal at the matched cell, ``p_k`` the new return.
Point-to-plane rather than point-to-point because a range scan slides freely
along the surface: point-to-point would report tangential motion that did not
happen. The normal matrix is rank deficient exactly when every visible normal is
parallel -- a robot looking at one flat face genuinely cannot observe motion along
that face -- so it is solved with a Tikhonov term and the unobservable component
comes out as zero rather than as noise.

Only translation is estimated. Yaw is not, and that is a stated limitation rather
than an approximation: the object-boundary rows and the transport controller both
use the estimate, so claiming SE(2) here without an error bound would put an
unmeasured quantity inside a safety constraint.

The same translation, accumulated, is the only thing any robot knows about how
far the cargo has travelled. It is what the transport controller closes its loop
on, so nothing in the control path reads a simulator pose.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .types import BoundaryObservation, BoundaryView

_CELL_BIAS = 1 << 20
_CELL_STRIDE = 1 << 21
_OBJECT_STRIDE = 1 << 42


@dataclass
class RegistrationResult:
    """One object's estimated frame-to-frame translation, with its diagnostics."""

    object_id: str
    translation: np.ndarray
    matches: int
    residual_rms: float
    conditioning: float
    clamped: bool

    @property
    def observable(self) -> bool:
        """False when every matched normal was parallel: motion is unobservable."""
        return self.conditioning > 1e-3


@dataclass
class LocalBoundaryMap:
    """Per-agent local memory of observed boundary, fused on a voxel grid.

    Cells are held as parallel arrays sorted by key, so a batch update is a
    ``searchsorted`` plus a handful of ``bincount`` reductions rather than a
    Python loop over returns.
    """

    voxel_size: float = 0.06
    age_decay: float = 0.30
    max_voxels_per_object: int = 600
    min_weight: float = 1e-3
    # Fusion weight saturates instead of accumulating. Without a cap a cell that
    # has been observed for a hundred steps carries a hundred units of prior, a
    # fresh return moves it by under a percent, and the map becomes an archive of
    # where the boundary first was rather than an estimate of where it is. The cap
    # turns the fusion into an exponentially weighted average whose time constant
    # is ``max_weight / mean_confidence`` steps, which is the quantity that has to
    # be short compared with the time the object takes to cross a voxel.
    max_weight: float = 4.0

    # --- motion compensation ---
    motion_compensation: bool = True
    registration_normal_cosine: float = 0.5
    registration_gate: float = 0.12
    registration_damping: float = 1e-3
    max_object_speed: float = 0.60
    velocity_filter: float = 0.35
    # Free-space carving. ``carve_margin`` is how much nearer than a return a cell
    # must be before the scan is taken to contradict it -- one voxel, so a cell
    # that merely straddles the surface survives. ``carve_aperture`` is the
    # arc-length tolerance on the bearing match.
    carve_enabled: bool = True
    carve_margin: float = 0.06
    carve_aperture: float = 0.05

    # --- storage, all sorted by ``_keys`` ---
    _keys: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.int64))
    _objects: np.ndarray = field(default_factory=lambda: np.empty(0, dtype="<U32"))
    _points: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    _normals: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    _confidence: np.ndarray = field(default_factory=lambda: np.empty(0))
    _arc: np.ndarray = field(default_factory=lambda: np.empty(0))
    _timestamp: np.ndarray = field(default_factory=lambda: np.empty(0))
    _weight: np.ndarray = field(default_factory=lambda: np.empty(0))

    _object_index: dict[str, int] = field(default_factory=dict)
    _step_motion: dict[str, np.ndarray] = field(default_factory=dict)
    displacement: dict[str, np.ndarray] = field(default_factory=dict)
    velocity: dict[str, np.ndarray] = field(default_factory=dict)
    last_registration: dict[str, RegistrationResult] = field(default_factory=dict)

    # ------------------------------------------------------------------ #
    # keys
    # ------------------------------------------------------------------ #

    @property
    def voxel_diagonal(self) -> float:
        return self.voxel_size * math.sqrt(2.0)

    def _object_code(self, object_ids: np.ndarray) -> np.ndarray:
        codes = np.empty(len(object_ids), dtype=np.int64)
        for k, name in enumerate(object_ids):
            key = str(name)
            code = self._object_index.get(key)
            if code is None:
                code = len(self._object_index)
                self._object_index[key] = code
            codes[k] = code
        return codes

    def _cell_keys(self, object_ids: np.ndarray, points: np.ndarray) -> np.ndarray:
        cells = np.rint(np.asarray(points, dtype=float) / self.voxel_size).astype(np.int64)
        codes = self._object_code(object_ids)
        return (
            codes * _OBJECT_STRIDE
            + (cells[:, 0] + _CELL_BIAS) * _CELL_STRIDE
            + (cells[:, 1] + _CELL_BIAS)
        )

    # ------------------------------------------------------------------ #
    # motion compensation
    # ------------------------------------------------------------------ #

    def register(self, scan: BoundaryView, dt: float) -> dict[str, RegistrationResult]:
        """Estimate each object's translation from ``scan`` against the stored map.

        ``scan`` must be the robot's *own* fresh returns. Relayed points are
        another robot's view of the same surface at the same instant, so they add
        no temporal information while multiplying the match cost.
        """
        results: dict[str, RegistrationResult] = {}
        shifted = False
        if not self.motion_compensation or len(scan) == 0 or len(self._keys) == 0 or dt <= 0.0:
            return results

        for object_id in np.unique(scan.object_ids):
            name = str(object_id)
            rows = np.flatnonzero(self._objects == name)
            if len(rows) < 4:
                continue
            picked = scan.object_ids == object_id
            result = self._register_object(name, scan.points[picked], scan.normals[picked], rows, dt)
            if result is None:
                continue
            results[name] = result
            self.last_registration[name] = result
            if result.matches:
                self._points[rows] += result.translation[None, :]
                self._step_motion[name] = self._step_motion.get(name, np.zeros(2)) + result.translation
                self._rekey()
        return results

    def _commit_motion(self, dt: float) -> None:
        """Fold this step's estimated motion into displacement and velocity.

        Two things move the map towards the object: the rigid shift from
        registration, and the pull that fusing a fresh scan exerts on the cells it
        lands in. Only counting the first underestimates the motion by exactly the
        share the second absorbed -- measured at a fusion cap of 4, the integrated
        estimate read 79% of the true displacement. Both are motion the robot
        observed, so both are counted, and the sum is what the transport loop and
        the stopping condition see.
        """
        if dt <= 0.0:
            self._step_motion.clear()
            return
        alpha = float(np.clip(self.velocity_filter, 0.0, 1.0))
        for name in set(self._step_motion) | set(self.velocity):
            motion = self._step_motion.get(name, np.zeros(2))
            self.displacement[name] = self.displacement.get(name, np.zeros(2)) + motion
            prior = self.velocity.get(name, np.zeros(2))
            self.velocity[name] = (1.0 - alpha) * prior + alpha * (motion / dt)
        self._step_motion.clear()

    def _rekey(self) -> None:
        """Recompute cell keys after a shift and fuse cells that collided.

        A record's key is derived from its position, so moving the map without
        rekeying leaves records filed under the cell they used to occupy. The next
        scan then lands in the cell the record has moved into, finds it empty,
        and creates a second record for the same piece of boundary -- the map
        doubles up along the direction of travel, the density smears over both
        copies, and the registration that caused it starts matching returns
        against the fresh duplicates instead of the older record, so the estimated
        translation collapses towards zero. Measured before this call existed: the
        estimate read 0.02 m/s against a true 0.085 m/s.
        """
        if len(self._keys) == 0:
            return
        self._keys = self._cell_keys(self._objects, self._points)
        unique, inverse = np.unique(self._keys, return_inverse=True)
        if len(unique) < len(self._keys):
            weight = self._weight
            total = np.bincount(inverse, weights=weight, minlength=len(unique))
            points = np.column_stack(
                [np.bincount(inverse, weights=weight * self._points[:, d], minlength=len(unique)) for d in (0, 1)]
            ) / total[:, None]
            normal_sum = np.column_stack(
                [np.bincount(inverse, weights=weight * self._normals[:, d], minlength=len(unique)) for d in (0, 1)]
            )
            norm = np.linalg.norm(normal_sum, axis=1)
            normals = np.where(norm[:, None] > 1e-9, normal_sum / np.maximum(norm, 1e-9)[:, None], 0.0)
            confidence = np.zeros(len(unique))
            np.maximum.at(confidence, inverse, self._confidence)
            arc = np.zeros(len(unique))
            np.maximum.at(arc, inverse, self._arc)
            timestamp = np.zeros(len(unique))
            np.maximum.at(timestamp, inverse, self._timestamp)
            first = np.zeros(len(unique), dtype=np.int64)
            first[inverse[::-1]] = np.arange(len(inverse))[::-1]

            self._objects = self._objects[first]
            self._points = points
            self._normals = normals
            self._confidence = confidence
            self._arc = arc
            self._timestamp = timestamp
            self._weight = np.minimum(total, self.max_weight)
            self._keys = unique
        else:
            self._reorder(np.argsort(self._keys, kind="stable"))

    def _register_object(
        self,
        object_id: str,
        points: np.ndarray,
        normals: np.ndarray,
        rows: np.ndarray,
        dt: float,
    ) -> RegistrationResult | None:
        matched_row, valid = self._match_rows(points, normals, rows)
        if not np.any(valid):
            return RegistrationResult(object_id, np.zeros(2), 0, 0.0, 0.0, False)

        p = points[valid]
        row = matched_row[valid]
        b = self._points[row]
        n = self._normals[row]

        residual = np.einsum("ij,ij->i", n, p - b)
        # Trim the tail: a return that landed on a face the map does not hold yet
        # produces a large residual and would drag the estimate with it.
        if len(residual) >= 8:
            keep = np.abs(residual - np.median(residual)) <= 2.5 * (np.median(np.abs(residual - np.median(residual))) + 1e-6)
            if np.count_nonzero(keep) >= 4:
                n, residual = n[keep], residual[keep]

        normal_matrix = n.T @ n
        rhs = n.T @ residual
        trace = float(np.trace(normal_matrix))
        if trace <= 1e-12:
            return RegistrationResult(object_id, np.zeros(2), len(residual), 0.0, 0.0, False)

        damping = self.registration_damping * trace
        translation = np.linalg.solve(normal_matrix + damping * np.eye(2), rhs)

        eigenvalues = np.linalg.eigvalsh(normal_matrix)
        conditioning = float(max(eigenvalues[0], 0.0) / max(eigenvalues[1], 1e-12))

        limit = self.max_object_speed * dt
        norm = float(np.linalg.norm(translation))
        clamped = norm > limit
        if clamped:
            translation = translation * (limit / norm)

        rms = float(np.sqrt(np.mean((residual - n @ translation) ** 2)))
        return RegistrationResult(object_id, translation, len(residual), rms, conditioning, clamped)

    def _match_rows(
        self, points: np.ndarray, normals: np.ndarray, rows: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Nearest stored cell for each return, searched over the 3x3 cell block.

        The voxel grid is already a spatial index, so the correspondence search is
        nine hash lookups per return instead of a scan over the whole map.
        """
        keys = self._keys[rows]
        base_cells = np.rint(points / self.voxel_size).astype(np.int64)
        object_key = self._keys[rows[0]] // _OBJECT_STRIDE * _OBJECT_STRIDE

        best_row = np.full(len(points), -1, dtype=np.int64)
        best_distance = np.full(len(points), np.inf)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                candidate = (
                    object_key
                    + (base_cells[:, 0] + dx + _CELL_BIAS) * _CELL_STRIDE
                    + (base_cells[:, 1] + dy + _CELL_BIAS)
                )
                slot = np.searchsorted(keys, candidate)
                slot = np.clip(slot, 0, len(keys) - 1)
                hit = keys[slot] == candidate
                if not np.any(hit):
                    continue
                row = rows[slot]
                delta = points - self._points[row]
                distance = np.linalg.norm(delta, axis=1)
                aligned = np.einsum("ij,ij->i", normals, self._normals[row]) >= self.registration_normal_cosine
                better = hit & aligned & (distance < best_distance) & (distance <= self.registration_gate)
                best_row = np.where(better, row, best_row)
                best_distance = np.where(better, distance, best_distance)
        return best_row, best_row >= 0

    def carve(self, origin: np.ndarray, scan: BoundaryView) -> int:
        """Delete cells the current scan sees straight through.

        Registration moves the map rigidly and fusion pulls each cell towards the
        latest return, but neither can remove a cell the object has walked away
        from: the new returns land in a *different* voxel, so the old one survives
        until age decay gets round to it, and in the meantime it is the nearest
        boundary point the robot has. Measured on the trailing arc, that ghost sat
        0.06 m inside the true surface -- so the pushing robots pressed to the
        barrier limit against a boundary that was no longer there, believed they
        were at full penetration, and applied no force at all.

        The test is the one an occupancy map uses. A return at range ``r`` on some
        bearing is evidence that everything nearer than ``r`` along that bearing is
        empty, so a cell on that bearing at a smaller range contradicts the scan
        and is dropped. One bearing lookup per cell, and it only ever removes cells
        this robot can currently see through.
        """
        if len(self._keys) == 0 or len(scan) == 0:
            return 0
        p = np.asarray(origin, dtype=float).reshape(2)

        observed = scan.points - p[None, :]
        observed_range = np.linalg.norm(observed, axis=1)
        live = observed_range > 1e-9
        if not np.any(live):
            return 0
        observed_angle = np.arctan2(observed[live, 1], observed[live, 0])
        observed_range = observed_range[live]
        order = np.argsort(observed_angle)
        observed_angle, observed_range = observed_angle[order], observed_range[order]

        rel = self._points - p[None, :]
        cell_range = np.linalg.norm(rel, axis=1)
        candidate = cell_range < np.max(observed_range)
        if not np.any(candidate):
            return 0
        cell_angle = np.arctan2(rel[candidate, 1], rel[candidate, 0])

        slot = np.clip(np.searchsorted(observed_angle, cell_angle), 0, len(observed_angle) - 1)
        left = np.maximum(slot - 1, 0)
        pick = np.where(
            np.abs(observed_angle[slot] - cell_angle) <= np.abs(observed_angle[left] - cell_angle), slot, left
        )
        # Only trust the comparison when a ray actually passed close to this
        # bearing; otherwise the nearest return describes a different direction and
        # says nothing about this cell.
        bearing_gap = np.abs(observed_angle[pick] - cell_angle)
        aperture = max(self.carve_aperture, 1e-6) / np.maximum(cell_range[candidate], 1e-6)
        free = (cell_range[candidate] < observed_range[pick] - self.carve_margin) & (bearing_gap <= aperture)
        if not np.any(free):
            return 0

        drop = np.flatnonzero(candidate)[free]
        keep = np.ones(len(self._keys), dtype=bool)
        keep[drop] = False
        self._reorder(np.flatnonzero(keep))
        return int(len(drop))

    def object_velocity(self, object_id: str) -> np.ndarray:
        return self.velocity.get(str(object_id), np.zeros(2)).copy()

    def object_displacement(self, object_id: str) -> np.ndarray:
        return self.displacement.get(str(object_id), np.zeros(2)).copy()

    # ------------------------------------------------------------------ #
    # update
    # ------------------------------------------------------------------ #

    def update(
        self,
        new_observations,
        timestamp: float,
        agent_codes: np.ndarray | None = None,
        dt: float = 0.0,
    ) -> None:
        """Fuse a batch of returns. Accepts a ``BoundaryView`` or a list."""
        if isinstance(new_observations, BoundaryView):
            view = new_observations
            if agent_codes is None:
                agent_codes = np.zeros(len(view), dtype=np.int64)
            timestamps = np.full(len(view), float(timestamp))
        else:
            observations = list(new_observations)
            if not observations:
                self.prune(timestamp)
                self._commit_motion(dt)
                return
            view = BoundaryView.from_observations(observations)
            sources = {}
            agent_codes = np.empty(len(observations), dtype=np.int64)
            for k, obs in enumerate(observations):
                agent_codes[k] = sources.setdefault(obs.agent_id, len(sources))
            timestamps = np.asarray([float(o.timestamp) for o in observations], dtype=float)

        if len(view) == 0:
            self.prune(timestamp)
            self._commit_motion(dt)
            return

        # A packet relayed over several paths arrives as several identical copies
        # inside a single update. Summing arc length over them inflated each cell
        # up to the diagonal cap; keying the deduplication on the *source* return
        # rather than on the cell keeps a genuinely denser scan contributing.
        signature = np.column_stack(
            [agent_codes.astype(float), timestamps, view.points[:, 0], view.points[:, 1]]
        )
        _, unique_rows = np.unique(signature, axis=0, return_index=True)
        if len(unique_rows) < len(view):
            unique_rows = np.sort(unique_rows)
            view = view.select(unique_rows)
            agent_codes = agent_codes[unique_rows]
            timestamps = timestamps[unique_rows]

        keys = self._cell_keys(view.object_ids, view.points)
        cell_keys, cell_of = np.unique(keys, return_inverse=True)
        n_cells = len(cell_keys)

        # Arc length is summed only within one scan of one robot; across robots and
        # across time it is a maximum, so relay adds no mass.
        pair = cell_of.astype(np.int64) * (int(agent_codes.max()) + 1) + agent_codes
        pair_ids, pair_of = np.unique(pair, return_inverse=True)
        pair_arc = np.bincount(pair_of, weights=view.arc_length, minlength=len(pair_ids))
        pair_arc = np.minimum(pair_arc, self.voxel_diagonal)
        pair_cell = (pair_ids // (int(agent_codes.max()) + 1)).astype(np.int64)
        cell_arc = np.zeros(n_cells)
        np.maximum.at(cell_arc, pair_cell, pair_arc)

        weight = np.maximum(view.confidence, 1e-6)
        cell_weight = np.bincount(cell_of, weights=weight, minlength=n_cells)
        cell_point = np.column_stack(
            [np.bincount(cell_of, weights=weight * view.points[:, d], minlength=n_cells) for d in (0, 1)]
        )
        cell_normal = np.column_stack(
            [np.bincount(cell_of, weights=weight * view.normals[:, d], minlength=n_cells) for d in (0, 1)]
        )
        cell_confidence = np.zeros(n_cells)
        np.maximum.at(cell_confidence, cell_of, view.confidence)
        cell_time = np.zeros(n_cells)
        np.maximum.at(cell_time, cell_of, timestamps)
        first = np.zeros(n_cells, dtype=np.int64)
        first[cell_of[::-1]] = np.arange(len(cell_of))[::-1]
        cell_object = view.object_ids[first]

        drift = self._merge(
            cell_keys, cell_object, cell_point, cell_normal, cell_weight, cell_confidence, cell_arc, cell_time
        )
        for name, translation in (drift or {}).items():
            self._step_motion[name] = self._step_motion.get(name, np.zeros(2)) + translation
        self.prune(timestamp)
        self._commit_motion(dt)

    def _merge(
        self,
        keys: np.ndarray,
        objects: np.ndarray,
        point_sum: np.ndarray,
        normal_sum: np.ndarray,
        weight: np.ndarray,
        confidence: np.ndarray,
        arc: np.ndarray,
        timestamp: np.ndarray,
    ) -> None:
        drift: dict[str, np.ndarray] = {}
        if len(self._keys):
            slot = np.clip(np.searchsorted(self._keys, keys), 0, len(self._keys) - 1)
            existing = self._keys[slot] == keys
        else:
            slot = np.zeros(len(keys), dtype=np.int64)
            existing = np.zeros(len(keys), dtype=bool)

        if np.any(existing):
            row = slot[existing]
            before = self._points[row].copy()
            total = self._weight[row] + weight[existing]
            self._points[row] = (self._points[row] * self._weight[row][:, None] + point_sum[existing]) / total[:, None]
            fused = self._normals[row] * self._weight[row][:, None] + normal_sum[existing]
            norm = np.linalg.norm(fused, axis=1)
            good = norm > 1e-9
            self._normals[row[good]] = fused[good] / norm[good][:, None]
            self._weight[row] = np.minimum(total, self.max_weight)
            self._confidence[row] = np.maximum(self._confidence[row], confidence[existing])
            self._arc[row] = np.maximum(self._arc[row], arc[existing])
            self._timestamp[row] = np.maximum(self._timestamp[row], timestamp[existing])
            drift = self._fusion_drift(row, self._points[row] - before)

        fresh = ~existing
        if not np.any(fresh):
            return drift
        norm = np.linalg.norm(normal_sum[fresh], axis=1)
        normals = np.where(norm[:, None] > 1e-9, normal_sum[fresh] / np.maximum(norm, 1e-9)[:, None], 0.0)
        self._keys = np.concatenate([self._keys, keys[fresh]])
        self._objects = np.concatenate([self._objects, objects[fresh]])
        self._points = np.vstack([self._points, point_sum[fresh] / weight[fresh][:, None]])
        self._normals = np.vstack([self._normals, normals])
        self._confidence = np.concatenate([self._confidence, confidence[fresh]])
        self._arc = np.concatenate([self._arc, arc[fresh]])
        self._timestamp = np.concatenate([self._timestamp, timestamp[fresh]])
        self._weight = np.concatenate([self._weight, weight[fresh]])
        self._reorder(np.argsort(self._keys, kind="stable"))
        return drift

    def _fusion_drift(self, rows: np.ndarray, delta: np.ndarray) -> dict[str, np.ndarray]:
        """Rigid translation that best explains how fusion moved existing cells.

        Same point-to-plane projection as the registration itself, so a scan that
        merely refined a cell tangentially -- which a range scan does constantly,
        since it slides freely along the surface -- contributes nothing.
        """
        out: dict[str, np.ndarray] = {}
        for name in np.unique(self._objects[rows]):
            picked = self._objects[rows] == name
            if np.count_nonzero(picked) < 4:
                continue
            n = self._normals[rows[picked]]
            residual = np.einsum("ij,ij->i", n, delta[picked])
            matrix = n.T @ n
            trace = float(np.trace(matrix))
            if trace <= 1e-12:
                continue
            translation = np.linalg.solve(matrix + self.registration_damping * trace * np.eye(2), n.T @ residual)
            out[str(name)] = translation
        return out

    def _reorder(self, order: np.ndarray) -> None:
        self._keys = self._keys[order]
        self._objects = self._objects[order]
        self._points = self._points[order]
        self._normals = self._normals[order]
        self._confidence = self._confidence[order]
        self._arc = self._arc[order]
        self._timestamp = self._timestamp[order]
        self._weight = self._weight[order]

    # ------------------------------------------------------------------ #
    # pruning and reading
    # ------------------------------------------------------------------ #

    def _decayed(self, timestamp: float) -> np.ndarray:
        age = np.maximum(0.0, float(timestamp) - self._timestamp)
        return self._confidence * np.exp(-self.age_decay * age)

    def prune(self, timestamp: float) -> None:
        """Drop faded cells, then apply the capacity cap per object.

        The cap is on *cells*, so it bounds the map's spatial extent rather than
        its update rate. When it binds, the cells kept are the ones with the
        largest current (decayed) weight.
        """
        if len(self._keys) == 0:
            return
        decayed = self._decayed(timestamp)
        keep = decayed >= self.min_weight
        if not np.all(keep):
            self._reorder(np.flatnonzero(keep))
            decayed = decayed[keep]
        if len(self._keys) == 0:
            return

        score = decayed * np.maximum(self._arc, 1e-6)
        drop: list[np.ndarray] = []
        for name in np.unique(self._objects):
            rows = np.flatnonzero(self._objects == name)
            if len(rows) <= self.max_voxels_per_object:
                continue
            ranked = rows[np.argsort(-score[rows], kind="stable")]
            drop.append(ranked[self.max_voxels_per_object:])
        if drop:
            removed = np.concatenate(drop)
            mask = np.ones(len(self._keys), dtype=bool)
            mask[removed] = False
            self._reorder(np.flatnonzero(mask))

    def view(self, timestamp: float | None = None) -> BoundaryView:
        """Fused cells as arrays, with age decay folded into confidence."""
        if timestamp is not None:
            self.prune(timestamp)
        if len(self._keys) == 0:
            return BoundaryView.empty()
        t = float(timestamp) if timestamp is not None else float(np.max(self._timestamp))
        return BoundaryView(
            points=self._points,
            normals=self._normals,
            confidence=self._decayed(t),
            arc_length=self._arc,
            object_ids=self._objects,
        )

    def all_observations(self, timestamp: float | None = None) -> list[BoundaryObservation]:
        t = 0.0 if timestamp is None else float(timestamp)
        return self.view(timestamp).to_observations(timestamp=t)

    def object_ids(self) -> list[str]:
        return sorted({str(name) for name in self._objects})

    def total_arc_length(self, object_id: str | None = None) -> float:
        """Estimated observed perimeter -- the total mass the density carries."""
        if len(self._arc) == 0:
            return 0.0
        if object_id is None:
            return float(np.sum(self._arc))
        return float(np.sum(self._arc[self._objects == str(object_id)]))

    def __len__(self) -> int:
        return len(self._keys)


__all__ = ["LocalBoundaryMap", "RegistrationResult"]
