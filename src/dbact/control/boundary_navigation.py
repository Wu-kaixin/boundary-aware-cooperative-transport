"""Experimental point-goal feedback from permitted boundary-map messages.

These are controller estimates, not object pose/contact certificates. Callers
explicitly supply own/one-hop messages; the shared phase supervisor separately
uses the all-team population, as the historical scalar supervisor already did.
No simulator geometry, physical contact or task start position is accepted.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class VectorGoalFeedback:
    displacement: np.ndarray
    velocity: np.ndarray
    error: np.ndarray
    direction: np.ndarray
    remaining: float
    voters: tuple[str, ...]


def point_goal_feedback(
    displacements: Mapping[str, np.ndarray],
    velocities: Mapping[str, np.ndarray],
    permitted_ids: Sequence[str],
    target_delta: np.ndarray,
    arrival_band: float,
    *, rotation_invariant: bool = False,
) -> VectorGoalFeedback | None:
    """Coordinate median of fresh, full-vector-credible permitted messages.

    Quality filtering happens before this function. No absent message is filled
    with a zero displacement. The arrival band belongs to the new control law;
    it does not replace the independent physical endpoint/settling evaluation.
    """
    target = np.asarray(target_delta, dtype=float).reshape(2)
    if not np.isfinite(target).all() or not np.isfinite(arrival_band) or arrival_band <= 0:
        raise ValueError("Finite target and a positive controller arrival band are required")
    ids = tuple(dict.fromkeys(i for i in permitted_ids if i in displacements and i in velocities))
    if not ids:
        return None
    d = np.asarray([displacements[i] for i in ids], dtype=float).reshape(-1, 2)
    v = np.asarray([velocities[i] for i in ids], dtype=float).reshape(-1, 2)
    if not np.isfinite(d).all() or not np.isfinite(v).all():
        raise ValueError("Non-finite boundary-map message")
    if rotation_invariant:
        from dbact.control.initial_task_reference import geometric_median
        displacement, velocity = geometric_median(d), geometric_median(v)
    else:
        displacement, velocity = np.median(d, axis=0), np.median(v, axis=0)
    error = target - displacement
    distance = float(np.linalg.norm(error))
    direction = error / distance if distance > 1e-12 else target / max(float(np.linalg.norm(target)), 1e-12)
    return VectorGoalFeedback(displacement, velocity, error, direction,
                              max(0., distance - arrival_band), ids)


def push_alignment(normal: np.ndarray, goal: np.ndarray, steered: np.ndarray, mode: str) -> float:
    """Membership only; all enacted contact commands remain compressive -normal."""
    return float(np.asarray(normal) @ (goal if mode == 'legacy' else steered))


class VoterContinuity:
    """Remove adjacent-roster offsets, not individual or common estimator bias.

    For the intersection C of consecutive fresh voter sets, bridge the output by
    median_C(d_now) - median_C(d_previous). With an unchanged roster this follows
    the raw median exactly, preserving any earlier offset. Each receiver owns an
    independent instance and can use only its own permitted messages.

    No overlap (including a missing-data gap) breaks the bridge. Abstain on the
    new anchor frame, then resume a *new* segment from its raw absolute estimate.
    Such a reanchor is explicitly logged and is not a continuity guarantee.
    Historical readings are used only for differencing, never as current votes.
    """

    def __init__(self) -> None:
        self._previous: dict[str, np.ndarray] = {}
        self._raw: np.ndarray | None = None
        self._offset = np.zeros(2)
        self._started = False
        self.trace: dict = {}

    def update(self, raw: VectorGoalFeedback | None,
               displacements: Mapping[str, np.ndarray], target_delta: np.ndarray,
               arrival_band: float) -> VectorGoalFeedback | None:
        self.trace = dict(status='no_readings', common_voters=[],
                          raw_displacement=None, displacement=None,
                          roster_delta=None, common_delta=None, offset=None)
        if raw is None:
            self._previous = {}
            self._raw = None
            self._offset = np.zeros(2)
            return None
        current = {i: np.array(displacements[i], dtype=float, copy=True) for i in raw.voters}
        common = tuple(i for i in raw.voters if i in self._previous)
        status = 'initialized'
        admitted = True
        if common:
            before = np.median([self._previous[i] for i in common], axis=0)
            after = np.median([current[i] for i in common], axis=0)
            # Removed old voters + added new voters. Never clip scientific input.
            roster_delta = (before - self._raw) + (raw.displacement - after)
            self._offset -= roster_delta
            self.trace.update(roster_delta=roster_delta.tolist(), common_delta=(after-before).tolist())
            status = 'continuous'
        elif self._started:
            self._offset = np.zeros(2)
            status, admitted = 'reanchored', False
        else:
            self._offset = np.zeros(2)
        self._started = True
        self._previous, self._raw = current, raw.displacement.copy()
        self.trace.update(status=status, common_voters=list(common),
                          raw_displacement=raw.displacement.tolist(), offset=self._offset.tolist())
        if not admitted:
            return None
        displacement = raw.displacement + self._offset
        target = np.asarray(target_delta, dtype=float).reshape(2)
        error = target - displacement
        distance = float(np.linalg.norm(error))
        direction = error / distance if distance > 1e-12 else target / max(float(np.linalg.norm(target)), 1e-12)
        self.trace['displacement'] = displacement.tolist()
        return VectorGoalFeedback(displacement, raw.velocity, error, direction,
                                  max(0., distance-arrival_band), raw.voters)
