from __future__ import annotations

from dataclasses import dataclass, field

from .types import BoundaryObservation


@dataclass
class LocalBoundaryMap:
    """Per-agent local memory of recently observed boundary points."""

    ttl: float = 4.0
    max_points_per_object: int = 160
    observations: dict[str, list[BoundaryObservation]] = field(default_factory=dict)

    def update(self, new_observations: list[BoundaryObservation], timestamp: float) -> None:
        for obs in new_observations:
            self.observations.setdefault(obs.object_id, []).append(obs)
        self.prune(timestamp)

    def prune(self, timestamp: float) -> None:
        for object_id in list(self.observations):
            fresh = [obs for obs in self.observations[object_id] if timestamp - obs.timestamp <= self.ttl]
            if len(fresh) > self.max_points_per_object:
                fresh = fresh[-self.max_points_per_object :]
            if fresh:
                self.observations[object_id] = fresh
            else:
                del self.observations[object_id]

    def all_observations(self, timestamp: float | None = None) -> list[BoundaryObservation]:
        if timestamp is not None:
            self.prune(timestamp)
        out: list[BoundaryObservation] = []
        for obs_list in self.observations.values():
            out.extend(obs_list)
        return out

    def object_ids(self) -> list[str]:
        return list(self.observations.keys())
