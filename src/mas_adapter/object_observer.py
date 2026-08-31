from __future__ import annotations

import numpy as np

from dbact.cargo import Cargo


class ObjectObserver:
    """Placeholder object observer for MAS integration.

    MAS WorldState currently contains robot states only. For real object-aware
    transport, replace this class with an OptiTrack rigid-body observer, vision
    detector, tactile estimator, or marker-based polygon provider.
    """

    def __init__(self, config: dict):
        self.config = config
        virtual = config.get("virtual_object", {})
        self.enabled = bool(virtual.get("enabled", False))
        self.virtual = virtual

    def observe(self) -> list[Cargo]:
        if not self.enabled:
            return []
        vertices = np.asarray(self.virtual["vertices"], dtype=float)
        return [Cargo(str(self.virtual.get("id", "cargo_0")), vertices)]

    def goal_directions(self) -> dict[str, np.ndarray]:
        """Task goal direction, kept apart from the body it acts on."""
        if not self.enabled:
            return {}
        direction = self.virtual.get("transport_direction")
        if direction is None:
            return {}
        return {str(self.virtual.get("id", "cargo_0")): np.asarray(direction, dtype=float)}
