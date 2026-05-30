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
        direction = np.asarray(self.virtual.get("transport_direction", [1.0, 0.0]), dtype=float)
        return [Cargo(str(self.virtual.get("id", "cargo_0")), vertices, direction)]
