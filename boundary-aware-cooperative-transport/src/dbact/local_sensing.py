from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .cargo import Cargo
from .types import AgentState, BoundaryObservation


@dataclass
class LocalBoundarySensor:
    """Simulated local boundary sensor.

    In real robots this module should be replaced by perception, tactile sensing,
    camera/LiDAR segmentation, or an external object observer. In simulation it
    samples cargo boundaries and keeps only points visible inside sensor_range.
    """

    sensor_range: float
    boundary_samples_per_object: int = 160
    max_points_per_object: int = 24
    noise_std: float = 0.0

    def sense(self, agent: AgentState, cargoes: list[Cargo], timestamp: float) -> list[BoundaryObservation]:
        observations: list[BoundaryObservation] = []
        rng = np.random.default_rng(abs(hash((agent.agent_id, round(timestamp, 2)))) % (2**32))
        for cargo in cargoes:
            points, normals = cargo.boundary_samples(self.boundary_samples_per_object)
            distances = np.linalg.norm(points - agent.position[None, :], axis=1)
            visible_idx = np.where(distances <= self.sensor_range)[0]
            if len(visible_idx) > self.max_points_per_object:
                # Spread samples uniformly instead of returning a cluster.
                pick = np.linspace(0, len(visible_idx) - 1, self.max_points_per_object).astype(int)
                visible_idx = visible_idx[pick]
            for idx in visible_idx:
                p = points[idx].copy()
                if self.noise_std > 0:
                    p += rng.normal(scale=self.noise_std, size=2)
                observations.append(
                    BoundaryObservation(
                        object_id=cargo.object_id,
                        agent_id=agent.agent_id,
                        point=p,
                        normal=normals[idx],
                        timestamp=timestamp,
                    )
                )
        return observations
