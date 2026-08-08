from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


def asvec2(value: Any) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(2)
    return arr


@dataclass
class AgentState:
    """Minimal planar state used by the simulator and MAS adapter."""

    agent_id: str
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    yaw: float = 0.0

    def __post_init__(self) -> None:
        self.position = asvec2(self.position)
        self.velocity = asvec2(self.velocity)


@dataclass
class BoundaryObservation:
    """A local boundary point observed by one robot.

    ``arc_length`` is the length of boundary this return stands for. It is what
    makes the density a *measure* on the boundary rather than a sum over however
    many samples the sensor happened to produce, so a densely scanned arc and a
    sparsely scanned arc of equal length carry equal mass.

    ``residual`` is the local plane-fit RMS from the normal estimate; the
    confidence is derived from it rather than from an eigenvalue ratio.
    """

    object_id: str
    agent_id: str
    point: np.ndarray
    normal: np.ndarray
    timestamp: float
    confidence: float = 1.0
    arc_length: float = 0.0
    residual: float = 0.0

    def __post_init__(self) -> None:
        self.point = asvec2(self.point)
        self.normal = asvec2(self.normal)
        norm = float(np.linalg.norm(self.normal))
        if norm > 1e-9:
            self.normal = self.normal / norm


@dataclass
class ControlCommand:
    """Planar velocity command for one robot."""

    agent_id: str
    velocity: np.ndarray
    mode: str = "dbact"

    def __post_init__(self) -> None:
        self.velocity = asvec2(self.velocity)
