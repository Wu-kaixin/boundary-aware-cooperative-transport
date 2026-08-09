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
    """Local boundary measurement z_ik = (b_hat, n_hat, c, t).

    The controller never observes complete object geometry. It only receives
    locally visible boundary measurements of this form. Optional ``arc_length``
    approximates the boundary measure element Δs_k for density integration.
    """

    object_id: str
    agent_id: str
    point: np.ndarray
    normal: np.ndarray
    timestamp: float
    confidence: float = 1.0
    arc_length: float = 1.0
    gap_score: float = 0.0

    def __post_init__(self) -> None:
        self.point = asvec2(self.point)
        self.normal = asvec2(self.normal)
        norm = float(np.linalg.norm(self.normal))
        if norm > 1e-9:
            self.normal = self.normal / norm
        self.confidence = float(np.clip(self.confidence, 0.0, 1.0))
        self.arc_length = float(max(0.0, self.arc_length))
        self.gap_score = float(max(0.0, self.gap_score))


@dataclass
class ControlCommand:
    """Planar velocity command for one robot."""

    agent_id: str
    velocity: np.ndarray
    mode: str = "dbact"

    def __post_init__(self) -> None:
        self.velocity = asvec2(self.velocity)
