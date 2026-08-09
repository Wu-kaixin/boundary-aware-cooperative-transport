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
class BoundaryView:
    """The same information as a list of ``BoundaryObservation``, as arrays.

    The list form is the right shape for a message and for a test. It is the
    wrong shape for the inner loop: at 16 robots and 96 rays the controller was
    building roughly three thousand dataclass instances per step purely to read
    its own map back, which measured 46% of one profiled step in ``update`` plus
    another 13% in the read. Nothing downstream wants one observation at a time --
    the density, the CVT and the safety rows all immediately re-stack them into
    arrays -- so the arrays are what the map stores and what it hands out.

    Empty is a valid view and every consumer must handle it; ``len(view) == 0``
    is a robot that has seen nothing yet, which is the initial condition of the
    entire task.
    """

    points: np.ndarray  # (N, 2)
    normals: np.ndarray  # (N, 2)
    confidence: np.ndarray  # (N,)
    arc_length: np.ndarray  # (N,)
    object_ids: np.ndarray  # (N,) unicode

    def __len__(self) -> int:
        return len(self.points)

    @staticmethod
    def empty() -> "BoundaryView":
        return BoundaryView(
            points=np.empty((0, 2)),
            normals=np.empty((0, 2)),
            confidence=np.empty(0),
            arc_length=np.empty(0),
            object_ids=np.empty(0, dtype="<U32"),
        )

    @staticmethod
    def from_observations(observations: list[BoundaryObservation]) -> "BoundaryView":
        if not observations:
            return BoundaryView.empty()
        return BoundaryView(
            points=np.vstack([o.point for o in observations]),
            normals=np.vstack([o.normal for o in observations]),
            confidence=np.asarray([o.confidence for o in observations], dtype=float),
            arc_length=np.asarray([o.arc_length for o in observations], dtype=float),
            object_ids=np.asarray([o.object_id for o in observations], dtype="<U32"),
        )

    def to_observations(self, timestamp: float = 0.0, agent_id: str = "map") -> list[BoundaryObservation]:
        return [
            BoundaryObservation(
                object_id=str(self.object_ids[k]),
                agent_id=agent_id,
                point=self.points[k].copy(),
                normal=self.normals[k].copy(),
                timestamp=timestamp,
                confidence=float(self.confidence[k]),
                arc_length=float(self.arc_length[k]),
            )
            for k in range(len(self.points))
        ]

    def select(self, mask: np.ndarray) -> "BoundaryView":
        return BoundaryView(
            points=self.points[mask],
            normals=self.normals[mask],
            confidence=self.confidence[mask],
            arc_length=self.arc_length[mask],
            object_ids=self.object_ids[mask],
        )


@dataclass
class ControlCommand:
    """Planar velocity command for one robot."""

    agent_id: str
    velocity: np.ndarray
    mode: str = "dbact"

    def __post_init__(self) -> None:
        self.velocity = asvec2(self.velocity)
