"""Local nominal motion for stationary acquisition; the existing QP owns safety."""
from __future__ import annotations

import numpy as np

from dbact.types import BoundaryView


def circulation_velocity(position, view: BoundaryView, standoff: float,
                         gain: float, max_speed: float) -> np.ndarray:
    """Counterclockwise following of the nearest point and its unit sensor normal."""
    if not len(view):
        return np.zeros(2)
    position = np.asarray(position, dtype=float).reshape(2)
    nearest = int(np.argmin(np.linalg.norm(view.points - position, axis=1)))
    normal = view.normals[nearest]
    gap = float((position - view.points[nearest]) @ normal)
    tangent = np.array([-normal[1], normal[0]])
    command = max_speed * tangent + gain * (standoff - gap) * normal
    speed = float(np.linalg.norm(command))
    return command * (max_speed / speed) if speed > max_speed else command
