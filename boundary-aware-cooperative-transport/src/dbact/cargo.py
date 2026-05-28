from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from .geometry import (
    closest_boundary_point_and_normal,
    ensure_ccw,
    make_circle,
    make_l_shape,
    make_nonconvex,
    make_rectangle,
    point_in_polygon,
    polygon_centroid,
    sample_polygon_boundary,
    normalize,
)


@dataclass
class Cargo:
    """Arbitrary-shaped cargo represented as a planar polygon."""

    object_id: str
    vertices: np.ndarray
    transport_direction: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0]))
    movable: bool = True

    def __post_init__(self) -> None:
        self.vertices = ensure_ccw(np.asarray(self.vertices, dtype=float))
        self.transport_direction = normalize(np.asarray(self.transport_direction, dtype=float), fallback=np.array([1.0, 0.0]))

    @property
    def center(self) -> np.ndarray:
        return polygon_centroid(self.vertices)

    def boundary_samples(self, count: int = 128) -> tuple[np.ndarray, np.ndarray]:
        return sample_polygon_boundary(self.vertices, count=count)

    def closest_boundary(self, point: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        return closest_boundary_point_and_normal(self.vertices, point)

    def contains(self, point: np.ndarray) -> bool:
        return point_in_polygon(point, self.vertices)

    def translate(self, delta: Iterable[float]) -> None:
        if not self.movable:
            return
        self.vertices = self.vertices + np.asarray(delta, dtype=float).reshape(2)

    @classmethod
    def circle(cls, object_id: str, center: Iterable[float], radius: float, transport_direction=(1.0, 0.0)) -> "Cargo":
        return cls(object_id, make_circle(center, radius), np.asarray(transport_direction, dtype=float))

    @classmethod
    def rectangle(
        cls,
        object_id: str,
        center: Iterable[float],
        width: float,
        height: float,
        yaw: float = 0.0,
        transport_direction=(1.0, 0.0),
    ) -> "Cargo":
        return cls(object_id, make_rectangle(center, width, height, yaw), np.asarray(transport_direction, dtype=float))

    @classmethod
    def l_shape(cls, object_id: str, center: Iterable[float], scale: float = 1.0, yaw: float = 0.0, transport_direction=(1.0, 0.0)) -> "Cargo":
        return cls(object_id, make_l_shape(center, scale, yaw), np.asarray(transport_direction, dtype=float))

    @classmethod
    def nonconvex(cls, object_id: str, center: Iterable[float], scale: float = 1.0, yaw: float = 0.0, transport_direction=(1.0, 0.0)) -> "Cargo":
        return cls(object_id, make_nonconvex(center, scale, yaw), np.asarray(transport_direction, dtype=float))

    @classmethod
    def from_config(cls, cfg: dict) -> "Cargo":
        object_id = str(cfg.get("id", "cargo"))
        direction = cfg.get("transport_direction", [1.0, 0.0])
        shape = str(cfg.get("shape", "rectangle"))
        if shape == "circle":
            return cls.circle(object_id, cfg.get("center", [0, 0]), float(cfg.get("radius", 0.5)), direction)
        if shape == "rectangle":
            return cls.rectangle(
                object_id,
                cfg.get("center", [0, 0]),
                float(cfg.get("width", 1.0)),
                float(cfg.get("height", 0.5)),
                float(cfg.get("yaw", 0.0)),
                direction,
            )
        if shape == "l_shape":
            return cls.l_shape(object_id, cfg.get("center", [0, 0]), float(cfg.get("scale", 1.0)), float(cfg.get("yaw", 0.0)), direction)
        if shape == "nonconvex":
            return cls.nonconvex(object_id, cfg.get("center", [0, 0]), float(cfg.get("scale", 1.0)), float(cfg.get("yaw", 0.0)), direction)
        if shape == "polygon":
            return cls(object_id, np.asarray(cfg["vertices"], dtype=float), np.asarray(direction, dtype=float))
        raise ValueError(f"Unknown cargo shape: {shape}")
