"""Planar rigid-body cargo.

The previous version stored a vertex array and offered ``translate`` only. A body
that cannot rotate hides an entire failure mode: a team that applies a net torque
it has no way to resist looks, in the log, exactly like a team that does not.

The cargo also no longer carries a transport direction. The task goal direction
is a property of the *task*, held by the controller and by the success criterion,
and is deliberately unreachable from the physics: with no such field on the body,
"the cargo moved the way the config said" is not an outcome the engine is able to
produce.
"""

from __future__ import annotations

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
    polygon_area,
    polygon_centroid,
    polygon_perimeter,
    polygon_second_moment,
    rotate,
    sample_polygon_boundary,
    signed_distance_and_gradient,
)


class Cargo:
    """Arbitrary-shaped planar rigid body.

    State is ``(position, angle, linear_velocity, angular_velocity)``. Vertices
    are derived from a body-frame outline, so the centroid stays exactly at
    ``position`` for the whole run and displacement is measured without drift.
    """

    def __init__(
        self,
        object_id: str,
        vertices: np.ndarray,
        movable: bool = True,
        surface_density: float = 1.0,
    ):
        v = ensure_ccw(np.asarray(vertices, dtype=float))
        centroid = polygon_centroid(v)
        self.object_id = str(object_id)
        self.local_vertices = v - centroid
        self.position = centroid.astype(float)
        self.angle = 0.0
        self.linear_velocity = np.zeros(2, dtype=float)
        self.angular_velocity = 0.0
        self.movable = bool(movable)
        self.surface_density = float(surface_density)

        self.area = abs(polygon_area(self.local_vertices))
        self.mass = max(self.surface_density * self.area, 1e-9)
        self.inertia = max(self.surface_density * polygon_second_moment(self.local_vertices, np.zeros(2)), 1e-9)
        self.initial_position = self.position.copy()

    # ------------------------------------------------------------------ #
    # geometry
    # ------------------------------------------------------------------ #

    @property
    def vertices(self) -> np.ndarray:
        return rotate(self.local_vertices, self.angle) + self.position[None, :]

    @property
    def center(self) -> np.ndarray:
        return self.position.copy()

    @property
    def perimeter(self) -> float:
        return polygon_perimeter(self.local_vertices)

    def boundary_samples(self, count: int = 128) -> tuple[np.ndarray, np.ndarray]:
        return sample_polygon_boundary(self.vertices, count=count)

    def closest_boundary(self, point: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        return closest_boundary_point_and_normal(self.vertices, point)

    def signed_distance(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Signed distance, outward unit normal and footpoint for each point."""
        return signed_distance_and_gradient(points, self.vertices)

    def contains(self, point: np.ndarray) -> bool:
        return point_in_polygon(point, self.vertices)

    def point_velocity(self, world_point: np.ndarray) -> np.ndarray:
        """Velocity of the material point currently at ``world_point``."""
        r = np.asarray(world_point, dtype=float).reshape(2) - self.position
        return self.linear_velocity + self.angular_velocity * np.array([-r[1], r[0]])

    # ------------------------------------------------------------------ #
    # rigid-body state
    # ------------------------------------------------------------------ #

    def translate(self, delta: Iterable[float]) -> None:
        if not self.movable:
            return
        self.position = self.position + np.asarray(delta, dtype=float).reshape(2)

    def rotate_by(self, delta_angle: float) -> None:
        if not self.movable:
            return
        self.angle = float(self.angle + delta_angle)

    def set_pose(self, position: np.ndarray, angle: float) -> None:
        self.position = np.asarray(position, dtype=float).reshape(2).copy()
        self.angle = float(angle)

    def set_twist(self, linear_velocity: np.ndarray, angular_velocity: float) -> None:
        self.linear_velocity = np.asarray(linear_velocity, dtype=float).reshape(2).copy()
        self.angular_velocity = float(angular_velocity)

    @property
    def displacement(self) -> np.ndarray:
        return self.position - self.initial_position

    # ------------------------------------------------------------------ #
    # factories
    # ------------------------------------------------------------------ #

    @classmethod
    def circle(cls, object_id: str, center: Iterable[float], radius: float, **kwargs) -> "Cargo":
        return cls(object_id, make_circle(center, radius), **kwargs)

    @classmethod
    def rectangle(
        cls,
        object_id: str,
        center: Iterable[float],
        width: float,
        height: float,
        yaw: float = 0.0,
        **kwargs,
    ) -> "Cargo":
        return cls(object_id, make_rectangle(center, width, height, yaw), **kwargs)

    @classmethod
    def l_shape(cls, object_id: str, center: Iterable[float], scale: float = 1.0, yaw: float = 0.0, **kwargs) -> "Cargo":
        return cls(object_id, make_l_shape(center, scale, yaw), **kwargs)

    @classmethod
    def nonconvex(cls, object_id: str, center: Iterable[float], scale: float = 1.0, yaw: float = 0.0, **kwargs) -> "Cargo":
        return cls(object_id, make_nonconvex(center, scale, yaw), **kwargs)

    @classmethod
    def from_config(cls, cfg: dict) -> "Cargo":
        object_id = str(cfg.get("id", "cargo"))
        shape = str(cfg.get("shape", "rectangle"))
        extra = {
            "movable": bool(cfg.get("movable", True)),
            "surface_density": float(cfg.get("surface_density", 1.0)),
        }
        if shape == "circle":
            return cls.circle(object_id, cfg.get("center", [0, 0]), float(cfg.get("radius", 0.5)), **extra)
        if shape == "rectangle":
            return cls.rectangle(
                object_id,
                cfg.get("center", [0, 0]),
                float(cfg.get("width", 1.0)),
                float(cfg.get("height", 0.5)),
                float(cfg.get("yaw", 0.0)),
                **extra,
            )
        if shape == "l_shape":
            return cls.l_shape(object_id, cfg.get("center", [0, 0]), float(cfg.get("scale", 1.0)), float(cfg.get("yaw", 0.0)), **extra)
        if shape == "nonconvex":
            return cls.nonconvex(object_id, cfg.get("center", [0, 0]), float(cfg.get("scale", 1.0)), float(cfg.get("yaw", 0.0)), **extra)
        if shape == "polygon":
            vertices = np.asarray(cfg["vertices"], dtype=float)
            frame = str(cfg.get("vertices_frame", "world"))
            if frame == "local":
                vertices = rotate(vertices, float(cfg.get("yaw", 0.0))) + np.asarray(
                    cfg.get("center", [0.0, 0.0]), dtype=float
                )
            elif frame != "world":
                raise ValueError("polygon vertices_frame must be 'world' or 'local'")
            return cls(object_id, vertices, **extra)
        raise ValueError(f"Unknown cargo shape: {shape}")


__all__ = ["Cargo"]
