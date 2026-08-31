import math

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.geometry import polygon_perimeter, triangulate_simple_polygon


def test_rectangle_boundary_samples_and_closest_point():
    cargo = Cargo.rectangle("box", [0, 0], width=2.0, height=1.0)
    pts, normals = cargo.boundary_samples(40)
    assert pts.shape == (40, 2)
    assert normals.shape == (40, 2)
    q, n, d = cargo.closest_boundary(np.array([2.0, 0.0]))
    assert d == pytest.approx(1.0)
    assert n == pytest.approx([1.0, 0.0])


def test_centroid_stays_at_the_position_under_motion():
    """Displacement is measured from the centroid, so it must not drift as the body
    translates and rotates."""
    cargo = Cargo.l_shape("obj", [1.0, 2.0], scale=1.5)
    cargo.translate([0.3, -0.2])
    cargo.rotate_by(0.7)
    from dbact.geometry import polygon_centroid

    assert polygon_centroid(cargo.vertices) == pytest.approx(cargo.position, abs=1e-9)


def test_rotation_moves_the_outline_but_not_the_centroid():
    cargo = Cargo.rectangle("box", [0.0, 0.0], 2.0, 1.0)
    before = cargo.vertices.copy()
    cargo.rotate_by(math.pi / 2)
    assert not np.allclose(before, cargo.vertices)
    assert cargo.position == pytest.approx([0.0, 0.0])
    assert polygon_perimeter(cargo.vertices) == pytest.approx(polygon_perimeter(before))


def test_signed_distance_is_negative_inside_and_positive_outside():
    cargo = Cargo.rectangle("box", [0.0, 0.0], 2.0, 1.0)
    signed, normals, foot = cargo.signed_distance(np.array([[0.0, 0.0], [0.0, 1.0]]))
    assert signed[0] < 0.0
    assert signed[1] == pytest.approx(0.5)
    assert normals[1] == pytest.approx([0.0, 1.0])
    assert foot[1] == pytest.approx([0.0, 0.5])


def test_point_velocity_includes_the_rotational_term():
    cargo = Cargo.rectangle("box", [0.0, 0.0], 2.0, 1.0)
    cargo.set_twist(np.array([0.1, 0.0]), 2.0)
    v = cargo.point_velocity(np.array([1.0, 0.0]))
    assert v == pytest.approx([0.1, 2.0])


def test_mass_and_inertia_scale_with_surface_density():
    light = Cargo.l_shape("a", [0, 0], scale=1.5, surface_density=1.0)
    heavy = Cargo.l_shape("b", [0, 0], scale=1.5, surface_density=3.0)
    assert heavy.mass == pytest.approx(3.0 * light.mass)
    assert heavy.inertia == pytest.approx(3.0 * light.inertia)
    assert light.mass == pytest.approx(light.area)


def test_immovable_cargo_ignores_translation_and_rotation():
    cargo = Cargo.rectangle("wall", [0.0, 0.0], 1.0, 1.0, movable=False)
    cargo.translate([1.0, 1.0])
    cargo.rotate_by(1.0)
    assert cargo.position == pytest.approx([0.0, 0.0])
    assert cargo.angle == 0.0


def test_cargo_has_no_transport_direction():
    """The task goal is a property of the task. With no such field on the body,
    "the cargo moved the way the config said" is not an outcome the physics can
    produce."""
    cargo = Cargo.from_config({"id": "c", "shape": "rectangle", "width": 1.0, "height": 1.0,
                              "transport_direction": [0.0, -1.0]})
    assert not hasattr(cargo, "transport_direction")


def test_displacement_tracks_the_initial_position():
    cargo = Cargo.rectangle("box", [1.0, 1.0], 1.0, 1.0)
    cargo.translate([0.25, 0.0])
    assert cargo.displacement == pytest.approx([0.25, 0.0])


@pytest.mark.parametrize("shape", ["circle", "rectangle", "l_shape", "nonconvex"])
def test_factories_produce_closed_outlines(shape):
    cfg = {"id": "c", "shape": shape, "center": [0, 0], "radius": 0.5, "width": 1.0, "height": 0.6, "scale": 1.0}
    cargo = Cargo.from_config(cfg)
    assert len(cargo.vertices) >= 3
    assert cargo.area > 0.0
    assert cargo.perimeter > 0.0


def test_unknown_shape_is_rejected():
    with pytest.raises(ValueError, match="Unknown cargo shape"):
        Cargo.from_config({"id": "c", "shape": "blob"})


def test_triangulation_covers_a_concave_outline():
    """Rigid-body solvers take convex shapes only, so a concave cargo is attached to
    its body as triangles."""
    cargo = Cargo.l_shape("obj", [0.0, 0.0], scale=1.5)
    triangles = triangulate_simple_polygon(cargo.local_vertices)
    assert len(triangles) == len(cargo.local_vertices) - 2
    def area_of(t: np.ndarray) -> float:
        u, v = t[1] - t[0], t[2] - t[0]
        return abs(0.5 * float(u[0] * v[1] - u[1] * v[0]))

    total = sum(area_of(t) for t in triangles)
    assert total == pytest.approx(cargo.area, rel=1e-9)
