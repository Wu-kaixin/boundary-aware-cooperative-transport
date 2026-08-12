"""T2 - SE(2) registration and the boundary-point velocity that reaches the barrier.

The premise of these tests is that ``estimate_yaw = False`` must be a *no-op* rather
than a cheaper approximation. v1's numbers are the branch's baseline, and a port that
changed them by a rounding-level amount while claiming to be off would make every
before/after comparison in the documentation meaningless. Several tests below
therefore assert exact equality rather than approximate.
"""

from __future__ import annotations

import numpy as np
import pytest

from dbact.boundary_map import LocalBoundaryMap, rot90
from dbact.safety_filter import SafetyFilter, SafetyFilterParams
from dbact.types import BoundaryView


def circle_view(radius: float = 0.5, count: int = 48, centre=(0.0, 0.0)) -> BoundaryView:
    """A full ring of returns with outward normals.

    Best case for *translation* observability and the worst possible case for
    rotation: see
    :func:`test_a_rotationally_symmetric_object_has_no_observable_yaw`.
    """
    angles = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    normals = np.column_stack([np.cos(angles), np.sin(angles)])
    points = np.asarray(centre, dtype=float)[None, :] + radius * normals
    return BoundaryView(
        points=points,
        normals=normals,
        confidence=np.ones(count),
        arc_length=np.full(count, 2.0 * np.pi * radius / count),
        object_ids=np.full(count, "obj", dtype="<U32"),
    )


def ellipse_view(a: float = 0.62, b: float = 0.32, count: int = 64, centre=(0.0, 0.0)) -> BoundaryView:
    """An outline with no rotational symmetry, so yaw is observable at all.

    Normals are the analytic ellipse normals ``(x/a^2, y/b^2)`` normalised, which is
    what makes the point-to-plane residual of a rotated copy non-zero.
    """
    t = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    local = np.column_stack([a * np.cos(t), b * np.sin(t)])
    raw = np.column_stack([local[:, 0] / a**2, local[:, 1] / b**2])
    normals = raw / np.linalg.norm(raw, axis=1, keepdims=True)
    perimeter = float(np.sum(np.linalg.norm(np.diff(np.vstack([local, local[:1]]), axis=0), axis=1)))
    return BoundaryView(
        points=local + np.asarray(centre, dtype=float)[None, :],
        normals=normals,
        confidence=np.ones(count),
        arc_length=np.full(count, perimeter / count),
        object_ids=np.full(count, "obj", dtype="<U32"),
    )


def transform_view(view: BoundaryView, translation=(0.0, 0.0), theta: float = 0.0,
                   about=(0.0, 0.0)) -> BoundaryView:
    cosine, sine = np.cos(theta), np.sin(theta)
    matrix = np.array([[cosine, -sine], [sine, cosine]])
    pivot = np.asarray(about, dtype=float)
    points = pivot[None, :] + (view.points - pivot[None, :]) @ matrix.T
    return BoundaryView(
        points=points + np.asarray(translation, dtype=float)[None, :],
        normals=view.normals @ matrix.T,
        confidence=view.confidence.copy(),
        arc_length=view.arc_length.copy(),
        object_ids=view.object_ids.copy(),
    )


def seeded_map(estimate_yaw: bool, view: BoundaryView | None = None, **kwargs) -> LocalBoundaryMap:
    local = LocalBoundaryMap(estimate_yaw=estimate_yaw, **kwargs)
    local.update(circle_view() if view is None else view, timestamp=0.0, dt=0.05)
    return local


def sorted_rows(array: np.ndarray) -> np.ndarray:
    """Rows in a canonical order.

    ``register`` calls ``_rekey``, which re-sorts the storage arrays by cell key, so a
    row-by-row comparison across a registration compares two different orderings of
    the same data. Sorting first is what makes "the normals did not change" testable.
    """
    values = np.asarray(array, dtype=float)
    return values[np.lexsort((values[:, 1], values[:, 0]))]


# --------------------------------------------------------------------------- #
# the rotation generator
# --------------------------------------------------------------------------- #


def test_rot90_is_the_planar_rotation_generator():
    """One definition, shared by the estimator, the velocity field and the audit.

    A sign error here would make the audit disagree with the estimator about which
    way the object is turning, and the audit would then report the error it had
    introduced itself.
    """
    assert rot90(np.array([[1.0, 0.0]]))[0] == pytest.approx([0.0, 1.0])
    assert rot90(np.array([[0.0, 1.0]]))[0] == pytest.approx([-1.0, 0.0])
    v = np.array([[3.0, -4.0], [0.5, 0.25]])
    # R90 is a rotation: norm-preserving, and orthogonal to its argument.
    assert np.allclose(np.linalg.norm(rot90(v), axis=1), np.linalg.norm(v, axis=1))
    assert np.allclose(np.einsum("ij,ij->i", rot90(v), v), 0.0)
    # Applied four times it is the identity.
    assert np.allclose(rot90(rot90(rot90(rot90(v)))), v)


# --------------------------------------------------------------------------- #
# yaw off is a no-op
# --------------------------------------------------------------------------- #


def test_translation_only_registration_reports_exactly_zero_rotation():
    """``0.0``, not ``approx(0.0)``.

    With the estimator off the third column is never built, so the value is a literal
    rather than a small number, and ``_commit_motion`` therefore never enters its
    angular branch. The translation itself is recovered to within the voxel
    quantisation -- 48 ring returns fused into 0.06 m cells under-report a 0.031 m
    shift by about 15%, which is v1's behaviour and not something this port changed.
    The claim that the yaw-off path *is* v1 rests on the 12-seed sweep reproducing
    J = 1.4908 and 68 barrier scalings exactly, which is a stronger check than any
    tolerance here.
    """
    shift = np.array([0.031, -0.017])
    off = seeded_map(False)
    result = off.register(transform_view(circle_view(), translation=shift), dt=0.05)["obj"]

    assert result.rotation == 0.0
    assert result.yaw_clamped is False
    # Right direction, right order of magnitude, under-reported by quantisation.
    assert float(np.dot(result.translation, shift)) > 0.0
    assert result.translation == pytest.approx(shift, rel=0.25)
    assert off.object_angular_velocity("obj") == 0.0
    assert off.object_rotation("obj") == 0.0


def test_point_velocities_reduce_exactly_to_the_translational_estimate_when_yaw_is_off():
    off = seeded_map(False)
    off.register(transform_view(circle_view(), translation=np.array([0.02, 0.0])), dt=0.05)
    off._commit_motion(0.05)

    linear = off.object_velocity("obj")
    assert float(np.linalg.norm(linear)) > 0.0
    field = off.object_point_velocities("obj", circle_view().points)
    # Exact: with omega == 0.0 the function returns the broadcast translation without
    # adding a term, so no floating-point difference can creep in.
    assert np.array_equal(field, np.repeat(linear[None, :], len(field), axis=0))


def test_registration_never_rotates_the_map_when_yaw_is_off():
    """``_rotate_rows`` is not called at all, which is the contract.

    Asserted by counting calls rather than by comparing the stored normals. ``register``
    calls ``_rekey``, which fuses cells that collided after the shift, and a fused
    normal is a confidence-weighted average of two originals -- a new orientation that
    has nothing to do with rotation. That is v1 behaviour, and a normal-set comparison
    would report it as a failure of this port.
    """
    off = seeded_map(False, view=ellipse_view())
    calls = []
    original = off._rotate_rows
    off._rotate_rows = lambda *args: calls.append(args) or original(*args)

    result = off.register(
        transform_view(ellipse_view(), translation=np.array([0.02, 0.01])), dt=0.05
    )["obj"]

    assert calls == []
    assert result.rotation == 0.0
    assert off._step_rotation == {}
    assert off.object_rotation("obj") == 0.0


# --------------------------------------------------------------------------- #
# yaw on
# --------------------------------------------------------------------------- #


def test_a_rotationally_symmetric_object_has_no_observable_yaw():
    """A rotating disc is geometrically indistinguishable from a stationary one.

    This is not a limitation of the estimator; it is a property of the measurement. A
    rotation about the centre maps a circle's boundary exactly onto itself, so the
    point-to-plane residual of the rotated copy is identically zero and *no* method
    reading boundary geometry can recover the rate. The estimator must therefore
    report zero rather than a small number, which is the right answer: the object's
    surface is not moving normal to itself, so the boundary-point velocity the barrier
    needs really is the translational one.

    It also bounds what SE(2) can buy on this branch's shape matrix. ``circle``,
    ``ellipse24`` and ``polygon32`` are all close to rotationally symmetric, so the
    yaw term is near-unobservable on a quarter of the twelve families by construction.
    """
    on = seeded_map(True)
    result = on.register(transform_view(circle_view(), theta=0.15), dt=0.05)["obj"]
    assert abs(result.rotation) < 1e-9
    assert on.object_angular_velocity("obj") == 0.0


def test_se2_registration_recovers_a_known_rotation():
    """An ellipse rotated about its own centre: pure rotation, no translation.

    The tolerance is on the linearisation -- the solve is first order in ``theta`` --
    and on the voxel quantisation of the stored map, which under-reports motion for
    the same reason it under-reports translation.
    """
    theta = 0.04
    on = seeded_map(True, view=ellipse_view())
    result = on.register(transform_view(ellipse_view(), theta=theta), dt=0.05)["obj"]

    assert result.rotation > 0.0, "the sign must follow the rotation"
    assert result.rotation == pytest.approx(theta, rel=0.4)
    assert np.linalg.norm(result.translation) < 0.02

    negative = seeded_map(True, view=ellipse_view())
    flipped = negative.register(transform_view(ellipse_view(), theta=-theta), dt=0.05)["obj"]
    assert flipped.rotation < 0.0


def test_se2_registration_separates_rotation_from_translation():
    theta, shift = 0.03, np.array([0.02, -0.012])
    on = seeded_map(True, view=ellipse_view())
    moved = transform_view(ellipse_view(), translation=shift, theta=theta)
    result = on.register(moved, dt=0.05)["obj"]

    assert result.rotation > 0.5 * theta
    assert float(np.dot(result.translation, shift)) > 0.0
    assert result.translation == pytest.approx(shift, abs=0.01)


def test_pure_translation_does_not_manufacture_a_rotation():
    """The failure mode that would make this feature harmful.

    A spurious yaw estimate on a translating object would inject
    ``omega |b_k - c|`` into every barrier row for no reason, and the arcs it inflates
    are exactly the ones the pushing robots are standing on.
    """
    on = seeded_map(True, view=ellipse_view())
    result = on.register(
        transform_view(ellipse_view(), translation=np.array([0.03, 0.015])), dt=0.05
    )["obj"]
    assert abs(result.rotation) < 0.01


def test_registration_rotates_stored_normals_with_their_cells():
    """A normal that did not turn with its own cell describes last frame's surface.

    The next step's point-to-plane residual is measured against these normals, and the
    barrier's half-plane orientation comes from them, so leaving them behind would put
    a one-frame-stale plane inside the safety constraint.
    """
    on = seeded_map(True, view=ellipse_view())

    # The rotation itself, on the routine that performs it: every cell and its normal
    # turn by the same angle about the same reference.
    rows = np.arange(len(on._points))
    reference = on.object_reference_point("obj")
    points_before, normals_before = on._points.copy(), on._normals.copy()
    on._rotate_rows(rows, reference, 0.10)

    cosine, sine = np.cos(0.10), np.sin(0.10)
    matrix = np.array([[cosine, -sine], [sine, cosine]])
    assert on._normals == pytest.approx(normals_before @ matrix.T)
    assert on._points == pytest.approx(
        reference[None, :] + (points_before - reference[None, :]) @ matrix.T
    )
    assert np.allclose(np.linalg.norm(on._normals, axis=1), 1.0, atol=1e-9)

    # And that ``register`` reaches it when the solve returns a rotation.
    fresh = seeded_map(True, view=ellipse_view())
    result = fresh.register(transform_view(ellipse_view(), theta=0.06), dt=0.05)["obj"]
    assert result.rotation != 0.0
    assert fresh._step_rotation["obj"] == pytest.approx(result.rotation)


def test_yaw_rate_estimate_is_clamped_to_its_declared_bound():
    """The angular counterpart of ``max_object_speed``, and a disturbance bound.

    The rate multiplies the object's reach in the boundary-point velocity, so an
    unclamped spike enters the barrier right-hand side amplified by the object radius.
    """
    on = seeded_map(True, view=ellipse_view(), max_object_yaw_rate=0.02)
    result = on.register(transform_view(ellipse_view(), theta=0.30), dt=0.05)["obj"]
    assert result.yaw_clamped is True
    assert abs(result.rotation) == pytest.approx(0.02 * 0.05)


def test_yaw_is_refused_when_no_cell_has_a_lever_arm():
    """No moment arm, no torque information; dividing by it would give noise.

    Cells clustered within a couple of voxels of the reference point cannot separate
    rotation from translation at all, and the estimate must be withheld rather than
    reported small.
    """
    tiny = LocalBoundaryMap(estimate_yaw=True, voxel_size=0.06, min_yaw_lever_voxels=2.0)
    tiny.update(circle_view(radius=0.02, count=24), timestamp=0.0, dt=0.05)
    result = tiny.register(
        transform_view(circle_view(radius=0.02, count=24), theta=0.2), dt=0.05
    ).get("obj")
    if result is not None:
        assert result.rotation == 0.0


def test_angular_velocity_uses_the_same_filter_constant_as_the_linear_one():
    """One twist, one time constant.

    A boundary-point velocity assembled from a fast translation and a slowly filtered
    rotation is not the velocity field of any rigid motion.
    """
    on = seeded_map(True, velocity_filter=0.5)
    on._step_motion["obj"] = np.array([0.01, 0.0])
    on._step_rotation["obj"] = 0.02
    on._commit_motion(0.05)

    assert on.object_velocity("obj") == pytest.approx([0.5 * 0.01 / 0.05, 0.0])
    assert on.object_angular_velocity("obj") == pytest.approx(0.5 * 0.02 / 0.05)
    assert on.object_rotation("obj") == pytest.approx(0.02)


def test_point_velocity_field_is_the_rigid_body_field():
    on = seeded_map(True)
    on.velocity["obj"] = np.array([0.05, 0.0])
    on.angular_velocity["obj"] = 0.3
    reference = on.object_reference_point("obj")

    query = np.array([[0.5, 0.0], [0.0, 0.5], [-0.5, 0.0]])
    field = on.object_point_velocities("obj", query)
    expected = np.array([0.05, 0.0])[None, :] + 0.3 * rot90(query - reference[None, :])
    assert field == pytest.approx(expected)
    # The velocity of a point on the axis is the translation alone.
    assert on.object_point_velocities("obj", reference[None, :])[0] == pytest.approx([0.05, 0.0])


def test_reference_point_is_the_maps_own_centroid_not_the_true_centre():
    """The estimate is expressed about a quantity the robot can compute.

    Two robots with different coverage of the same object hold different reference
    points, and each one's boundary-point velocity is consistent with its own
    estimate. A shared reference would have to come from somewhere, and the only
    available somewhere is the simulator.
    """
    view = circle_view(centre=(3.0, 4.0))
    half = LocalBoundaryMap(estimate_yaw=True)
    half.update(view.select(view.points[:, 0] >= 3.0), timestamp=0.0, dt=0.05)
    full = LocalBoundaryMap(estimate_yaw=True)
    full.update(view, timestamp=0.0, dt=0.05)

    partial_reference = half.object_reference_point("obj")
    full_reference = full.object_reference_point("obj")
    # Close to the ring's centre, but not equal to it: the cells are voxel-quantised
    # and fused, so even full coverage gives a centroid off by a fraction of a voxel.
    # Which is the point -- this is a map quantity, not a pose.
    assert full_reference == pytest.approx([3.0, 4.0], abs=0.03)
    # The half-covered map's centroid is displaced towards the arc it has seen. It is
    # *not* the object's centre, and that is the correct behaviour.
    assert partial_reference[0] > 3.1
    assert half.object_reference_point("missing") is None


# --------------------------------------------------------------------------- #
# what reaches the barrier
# --------------------------------------------------------------------------- #


def filter_for(mode: str = "aggregate") -> SafetyFilter:
    return SafetyFilter(
        SafetyFilterParams(
            d_min=0.30, gamma_obj=4.0, rho=0.05, r_safe=0.10, max_speed=0.30,
            object_row_mode=mode, dt=0.05,
        )
    )


def test_object_rows_are_identical_when_point_velocities_are_none():
    """The v1 path, reproduced exactly rather than approximately."""
    filt = filter_for()
    position = np.array([0.75, 0.0])
    view = circle_view()
    v_obj = np.array([0.04, 0.01])

    legacy = filt._object_rows(position, view.points, view.normals, v_obj, None)
    equivalent = filt._object_rows(
        position, view.points, view.normals, v_obj,
        np.repeat(v_obj[None, :], len(view.points), axis=0),
    )
    for a, b in zip(legacy, equivalent):
        assert a == pytest.approx(b), "a uniform point-velocity field must reproduce v1"


def test_rotation_raises_the_barrier_demand_on_an_approaching_arc():
    """The quantity the whole item is about, isolated.

    A robot standing off the ``+x`` face of a rotating ring sees the material point
    beneath it moving tangentially, and the row's right-hand side must reflect the
    component of that motion along the row normal. Feeding the body's translational
    velocity to every row understates it.
    """
    filt = filter_for(mode="pointwise")
    position = np.array([0.62, 0.0])
    view = circle_view()
    v_obj = np.zeros(2)

    # A rotating object whose centre is stationary: the translational estimate is
    # zero, so any change in the row demand comes from the rotation alone.
    omega = 0.4
    point_velocities = omega * rot90(view.points)
    _, _, quiet, _ = filt._object_rows(position, view.points, view.normals, v_obj, None)
    _, _, turning, _ = filt._object_rows(
        position, view.points, view.normals, v_obj, point_velocities
    )
    assert np.max(turning) > np.max(quiet), "rotation must be visible in the row demand"


def test_aggregate_face_takes_the_worst_point_velocity_not_the_mean():
    """One plane needs one velocity, and averaging would hide the fast arc.

    The aggregated row stands for the whole face, so it must demand at least as much
    retreat as the fastest-approaching point on that face requires. A weighted mean
    lets a fast arc be averaged away by the slow cells beside it, which is the
    aggregate under-reporting the disturbance it exists to summarise.
    """
    filt = filter_for()
    position = np.array([0.0, 0.0])
    points = np.array([[0.4, -0.1], [0.4, 0.0], [0.4, 0.1]])
    normals = np.array([[1.0, 0.0], [1.0, 0.0], [1.0, 0.0]])
    distance = np.array([0.41, 0.40, 0.41])
    velocities = np.array([[-0.01, 0.0], [0.0, 0.0], [0.09, 0.0]])

    _, _, aggregated = filt._aggregate_face(position, points, normals, distance, velocities)
    assert aggregated is not None
    # The maximum along n_bar = +x, which is the third row, not the mean 0.0267.
    assert aggregated[0] == pytest.approx([0.09, 0.0])


def test_aggregate_face_returns_no_velocity_when_none_is_supplied():
    filt = filter_for()
    points = np.array([[0.4, 0.0], [0.4, 0.1]])
    normals = np.array([[1.0, 0.0], [1.0, 0.0]])
    _, _, aggregated = filt._aggregate_face(
        np.zeros(2), points, normals, np.array([0.40, 0.41]), None
    )
    assert aggregated is None


def test_per_row_velocity_is_clamped_by_the_issf_disturbance_bound():
    """The bound has to apply per point, not to the body's translation.

    The row's right-hand side contains ``n_k^T v_{b_k}``. Bounding only the
    translational part would leave ``omega |b_k - c|`` unbounded, and the ISSf constant
    would then be stated over a quantity nothing constrains.
    """
    filt = filter_for(mode="pointwise")
    filt.params.object_velocity_bound = 0.05
    position = np.array([0.62, 0.0])
    view = circle_view()

    wild = np.repeat(np.array([[5.0, 0.0]]), len(view.points), axis=0)
    _, _, demand, _ = filt._object_rows(position, view.points, view.normals, np.zeros(2), wild)
    # Capped at the bound and then at what a speed-limited robot can deliver.
    assert np.max(demand) <= filt.params.recovery_fraction * filt.params.max_speed + 1e-9


def test_mismatched_point_velocity_length_raises():
    """Silently pairing a row with another point's motion is worse than an error."""
    filt = filter_for()
    view = circle_view()
    with pytest.raises(ValueError, match="point_velocities"):
        filt._object_rows(
            np.array([0.62, 0.0]), view.points, view.normals, np.zeros(2), np.zeros((3, 2))
        )


def test_filter_velocity_accepts_and_ignores_absent_point_velocities():
    """End to end, both call forms, with the QP actually solved."""
    filt = filter_for()
    view = circle_view()
    position = np.array([0.62, 0.0])
    nominal = np.array([-0.2, 0.0])

    without = filt.filter_velocity(
        position, nominal, [], boundary_points=view.points, boundary_normals=view.normals,
        object_velocity=np.array([0.02, 0.0]),
    )
    with_uniform = filt.filter_velocity(
        position, nominal, [], boundary_points=view.points, boundary_normals=view.normals,
        object_velocity=np.array([0.02, 0.0]),
        boundary_point_velocities=np.repeat(np.array([[0.02, 0.0]]), len(view.points), axis=0),
    )
    assert without.status == with_uniform.status
    assert without.velocity == pytest.approx(with_uniform.velocity)
