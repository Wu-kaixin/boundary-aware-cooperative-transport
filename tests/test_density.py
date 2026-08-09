"""The boundary-aware density: a measure on the boundary, in two models."""

import numpy as np
import pytest

from dbact.boundary_density import BoundaryAwareDensity, DensityParams
from dbact.cargo import Cargo
from dbact.geometry import points_in_polygon, signed_distance_to_polygon
from dbact.types import BoundaryObservation


def wall(count: int = 41, spacing: float = 0.06, confidence: float = 1.0):
    xs = (np.arange(count) - count // 2) * spacing
    return [
        BoundaryObservation("obj", "m", np.array([x, 0.0]), np.array([0.0, 1.0]), 0.0, confidence, arc_length=spacing)
        for x in xs
    ]


def params(**overrides) -> DensityParams:
    kwargs = dict(mode="offset", cage_offset=0.135, sigma=0.12, base_density=1e-3)
    kwargs.update(overrides)
    return DensityParams(**kwargs)


def full_map_of(cargo: Cargo, spacing: float = 0.05):
    """Observations covering the whole outline, with correct outward normals."""
    points, normals = cargo.boundary_samples(int(cargo.perimeter / spacing))
    return [
        BoundaryObservation("obj", "m", points[i], normals[i], 0.0, 1.0, arc_length=spacing)
        for i in range(len(points))
    ]


# --------------------------------------------------------------------------- #
# offset model
# --------------------------------------------------------------------------- #


def test_density_peaks_at_the_cage_offset():
    density = BoundaryAwareDensity.from_observations(wall(), params())
    ys = np.linspace(0.0, 0.6, 121)
    values = density(np.column_stack([np.zeros_like(ys), ys]))
    assert ys[int(np.argmax(values))] == pytest.approx(0.135, abs=0.01)


def test_base_density_is_present_far_from_the_boundary():
    density = BoundaryAwareDensity.from_observations(wall(), params())
    assert density(np.array([0.0, 8.0])) == pytest.approx(1e-3, rel=1e-6)


def test_arc_length_makes_mass_independent_of_sampling_density():
    """Half as many observations, each standing for twice the arc, must carry the
    same mass -- that is what makes phi a boundary measure rather than a sample
    count."""
    dense = BoundaryAwareDensity.from_observations(wall(count=81, spacing=0.03), params())
    coarse = BoundaryAwareDensity.from_observations(wall(count=41, spacing=0.06), params())
    assert dense.total_mass == pytest.approx(coarse.total_mass, rel=0.05)


def test_confidence_scales_mass():
    confident = BoundaryAwareDensity.from_observations(wall(confidence=1.0), params())
    unsure = BoundaryAwareDensity.from_observations(wall(confidence=0.25), params())
    assert unsure.total_mass == pytest.approx(0.25 * confident.total_mass, rel=1e-9)


def test_gap_term_prefers_boundary_no_robot_is_holding():
    held = np.array([[0.0, 0.135]])
    density = BoundaryAwareDensity.from_observations(wall(), params(gap_gain=2.0), robot_positions=held)
    # The observation directly under the robot is discounted relative to a distant one.
    near_index = len(density.gap) // 2
    assert density.gap[near_index] < 0.2
    assert density.gap[0] > 0.9


def test_unheld_field_vanishes_where_robots_already_sit():
    crowd = np.array([[x, 0.135] for x in np.linspace(-1.2, 1.2, 13)])
    density = BoundaryAwareDensity.from_observations(wall(), params(gap_gain=2.0), robot_positions=crowd)
    query = np.array([[0.0, 0.135]])
    assert float(density.unheld_field(query)[0]) < 0.2 * float(np.atleast_1d(density(query))[0])


def test_restrict_agrees_with_the_full_field_on_the_disk():
    density = BoundaryAwareDensity.from_observations(wall(count=201, spacing=0.03), params())
    center = np.array([0.0, 0.135])
    restricted = density.restrict(center, 0.4)
    assert len(restricted.points) < len(density.points)
    query = center + np.column_stack([np.linspace(-0.4, 0.4, 25), np.zeros(25)])
    assert np.allclose(np.atleast_1d(density(query)), np.atleast_1d(restricted(query)), atol=2e-3)


# --------------------------------------------------------------------------- #
# arc allocation for transport
# --------------------------------------------------------------------------- #


def test_leading_arc_is_lifted_out_of_contact():
    """A robot in contact applies force along -n, so its contribution along the
    goal is -(n . u_goal)|F|: everything on the leading half subtracts."""
    goal = np.array([1.0, 0.0])
    p = params(lead_offset=0.22, lead_threshold=0.35)
    trailing = np.array([[-1.0, 0.0]])
    lateral = np.array([[0.0, 1.0]])
    leading = np.array([[1.0, 0.0]])
    assert p.offsets_for(trailing, goal)[0] == pytest.approx(0.135)
    assert p.offsets_for(lateral, goal)[0] == pytest.approx(0.135)
    assert p.offsets_for(leading, goal)[0] == pytest.approx(0.22)


def test_arc_allocation_is_continuous_in_the_alignment():
    """A hard switch makes a robot near the arc boundary chatter between two
    targets."""
    goal = np.array([1.0, 0.0])
    p = params(lead_offset=0.22, lead_threshold=0.35)
    angles = np.linspace(-np.pi, np.pi, 400)
    normals = np.column_stack([np.cos(angles), np.sin(angles)])
    offsets = p.offsets_for(normals, goal)
    assert float(np.max(np.abs(np.diff(offsets)))) < 0.01


def test_no_goal_direction_means_a_uniform_cage():
    p = params(lead_offset=0.22)
    normals = np.array([[1.0, 0.0], [-1.0, 0.0]])
    assert np.allclose(p.offsets_for(normals, None), 0.135)


# --------------------------------------------------------------------------- #
# distance-field model
# --------------------------------------------------------------------------- #


def test_distance_field_puts_far_less_mass_inside_a_concave_object():
    """Pointwise offsetting folds into a swallowtail once d_c approaches the local
    curvature radius, and some cage targets land inside the object -- geometrically
    unreachable. The level set of the distance field is the boundary of a Minkowski
    sum and cannot self-intersect."""
    cargo = Cargo.l_shape("obj", [0.0, 0.0], scale=1.5)
    observations = full_map_of(cargo)
    lo, hi = cargo.vertices.min(axis=0) - 0.5, cargo.vertices.max(axis=0) + 0.5
    grid = np.column_stack([g.ravel() for g in np.meshgrid(np.linspace(lo[0], hi[0], 150),
                                                          np.linspace(lo[1], hi[1], 150))])
    inside = points_in_polygon(grid, cargo.vertices)

    fractions = {}
    for mode in ("offset", "distance_field"):
        density = BoundaryAwareDensity.from_observations(observations, params(mode=mode, sigma=0.20))
        values = np.atleast_1d(density(grid))
        fractions[mode] = float(np.sum(values[inside]) / np.sum(values))
    assert fractions["distance_field"] < fractions["offset"]


def test_distance_field_ridge_sits_at_the_cage_offset_outside_only():
    cargo = Cargo.rectangle("obj", [0.0, 0.0], 2.0, 2.0)
    observations = full_map_of(cargo)
    density = BoundaryAwareDensity.from_observations(observations, params(mode="distance_field", sigma=0.10))

    ys = np.linspace(1.0, 1.6, 121)
    outside = density(np.column_stack([np.zeros_like(ys), ys]))
    assert ys[int(np.argmax(outside))] - 1.0 == pytest.approx(0.135, abs=0.015)

    # Inside the object the field falls back to the base density.
    assert density(np.array([0.0, 0.0])) == pytest.approx(1e-3, rel=1e-6)


def test_offset_model_piles_up_density_at_a_concave_corner():
    """The offset curve folds in a concavity, so targets from the two incident faces
    land on top of each other and the mass there is over-counted. On a polygon with a
    sharp reflex corner the folded targets stay outside the outline, so the symptom is
    the pile-up rather than targets inside -- which is why the peak-to-median ratio is
    the quantity worth reporting."""
    cargo = Cargo.l_shape("obj", [0.0, 0.0], scale=0.9)
    observations = full_map_of(cargo)
    reflex = np.array([-0.09, -0.135]) * 0.9  # the L's inner corner

    ratios = {}
    for mode in ("offset", "distance_field"):
        density = BoundaryAwareDensity.from_observations(observations, params(mode=mode, cage_offset=0.32, sigma=0.20))
        lo, hi = cargo.vertices.min(axis=0) - 0.5, cargo.vertices.max(axis=0) + 0.5
        grid = np.column_stack([g.ravel() for g in np.meshgrid(np.linspace(lo[0], hi[0], 160),
                                                              np.linspace(lo[1], hi[1], 160))])
        signed = signed_distance_to_polygon(grid, cargo.vertices)
        band = np.abs(signed - 0.32) <= 0.40
        near_corner = np.linalg.norm(grid - reflex[None, :], axis=1) <= 0.96
        values = np.atleast_1d(density(grid))
        ratios[mode] = float(np.max(values[band & near_corner])) / float(np.median(values[band]))

    assert ratios["offset"] > ratios["distance_field"]


def test_signed_distance_helper_agrees_with_containment():
    cargo = Cargo.l_shape("obj", [0.0, 0.0], scale=1.2)
    grid = np.column_stack([g.ravel() for g in np.meshgrid(np.linspace(-1, 1, 40), np.linspace(-1, 1, 40))])
    signed = signed_distance_to_polygon(grid, cargo.vertices)
    inside = points_in_polygon(grid, cargo.vertices)
    assert np.all(signed[inside] <= 0.0)
    assert np.all(signed[~inside] >= 0.0)


# --------------------------------------------------------------------------- #
# degenerate inputs
# --------------------------------------------------------------------------- #


def test_empty_observation_set_gives_the_base_density():
    density = BoundaryAwareDensity.from_observations([], params())
    assert density(np.array([1.0, 1.0])) == pytest.approx(1e-3)
    assert density.total_mass == 0.0


def test_observations_without_arc_length_still_produce_a_usable_field():
    obs = [BoundaryObservation("obj", "m", np.array([0.0, 0.0]), np.array([0.0, 1.0]), 0.0, 1.0)]
    density = BoundaryAwareDensity.from_observations(obs, params())
    assert density(np.array([0.0, 0.135])) > density(np.array([0.0, 2.0]))


# --------------------------------------------------------------------------- #
# D10 - exploration demand past the ends of what has been observed
# --------------------------------------------------------------------------- #


def test_exploration_is_off_by_default_and_changes_nothing():
    observations = wall()
    off = BoundaryAwareDensity.from_observations(observations, params())
    explicit = BoundaryAwareDensity.from_observations(observations, params(explore_gain=0.0))
    q = np.array([[0.0, 0.135], [1.5, 0.135], [-1.5, 0.135]])
    assert np.allclose(off(q), explicit(q))
    assert len(off.points) == len(observations)


def test_exploration_adds_one_target_at_each_end_of_an_open_arc():
    observations = wall(count=17)
    density = BoundaryAwareDensity.from_observations(
        observations, params(explore_gain=4.0, explore_step=0.25, explore_window=0.18)
    )
    assert len(density.points) == len(observations) + 2
    added = density.points[len(observations):]
    xs = np.sort(added[:, 0])
    ends = np.array([observations[0].point[0], observations[-1].point[0]])
    assert xs[0] == pytest.approx(ends.min() - 0.25, abs=1e-9)
    assert xs[1] == pytest.approx(ends.max() + 0.25, abs=1e-9)


def test_exploration_raises_the_density_past_the_end_and_not_in_the_middle():
    observations = wall(count=17)
    off = BoundaryAwareDensity.from_observations(observations, params())
    on = BoundaryAwareDensity.from_observations(observations, params(explore_gain=4.0))
    middle = np.array([[0.0, 0.135]])
    beyond = np.array([[observations[-1].point[0] + 0.25, 0.135]])
    # Not exactly equal: the added target has a Gaussian tail. It is 1.5e-8 of the
    # value at the middle of a 1 m arc, which is what "local perturbation" means.
    assert on(middle) == pytest.approx(off(middle), rel=1e-6)
    assert on(beyond) > 1.5 * off(beyond)


def test_a_closed_outline_has_no_frontier_to_explore():
    """The demand has to switch itself off, or it competes with the cage forever."""
    cargo = Cargo(object_id="c", vertices=np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]))
    observations = full_map_of(cargo, spacing=0.05)
    density = BoundaryAwareDensity.from_observations(
        observations, params(explore_gain=4.0, explore_step=0.25, explore_window=0.18)
    )
    assert len(density.points) == len(observations)


def test_a_continuation_into_already_mapped_boundary_is_dropped():
    """A gap smaller than one step is not a frontier: the boundary is known there."""
    left = wall(count=9)
    right = [
        BoundaryObservation("obj", "m", np.array([p.point[0] + 0.60, 0.0]), np.array([0.0, 1.0]),
                            0.0, 1.0, arc_length=0.06)
        for p in left
    ]
    density = BoundaryAwareDensity.from_observations(
        left + right, params(explore_gain=4.0, explore_step=0.25, explore_window=0.18)
    )
    added = len(density.points) - (len(left) + len(right))
    # Two outer ends are genuine frontiers; the 0.36 m interior gap is spanned by
    # a 0.25 m step that lands within 0.11 m of the far segment, so it is dropped.
    assert added == 2


def test_a_negative_exploration_gain_is_rejected():
    with pytest.raises(ValueError):
        params(explore_gain=-1.0)


def test_invalid_mode_is_rejected():
    with pytest.raises(ValueError, match="density mode"):
        DensityParams(mode="magic")
