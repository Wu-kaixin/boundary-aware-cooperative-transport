"""D1 - the random constrained task: the admissible set has to be a definition."""

import math

import numpy as np
import pytest

from dbact.contracts import ContractViolation
from dbact.provenance import frame_rng
from dbact.task import TaskSampler, TransportTask

DOMAIN = (0.0, 8.0, 0.0, 8.0)


def sampler(**kwargs) -> TaskSampler:
    defaults = dict(distance_min=0.35, distance_max=0.60, wall_margin=0.35, tolerance=0.12)
    defaults.update(kwargs)
    return TaskSampler(**defaults)


def sample(seed: int = 0, start=(4.0, 4.0), **kwargs) -> TransportTask:
    return sampler(**kwargs).sample(
        frame_rng("transport_task", "cargo_0", base=seed),
        object_id="cargo_0",
        start=np.asarray(start, dtype=float),
        object_radius=1.27,
        cage_offset=0.135,
        robot_radius=0.16,
        domain=DOMAIN,
    )


def test_the_same_seed_gives_the_same_task():
    """Without this a multi-seed table cannot attribute its own variance."""
    first, second = sample(seed=3), sample(seed=3)
    assert np.allclose(first.direction, second.direction)
    assert first.distance == pytest.approx(second.distance)


def test_different_seeds_give_different_directions():
    angles = {round(sample(seed=s).angle_deg, 3) for s in range(12)}
    assert len(angles) == 12


def test_the_direction_is_a_unit_vector_and_the_distance_is_in_range():
    for seed in range(20):
        task = sample(seed=seed)
        assert float(np.linalg.norm(task.direction)) == pytest.approx(1.0)
        assert 0.35 <= task.distance <= 0.60


def test_the_goal_leaves_room_for_the_object_and_its_ring():
    """The rejection is the definition of 'within the controllable range'."""
    for seed in range(30):
        task = sample(seed=seed)
        clearance = task.clearance
        assert DOMAIN[0] + clearance <= task.goal_point[0] <= DOMAIN[1] - clearance
        assert DOMAIN[2] + clearance <= task.goal_point[1] <= DOMAIN[3] - clearance


def test_a_start_too_close_to_a_wall_is_rejected_rather_than_sampled():
    with pytest.raises(ContractViolation, match="workspace wall"):
        sample(start=(0.4, 4.0))


def test_an_unreachable_distance_raises_instead_of_returning_a_bad_task():
    with pytest.raises(ContractViolation, match="no admissible transport direction"):
        sample(distance_min=6.0, distance_max=6.5)


def test_progress_is_signed_and_cross_track_is_not():
    task = sampler().sample(
        frame_rng("t", base=0),
        object_id="c",
        start=np.array([4.0, 4.0]),
        object_radius=1.0,
        cage_offset=0.135,
        robot_radius=0.16,
        domain=DOMAIN,
    )
    d = task.direction
    perpendicular = np.array([-d[1], d[0]])
    assert task.progress(task.start + 0.5 * d) == pytest.approx(0.5)
    assert task.progress(task.start - 0.5 * d) == pytest.approx(-0.5)
    assert task.cross_track(task.start + 0.5 * d) == pytest.approx(0.0, abs=1e-9)
    assert task.cross_track(task.start + 0.3 * perpendicular) == pytest.approx(0.3)
    assert task.cross_track(task.start - 0.3 * perpendicular) == pytest.approx(0.3)


def test_the_acceptance_rate_is_reported_rather_than_assumed():
    rate = sampler().acceptance_rate(
        frame_rng("acceptance", base=0),
        start=np.array([4.0, 4.0]),
        object_radius=1.27,
        cage_offset=0.135,
        robot_radius=0.16,
        domain=DOMAIN,
        draws=4000,
    )
    # A 0.6 m reach from the middle of an 8 m room clears the walls in every
    # direction, so the admissible set here is the full circle.
    assert rate == pytest.approx(1.0)

    tight = sampler(distance_max=1.0).acceptance_rate(
        frame_rng("acceptance", base=0),
        start=np.array([2.3, 4.0]),
        object_radius=1.27,
        cage_offset=0.135,
        robot_radius=0.16,
        domain=DOMAIN,
        draws=4000,
    )
    assert 0.0 < tight < 1.0


def test_the_sampled_angle_range_is_honoured():
    for seed in range(15):
        task = sample(seed=seed, angle_min_deg=30.0, angle_max_deg=60.0)
        assert 30.0 - 1e-9 <= task.angle_deg <= 60.0 + 1e-9


def test_as_dict_carries_the_direction_that_explains_the_run():
    payload = sample(seed=1).as_dict()
    for field in ("direction", "angle_deg", "target_distance", "goal_point", "sampling_attempts"):
        assert field in payload
    assert math.isfinite(payload["angle_deg"])
