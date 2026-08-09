"""D3/D4 - the outer loop: the property that matters is that it escapes a stall."""

import numpy as np
import pytest

from dbact.transport_control import DirectionalProgressController, TransportControlParams

GOAL = np.array([1.0, 0.0])
DT = 0.05


def params(**kwargs) -> TransportControlParams:
    defaults = dict(
        reference_speed=0.055, kp=2.4, ki=3.0, effort_max=0.30,
        deadband_fraction=0.05, brake_gain=0.55, convoy_gain=1.0, convoy_max=0.25,
    )
    defaults.update(kwargs)
    return TransportControlParams(**defaults)


def run(loop: DirectionalProgressController, speed, steps: int, progress=0.0, target=1.0):
    """Drive the loop with a fixed or callable object speed; return the efforts."""
    out = []
    for k in range(steps):
        v = speed(k) if callable(speed) else speed
        p = progress(k) if callable(progress) else progress
        out.append(loop.update(GOAL, np.array([v, 0.0]), p, target, DT))
    return out


def test_a_stalled_cargo_makes_the_effort_grow_without_bound_until_it_saturates():
    """This is the whole point. The A-branch law produced a constant press whose
    magnitude carried no information about whether the cargo was moving, and J was
    flat at 0.0561 m for 400 frames."""
    loop = DirectionalProgressController(params())
    efforts = [e.effort for e in run(loop, speed=0.0, steps=200)]
    assert efforts[0] < efforts[10] < efforts[40]
    assert efforts[-1] == pytest.approx(0.30)


def test_the_effort_falls_back_once_the_cargo_moves_faster_than_the_reference():
    loop = DirectionalProgressController(params())
    run(loop, speed=0.0, steps=120)
    stalled = loop.integral
    fast = run(loop, speed=0.20, steps=40)
    assert loop.integral < stalled
    assert fast[-1].effort < 0.30


def test_the_integral_bound_comes_from_the_actuator_limit_not_from_tuning():
    p = params()
    assert p.integral_max == pytest.approx((p.effort_max - p.kp * p.reference_speed) / p.ki)
    loop = DirectionalProgressController(p)
    run(loop, speed=0.0, steps=2000)
    assert loop.integral <= p.integral_max + 1e-12


def test_a_blocked_robot_does_not_accumulate_demand_it_will_have_to_unwind():
    """At the barrier the robot is not short of effort, so integrating there only
    stores demand."""
    free = DirectionalProgressController(params())
    blocked = DirectionalProgressController(params())
    for _ in range(60):
        free.update(GOAL, np.zeros(2), 0.0, 1.0, DT, blocked=False)
        blocked.update(GOAL, np.zeros(2), 0.0, 1.0, DT, blocked=True)
    assert blocked.integral == pytest.approx(0.0)
    assert free.integral > 0.0


def test_leaving_saturation_releases_the_press_at_once_and_the_state_shortly_after():
    """Twenty seconds of stall must not cost twenty seconds of unwinding."""
    loop = DirectionalProgressController(params())
    run(loop, speed=0.0, steps=400)
    assert loop.integral == pytest.approx(loop.params.integral_max)

    # The press stops on the first step the cargo runs away, because the
    # proportional term alone already drives the output below zero.
    first = loop.update(GOAL, np.array([0.5, 0.0]), 0.0, 1.0, DT)
    assert first.effort == pytest.approx(0.0)
    assert loop.integral < loop.params.integral_max

    # And the stored demand is gone within a handful of steps, so the loop is not
    # still pressing on stale state when the cargo next needs it.
    run(loop, speed=0.5, steps=4)
    assert loop.integral == pytest.approx(0.0)


def test_the_reference_falls_linearly_inside_braking_distance():
    loop = DirectionalProgressController(params(reference_speed=0.055, brake_gain=0.55))
    assert loop.reference_for(1.0) == pytest.approx(0.055)
    assert loop.reference_for(0.10) == pytest.approx(0.055)
    assert loop.reference_for(0.05) == pytest.approx(0.0275)
    assert loop.reference_for(0.0) == pytest.approx(0.0)


def test_reaching_the_target_latches_hold_and_releases_the_press():
    loop = DirectionalProgressController(params())
    run(loop, speed=0.0, steps=200)
    arrived = loop.update(GOAL, np.zeros(2), progress=1.0, target_distance=1.0, dt=DT)
    assert arrived.holding
    assert arrived.effort == pytest.approx(0.0)
    # The latch does not come undone if the estimate drifts back below the target.
    after = loop.update(GOAL, np.zeros(2), progress=0.5, target_distance=1.0, dt=DT)
    assert after.holding
    assert after.effort == pytest.approx(0.0)


def test_effort_is_never_negative_so_the_arc_cannot_pull_the_cargo_back():
    loop = DirectionalProgressController(params())
    for result in run(loop, speed=0.4, steps=50):
        assert result.effort >= 0.0


def test_an_inactive_robot_holds_no_state():
    loop = DirectionalProgressController(params())
    run(loop, speed=0.0, steps=80)
    idle = loop.update(GOAL, np.zeros(2), 0.0, 1.0, DT, active=False)
    assert idle.effort == pytest.approx(0.0)
    assert loop.integral == pytest.approx(0.0)


def test_the_convoy_feed_forward_is_the_estimate_itself_up_to_its_cap():
    loop = DirectionalProgressController(params(convoy_max=0.25))
    slow = loop.convoy_velocity(np.array([0.05, 0.02]))
    assert np.allclose(slow, [0.05, 0.02])
    fast = loop.convoy_velocity(np.array([1.0, 0.0]))
    assert float(np.linalg.norm(fast)) == pytest.approx(0.25)


def test_the_dead_band_keeps_the_integral_off_estimator_noise():
    loop = DirectionalProgressController(params(deadband_fraction=0.9))
    run(loop, speed=0.054, steps=100)
    assert loop.integral == pytest.approx(0.0)
