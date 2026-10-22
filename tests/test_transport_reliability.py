"""Control-level regressions: a latched quorum cannot bypass stop permissions."""
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.control.controller import DBACTController, DBACTParams
from dbact.control.phase import Phase
from dbact.control.transport import DirectionalProgressController
from dbact.types import AgentState, BoundaryView
from dbact_sim.experiments.reliability import classify


def setup_controller(phase, quality="credible"):
    c = DBACTController(DBACTParams(task_mode="transport", min_push_agents=1,
                                   transport_feedback_guard=True, brake_release_integral=True,
                                   press_margin=.015, object_velocity_bound=.02, rho=.01,
                                   cage_offset=.135), (0, 8, 0, 8))
    a = AgentState("a", np.array([0.05, 0.0]))
    c.tasks = {"cargo": SimpleNamespace(direction=np.array([1., 0.]), distance=1.)}
    c.goal_directions = {"cargo": np.array([1., 0.])}
    c.maps["a"] = SimpleNamespace(object_velocity=lambda _: np.array([0.1, 0.]),
                                  object_displacement=lambda _: np.array([.9, 0.]))
    c.transport_loops["a"] = DirectionalProgressController(c.params.transport_control())
    c.transport_loops["a"].integral = 0.05
    c._progress = {"a": {"cargo": 0.9}}
    c._progress_quality = {"a": {"cargo": quality}}
    c.team_estimate_quality = quality
    c.phase_monitor.phase = phase
    c.phase_monitor.brake_push_quorum = True
    view = BoundaryView(points=np.array([[0.2, 0.]]), normals=np.array([[-1., 0.]]),
                        confidence=np.ones(1), arc_length=np.ones(1), object_ids=np.array(["cargo"]))
    return c, a, view


def command(c, a, view):
    return c._transport_command(0, [a], [], view, [True], .05, c.density_params)


def test_rejected_transport_candidates_are_not_enabled_by_default():
    params = DBACTParams()
    assert not params.transport_feedback_guard
    assert not params.brake_release_integral


def test_full_quorum_brake_releases_integral_at_excess_speed():
    c, a, view = setup_controller(Phase.BRAKE)
    _, pushing, effort, _ = command(c, a, view)
    assert pushing
    assert effort == 0.0  # 0.1 m/s exceeds the 0.055 m/s falling reference
    assert c.transport_loops["a"].integral == 0.0


def test_credible_slow_brake_can_escape_static_friction_then_releases_at_reference():
    loop = DirectionalProgressController(DBACTParams().transport_control())
    for _ in range(30):
        output = loop.update(np.array([1., 0.]), np.zeros(2), .9, 1., .05,
                             braking=True, allow_braking_integral=True)
    assert output.integral > 0.
    assert output.effort > loop.params.kp * output.reference_speed
    released = loop.update(np.array([1., 0.]), np.array([.2, 0.]), .9, 1., .05,
                           braking=True, allow_braking_integral=True)
    assert released.effort == released.integral == 0.


@pytest.mark.parametrize("phase,quality", [(Phase.HOLD, "credible"),
                                          (Phase.TRANSPORT, "unavailable"),
                                          (Phase.BRAKE, "unavailable")])
def test_hold_or_missing_feedback_cannot_press_or_follow_stale_velocity(phase, quality):
    c, a, view = setup_controller(phase, quality)
    velocity, pushing, effort, _ = command(c, a, view)
    assert not pushing
    assert effort == 0.0
    assert c.transport_loops["a"].integral == 0.0
    # At this point the robot is already outside its configured cage offset.
    assert np.allclose(velocity, 0.)


@pytest.mark.parametrize("signed,endpoint,outcome", [(-.2, .21, "undershoot"), (.2, .21, "overshoot"),
                                                    (.01, .2, "cross_track_error"), (.1, .13, "combined_endpoint_error")])
def test_outcome_geometry_is_not_a_causal_label(signed, endpoint, outcome):
    row = classify({"case_id": "x", "task": {"cargo": {"tolerance": .12}},
                    "signed_progress_error": signed, "euclidean_endpoint_error": endpoint,
                    "task_completed": False, "settled": True})
    assert row["outcome"] == outcome
    assert row["trace"]["causal_attribution"] == "unresolved"
    assert not row["trace"]["available"]


def test_trace_bias_and_contact_absence_remain_observations():
    result = {"case_id": "x", "task": {"cargo": {"tolerance": .12}}, "task_completed": False,
              "signed_progress_error": -.2, "euclidean_endpoint_error": .2}
    row = classify(result, [{"phase": "BRAKE", "true_progress": .8, "estimated_progress": 1.,
                             "positive_contacts": 0, "phase_guards": {"estimate_quality": "unavailable"},
                             "phase_response": {"estimate_stopped": True}}])
    assert row["trace"]["brake_progress_bias_m"] == pytest.approx(.2)
    assert row["trace"]["zero_positive_contact_frames"] == 1
    assert "estimate_stopped" in row["observed_tags"]


def test_phase_change_invalidates_a_slow_planners_held_push(monkeypatch):
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg = load_yaml("configs/sim/d/l_shape_closed_loop.yaml")
    cfg["controller"]["planning_every"] = 10
    env = SimulationEnvironment(cfg, seed=2)
    env.step()
    env.controller.phase_monitor.phase = Phase.HOLD
    calls = []
    original = env.controller._nominal_command
    def counted(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)
    monkeypatch.setattr(env.controller, "_nominal_command", counted)
    env.step()
    assert len(calls) == len(env.agents)
