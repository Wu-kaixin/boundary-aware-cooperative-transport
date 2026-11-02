"""Information permissions and controller/evaluator separation for new arms."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.control.boundary_navigation import point_goal_feedback, push_alignment
from dbact.control.controller import DBACTParams
from dbact.control.phase import Phase, PhaseMonitor, PhaseSignals


def environment(mode="legacy"):
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg = load_yaml("configs/sim/d/l_shape_closed_loop.yaml")
    cfg["controller"]["boundary_navigation"] = mode
    return SimulationEnvironment(cfg, seed=2)


def test_disconnected_estimates_cannot_vote_and_absent_is_not_zero():
    ds = {"own": np.array([.5, .2]), "neighbor": np.array([.7, .4]), "remote": np.array([100., -100.])}
    vs = {k: np.zeros(2) for k in ds}
    value = point_goal_feedback(ds, vs, ["own", "neighbor", "own", "absent"], [1.2, 0.], .02)
    np.testing.assert_allclose(value.displacement, [.6, .3])
    assert value.voters == ("own", "neighbor")
    assert point_goal_feedback(ds, vs, ["absent"], [1.2, 0.], .02) is None


def test_scalar_arrival_with_large_lateral_error_cannot_authorize_hold():
    value = point_goal_feedback({"a": np.array([1.2, -.5])}, {"a": np.zeros(2)}, ["a"], [1.2, 0.], .02)
    assert value.remaining == pytest.approx(.48)
    np.testing.assert_allclose(value.direction, [0., 1.])
    monitor = PhaseMonitor(phase=Phase.BRAKE)
    monitor.update(PhaseSignals(progress=1.2-value.remaining, target_distance=1.2, estimated_speed=0.), 1)
    assert monitor.phase == Phase.BRAKE


def test_two_dimensional_control_tube_uses_no_physical_success_flag():
    value = point_goal_feedback({"a": np.array([1.195, .005])}, {"a": np.zeros(2)}, ["a"], [1.2, 0.], .02)
    assert value.remaining == 0.
    monitor = PhaseMonitor(phase=Phase.BRAKE)
    monitor.update(PhaseSignals(progress=1.2-value.remaining, target_distance=1.2, estimated_speed=0.), 1)
    assert monitor.phase == Phase.HOLD


def test_reversed_goal_can_be_served_by_compressive_contact():
    value = point_goal_feedback({"a": np.array([1.5, 0.])}, {"a": np.zeros(2)}, ["a"], [1.2, 0.], .02)
    normal = np.array([1., 0.])
    assert push_alignment(normal, [1., 0.], value.direction, "vector_goal") == -1.
    np.testing.assert_array_equal(-normal, value.direction)


@pytest.mark.parametrize("mode", ["legacy", "adaptive_arc", "vector_goal"])
def test_membership_keeps_the_same_physical_inward_normal(mode):
    assert push_alignment([0., -1.], [1., 0.], [0., 1.], mode) == (0. if mode == "legacy" else -1.)


def test_only_fresh_full_vector_credible_neighbors_enter_navigation():
    env = environment("vector_goal")
    c, agents = env.controller, env.agents
    c._ensure_state(agents)
    oid = next(iter(c.tasks))
    for k, a in enumerate(agents):
        m = c.maps[a.agent_id]
        m.displacement[oid] = np.array([k*.01, k*.01])
        m.velocity[oid] = np.zeros(2)
        m.motion_quality[oid] = "credible" if k in (0, 2) else "degraded"
        m.last_registration[oid] = SimpleNamespace(matches=5)
    neighbors = [[1] for _ in agents]
    neighbors[0] = [1]
    c._update_navigation(agents, neighbors)
    assert c._navigation[agents[0].agent_id].voters == (agents[0].agent_id,)
    assert c._navigation[agents[1].agent_id] is None
    assert set(c.team_navigation.voters) == {agents[0].agent_id, agents[2].agent_id}


def test_disabled_path_and_default_are_exactly_equal():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg = load_yaml("configs/sim/d/l_shape_closed_loop.yaml")
    other = copy.deepcopy(cfg)
    other["controller"]["boundary_navigation"] = "legacy"
    a, b = SimulationEnvironment(cfg, seed=2), SimulationEnvironment(other, seed=2)
    for _ in range(8):
        a.step(); b.step()
        np.testing.assert_array_equal([x.position for x in a.agents], [x.position for x in b.agents])
        np.testing.assert_array_equal(a.cargoes[0].position, b.cargoes[0].position)


@pytest.mark.parametrize("mode", ["adaptive_arc", "vector_goal"])
def test_candidate_cannot_reopen_shared_hold_or_change_physics(mode):
    env = environment(mode)
    reference = environment()
    assert env.contact_params == reference.contact_params
    assert env.controller.params.contact_contract() == reference.controller.params.contact_contract()
    env.controller.trace_enabled = True
    for _ in range(35): env.step()
    env.controller.phase_monitor.phase = Phase.HOLD
    env.step()
    assert not any(d.push_side for d in env.controller.diagnostics)
    assert all(np.isfinite(a.position).all() for a in env.agents)


def test_navigation_cannot_silently_combine_with_old_contact_candidate():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg = load_yaml("configs/sim/d/l_shape_closed_loop.yaml")
    cfg["controller"].update(boundary_navigation="vector_goal", confirmed_contact_handoff=True)
    with pytest.raises(ValueError, match="no other contact candidate"):
        SimulationEnvironment(cfg, seed=2)
    assert DBACTParams().boundary_navigation == "legacy"


def test_transport_command_actually_admits_a_compressive_side_pusher(monkeypatch):
    from dbact.types import BoundaryView
    outputs = {}
    for mode in ("legacy", "adaptive_arc"):
        env = environment(mode)
        c = env.controller
        c._ensure_state(env.agents)
        oid = next(iter(c.tasks))
        goal = c.goal_directions[oid]
        side = np.array([-goal[1], goal[0]])
        direction = (goal+side)/np.sqrt(2)
        normal = -side
        point = np.array([4., 4.])
        env.agents[0].position = point + (c._press_floor+.02)*normal
        view = BoundaryView(points=point[None,:], normals=normal[None,:], confidence=np.ones(1),
                            arc_length=np.ones(1), object_ids=np.array([oid]))
        monkeypatch.setattr(c, "_steered_direction", lambda *args: direction)
        c.phase_monitor.phase = Phase.CONTACT_READY
        command, pushing, effort, _ = c._transport_command(0, env.agents, list(range(1,5)),
            view, [True]*len(env.agents), .05, c.density_params)
        outputs[mode] = pushing
        if pushing:
            assert effort > 0.
            assert float(command @ (-normal)) > 0.
            assert abs(float(command @ goal)) < 1e-12
    assert outputs == {"legacy":False, "adaptive_arc":True}


@pytest.mark.parametrize("band", [0., -1., float("nan")])
def test_nonpositive_or_nonfinite_control_band_rejected(band):
    with pytest.raises(ValueError): point_goal_feedback({}, {}, [], [1., 0.], band)
