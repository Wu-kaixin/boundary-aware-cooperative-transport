"""Geometry, permissions and control-step regressions for the bounded candidate."""
import numpy as np

from dbact.control.contact_roles import (
    local_contact_roles, select_observed_target, redeployment_velocity,
)


def test_orthogonal_contacts_can_supply_correction_without_tension():
    normals = np.array([[-1., 0.], [0., -1.], [1., 0.]])
    points = np.array([[0., 0.], [1., 0.], [2., 0.]])
    result = local_contact_roles(points, normals, points + .1 * normals,
                                 np.array([1., 1.]) / np.sqrt(2), .2)
    assert result.residual < .001
    assert np.allclose(result.weights, [1., 1., 0.])
    assert np.all((result.weights >= 0) & (result.weights <= 1))


def test_forward_only_contacts_do_not_certify_lateral_correction():
    result = local_contact_roles([[0., 0.]], [[-1., 0.]], [[-.1, 0.]], [0., 1.], .2)
    assert result.residual == 1.
    assert result.weights[0] == 0.


def test_unobserved_neighbour_does_not_supply_fictitious_contact():
    result = local_contact_roles([[0., 0.]], [[-1., 0.]], [[-.1, 0.], [5., 5.]], [1., 0.], .2)
    assert result.observed.tolist() == [True, False]
    assert result.weights[1] == 0.


def test_redeployment_retracts_before_tangential_travel():
    kw = dict(clearance=.22, gain=1., max_speed=.35)
    first = redeployment_velocity(np.array([.1, 0.]), np.zeros(2), np.array([1., 0.]),
                                 np.array([.22, 1.]), np.zeros(2), **kw)
    assert first[0] > 0 and first[1] == 0
    second = redeployment_velocity(np.array([.22, 0.]), np.zeros(2), np.array([1., 0.]),
                                  np.array([.22, 1.]), np.zeros(2), **kw)
    assert second[0] == 0 and second[1] > 0
    assert np.linalg.norm(second) <= .35


def test_observed_target_rejects_an_occupied_patch_and_a_concavity_shortcut():
    theta = np.array([0., .6, 1.2])
    points = np.c_[np.cos(theta), np.sin(theta)]
    kw = dict(clearance=.22, radius=1., separation=.34)
    target = select_observed_target(points, points, [1.22, 0.], [], [0., -1.], **kw)
    assert target is not None
    assert select_observed_target(points, points, [1.22, 0.], [target], [0., -1.], **kw) is None
    assert select_observed_target([[1., 0.], [.9, .5]], [[1., 0.], [-1., 0.]],
                                 [1.22, 0.], [], [1., 0.], **kw) is None


def test_previous_safety_feedback_survives_the_whole_control_step(monkeypatch):
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    env = SimulationEnvironment(load_yaml('configs/sim/d/l_shape_closed_loop.yaml'), seed=2)
    env.controller.params.transport_safety_feedback = True
    env.step()
    controller = env.controller
    expected = {agent.agent_id: (i % 2 == 0) for i, agent in enumerate(env.agents)}
    for diagnostic in controller.diagnostics:
        diagnostic.modification = .1 if expected[diagnostic.agent_id] else 0.
    original = controller._nominal_command
    observed = {}

    def checking(i, agents, *args):
        observed[agents[i].agent_id] = controller._blocked_along(agents[i].agent_id)
        return original(i, agents, *args)

    monkeypatch.setattr(controller, '_nominal_command', checking)
    env.step()
    assert observed == expected


def test_candidate_is_off_by_default_and_cannot_change_predeployment():
    from dbact.control.controller import DBACTParams
    from dbact.control.contact_roles import DynamicContactPolicy
    p = DBACTParams()
    assert not p.dynamic_contact_roles
    assert not p.transport_safety_feedback
    policy = DynamicContactPolicy(p)
    roles, command, trace = policy.decide(
        'robot', np.array([[0., 0.]]), np.array([[-1., 0.]]), np.array([-.1, 0.]),
        [], np.array([1., 0.]), np.array([1., 0.]), np.array([0., .5]), np.zeros(2), .05, active=False)
    assert command is None and trace['contact_role'] == 'inactive'
    assert policy.states['robot'].attempts == 0


def test_contact_ready_robot_can_redeploy_but_has_a_finite_lease_and_attempt_budget():
    from dbact.control.controller import DBACTParams
    from dbact.control.contact_roles import DynamicContactPolicy
    p = DBACTParams()
    policy = DynamicContactPolicy(p)
    theta = np.array([0., .6])
    points = np.c_[np.cos(theta), np.sin(theta)]
    args = ('robot', points, points, np.array([1.13, 0.]), [], np.array([0., -1.]),
            np.array([1., 0.]), np.array([0., .2]), np.zeros(2), .05)
    outputs = [policy.decide(*args, active=True) for _ in range(500)]
    assert any(command is not None for _, command, _ in outputs)
    assert any(trace['contact_redeploy_reason'] == 'lease_expired' for _, _, trace in outputs)
    assert policy.states['robot'].attempts == 3
    assert outputs[-1][1] is None
    _, command, trace = policy.decide(*args, active=False)
    assert command is None and trace['contact_role'] == 'inactive'


def test_default_and_explicit_legacy_controls_match_with_candidate_code_installed():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg = load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    first = SimulationEnvironment(cfg, seed=2)
    second = SimulationEnvironment(cfg, seed=2)
    second.controller.params.transport_safety_feedback = False
    second.controller.params.dynamic_contact_roles = False
    for _ in range(5):
        first.step()
        second.step()
        np.testing.assert_array_equal([a.position for a in first.agents], [a.position for a in second.agents])


def test_dynamic_integration_and_hold_keep_original_stop_permissions():
    from dbact.control.phase import Phase
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg = load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(dynamic_contact_roles=True, transport_safety_feedback=True)
    env = SimulationEnvironment(cfg, seed=2)
    env.controller.trace_enabled = True
    for _ in range(40):
        env.step()
    assert all(np.isfinite(a.position).all() for a in env.agents)
    assert any(d.contact_role != 'baseline' for d in env.controller.diagnostics)
    env.controller.phase_monitor.phase = Phase.HOLD
    env.step()
    assert not any(d.push_side for d in env.controller.diagnostics)
    assert all(s.target is None for s in env.controller.contact_roles.states.values())
