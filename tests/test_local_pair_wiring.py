"""Execution accounting and controller information flow for acquisition pairs."""
from dataclasses import asdict
import copy

import numpy as np
import pytest

from dbact.safety.filter import SafetyFilter
from dbact.types import AgentState, ControlCommand
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml
from test_local_pair_safety import module, pair


def config(enabled=True):
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True,
        initial_reference_motion_model='translation', initial_reference_use_all_captured=True,
        initial_reference_map_motion=True, reference_map_reacquisition=True,
        initial_acquisition_circulation=True, acquisition_finite_support=True,
        acquisition_pair_safety=enabled)
    return cfg


def test_negotiation_precedes_executed_fallback_accounting():
    m = module()
    a, b, pa, pb = pair()
    accepted, trace = m.exchange({'a': a, 'b': b})
    safety = SafetyFilter(pa.parameters)
    assert trace['a']['independent_feasible'] is False
    result = safety.finish_velocity(pa, pair_command=accepted['a'])
    safety.finish_velocity(pb, pair_command=accepted['b'])
    assert result.status == 'paired' and result.barrier_scale == 1.
    assert result.feasible and result.agent_residual_min >= -1e-7 and result.object_residual_min >= -1e-7
    # Original individual zero feasibility is retained, despite a feasible pair.
    assert safety.stats.zero_input_feasible_failures == 1
    assert safety.stats.solves == 2 and safety.stats.statuses == {'paired': 2}
    assert safety.stats.barrier_scalings == safety.stats.margin_relaxations == safety.stats.fallbacks == 0
    np.testing.assert_array_equal(result.b_original, a.b)
    assert not np.array_equal(result.b_effective, result.b_original)


def test_failed_negotiation_keeps_original_executed_scaling_classification():
    m = module()
    a, b, pa, _ = pair(peer_rows=((-1., 0., 0.),))
    accepted, _ = m.exchange({'a': a, 'b': b})
    assert not accepted
    safety = SafetyFilter(pa.parameters)
    result = safety.finish_velocity(pa, pair_command=accepted.get('a'))
    assert result.status == 'scaled_barrier' and result.barrier_scale < 1.
    assert safety.stats.barrier_scalings == safety.stats.solves == 1


def test_only_new_boolean_differs_from_existing_numeric_parameters():
    before, after = SimulationEnvironment(config(False), seed=1), SimulationEnvironment(config(True), seed=1)
    left, right = asdict(before.controller.params), asdict(after.controller.params)
    assert left.pop('acquisition_pair_safety') is False
    assert right.pop('acquisition_pair_safety') is True
    assert left == right and before.contact_params == after.contact_params
    assert before.controller.phase_monitor.gates == after.controller.phase_monitor.gates


@pytest.mark.parametrize('change', [dict(acquisition_pair_safety='false'),
    dict(acquisition_finite_support=False), dict(backend='cvxpy')])
def test_incompatible_pair_selection_is_rejected(change):
    cfg = config()
    cfg['controller'].update(change)
    with pytest.raises(ValueError, match='[Pp]air'):
        SimulationEnvironment(cfg, seed=1)


def test_missing_and_disabled_selector_are_exact_short_physical_controls():
    cfg, missing = config(False), config(False)
    missing['controller'].pop('acquisition_pair_safety')
    first, second = SimulationEnvironment(cfg, seed=1), SimulationEnvironment(missing, seed=1)
    assert getattr(first.controller.params, 'acquisition_pair_safety', None) is False
    first.controller.trace_enabled = second.controller.trace_enabled = True
    for _ in range(6):
        first.step(); second.step()
        np.testing.assert_array_equal([a.position for a in first.agents], [a.position for a in second.agents])
        np.testing.assert_array_equal(first.cargoes[0].center, second.cargoes[0].center)
        assert first.controller.safety.stats.as_dict() == second.controller.safety.stats.as_dict()
        assert [vars(d) for d in first.controller.diagnostics] == [vars(d) for d in second.controller.diagnostics]
        assert all(not hasattr(d, 'pair_safety') for d in first.controller.diagnostics)


def test_controller_routes_only_current_local_supported_offers(monkeypatch):
    m = module()
    env = SimulationEnvironment(config(True), seed=1)
    env.agents = env.agents[:3]
    for agent, point in zip(env.agents, ([4.3, 3.0], [4.0, 3.0], [.5, .5])):
        agent.position = np.array(point)
    captured = []
    original = m.exchange
    def observe(offers):
        captured.append(offers)
        return original(offers)
    monkeypatch.setattr(m, 'exchange', observe)
    env.controller.trace_enabled = True
    env.controller.step(env.agents, env.cargoes, 0., env.dt)
    assert len(captured) == 1 and captured[0]
    allowed = {a.agent_id for a in env.agents[:2]}
    assert set(captured[0]) <= allowed
    for offer in captured[0].values():
        assert {aid for aid, _ in offer.neighbors} <= allowed
        assert offer.timestamp == 0. and offer.dt == env.dt
    assert env.controller.safety.stats.solves == len(env.agents)
    assert all(hasattr(d, 'pair_safety') for d in env.controller.diagnostics)
    assert env.controller.diagnostics[-1].pair_safety['eligible'] is False


def test_apply_checks_both_pair_commands_before_any_position_changes():
    m = module()
    a, b, pa, pb = pair(domain=(0., 8., 0., 8.))
    accepted, _ = m.exchange({'a': a, 'b': b})
    env = SimulationEnvironment(config(True), seed=1)
    env.controller._pair_commands = accepted
    env.controller._pair_applied = False
    agents = [AgentState('a', np.array(a.position)), AgentState('b', np.array(b.position))]
    commands = [ControlCommand(aid, accepted[aid].for_problem(p)[0]) for aid, p in [('a', pa), ('b', pb)]]
    original_positions = np.array([x.position.copy() for x in agents])
    bad = copy.deepcopy(commands)
    bad[1].velocity *= -1.
    with pytest.raises(ValueError, match='[Pp]air'):
        env.controller.apply_commands(agents, bad, .05)
    np.testing.assert_array_equal([x.position for x in agents], original_positions)
    env.controller.apply_commands(agents, commands, .05)
    assert np.linalg.norm(agents[0].position-agents[1].position) >= .28-1e-7
    with pytest.raises(ValueError, match='[Pp]air'):
        env.controller.apply_commands(agents, commands, .05)


def test_changed_clipping_domain_cannot_execute_an_old_pair_certificate():
    m = module()
    a, b, pa, pb = pair(domain=(0., 8., 0., 8.))
    accepted, _ = m.exchange({'a': a, 'b': b})
    env = SimulationEnvironment(config(True), seed=1)
    env.controller._pair_commands = accepted
    env.controller._pair_applied = False
    env.controller.domain = (0., .3, 0., 8.)
    agents = [AgentState('a', np.array(a.position)), AgentState('b', np.array(b.position))]
    commands = [ControlCommand(aid, accepted[aid].for_problem(p)[0]) for aid, p in [('a', pa), ('b', pb)]]
    with pytest.raises(ValueError, match='[Pp]air'):
        env.controller.apply_commands(agents, commands, .05)
