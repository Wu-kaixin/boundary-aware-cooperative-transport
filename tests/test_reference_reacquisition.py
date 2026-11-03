"""Missing reference must allow observation recovery without permitting transport."""
from dataclasses import asdict

import numpy as np
import pytest

from dbact.control.phase import Phase
from dbact.types import BoundaryView
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def environment(enabled=True):
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True,
        initial_reference_motion_model='translation', initial_reference_use_all_captured=True,
        initial_reference_map_motion=True, reference_map_reacquisition=enabled)
    env = SimulationEnvironment(cfg, seed=1)
    env.agents = env.agents[:1]
    env.agents[0].position = np.array([.5, .5])
    c = env.controller
    c._ensure_state(env.agents)
    c.initial_reference_status = 'frozen'
    c.initial_reference_epoch = 0.
    c.trace_enabled = True
    return env


@pytest.mark.parametrize('recall', [False, True])
def test_complete_controller_reacquires_missing_reference_with_search_or_recall(recall):
    commands = []
    for enabled in (False, True):
        env = environment(enabled)
        c = env.controller
        if recall:
            c._tokens[env.agents[0].agent_id] = {'cargo_0': (np.array([1., 1.]), 0.)}
        command = c.step(env.agents, env.cargoes, 0., env.dt)[0]
        assert c.diagnostics[0].map_reference_status == 'unavailable'
        assert not c.diagnostics[0].push_side and c.diagnostics[0].effort == 0
        commands.append(command)
    np.testing.assert_array_equal(commands[0].velocity, [0., 0.])
    assert commands[1].mode == 'map_reference_reacquire_' + ('recall' if recall else 'search')
    assert 0 < np.linalg.norm(commands[1].velocity) <= c.params.max_speed


@pytest.mark.parametrize('phase', [Phase.TRANSPORT, Phase.BRAKE, Phase.HOLD])
def test_reacquisition_does_not_bypass_later_phase_abstention(phase):
    env = environment()
    env.controller.phase_monitor.phase = phase
    command = env.controller.step(env.agents, env.cargoes, 0., env.dt)[0]
    assert command.mode == 'map_reference_unavailable'
    np.testing.assert_array_equal(env.controller.diagnostics[0].nominal_command, [0., 0.])


@pytest.mark.parametrize('mode,pushing,effort,allowed', [
    ('approach', False, 0., True),
    ('redeploy', False, 0., False),
    ('cage', False, 0., False),
    ('push', True, .1, False),
    ('search', True, 0., False),
    ('recall', False, .1, False),
    ('unknown', False, 0., False),
])
def test_complete_controller_only_releases_nonpressing_fresh_acquisition_modes(monkeypatch, mode, pushing, effort, allowed):
    env = environment()
    # Supply a nominal branch result; reference/map update, override and QP remain real.
    monkeypatch.setattr(env.controller, '_nominal_command',
        lambda *args: (np.array([2., 0.]), mode, 1., pushing, effort))
    command = env.controller.step(env.agents, env.cargoes, 0., env.dt)[0]
    assert command.mode == ('map_reference_reacquire_'+mode if allowed else 'map_reference_unavailable')
    assert (np.linalg.norm(command.velocity) > 0) == allowed
    assert np.linalg.norm(command.velocity) <= env.controller.params.max_speed + 1e-12
    assert not env.controller.diagnostics[0].push_side and env.controller.diagnostics[0].effort == 0


def test_failed_initial_capture_still_abstains():
    env = environment()
    env.controller.initial_reference_status = 'unavailable'
    command = env.controller.step(env.agents, env.cargoes, 0., env.dt)[0]
    assert command.mode == 'map_reference_unavailable'
    np.testing.assert_array_equal(env.controller.diagnostics[0].nominal_command, [0., 0.])


def test_reacquisition_keeps_missing_motion_bounds_and_all_existing_parameters():
    baseline, candidate = environment(False), environment(True)
    values = asdict(candidate.controller.params)
    values['reference_map_reacquisition'] = False
    assert values == asdict(baseline.controller.params)
    assert baseline.contact_params == candidate.contact_params
    assert baseline.controller.phase_monitor.gates == candidate.controller.phase_monitor.gates
    default = SimulationEnvironment(load_yaml('configs/research/best_known_transport_v1.yaml'), seed=1)
    assert default.controller.params.reference_map_reacquisition is False
    c = candidate.controller
    aid = candidate.agents[0].agent_id
    c._map_reference_state[aid] = dict(status='unavailable', voters=[])
    view = BoundaryView(np.array([[.5, .3]]), np.array([[0., 1.]]), np.ones(1), np.ones(1), np.full(1, 'cargo_0'))
    _, normal, _, velocities = c._object_rows_from_map(aid, candidate.agents[0].position, view)
    np.testing.assert_array_equal(velocities, c.params.max_object_speed * normal)


@pytest.mark.parametrize('updates', [
    {'reference_map_reacquisition': 'false'},
    {'reference_map_reacquisition': True, 'initial_reference_map_motion': False},
])
def test_invalid_reacquisition_contract_is_rejected(updates):
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True,
        initial_reference_motion_model='translation', initial_reference_use_all_captured=True,
        initial_reference_map_motion=True)
    cfg['controller'].update(updates)
    with pytest.raises(ValueError, match='Reference reacquisition'):
        SimulationEnvironment(cfg, seed=1)
