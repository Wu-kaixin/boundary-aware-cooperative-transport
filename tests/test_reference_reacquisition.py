"""Missing reference must allow observation recovery without permitting transport."""
from dataclasses import asdict
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.control.phase import Phase, PhaseSignals
from dbact.deployment.cvt import CVTResult
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


def packet(points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return BoundaryView(points, np.tile([0., -1.], (len(points), 1)),
                        np.ones(len(points)), np.full(len(points), .02),
                        np.full(len(points), 'cargo_0'))


def test_fresh_reference_is_identical_with_reacquisition_enabled_or_disabled(monkeypatch):
    results = []
    for enabled in (False, True):
        env = environment(enabled)
        c, aid = env.controller, env.agents[0].agent_id
        fresh = packet([[1.5, 1.5], [1.7, 1.5]])
        reading = SimpleNamespace(displacement=np.array([.2, .1]),
                                  velocity=np.array([.01, 0.]), angle=0.,
                                  matches=10, residual_rms=0.)
        # Inject sensor/reference measurements, retaining real map, nominal,
        # supervisor, permission override and QP execution.
        monkeypatch.setattr(c.sensor, 'sense_view', lambda *args: fresh)
        monkeypatch.setattr(c, '_update_reference',
                            lambda *args, c=c, aid=aid, r=reading:
                            setattr(c, '_reference_readings', {aid: r}))
        command = c.step(env.agents, env.cargoes, 0., env.dt)[0]
        assert c._map_reference_state[aid]['status'] == 'fresh_reference'
        assert not command.mode.startswith('map_reference_')
        results.append((command, asdict(c.diagnostics[0]), c._views[aid].points,
                        c.phase_monitor.as_dict(), c.safety.stats.as_dict()))
    assert results[0][0].mode == results[1][0].mode
    np.testing.assert_array_equal(results[0][0].velocity, results[1][0].velocity)
    for baseline, candidate in zip(results[0][1:], results[1][1:]):
        np.testing.assert_equal(baseline, candidate)


@pytest.mark.parametrize('occupied', [False, True])
def test_real_approach_uses_current_packet_despite_stale_targets(monkeypatch, occupied):
    env = environment()
    c, agent = env.controller, env.agents[0]
    aid = agent.agent_id
    fresh = packet([[1.5, 1.5], [1.7, 1.5]])
    targets = fresh.points + c.density_params.cage_offset * fresh.normals
    if occupied:
        # Both ring targets are within the original gap radius of this robot.
        agent.position = targets[0].copy()
        assert np.all(np.linalg.norm(targets-agent.position, axis=1) <= c.params.gap_radius)
        expected_target = fresh.points.mean(axis=0)
    else:
        assert np.all(np.linalg.norm(targets-agent.position, axis=1) > c.params.gap_radius)
        expected_target = targets[0]
    stale = packet([[6., 6.]])
    c.maps[aid].update(stale, 0.)
    c.maps[aid].align_translation('cargo_0', np.zeros(2), np.zeros(2))
    c._views[aid] = stale
    c._redeploy_target[aid] = np.array([7., .5])
    c._held_nominal[aid] = (np.array([-3., -2.]), 'approach', 1., True, .5)
    monkeypatch.setattr(c.sensor, 'sense_view', lambda *args: fresh)
    # Supply an empty CVT cell to exercise the actual approach branch, including
    # its occupied-target fallback; neither nominal generation nor QP is mocked.
    monkeypatch.setattr(c.cvt, 'compute',
                        lambda *args: CVTResult(agent.position.copy(), 0., 0, 0))
    command = c.step(env.agents, env.cargoes, 0., env.dt)[0]
    d = c.diagnostics[0]
    assert d.map_reference_status == 'unavailable'
    assert command.mode == 'map_reference_reacquire_approach'
    np.testing.assert_array_equal(c._views[aid].points, fresh.points)
    np.testing.assert_array_equal(c.maps[aid].view().points, stale.points)
    np.testing.assert_allclose(d.nominal_command,
                               c.params.kp_cage * (expected_target-agent.position), rtol=0, atol=1e-12)
    assert not d.push_side and d.effort == 0
    assert np.linalg.norm(command.velocity) <= c.params.max_speed + 1e-12


@pytest.mark.parametrize('destination', [Phase.TRANSPORT, Phase.BRAKE, Phase.HOLD])
def test_recovery_stops_on_the_controller_step_entering_a_later_phase(monkeypatch, destination):
    env = environment()
    c = env.controller
    c.phase_monitor.phase = Phase.CONTACT_READY
    signals = PhaseSignals(agent_count=16, target_distance=1., estimate_quality='credible')
    # Control the measured supervisor input; execute the real transition/dwell
    # machine, nominal search, permission check and QP in their original order.
    monkeypatch.setattr(c, '_phase_signals', lambda *args: signals)
    command = c.step(env.agents, env.cargoes, 0., env.dt)[0]
    assert c.phase == Phase.CONTACT_READY
    assert command.mode == 'map_reference_reacquire_search'
    assert np.linalg.norm(command.velocity) > 0
    if destination == Phase.TRANSPORT:
        signals.transport_active = c.phase_monitor.gates.transport_quorum
    else:
        signals.progress = 1.
        signals.estimated_speed = (0. if destination == Phase.HOLD
                                   else 2*c.phase_monitor.gates.hold_speed)
    for _ in range(c.phase_monitor.gates.hold_dwell + 2):
        entering_frame = c._frame
        command = c.step(env.agents, env.cargoes, c._frame*env.dt, env.dt)[0]
        assert c.phase >= Phase.TRANSPORT
        assert command.mode == 'map_reference_unavailable'
        np.testing.assert_array_equal(c.diagnostics[0].nominal_command, [0., 0.])
        np.testing.assert_array_equal(command.velocity, [0., 0.])
        if c.phase == destination:
            assert c.phase_monitor.entered[destination.label] == entering_frame
            break
    else:
        pytest.fail('The real supervisor did not enter the requested phase')
