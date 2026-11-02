"""A map must transport retained observations in the task reference frame."""
from dataclasses import asdict
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.sensing.boundary_map import LocalBoundaryMap
from dbact.types import BoundaryView
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def scan(points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    normals = np.tile([0., -1.], (len(points), 1))
    return BoundaryView(points, normals, np.ones(len(points)), np.full(len(points), .02),
                        np.full(len(points), 'cargo_0'))


def environment(enabled=True):
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True,
        initial_reference_motion_model='translation', initial_reference_use_all_captured=True,
        initial_reference_map_motion=enabled)
    return SimulationEnvironment(cfg, seed=20261103)


def test_retained_point_uses_absolute_reference_and_preserves_age_and_normal():
    local = LocalBoundaryMap()
    point = np.array([2.3571386801443044, 4.699607958051123])
    local.update(scan([point]), 25.2)
    local.align_translation('cargo_0', np.zeros(2), np.zeros(2))
    shift = np.array([1.147065975710992, .1955506323820967])
    local.align_translation('cargo_0', shift, np.array([.02, .01]))
    np.testing.assert_allclose(local.view().points, [point + shift], rtol=0, atol=1e-12)
    np.testing.assert_array_equal(local._timestamp, [25.2])
    np.testing.assert_array_equal(local.view().normals, [[0., -1.]])
    local.align_translation('cargo_0', shift, np.array([.03, .01]))
    np.testing.assert_allclose(local.view().points, [point + shift], rtol=0, atol=1e-12)
    np.testing.assert_array_equal(local.object_velocity('cargo_0'), [.03, .01])


def test_registration_and_fusion_cannot_add_a_second_reference_motion():
    local = LocalBoundaryMap()
    points = np.column_stack([np.linspace(0., .5, 10), np.zeros(10)])
    local.update(scan(points), 0.)
    local.align_translation('cargo_0', np.array([.1, .2]), np.array([.2, .1]))
    before = local.view().points.copy()
    registration = local.register(scan(points + [.1, .22]), .05, apply_motion=False)
    assert registration['cargo_0'].matches > 0
    np.testing.assert_array_equal(local.view().points, before)
    local.update(scan(points + [.1, .22]), .05, dt=.05, integrate_motion=False)
    np.testing.assert_array_equal(local.object_displacement('cargo_0'), [.1, .2])
    np.testing.assert_array_equal(local.object_velocity('cargo_0'), [.2, .1])
    assert not local._step_motion


def test_nonfinite_reference_rejected_without_changing_map():
    local = LocalBoundaryMap()
    local.update(scan([[1., 2.]]), 0.)
    before = local.view().points.copy()
    with pytest.raises(ValueError, match='Finite'):
        local.align_translation('cargo_0', np.array([np.nan, 0.]), np.zeros(2))
    np.testing.assert_array_equal(local.view().points, before)


def test_candidate_does_not_change_any_existing_parameter_and_is_default_off():
    baseline, candidate = environment(False), environment(True)
    values = asdict(candidate.controller.params)
    values['initial_reference_map_motion'] = False
    assert values == asdict(baseline.controller.params)
    assert baseline.contact_params == candidate.contact_params
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    assert SimulationEnvironment(cfg, seed=1).controller.params.initial_reference_map_motion is False


@pytest.mark.parametrize('change', [
    {'initial_reference_map_motion': 'false'},
    {'initial_reference_motion_model': 'se2'},
    {'initial_task_reference': False, 'initial_reference_motion_model': 'se2', 'initial_reference_use_all_captured': False},
    {'perception_every': 2},
])
def test_invalid_reference_map_contract_rejected(change):
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True,
        initial_reference_motion_model='translation', initial_reference_use_all_captured=True,
        initial_reference_map_motion=True)
    cfg['controller'].update(change)
    with pytest.raises(ValueError, match='Reference map|every-frame'):
        SimulationEnvironment(cfg, seed=1)


def test_one_hop_reference_dropout_and_recovery_preserves_anchor(monkeypatch):
    from dbact.control.reference_map import update_reference_maps
    env = environment()
    c = env.controller
    agents = env.agents[:3]
    c._ensure_state(agents)
    c.initial_reference_status = 'frozen'
    aid, peer, outsider = [a.agent_id for a in agents]
    own = scan([[.1, 0.], [.2, 0.], [.3, 0.], [.4, 0.]])
    readings = {peer: SimpleNamespace(displacement=np.array([.2, .1]), velocity=np.array([.03, .02])),
                outsider: SimpleNamespace(displacement=np.array([80., 90.]), velocity=np.array([1., 1.]))}
    monkeypatch.setattr(c, '_update_reference', lambda *args: setattr(c, '_reference_readings', dict(readings)))
    neighbors = [[1], [0], []]
    update_reference_maps(c, agents, neighbors, [own, BoundaryView.empty(), BoundaryView.empty()], 1., .05)
    assert c._map_reference_state[aid]['voters'] == [peer]
    local = c.maps[aid]
    original = local.view().points.copy()
    readings.clear()
    update_reference_maps(c, agents, neighbors, [scan([[5., 5.]]), BoundaryView.empty(), BoundaryView.empty()], 1.05, .05)
    assert c._map_reference_state[aid]['status'] == 'unavailable'
    np.testing.assert_array_equal(local.view().points, original)
    np.testing.assert_array_equal(c._views[aid].points, [[5., 5.]])
    _, normals, _, velocities = c._object_rows_from_map(aid, np.array([5., 4.]), c._views[aid])
    np.testing.assert_allclose(velocities, c.params.max_object_speed * normals)
    readings[peer] = SimpleNamespace(displacement=np.array([.4, .2]), velocity=np.array([.03, .02]))
    update_reference_maps(c, agents, neighbors, [BoundaryView.empty()] * 3, 1.1, .05)
    np.testing.assert_allclose(local.view().points, original + [.2, .1], atol=1e-12)
    np.testing.assert_array_equal(local.object_velocity('cargo_0'), [.03, .02])


def test_stationary_acquisition_and_freeze_frame_use_zero_epoch(monkeypatch):
    from dbact.control.reference_map import update_reference_maps
    env = environment()
    c = env.controller
    agents = env.agents[:1]
    c._ensure_state(agents)
    c.initial_reference_status = 'acquiring'
    monkeypatch.setattr(c, '_update_reference', lambda *args: setattr(c, 'initial_reference_status', 'frozen'))
    update_reference_maps(c, agents, [[]], [scan([[.1, 0.]])], 1., .05)
    assert c._map_reference_state[agents[0].agent_id]['status'] == 'stationary_acquisition'
    np.testing.assert_array_equal(c.maps[agents[0].agent_id].object_displacement('cargo_0'), [0., 0.])


def test_missing_reference_inhibits_nominal_motion_in_the_complete_controller():
    env = environment()
    c = env.controller
    c.initial_reference_status = 'frozen'
    c.trace_enabled = True
    commands = c.step(env.agents, env.cargoes, 0., env.dt)
    assert all(command.mode == 'map_reference_unavailable' for command in commands)
    assert all(d.map_reference_status == 'unavailable' for d in c.diagnostics)
    np.testing.assert_array_equal([d.nominal_command for d in c.diagnostics], np.zeros((len(env.agents), 2)))
