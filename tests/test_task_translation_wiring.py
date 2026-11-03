"""Opt-in wiring contracts; physical candidate success is tested separately."""
from dataclasses import asdict
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from dbact.control.boundary_navigation import point_goal_feedback
from dbact.control.phase import Phase
from dbact.types import BoundaryView
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def config():
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True)
    return cfg


def rectangle():
    u = np.linspace(-.47, .47, 40)
    points = np.vstack([np.c_[u, np.full(40, -.5)], np.c_[np.full(40, .5), u],
                       np.c_[u, np.full(40, .5)], np.c_[np.full(40, -.5), u]])+[3., 5.]
    normals = np.repeat([[0., -1.], [1., 0.], [0., 1.], [-1., 0.]], 40, axis=0)
    return BoundaryView(points, normals, np.ones(160), np.full(160, .025), np.full(160, 'cargo_0'))


def shifted(scan, delta):
    return BoundaryView(scan.points+delta, scan.normals, scan.confidence, scan.arc_length, scan.object_ids)


def test_reference_selectors_default_off_preserve_all_protected_numerical_values():
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    env = SimulationEnvironment(cfg, seed=20261102)
    values = asdict(env.controller.params)
    assert values.pop('initial_reference_motion_model') == 'se2'
    assert values.pop('initial_reference_use_all_captured') is False
    assert values.pop('initial_reference_safe_geometry') is False
    assert values.pop('initial_reference_map_motion') is False
    assert values.pop('initial_acquisition_circulation') is False
    assert values.pop('reference_map_reacquisition') is False
    assert values.pop('acquisition_finite_support') is False
    assert values.pop('acquisition_pair_safety') is False
    assert values.pop('reference_surface_refinement') is False
    receipt = json.loads(Path('docs/results/reference_surface_v1/protected_baseline.json').read_text(encoding='utf-8'))
    assert values == receipt['effective_controller']


@pytest.mark.parametrize('updates, message', [
    ({'initial_reference_motion_model': 'unknown'}, 'Initial reference motion model'),
    ({'initial_reference_use_all_captured': 'false'}, 'Initial reference template admission'),
    ({'initial_task_reference': False, 'initial_reference_use_all_captured': True}, 'Initial reference options require'),
    ({'initial_task_reference': False, 'initial_reference_motion_model': 'translation'}, 'Initial reference options require'),
])
def test_invalid_reference_selector_combinations_fail_explicitly(updates, message):
    cfg = config()
    cfg['controller'].update(updates)
    with pytest.raises(ValueError, match=message):
        SimulationEnvironment(cfg, seed=20261102)


@pytest.mark.parametrize('all_captured, model', [(False, 'se2'), (True, 'translation')])
@pytest.mark.parametrize('safe_geometry', [False, True])
def test_original_freeze_is_preserved_and_late_data_cannot_create_a_new_zero(all_captured, model, safe_geometry):
    cfg = config()
    cfg['controller'].update(initial_reference_motion_model=model,
                             initial_reference_safe_geometry=safe_geometry,
                             initial_reference_use_all_captured=all_captured)
    env = SimulationEnvironment(cfg, seed=20261102)
    c = env.controller
    c._ensure_state(env.agents)
    ids = [a.agent_id for a in env.agents]
    scan = rectangle()
    # Ten points on two faces can bootstrap translation, but cannot meet the
    # original 32-point acquisition-support criterion. Remaining agents saw none.
    weak = scan.select(np.r_[np.arange(0, 40, 8), np.arange(40, 80, 8)])
    batches = {aid: scan if i < 4 else weak if i == 4 else BoundaryView.empty()
               for i, aid in enumerate(ids)}
    c._update_reference(env.agents, batches, .1)
    assert c.initial_reference_epoch == .1
    assert c.initial_reference_supporters == ids[:4]
    assert c._initial_capture.frozen
    assert c._reference_readings == {}  # Initialization alone is never a vote.
    initialized = {aid for aid, tracker in c._reference_trackers.items() if tracker.initialized}
    assert initialized == set(ids[:5] if all_captured else ids[:4])
    records = copy.deepcopy({aid: t.initialization_record() for aid, t in c._reference_trackers.items()})
    if all_captured:
        assert all(r['rotation_model'] == 'translation_hypothesis_not_measured_zero_yaw'
                   and r['registration_certified'] is False for r in records.values())
    delta = np.array([.004, 0.])
    moved = {aid: shifted(batches[aid], delta) if i < 5 else shifted(scan, delta)
             for i, aid in enumerate(ids)}
    c._update_reference(env.agents, moved, .15)
    assert set(c._reference_readings) == initialized
    for reading in c._reference_readings.values():
        np.testing.assert_allclose(reading.displacement, delta, atol=1e-9)
    assert {aid: t.initialization_record() for aid, t in c._reference_trackers.items()} == records
    assert c.initial_reference_epoch == .1 and c.initial_reference_supporters == ids[:4]
    # A subsequent no-return frame revokes every fresh navigation/stop vote.
    c._update_reference(env.agents, {aid: BoundaryView.empty() for aid in ids}, .2)
    c._update_navigation(env.agents, [[] for _ in ids])
    assert c._reference_readings == {} and c.team_navigation is None
    assert c.task_reference_hold_ready is False


def test_vector_goal_can_request_more_motion_in_brake_before_common_hold():
    env = SimulationEnvironment(config(), seed=20261102)
    c = env.controller
    task = next(iter(c.tasks.values()))
    target = task.distance*task.direction
    c.phase_monitor.phase = Phase.BRAKE
    c.team_navigation = point_goal_feedback({'a': target-.044*task.direction}, {'a': np.zeros(2)},
        ['a'], target, c.params.cross_track_deadband, rotation_invariant=True)
    assert c.team_navigation.remaining > 0 and c._should_retract() is False
    c.team_navigation = point_goal_feedback({'a': target}, {'a': np.zeros(2)},
        ['a'], target, c.params.cross_track_deadband, rotation_invariant=True)
    assert c._should_retract() is True
    c.phase_monitor.phase = Phase.HOLD
    assert c._should_retract() is True
