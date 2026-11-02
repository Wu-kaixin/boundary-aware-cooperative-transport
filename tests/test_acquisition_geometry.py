"""Stationary acquisition targets must agree with its enlarged safety radius."""
from dataclasses import asdict, replace

import numpy as np
import pytest

from dbact.control.phase import Phase
from dbact.types import BoundaryView
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def environment(enabled=False, uniform=False):
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True,
        initial_reference_motion_model='translation', initial_reference_use_all_captured=True,
        initial_reference_safe_geometry=enabled, enclose_uniform_ring=uniform)
    return SimulationEnvironment(cfg, seed=20261102)


@pytest.mark.parametrize('uniform', [False, True])
def test_acquisition_targets_outside_active_safety_band_and_restore_after_freeze(uniform):
    env = environment(True, uniform)
    c = env.controller
    original = asdict(c.params)
    c._ensure_state(env.agents)
    c.phase_monitor.phase = Phase.ENCLOSE
    ids = [a.agent_id for a in env.agents]
    c._update_reference(env.agents, {aid: BoundaryView.empty() for aid in ids}, .05)
    normals = np.array([[1., 0.], [0., 1.], [-1., 0.], [0., -1.]])
    scan = BoundaryView(normals.copy(), normals, np.ones(4), np.ones(4), np.full(4, 'cargo_0'))
    shape = c._enclosure_geometry(ids[0], scan)
    distances = np.sum((c._cage_targets(scan, np.array([1., 0.]), shape)-scan.points)*normals, axis=1)
    assert c.safety.params.r_safe == pytest.approx(.195)
    assert np.min(distances) >= c.safety.params.r_safe+c.feasibility.press_margin-1e-12
    assert shape.lead_offset is None if uniform else shape.lead_offset == c.density_params.lead_offset
    assert asdict(c.params) == original  # Derived geometry does not retune parameters.
    assert c.initial_reference_epoch is None
    c.initial_reference_status = 'frozen'
    c.safety.params.r_safe = c._transport_safe_radius
    restored = c._enclosure_geometry(ids[0], scan)
    expected = replace(c.density_params, lead_offset=None) if uniform else c.density_params
    assert restored == expected


def test_disabled_geometry_preserves_old_targets_and_larger_offsets_are_not_shrunk():
    env = environment()
    c = env.controller
    c.phase_monitor.phase = Phase.ENCLOSE
    c._update_initial_reference(env.agents, {}, .05)
    assert c._enclosure_geometry(env.agents[0].agent_id, BoundaryView.empty()) == c.density_params
    c.params.initial_reference_safe_geometry = True
    c.density_params = replace(c.density_params, cage_offset=.4, lead_offset=.5)
    assert c._enclosure_geometry(env.agents[0].agent_id, BoundaryView.empty()) == c.density_params


@pytest.mark.parametrize('updates, message', [
    ({'initial_reference_safe_geometry': 'false'}, 'Acquisition geometry'),
    ({'initial_reference_safe_geometry': True, 'initial_reference_acquisition': False, 'initial_task_reference': False}, 'Acquisition geometry'),
    ({'initial_reference_safe_geometry': True, 'task_mode': 'deployment'}, 'requires.*transport'),
])
def test_invalid_geometry_selector_rejected(updates, message):
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True, initial_task_reference=True)
    cfg['controller'].update(updates)
    with pytest.raises(ValueError, match=message):
        SimulationEnvironment(cfg, seed=20261102)
