"""Post-acquisition finite rows must keep noise and motion in explicit roles."""
import importlib
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest

from dbact.safety.filter import SafetyFilterParams
from dbact.sensing.finite_support import ScanSupport


def load(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    assert (root/'scripts/probe_map_bridge_finite_support.py').exists(), 'Missing finite observer'
    return importlib.import_module('probe_map_bridge_finite_support')


def state():
    params = SafetyFilterParams(r_safe=.065, max_speed=.35, dt=.05,
        rho=.02, gamma_obj=8., object_velocity_bound=.6, object_row_range=.6)
    packet = ScanSupport(points=np.array([[0., -.01], [0., .01]]),
        object_ids=np.array(['box', 'box']), origin=np.array([.2, 0.]), timestamp=3.,
        noise_allowance=.03, chords=np.array([[0, 1]]), half_gaps=np.array([.01]))
    return dict(agent_id='self', position=[.2, 0.], neighbors=['peer'],
        A=[[0., -1.], [-1., 0.]], b=[-.1, -.2], kinds=['agent', 'wall'],
        params=asdict(params), object_velocity=[.03, .04], point_velocities=None,
        permitted_packets={'self': asdict(packet)})


def test_noise_is_subtracted_once_and_speed_changes_rhs_not_cover(monkeypatch):
    observer = load(monkeypatch)
    full = observer.observed_system(state(), 3., 'full_bound')
    received = observer.observed_system(state(), 3., 'received_speed')
    assert full['ready'] and received['ready']
    # .20 distance - .01 half-gap - .03 noise - .065 protected radius.
    np.testing.assert_allclose(full['h'], [.095])
    assert received['cover_motion_bound'] == full['cover_motion_bound'] == .6
    np.testing.assert_allclose(full['A'], received['A'])
    np.testing.assert_allclose(full['normal_motion'], [.6])
    np.testing.assert_allclose(received['normal_motion'], [.05])
    np.testing.assert_allclose(received['b'][:2], [-.1, -.2])
    assert observer.systems(received)['uncapped_full'][-1] == pytest.approx(-.69)
    assert observer.systems(full)['uncapped_full'][-1] == pytest.approx(-.14)


def test_stale_own_packet_does_not_become_empty_success(monkeypatch):
    observer = load(monkeypatch)
    saved = state()
    saved['permitted_packets']['peer'] = dict(saved['permitted_packets']['self'], timestamp=4.)
    result = observer.observed_system(saved, 4., 'received_speed')
    assert not result['ready'] and result['reason'] == 'no_current_own_returns'
    assert 'A' not in result


def test_isotropic_received_point_speed_keeps_original_cap(monkeypatch):
    observer = load(monkeypatch)
    saved = state()
    saved['point_velocities'] = [[0., .2], [0., 1.2]]
    result = observer.observed_system(saved, 3., 'received_speed')
    np.testing.assert_allclose(result['normal_motion'], [.6])
    assert result['cover_motion_bound'] == .6
