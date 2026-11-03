"""Disentangle finite geometry, normal changes and received-motion demands."""
import copy
import importlib
from pathlib import Path

import numpy as np
import pytest


def load(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root / 'scripts'))
    assert (root / 'scripts/analyze_map_bridge_conflicts.py').exists(), 'Missing geometry and row analysis'
    return importlib.import_module('analyze_map_bridge_conflicts')


def test_finite_edge_projection_distinguishes_line_extension_and_distance(monkeypatch):
    module = load(monkeypatch)
    polygon = np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.]])
    records = module.edge_geometry([1.2, 1.3], polygon)
    top, right = records[2], records[1]
    assert top['projection_t'] == pytest.approx(-.2)
    assert right['projection_t'] == pytest.approx(1.3)
    assert top['signed_plane_distance'] == pytest.approx(.3)
    assert right['signed_plane_distance'] == pytest.approx(.2)
    assert top['finite_distance'] == pytest.approx(np.sqrt(.13))
    assert right['finite_distance'] == pytest.approx(np.sqrt(.13))
    assert not top['projection_inside'] and not right['projection_inside']
    rotation = np.array([[0., -1.], [1., 0.]])
    shifted = module.edge_geometry(np.array([1.2, 1.3]) @ rotation.T + [7., -2.], polygon @ rotation.T + [7., -2.])
    for old, new in zip(records, shifted):
        for key in ('projection_t', 'signed_plane_distance', 'finite_distance', 'endpoint_excess'):
            assert new[key] == pytest.approx(old[key])


def fixture():
    from dataclasses import asdict
    from dbact.safety.filter import SafetyFilterParams
    params = SafetyFilterParams(r_safe=.1, gamma_obj=2., rho=.02,
        max_speed=.35, object_velocity_bound=.6, object_row_range=2.,
        object_row_window=1., aggregate_frame_consistent=True)
    return dict(position=[.15, 0.], A=[[0., -2.], [0., 1.], [-1., 0.]],
        b=[-.4, .21, -.2], kinds=['agent', 'object', 'wall'],
        h=[-.1], normal_motion=[.48], params=asdict(params),
        boundary_points=[[0., -.1], [0., .1]], boundary_normals=[[0., 1.], [0., 1.]],
        object_velocity=[.6, .8], point_velocities=None,
        map_true_feet=[[.01, -.1], [.01, .1]], map_true_normals=[[1., 0.], [1., 0.]],
        true_normal=[1., 0.], true_gap=.15, true_foot=[0., 0.])


def test_substitutions_preserve_hard_rows_and_reproject_same_bounded_motion(monkeypatch):
    module = load(monkeypatch)
    saved = fixture()
    before = copy.deepcopy(saved)
    normals = module.geometry_system(saved, 'normals_only')
    np.testing.assert_array_equal(normals['A'][:2], [[0., -2.], [-1., 0.]])
    np.testing.assert_array_equal(normals['b'][:2], [-.4, -.2])
    assert normals['hard_original_indices'] == [0, 2]
    np.testing.assert_allclose(normals['h'], [.05])
    np.testing.assert_allclose(normals['normal_motion'], [.36])
    np.testing.assert_allclose(normals['b'][-1], .21)
    assert module.systems(normals)['uncapped_full'][-1] == pytest.approx(.28)
    points = module.geometry_system(saved, 'points_only')
    np.testing.assert_allclose(points['A'][-1], [0., 1.])
    np.testing.assert_allclose(points['h'], [-.1])
    both = module.geometry_system(saved, 'points_and_normals')
    np.testing.assert_allclose(both['h'], [.04])
    assert saved == before


def test_true_nearest_row_keeps_received_point_velocity_envelope(monkeypatch):
    module = load(monkeypatch)
    saved = fixture()
    saved['point_velocities'] = [[0., 1.], [.8, 0.]]
    row = module.geometry_system(saved, 'true_nearest')
    np.testing.assert_allclose(row['normal_motion'], [.6])
    np.testing.assert_allclose(row['h'], [.05])
    assert module.systems(row)['uncapped_full'][-1] == pytest.approx(.52)
    np.testing.assert_allclose(row['b'][-1], .21)
