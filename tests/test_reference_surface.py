"""Mechanism checks for the opt-in surface model, not task success evidence."""
from dataclasses import asdict
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from dbact.control.reference_surface import SurfaceReferenceTracker, refine_reference_surface
from dbact.types import BoundaryView
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def view(points, normals):
    return BoundaryView(points=np.asarray(points), normals=np.asarray(normals),
        confidence=np.ones(len(points)), arc_length=np.full(len(points), .01),
        object_ids=np.full(len(points), 'cargo_0', dtype='<U32'))


def rectangle():
    u = np.linspace(-.47, .47, 100)
    return view(np.vstack([np.c_[u, np.full(100, -.5)], np.c_[np.full(100, .5), u],
                          np.c_[u, np.full(100, .5)], np.c_[np.full(100, -.5), u]]),
                np.repeat([[0., -1.], [1., 0.], [0., 1.], [-1., 0.]], 100, axis=0))


def test_fitted_flat_surface_does_not_invent_tangential_observability():
    x = np.linspace(-.5, .5, 100)
    theta = .15 * np.where(np.arange(100) % 2, 1., -1.)
    scan = view(np.c_[x, np.zeros(100)], np.c_[np.sin(theta), np.cos(theta)])
    original = ObservedReferenceTracker()
    candidate = SurfaceReferenceTracker(surface_residual_tolerance=.03)
    for tr in (original, candidate):
        tr.initialize(scan, [.04, -.01], 0.)
        assert tr.initialized  # Same original bootstrap admission and epoch.
    assert original.update(scan, .05) is not None
    assert candidate.update(scan, .05) is None
    assert candidate.reason == 'translation_unobservable'
    assert candidate.surface_diagnostics['refined_points'] == len(scan)
    np.testing.assert_array_equal(candidate.translation, [0., 0.])
    assert candidate.last_time == 0.


def test_exact_motion_immutable_template_and_original_anchor_offset_pivot():
    scan = rectangle()
    scan = view(scan.points + [3., 4.], scan.normals)
    original = ObservedReferenceTracker(radial_readout=True)
    candidate = SurfaceReferenceTracker(surface_residual_tolerance=.03, radial_readout=True)
    for tr in (original, candidate):
        tr.initialize(scan, [.04, -.01], 0.)
    for field in ('anchor', 'offset', 'pivot_shift'):
        np.testing.assert_array_equal(getattr(candidate, field), getattr(original, field))
    assert candidate.pivot_status == original.pivot_status
    frozen = candidate.initialization_record()
    for k in range(1, 21):
        angle, delta = k*.002, np.array([k*.004, k*.001])
        r = rotation(angle)
        current = view((scan.points-[3., 4.])@r.T+[3., 4.]+delta, scan.normals@r.T)
        reading = candidate.update(current, k*.05)
        assert reading is not None
        np.testing.assert_allclose(reading.displacement, [.04, -.01]+delta, atol=2e-5)
        assert reading.angle == pytest.approx(angle, abs=2e-5)
    assert candidate.initialization_record() == frozen
    assert candidate.update(BoundaryView.empty(), 1.05) is None
    assert candidate.last_reading is None
    with pytest.raises(RuntimeError, match='silently replaced'):
        candidate.initialize(scan, [0., 0.], 2.)


def test_corner_with_excessive_line_residual_keeps_original_geometry():
    u = np.linspace(-.115, .115, 40)
    points = np.vstack([[0., 0.], np.c_[u, np.zeros(40)], np.c_[np.zeros(40), u]])
    normals = np.tile([1., 0.], (len(points), 1))
    fitted, fitted_normals, accepted = refine_reference_surface(
        points, normals, gate=.12, normal_cosine=.5, residual_tolerance=.03)
    assert not accepted[0]
    np.testing.assert_array_equal(fitted[0], points[0])
    np.testing.assert_array_equal(fitted_normals[0], normals[0])


def test_sparse_support_is_unchanged_and_input_arrays_are_never_modified():
    scan = rectangle().select(np.arange(0, 400, 10))
    points, normals = scan.points.copy(), scan.normals.copy()
    fitted, fitted_normals, accepted = refine_reference_surface(
        points, normals, gate=.12, normal_cosine=.5, residual_tolerance=.03)
    assert not accepted.any()
    np.testing.assert_array_equal(fitted, points)
    np.testing.assert_array_equal(fitted_normals, normals)
    np.testing.assert_array_equal(scan.points, points)
    np.testing.assert_array_equal(scan.normals, normals)


def test_opt_in_preserves_every_numeric_parameter_physics_and_stop_gate():
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    original = SimulationEnvironment(cfg, seed=20261102)
    variant = copy.deepcopy(cfg)
    variant['controller']['reference_surface_refinement'] = True
    candidate = SimulationEnvironment(variant, seed=20261102)
    a, b = asdict(original.controller.params), asdict(candidate.controller.params)
    assert {k for k in a if a[k] != b[k]} == {'reference_surface_refinement'}
    assert a['max_object_speed'] == .45
    assert candidate.contact_params == original.contact_params
    assert candidate.controller.phase_monitor.gates == original.controller.phase_monitor.gates
    assert candidate.controller.params.contact_contract() == original.controller.params.contact_contract()
    variant['controller']['initial_reference_acquisition'] = True
    with pytest.raises(ValueError, match='Surface refinement'):
        SimulationEnvironment(variant, seed=20261102)
    variant['controller']['initial_reference_acquisition'] = False
    variant['controller']['reference_radial_pivot'] = False
    variant['controller']['navigation_estimator'] = 'accumulated'
    with pytest.raises(ValueError, match='Surface refinement'):
        SimulationEnvironment(variant, seed=20261102)


def test_protected_configuration_and_resolved_values_match_saved_receipt():
    receipt = json.loads(Path('docs/results/reference_surface_v1/protected_baseline.json').read_text(encoding='utf-8'))
    path = Path(receipt['canonical_config'])
    assert hashlib.sha256(path.read_bytes()).hexdigest() == receipt['canonical_sha256']
    resolved = asdict(SimulationEnvironment(load_yaml(path), seed=receipt['canonical_seed']).controller.params)
    assert resolved.pop('reference_surface_refinement') is False
    assert resolved.pop('initial_reference_motion_model') == 'se2'
    assert resolved.pop('initial_reference_use_all_captured') is False
    assert resolved.pop('initial_reference_safe_geometry') is False
    assert resolved.pop('initial_reference_map_motion') is False
    assert resolved.pop('initial_acquisition_circulation') is False
    assert resolved.pop('reference_map_reacquisition') is False
    assert resolved.pop('acquisition_finite_support') is False
    assert resolved.pop('acquisition_pair_safety') is False
    assert resolved == receipt['effective_controller']


@pytest.mark.parametrize('enabled', [False, True])
def test_controller_uses_only_requested_tracker_without_changing_safety_map(enabled):
    from types import SimpleNamespace
    from dbact.control.phase import Phase
    cfg = load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller']['reference_surface_refinement'] = enabled
    env = SimulationEnvironment(cfg, seed=20261102)
    c = env.controller
    c._ensure_state(env.agents)
    c.phase_monitor.phase = Phase.CONTACT_READY
    aid = env.agents[0].agent_id
    oid = next(iter(c.tasks))
    local_map = c.maps[aid]
    local_map.motion_quality[oid] = 'credible'
    local_map.displacement[oid] = np.array([.03, 0.])
    local_map.velocity[oid] = np.array([.1, .2])
    agents, scan = [SimpleNamespace(agent_id=aid)], rectangle()
    c._update_reference(agents, {aid: scan}, 0.)
    assert type(c._reference_trackers[aid]) is (SurfaceReferenceTracker if enabled else ObservedReferenceTracker)
    c._update_reference(agents, {aid: view(scan.points+[.01, 0.], scan.normals)}, .05)
    assert aid in c._reference_readings
    np.testing.assert_array_equal(local_map.displacement[oid], [.03, 0.])
    np.testing.assert_array_equal(local_map.velocity[oid], [.1, .2])


@pytest.mark.parametrize('tolerance', [0., -1., float('nan')])
def test_invalid_residual_tolerance_rejected(tolerance):
    with pytest.raises(ValueError):
        SurfaceReferenceTracker(surface_residual_tolerance=tolerance)
