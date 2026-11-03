"""Review witnesses: structural allowance must precede feature exclusion."""
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact.sensing.finite_support import scan_support, support_rows


def packet(points, noise=0., objects=None):
    return scan_support([
        dict(point=p, ray_index=ray, object_id='cargo' if objects is None else objects[i])
        for i, (p, ray) in enumerate(zip(points, [0, 1, 5, 7]))
    ], [0, 0], 0., noise)


def params():
    return SimpleNamespace(r_safe=.195, max_speed=.35, dt=.05, object_row_range=.6)


def test_structural_allowance_keeps_stricter_isolated_guard():
    scan = packet([[-.001, -.16], [.001, -.16], [.197, 0], [.276, 0]])
    rows = support_rows([0, 0], [scan], params(), 0., 0., isolated_point_guards=True)
    assert rows.ready and rows.point_guard_rows == 1
    np.testing.assert_allclose(rows.A, [[0, 1], [-1, 0]], atol=1e-12)
    np.testing.assert_allclose(rows.h, [-.036, -.0375], atol=1e-12)
    np.testing.assert_allclose(rows.lower_distance, [.159, .1575], atol=1e-12)


def test_guarded_cover_prevents_optimal_qp_violating_measured_point():
    scan = packet([[-.001, -.163], [.001, -.163], [.260, 0], [.339, 0]], noise=.01)
    safety = SafetyFilter(SafetyFilterParams(r_safe=.195, gamma_obj=8., rho=.02,
        max_speed=.35, aggregate_frame_consistent=True))
    trace = {}
    result = safety.filter_velocity([0, 0], [.2, .285], finite_support_packets=[scan],
        finite_support_motion_bound=0., finite_support_trace=trace)
    # Hand-derived point row: h=.260-.0395-.195=.0255, -u_x >= .02-8h=-.184.
    assert -result.velocity[0] >= -.184-1e-10
    assert result.status == 'optimal' and result.barrier_scale == 1.
    assert trace['applied'] and trace['point_guard_rows'] == 1
    np.testing.assert_allclose(result.velocity, [.184, .285], atol=1e-10)


@pytest.mark.parametrize('points,objects,reason', [
    ([[-.001, -.16], [.001, -.16], [.197, 0], [.278, 0]], None, 'isolated_gap_exceeds_cap'),
    ([[-.001, -.16], [.001, -.16], [.197, 0]], ['cargo', 'cargo', 'other'], 'isolated_gap_unknown'),
])
def test_unsupported_potential_competitor_keeps_original_filter(points, objects, reason):
    scan = packet(points, objects=objects)
    rows = support_rows([0, 0], [scan], params(), 0., 0., isolated_point_guards=True)
    assert not rows.ready and rows.reason == reason
    inputs = dict(boundary_points=np.array([[-.02, -.16], [.02, -.16]]),
                  boundary_normals=np.array([[0, 1], [0, 1]]))
    par = SafetyFilterParams(r_safe=.195, gamma_obj=8., rho=.02, max_speed=.35,
                            aggregate_frame_consistent=True)
    original = SafetyFilter(par).filter_velocity([0, 0], [0, .2], **inputs)
    trace = {}
    candidate = SafetyFilter(par).filter_velocity([0, 0], [0, .2], **inputs,
        finite_support_packets=[scan], finite_support_motion_bound=0., finite_support_trace=trace)
    assert not trace['applied'] and trace['reason'] == reason
    for field in ('A_rows', 'b_original', 'b_effective', 'h_object', 'velocity'):
        np.testing.assert_array_equal(getattr(candidate, field), getattr(original, field))
    assert candidate.status == original.status


def test_distant_noncompeting_valid_guard_does_not_add_constraints():
    scan = packet([[-.001, -.16], [.001, -.16], [1., 0], [1.02, 0]])
    rows = support_rows([0, 0], [scan], params(), 0., 0., isolated_point_guards=True)
    assert rows.ready and rows.point_guard_rows == 0
    np.testing.assert_allclose(rows.A, [[0, 1]], atol=1e-12)
    np.testing.assert_allclose(rows.h, [-.036], atol=1e-12)
