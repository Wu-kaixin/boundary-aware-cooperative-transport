import numpy as np
from dbact.local_boundary_features import gap_guard_report, gap_guarded_samples


def test_gap_guard_pulls_a_sample_toward_the_sensor():
    origin = np.zeros(2)
    points = np.array([[1.0, 0.0], [1.0, 0.04]])
    normals = np.array([[-1.0, 0.0], [-1.0, 0.0]])
    guarded, out_normals = gap_guarded_samples(origin, points, normals)
    assert guarded.shape == points.shape
    assert np.linalg.norm(guarded[0]) < np.linalg.norm(points[0])
    assert np.allclose(out_normals, normals)


def test_gap_above_40mm_is_truncated_and_not_a_completed_guard():
    origin = np.zeros(2)
    points = np.array([[1.0, 0.0], [1.0, 0.10]])
    normals = np.array([[-1.0, 0.0], [-1.0, 0.0]])
    guarded, _normals, report = gap_guard_report(origin, points, normals)
    assert report["status"] == "CAP_TRUNCATED"
    assert report["truncated_samples"] == 2
    assert report["max_half_gap_m"] == 0.05
    assert abs(float(np.linalg.norm(points[0] - guarded[0])) - 0.04) < 1e-12
