import numpy as np
from dbact.local_boundary_features import gap_guarded_samples


def test_gap_guard_pulls_a_sample_toward_the_sensor():
    origin = np.zeros(2)
    points = np.array([[1.0, 0.0], [1.0, 0.04]])
    normals = np.array([[-1.0, 0.0], [-1.0, 0.0]])
    guarded, out_normals = gap_guarded_samples(origin, points, normals)
    assert guarded.shape == points.shape
    assert np.linalg.norm(guarded[0]) < np.linalg.norm(points[0])
    assert np.allclose(out_normals, normals)
