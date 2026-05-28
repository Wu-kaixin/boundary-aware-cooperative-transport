import numpy as np

from dbact.cargo import Cargo


def test_rectangle_boundary_samples_and_closest_point():
    cargo = Cargo.rectangle("box", [0, 0], width=2.0, height=1.0)
    pts, normals = cargo.boundary_samples(40)
    assert pts.shape == (40, 2)
    assert normals.shape == (40, 2)
    q, n, d = cargo.closest_boundary(np.array([2.0, 0.0]))
    assert d > 0
    assert np.linalg.norm(n) > 0.9
