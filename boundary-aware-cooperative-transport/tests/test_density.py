import numpy as np

from dbact.boundary_density import BoundaryAwareDensity
from dbact.types import BoundaryObservation


def test_boundary_density_peak_near_target():
    obs = BoundaryObservation("obj", "a0", np.array([0.0, 0.0]), np.array([1.0, 0.0]), 0.0)
    field = BoundaryAwareDensity.from_observations([obs], cage_offset=0.5, sigma=0.2)
    near = field(np.array([0.5, 0.0]))
    far = field(np.array([2.0, 0.0]))
    assert near > far
