import numpy as np
from dbact.deployment_variants import push_cone_index


def test_push_cone_ignores_the_leading_face_and_falls_back_when_unseen():
    points = np.array([[0.0, -1.0], [0.0, 1.0]])
    normals = np.array([[0.0, -1.0], [0.0, 1.0]])
    offsets = np.array([0.135, 0.135])
    goal = np.array([1.0, 0.0])
    # Neither normal faces the trailing direction (-goal).
    assert push_cone_index(points, normals, offsets, np.zeros(2), [], goal, threshold=0.35, local_radius=1.0, d_min=0.2) is None

    trailing = np.array([[0.0, 0.0], [1.0, 0.0]])
    trailing_normals = np.array([[-1.0, 0.0], [1.0, 0.0]])
    choice = push_cone_index(
        trailing, trailing_normals, offsets, np.array([-0.2, 0.0]), [], goal,
        threshold=0.35, local_radius=2.0, d_min=0.2,
    )
    assert choice == 0
