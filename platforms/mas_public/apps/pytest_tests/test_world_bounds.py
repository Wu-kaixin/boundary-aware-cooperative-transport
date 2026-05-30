from src.common.messages import RobotState, WorldState
from src.controller.world_bounds import out_of_bounds_robot_ids


WORLD_CONFIG = {
    "x_min": -1.0,
    "x_max": 2.0,
    "y_min": -1.0,
    "y_max": 2.0,
    "z_min": -0.1,
    "z_max": 1.0,
    "stop_on_out_of_bounds": True,
}


def test_out_of_bounds_robot_ids_empty_when_inside():
    state = WorldState(1.0, 1, [RobotState("robot_1", True, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 1.0)])
    assert out_of_bounds_robot_ids(state, WORLD_CONFIG) == []


def test_out_of_bounds_robot_ids_reports_tracked_robot():
    state = WorldState(1.0, 1, [RobotState("robot_1", True, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 1.0)])
    assert out_of_bounds_robot_ids(state, WORLD_CONFIG) == ["robot_1"]


def test_out_of_bounds_robot_ids_ignores_untracked_robot():
    state = WorldState(1.0, 1, [RobotState("robot_1", False, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 1.0)])
    assert out_of_bounds_robot_ids(state, WORLD_CONFIG) == []

