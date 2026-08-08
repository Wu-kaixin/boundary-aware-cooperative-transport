from dataclasses import dataclass

import numpy as np

from mas_adapter.decentralized_transport_controller import DecentralizedTransportController


@dataclass
class MockRobotState:
    robot_id: str
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    tracked: bool = True


@dataclass
class MockWorldState:
    timestamp: float
    robots: list[MockRobotState]


def _make_controller() -> DecentralizedTransportController:
    config = {
        "controller": {
            "type": "dtransport",
            "robot_mode": "free",
        },
        "controller_params": {
            "dtransport": {
                "sensor_range": 1.20,
                "comm_range": 2.40,
                "cage_offset": 0.135,
                "robot_radius": 0.16,
                "delta_max": 0.05,
                "gamma_obj": 8.0,
                "rho": 0.05,
                "backend": "qp",
                "local_radius": 0.90,
                "sigma": 0.20,
                "d_min": 0.34,
                "max_speed": 0.30,
                "kp_explore": 0.20,
                "kp_cage": 1.20,
                "kp_transport": 0.0,
                "grid_resolution": 24,
                "age_decay": 0.30,
                "gamma_agent": 6.0,
                "virtual_object": {
                    "enabled": True,
                    "id": "cargo_0",
                    "vertices": [
                        [3.10, 4.55],
                        [4.45, 4.30],
                        [5.10, 4.90],
                        [4.80, 5.75],
                        [3.70, 6.05],
                        [3.05, 5.30],
                    ],
                    "transport_direction": [0.0, 1.0],
                },
            }
        },
    }

    robot_ids = ["agent_00", "agent_01", "agent_02", "agent_03"]

    world_config = {
        "xmin": 0.0,
        "xmax": 8.0,
        "ymin": 0.0,
        "ymax": 8.0,
    }

    limits_config = {
        "chassis": {
            "max_vx": 0.30,
            "max_vy": 0.30,
            "max_wz": 0.60,
        }
    }

    return DecentralizedTransportController(
        config=config,
        robot_ids=robot_ids,
        world_config=world_config,
        limits_config=limits_config,
    )


def test_mock_worldstate_to_planar_velocities():
    controller = _make_controller()

    world_state = MockWorldState(
        timestamp=0.0,
        robots=[
            MockRobotState("agent_00", 3.4, 4.0),
            MockRobotState("agent_01", 4.0, 4.0),
            MockRobotState("agent_02", 4.6, 4.0),
            MockRobotState("agent_03", 5.2, 4.4),
        ],
    )

    velocities = controller.compute_planar_velocities(world_state)

    assert set(velocities.keys()) == {
        "agent_00",
        "agent_01",
        "agent_02",
        "agent_03",
    }

    for velocity in velocities.values():
        assert velocity.shape == (2,)
        assert np.all(np.isfinite(velocity))
        assert np.linalg.norm(velocity) <= 0.31

    assert any(np.linalg.norm(v) > 1e-6 for v in velocities.values())


def test_mock_worldstate_untracked_robot_gets_zero_velocity():
    controller = _make_controller()

    world_state = MockWorldState(
        timestamp=0.0,
        robots=[
            MockRobotState("agent_00", 3.4, 4.0, tracked=True),
            MockRobotState("agent_01", 4.0, 4.0, tracked=False),
            MockRobotState("agent_02", 4.6, 4.0, tracked=True),
            MockRobotState("agent_03", 5.2, 4.4, tracked=True),
        ],
    )

    velocities = controller.compute_planar_velocities(world_state)

    assert np.allclose(velocities["agent_01"], np.zeros(2))