from mas_adapter.decentralized_transport_controller import DecentralizedTransportController


def test_mas_adapter_import_and_init_without_mas_public():
    config = {
        "controller": {
            "type": "dtransport",
            "robot_mode": "free",
        },
        "controller_params": {
            "dtransport": {
                "sensor_range": 1.0,
                "comm_range": 2.0,
                "cage_offset": 0.28,
                "sigma": 0.34,
                "d_min": 0.30,
                "max_speed": 0.30,
                "kp_explore": 0.20,
                "kp_cage": 1.20,
                "kp_transport": 0.0,
                "grid_resolution": 24,
                "map_ttl": 8.0,
                "cbf_gamma": 6.0,
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

    robot_ids = ["agent_00", "agent_01", "agent_02"]

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

    controller = DecentralizedTransportController(
        config=config,
        robot_ids=robot_ids,
        world_config=world_config,
        limits_config=limits_config,
    )

    assert controller.robot_ids == robot_ids
    assert controller.robot_mode == "free"
    assert controller.max_vx == 0.30
    assert controller.max_vy == 0.30
    assert controller.object_observer is not None
    assert controller.dbact is not None