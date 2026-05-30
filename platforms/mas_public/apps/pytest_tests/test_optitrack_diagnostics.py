from src.optitrack.natnet_adapter import NatNetRigidBody
from src.optitrack.optitrack_module import OptiTrackModule
from src.optitrack.rigid_body_mapper import RigidBodyMapper
from src.common.messages import SystemCommand
from src.messaging.topics import OPTITRACK_COMMAND


class CapturingLogger:
    def __init__(self):
        self.info_calls = []

    def info(self, *args):
        self.info_calls.append(args)


def test_optitrack_logs_mapped_and_unmapped_rigid_body_ids():
    module = OptiTrackModule.__new__(OptiTrackModule)
    module.log_rigid_bodies = True
    module.rigid_body_log_interval_s = 0.0
    module.last_rigid_body_log_s = 0.0
    module.logger = CapturingLogger()
    module.mapper = RigidBodyMapper(
        {
            "robots": {
                "list": [
                    {"robot_id": "robot_1", "rigid_body_name": "Rigid Body 001", "rigid_body_id": 1},
                ]
            }
        }
    )
    bodies = [
        NatNetRigidBody("1", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), True, 1.0, rigid_body_id=1),
        NatNetRigidBody("5", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), True, 1.0, rigid_body_id=5),
    ]

    module._log_rigid_body_diagnostics(bodies)

    args = module.logger.info_calls[0]
    assert "NatNet rigid bodies" in args[0]
    assert args[3] == {"1": "robot_1"}
    assert args[4] == [{"id": 5, "name": "5", "tracked": True}]


class FakeCommandServer:
    def __init__(self, messages):
        self.messages = list(messages)

    def receive_command(self, timeout_ms=0):
        if not self.messages:
            return None
        return self.messages.pop(0)


def test_set_frequency_updates_publish_period():
    module = OptiTrackModule.__new__(OptiTrackModule)
    module.system_config = {"frequency": {"optitrack_publish_hz": 100}}
    module.publish_period_s = module._publish_period_s()
    module.command_server = FakeCommandServer(
        [
            (
                OPTITRACK_COMMAND,
                SystemCommand("set_frequency", "optitrack", {"frequency_hz": 50}, 1.0).__dict__,
            )
        ]
    )
    module.logger = CapturingLogger()

    module._handle_commands()

    assert module.system_config["frequency"]["optitrack_publish_hz"] == 50.0
    assert module.publish_period_s == 0.02

