import threading
from collections import deque

from src.common.messages import RobotCommand
from src.robot.robomaster_adapter import RoboMasterAdapter
from src.robot.robot_registry import RobotInfo


class FakeChassis:
    def __init__(self):
        self.calls = []

    def drive_speed(self, **kwargs):
        self.calls.append(kwargs)


class FakeGimbal:
    def __init__(self):
        self.calls = []
        self.moveto_calls = []
        self.angle_subscriptions = []
        self.unsub_angle_count = 0

    def drive_speed(self, **kwargs):
        self.calls.append(kwargs)

    def moveto(self, **kwargs):
        self.moveto_calls.append(kwargs)
        return type("Action", (), {"wait_for_completed": lambda _self, timeout=None: None})()

    def sub_angle(self, **kwargs):
        self.angle_subscriptions.append(kwargs)
        return True

    def unsub_angle(self):
        self.unsub_angle_count += 1
        return True


class FakeRobot:
    def __init__(self):
        self.chassis = FakeChassis()
        self.gimbal = FakeGimbal()
        self.modes = []
        self.closed = False
        self.robot_mode = None

    def set_robot_mode(self, **kwargs):
        self.modes.append(kwargs)
        self.robot_mode = kwargs["mode"]

    def get_robot_mode(self):
        return self.robot_mode

    def close(self):
        self.closed = True


class FakeRobotSdk:
    CHASSIS_LEAD = "sdk_chassis_lead"
    FREE = "sdk_free"
    GIMBAL_LEAD = "sdk_gimbal_lead"


def make_adapter() -> tuple[RoboMasterAdapter, FakeRobot]:
    adapter = RoboMasterAdapter.__new__(RoboMasterAdapter)
    adapter.logger = type(
        "Logger",
        (),
        {
            "warning": lambda *args, **kwargs: None,
            "debug": lambda *args, **kwargs: None,
            "info": lambda *args, **kwargs: None,
        },
    )()
    adapter.drive_timeout_s = 0.1
    adapter.angular_unit = "rad_per_s"
    adapter.sdk_z_unit = "rad_per_s"
    adapter.gimbal_config = {}
    adapter.sdk_available = True
    adapter.robot_sdk = None
    adapter.zero_sent_by_robot = {}
    adapter.latest_robot_status = {}
    adapter.pending_robot_statuses = deque()
    adapter.robot_status_lock = threading.Lock()
    adapter.gimbal_angle_subscribed = set()
    adapter.robot_mode = "free"
    adapter.valid_robot_modes = {"chassis_lead", "free", "gimbal_lead"}
    adapter.requested_mode_by_robot = {}
    adapter.actual_mode_by_robot = {}
    adapter.robot_info = {
        "r1": RobotInfo(
            robot_id="r1",
            sn="sn",
            group="g",
            rigid_body_name="rb",
            chassis_enabled=True,
            gimbal_enabled=True,
        )
    }
    robot = FakeRobot()
    adapter.instances = {"r1": robot}
    return adapter, robot


def cmd(vx=0.0, vy=0.0, wz=0.0, gy=0.0, gp=0.0) -> RobotCommand:
    return RobotCommand("r1", vx, vy, wz, gy, gp, "test")


def total_sdk_calls(robot: FakeRobot) -> int:
    return len(robot.chassis.calls) + len(robot.gimbal.calls)


def test_nonzero_command_is_sent():
    adapter, robot = make_adapter()
    adapter.send_command(cmd(vx=0.1))
    assert total_sdk_calls(robot) == 2
    assert adapter.zero_sent_by_robot["r1"] is False


def test_first_zero_after_nonzero_is_sent():
    adapter, robot = make_adapter()
    adapter.send_command(cmd(vx=0.1))
    adapter.send_command(cmd())
    assert total_sdk_calls(robot) == 4
    assert adapter.zero_sent_by_robot["r1"] is True


def test_second_consecutive_zero_is_skipped():
    adapter, robot = make_adapter()
    adapter.send_command(cmd())
    adapter.send_command(cmd())
    assert total_sdk_calls(robot) == 2
    assert adapter.zero_sent_by_robot["r1"] is True


def test_nonzero_after_zero_is_sent_and_clears_state():
    adapter, robot = make_adapter()
    adapter.send_command(cmd())
    adapter.send_command(cmd(vx=0.1))
    assert total_sdk_calls(robot) == 4
    assert adapter.zero_sent_by_robot["r1"] is False


def test_zero_chassis_nonzero_gimbal_is_not_complete_zero():
    adapter, robot = make_adapter()
    adapter.send_command(cmd(gy=5.0))
    adapter.send_command(cmd(gy=5.0))
    assert total_sdk_calls(robot) == 4
    assert adapter.zero_sent_by_robot["r1"] is False


def test_repeated_stop_all_forces_sdk_zero():
    adapter, robot = make_adapter()
    adapter.stop_all()
    adapter.stop_all()
    assert total_sdk_calls(robot) == 4
    assert adapter.zero_sent_by_robot["r1"] is True


def test_mock_mode_keeps_same_zero_state_logic():
    adapter, _ = make_adapter()
    adapter.sdk_available = False
    adapter.send_command(cmd())
    adapter.send_command(cmd())
    assert adapter.zero_sent_by_robot["r1"] is True
    adapter.send_command(cmd(vx=0.1))
    assert adapter.zero_sent_by_robot["r1"] is False


def test_chassis_lead_ignores_gimbal_yaw_for_effective_zero():
    adapter, robot = make_adapter()
    adapter.robot_mode = "chassis_lead"
    adapter.send_command(cmd(gy=5.0))
    adapter.send_command(cmd(gy=5.0))

    assert total_sdk_calls(robot) == 1
    assert len(robot.chassis.calls) == 1
    assert robot.gimbal.calls == []
    assert adapter.zero_sent_by_robot["r1"] is True


def test_chassis_lead_does_not_send_gimbal_pitch():
    adapter, robot = make_adapter()
    adapter.robot_mode = "chassis_lead"
    adapter.send_command(cmd(vx=0.1, gp=5.0))

    assert total_sdk_calls(robot) == 1
    assert robot.chassis.calls[0]["x"] == 0.1
    assert robot.gimbal.calls == []
    assert adapter.zero_sent_by_robot["r1"] is False


def test_gimbal_lead_ignores_all_chassis_channels_for_effective_zero():
    adapter, robot = make_adapter()
    adapter.robot_mode = "gimbal_lead"
    adapter.send_command(cmd(vx=0.1, vy=0.2, wz=0.3))
    adapter.send_command(cmd(vx=0.1, vy=0.2, wz=0.3))

    assert total_sdk_calls(robot) == 1
    assert robot.chassis.calls == []
    assert len(robot.gimbal.calls) == 1
    assert adapter.zero_sent_by_robot["r1"] is True


def test_gimbal_lead_sends_only_gimbal_data():
    adapter, robot = make_adapter()
    adapter.robot_mode = "gimbal_lead"
    adapter.send_command(cmd(vx=0.1, vy=0.2, wz=0.3, gy=5.0, gp=6.0))

    assert total_sdk_calls(robot) == 1
    assert robot.chassis.calls == []
    assert robot.gimbal.calls[0] == {"yaw_speed": 5.0, "pitch_speed": 6.0}
    assert adapter.zero_sent_by_robot["r1"] is False


def test_free_sends_chassis_and_gimbal_data():
    adapter, robot = make_adapter()
    adapter.robot_mode = "free"
    adapter.send_command(cmd(vx=0.1, vy=0.2, wz=0.3, gy=5.0, gp=6.0))

    assert total_sdk_calls(robot) == 2
    assert robot.chassis.calls[0]["x"] == 0.1
    assert robot.chassis.calls[0]["y"] == 0.2
    assert robot.chassis.calls[0]["z"] == 0.3
    assert robot.gimbal.calls[0] == {"yaw_speed": 5.0, "pitch_speed": 6.0}


def test_set_robot_mode_calls_sdk_robot_mode():
    adapter, robot = make_adapter()
    adapter.robot_sdk = FakeRobotSdk
    adapter.set_robot_mode("chassis_lead")

    assert adapter.robot_mode == "chassis_lead"
    assert robot.modes == [{"mode": "sdk_chassis_lead"}]
    status = adapter.get_robot_statuses()[0]
    assert status.status_type == "robot_mode"
    assert status.requested_mode == "chassis_lead"
    assert status.actual_mode == "chassis_lead"


def test_set_robot_mode_rejects_unknown_mode():
    adapter, _ = make_adapter()
    try:
        adapter.set_robot_mode("unknown")
    except ValueError as exc:
        assert "Unsupported robot_mode" in str(exc)
    else:
        raise AssertionError("Expected ValueError")


def test_subscribe_gimbal_angle_status_records_latest_status():
    adapter, robot = make_adapter()
    adapter.gimbal_config = {"angle_status": {"enabled": True, "freq_hz": 10}}
    adapter.requested_mode_by_robot["r1"] = "free"
    adapter.actual_mode_by_robot["r1"] = "free"
    info = adapter.robot_info["r1"]

    adapter._subscribe_gimbal_angle_status(info, robot)
    robot.gimbal.angle_subscriptions[0]["callback"]((1.0, 2.0, 3.0, 4.0))

    status = adapter.get_robot_statuses()[0]
    assert robot.gimbal.angle_subscriptions[0]["freq"] == 10
    assert status.robot_id == "r1"
    assert status.status_type == "gimbal_angle"
    assert status.pitch_angle == 1.0
    assert status.yaw_angle == 2.0
    assert status.pitch_ground_angle == 3.0
    assert status.yaw_ground_angle == 4.0
    assert status.requested_mode == "free"
    assert status.actual_mode == "free"


def test_gimbal_angle_status_uses_current_robot_mode_when_mode_not_cached():
    adapter, _ = make_adapter()
    adapter.robot_mode = "gimbal_lead"

    adapter._update_gimbal_angle_status("r1", (1.0, 2.0, 3.0, 4.0))

    status = adapter.get_robot_statuses()[0]
    assert status.requested_mode == "gimbal_lead"
    assert status.actual_mode is None


def test_gimbal_init_uses_nested_zero_on_connect_config():
    adapter, robot = make_adapter()
    adapter.gimbal_config = {
        "init_zero_on_connect": {
            "enabled": True,
            "pitch_deg": 1.0,
            "yaw_deg": 2.0,
            "pitch_speed": 3.0,
            "yaw_speed": 4.0,
            "wait_timeout_s": 5.0,
        }
    }

    adapter._move_gimbal_to_initial_position("r1", robot)

    assert robot.gimbal.moveto_calls == [
        {"pitch": 1.0, "yaw": 2.0, "pitch_speed": 3.0, "yaw_speed": 4.0}
    ]
    assert robot.gimbal.calls[-1] == {"pitch_speed": 0.0, "yaw_speed": 0.0}


def test_close_unsubscribes_gimbal_angle_status():
    adapter, robot = make_adapter()
    adapter.gimbal_angle_subscribed.add("r1")

    adapter.close()

    assert robot.gimbal.unsub_angle_count == 1

