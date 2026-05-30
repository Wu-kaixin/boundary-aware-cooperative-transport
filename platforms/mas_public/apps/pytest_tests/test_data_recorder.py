import csv

from src.common.messages import ControlCommand, ModuleStatus, RobotCommand, RobotState, RobotStatus, WorldState
from src.controller.data_recorder import DataRecorder


def test_data_recorder_writes_csv(tmp_path):
    recorder = DataRecorder(tmp_path)
    recorder.record_world_state(WorldState(1.0, 1, [RobotState("r1", True, 1, 2, 0, 0.1, 0.2, 0.3, 0, 0, 0, 1.0)]))
    recorder.record_control_command(ControlCommand(1.0, "chassis_lead", [RobotCommand("r1", 0, 0, 0, 0, 0, "test")]))
    recorder.record_module_status(ModuleStatus("controller", "running", "ok", 1.0))
    recorder.record_robot_status(RobotStatus("r1", "gimbal_angle", 1.0, 1.0, 2.0, 3.0, 4.0))
    recorder.close()
    world_text = (tmp_path / "world_state.csv").read_text(encoding="utf-8")
    command_text = (tmp_path / "control_command.csv").read_text(encoding="utf-8")
    robot_status_text = (tmp_path / "robot_status.csv").read_text(encoding="utf-8")
    assert "roll,pitch,yaw" in world_text
    assert "robot_mode" in command_text
    assert "pitch_angle,yaw_angle" in robot_status_text
    assert "requested_mode,actual_mode" in robot_status_text
    assert "gimbal_angle" in robot_status_text
    assert world_text.count("\n") >= 2
    assert command_text.count("\n") >= 2
    assert (tmp_path / "system_status.csv").read_text(encoding="utf-8").count("\n") >= 2


def test_data_recorder_writes_inactive_command_channels_as_blank(tmp_path):
    recorder = DataRecorder(tmp_path)
    recorder.record_control_command(
        ControlCommand(1.0, "gimbal_lead", [RobotCommand("r1", None, None, None, 5.0, 0.0, "test")])
    )
    recorder.close()

    with (tmp_path / "control_command.csv").open(encoding="utf-8", newline="") as file:
        row = next(csv.DictReader(file))
    assert row["chassis_vx"] == ""
    assert row["chassis_vy"] == ""
    assert row["chassis_wz"] == ""
    assert row["gimbal_yaw_speed"] == "5.0"

