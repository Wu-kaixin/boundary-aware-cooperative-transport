from __future__ import annotations

import signal
import math
from dataclasses import dataclass
from typing import Any

from src.common.config_loader import load_all_configs
from src.common.logger import setup_logging
from src.common.math_utils import clamp, wrap_angle_rad
from src.common.messages import ControlCommand, ModuleStatus, RobotCommand, RobotState, RobotStatus, WorldState
from src.common.time_utils import monotonic_s, now_s, sleep_until_next
from src.controller.base_controller import BaseController
from src.controller.coordinate_transform import CoordinateTransformer
from src.controller.cvt_controller import CVTController
from src.controller.data_recorder import DataRecorder
from src.controller.experiment_logger import ExperimentLogger
from src.controller.decentralized_transport_controller import DecentralizedTransportController
from src.controller.manual_controller import ManualController
from src.controller.point_controller import PointController
from src.controller.plotting.experiment_plotter import ExperimentPlotter
from src.controller.world_bounds import out_of_bounds_robot_ids
from src.messaging.factory import TransportFactory
from src.messaging.topics import CONTROL_COMMAND, MODULE_STATUS, ROBOT_STATUS, WORLD_STATE


@dataclass
class SmoothedPose:
    x: float
    y: float
    yaw: float


class ControllerModule:
    """Internal module."""

    def __init__(self):
        self.configs = load_all_configs()
        self.system_config = self.configs["system"]
        self.controller_config = self.configs["controller"]
        self.robots_config = self.configs["robots"]
        self.logger = setup_logging("controller")
        transport = TransportFactory(self.system_config["network"], self.logger)
        self.publisher = transport.create_publisher("control_command")
        self.subscriber = transport.create_subscriber("world_state", [WORLD_STATE, MODULE_STATUS])
        self.robot_status_subscriber = transport.create_subscriber("module_status", [MODULE_STATUS, ROBOT_STATUS])
        self.robot_ids = [item["robot_id"] for item in self.robots_config["robots"]["list"]]
        self.controller = self._build_controller()
        self.coordinate_transformer = self._build_coordinate_transformer()
        self.running = False
        self.last_world_state: WorldState | None = None
        self.last_state_monotonic: float | None = None
        self.previous_control_frame_yaw_by_id: dict[str, tuple[float, float]] = {}
        self.smoothed_pose_by_id: dict[str, SmoothedPose] = {}
        self.latest_robot_status_by_id: dict[str, RobotStatus] = {}
        self.latest_module_status_by_name: dict[str, ModuleStatus] = {}
        self.untracked_since_by_id: dict[str, float] = {}
        self.out_of_bounds_since: float | None = None
        self.recorder: DataRecorder | None = None
        self.task_completed = False
        self.task_failed = False
        self.failure_message = ""
        self.safety_event_messages: list[str] = []
        self.interrupted = False
        self._setup_recording()
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)
        if hasattr(signal, "SIGBREAK"):
            signal.signal(signal.SIGBREAK, self._handle_signal)

    def _build_controller(self) -> BaseController:
        controller_type = self.controller_config["controller"].get("type", "manual")
        limits_config = getattr(self, "system_config", {}).get("limits", {})
        if controller_type == "point":
            return PointController(self.controller_config, self.robot_ids, limits_config)
        if controller_type == "cvt":
            return CVTController(
                self.controller_config,
                self.robot_ids,
                self.system_config["world"],
                limits_config,
            )
        if controller_type == "dtransport":
            return DecentralizedTransportController(
                self.controller_config,
                self.robot_ids,
                self.system_config["world"],
                limits_config,
            )
        if controller_type == "manual":
            return ManualController(self.controller_config, self.robot_ids)
        raise ValueError(f"Unsupported controller type: {controller_type}")

    def _build_coordinate_transformer(self) -> CoordinateTransformer | None:
        transform_config = self.system_config.get("z_up_transform", {})
        if not transform_config.get("enabled", False):
            return None
        return CoordinateTransformer(float(transform_config.get("motive_to_world_rx_deg", -90.0)))

    def _world_state_for_control_frame(
        self, world_state: WorldState | None, update_yaw_rate: bool = False
    ) -> WorldState | None:
        # 先在 Motive 原始坐标中平滑，再转换到控制坐标；控制器和边界检查使用该控制坐标。
        # Smooth in the raw Motive frame first, then transform into the controller frame used by controllers/bounds.
        smoothed_raw = self._smooth_world_state(world_state)
        if self.coordinate_transformer is None:
            control_frame = smoothed_raw
        else:
            control_frame = self.coordinate_transformer.world_state(smoothed_raw)
        if control_frame is None or not update_yaw_rate:
            return control_frame
        return self._with_control_frame_yaw_rates(control_frame)

    def _smooth_world_state(self, world_state: WorldState | None) -> WorldState | None:
        smoothing = self.system_config.get("worldstate_smoothing", {})
        if world_state is None or not smoothing.get("enabled", False):
            return world_state
        method = str(smoothing.get("method", "ema"))
        if method != "ema":
            raise ValueError(f"Unsupported worldstate_smoothing method: {method}")
        alpha_xy = clamp(float(smoothing.get("alpha_xy", 0.5)), 0.0, 1.0)
        alpha_yaw = clamp(float(smoothing.get("alpha_yaw", 0.5)), 0.0, 1.0)
        near_target_only = bool(smoothing.get("near_target_only", False))
        near_target_distance_m = float(smoothing.get("near_target_distance_m", 0.3))
        robots = []
        for robot in world_state.robots:
            if not robot.tracked:
                self.smoothed_pose_by_id.pop(robot.robot_id, None)
                robots.append(robot)
                continue
            if near_target_only and self._target_distance_for_smoothing(robot) > near_target_distance_m:
                self.smoothed_pose_by_id.pop(robot.robot_id, None)
                robots.append(robot)
                continue
            previous = self.smoothed_pose_by_id.get(robot.robot_id)
            if previous is None:
                smoothed = SmoothedPose(robot.x, robot.y, robot.yaw)
            else:
                smoothed = SmoothedPose(
                    x=alpha_xy * robot.x + (1.0 - alpha_xy) * previous.x,
                    y=alpha_xy * robot.y + (1.0 - alpha_xy) * previous.y,
                    yaw=wrap_angle_rad(previous.yaw + alpha_yaw * wrap_angle_rad(robot.yaw - previous.yaw)),
                )
            self.smoothed_pose_by_id[robot.robot_id] = smoothed
            robots.append(
                RobotState(
                    robot_id=robot.robot_id,
                    tracked=robot.tracked,
                    x=smoothed.x,
                    y=smoothed.y,
                    z=robot.z,
                    roll=robot.roll,
                    pitch=robot.pitch,
                    yaw=smoothed.yaw,
                    vx=robot.vx,
                    vy=robot.vy,
                    wz=robot.wz,
                    timestamp=robot.timestamp,
                    vz=robot.vz,
                )
            )
        return WorldState(timestamp=world_state.timestamp, frame_id=world_state.frame_id, robots=robots)

    def _target_distance_for_smoothing(self, robot: RobotState) -> float:
        controller_type = self.controller_config.get("controller", {}).get("type", "")
        targets = self.controller_config.get("controller_params", {}).get(controller_type, {}).get("targets", {})
        target = targets.get(robot.robot_id)
        if target is None:
            return 0.0
        transformer = getattr(self, "coordinate_transformer", None)
        control_frame_robot = transformer.robot_state(robot) if transformer else robot
        return math.hypot(
            float(target.get("x", 0.0)) - control_frame_robot.x,
            float(target.get("y", 0.0)) - control_frame_robot.y,
        )

    def _with_control_frame_yaw_rates(self, world_state: WorldState) -> WorldState:
        robots = []
        seen_robot_ids = set()
        for robot in world_state.robots:
            seen_robot_ids.add(robot.robot_id)
            previous = self.previous_control_frame_yaw_by_id.get(robot.robot_id)
            self.previous_control_frame_yaw_by_id[robot.robot_id] = (robot.yaw, robot.timestamp)
            if not robot.tracked:
                self.previous_control_frame_yaw_by_id.pop(robot.robot_id, None)
                robots.append(robot)
                continue
            if previous is None:
                wz = 0.0
            else:
                previous_yaw, previous_timestamp = previous
                dt = robot.timestamp - previous_timestamp
                wz = 0.0 if dt <= 1e-6 or dt > 1.0 else wrap_angle_rad(robot.yaw - previous_yaw) / dt
            robots.append(
                RobotState(
                    robot_id=robot.robot_id,
                    tracked=robot.tracked,
                    x=robot.x,
                    y=robot.y,
                    z=robot.z,
                    roll=robot.roll,
                    pitch=robot.pitch,
                    yaw=robot.yaw,
                    vx=robot.vx,
                    vy=robot.vy,
                    wz=wz,
                    timestamp=robot.timestamp,
                    vz=robot.vz,
                )
            )
        for robot_id in list(self.previous_control_frame_yaw_by_id):
            if robot_id not in seen_robot_ids:
                self.previous_control_frame_yaw_by_id.pop(robot_id, None)
        return WorldState(timestamp=world_state.timestamp, frame_id=world_state.frame_id, robots=robots)

    def _setup_recording(self) -> None:
        recording = self.controller_config.get("recording", {})
        if not recording.get("enable", True):
            return
        experiment_name = self.system_config.get("experiment_name", "experiment")
        output_dir = recording.get("output_dir", "data/experiments")
        experiment_dir = ExperimentLogger(experiment_name, output_dir).create()
        self.recorder = DataRecorder(experiment_dir)
        self.logger.info("Recording enabled: %s", experiment_dir)

    def run(self) -> None:
        self.running = True
        self._publish_status("running", "controller module started")
        hz = float(self.system_config["frequency"].get("controller_hz", 100))
        period = 1.0 / hz
        timeout_ms = int(self.controller_config["input"].get("state_timeout_ms", 200))
        require_world_state = self._requires_world_state()
        world_config = self.system_config["world"]
        try:
            while self.running:
                loop_start = monotonic_s()
                self._drain_inputs()
                world_state_fresh = self._world_state_is_fresh(timeout_ms)
                state_valid = self._state_is_valid(timeout_ms)
                control_frame_world_state = (
                    self._world_state_for_control_frame(self.last_world_state, update_yaw_rate=True)
                    if self.last_world_state and state_valid
                    else None
                )
                tracking_lost_message = self._tracking_lost_message(world_state_fresh)
                if tracking_lost_message:
                    command = self._zero_command("tracking_lost")
                    self.task_failed = True
                    self.failure_message = tracking_lost_message
                    self.running = False
                    self.logger.error(tracking_lost_message)
                elif require_world_state and not state_valid:
                    command = self._zero_command("state_timeout")
                    self.logger.warning("WorldState timeout, sending zero command")
                elif self._bounds_check_enabled(world_state_fresh) and self._is_out_of_bounds(
                    control_frame_world_state, world_config
                ):
                    command = self._zero_command("world_out_of_bounds")
                else:
                    command = self.controller.compute(control_frame_world_state)
                command = self._apply_gimbal_control(command)
                command = self._normalize_command_for_mode(command)
                self.publisher.publish(CONTROL_COMMAND, command)
                if self.recorder:
                    if self.last_world_state and state_valid:
                        self.recorder.record_world_state(self.last_world_state)
                        if self.coordinate_transformer is not None and control_frame_world_state is not None:
                            self.recorder.record_world_state_zup(control_frame_world_state)
                    self.recorder.record_control_command(command)
                if self._controller_task_completed():
                    self.task_completed = True
                    self.running = False
                sleep_until_next(loop_start, period)
        except Exception as exc:
            self.logger.exception("Controller error: %s", exc)
            self._publish_status("error", str(exc))
            raise
        finally:
            self.shutdown()

    def _drain_inputs(self) -> None:
        self._drain_subscriber(self.subscriber)
        self._drain_subscriber(self.robot_status_subscriber)

    def _drain_subscriber(self, subscriber: Any) -> None:
        while True:
            received = subscriber.receive(timeout_ms=0)
            if received is None:
                return
            topic, payload = received
            if topic == WORLD_STATE:
                self.last_world_state = WorldState.from_dict(payload)
                self.last_state_monotonic = monotonic_s()
            elif topic == MODULE_STATUS:
                status = ModuleStatus.from_dict(payload)
                self.latest_module_status_by_name[status.module_name] = status
                if self.recorder:
                    self.recorder.record_module_status(status)
            elif topic == ROBOT_STATUS:
                status = RobotStatus.from_dict(payload)
                if status.status_type == "gimbal_angle":
                    self.latest_robot_status_by_id[status.robot_id] = status
                if self.recorder:
                    self.recorder.record_robot_status(status)

    def _state_is_valid(self, timeout_ms: int) -> bool:
        if not self._world_state_is_fresh(timeout_ms):
            return False
        if self.controller_config["input"].get("require_all_tracked_for_valid_state", False):
            return all(robot.tracked for robot in self.last_world_state.robots)
        return True

    def _requires_world_state(self) -> bool:
        controller_type = str(self.controller_config.get("controller", {}).get("type", "manual"))
        use_optitrack = bool(self.configs.get("supervisor", {}).get("use_optitrack", True))
        return controller_type != "manual" and use_optitrack

    def _world_state_is_fresh(self, timeout_ms: int) -> bool:
        if self.last_world_state is None or self.last_state_monotonic is None:
            return False
        return (monotonic_s() - self.last_state_monotonic) * 1000.0 <= timeout_ms

    def _tracking_lost_message(self, world_state_fresh: bool) -> str | None:
        # untracked 超时是全局安全停止条件；manual 模式也会参与该判断。
        # Persistent untracked robots are a global safety-stop condition, including manual mode.
        experiment_config = self.system_config.get("experiment", {})
        if not experiment_config.get("auto_stop_on_untracked", False):
            return None
        if not world_state_fresh or self.last_world_state is None:
            return None
        timeout_s = float(experiment_config.get("untracked_timeout_s", 1.0))
        current_time = monotonic_s()
        state_by_id = {robot.robot_id: robot for robot in self.last_world_state.robots}
        timed_out_robot_ids = []
        for robot_id in self.robot_ids:
            state = state_by_id.get(robot_id)
            if state is not None and state.tracked:
                self.untracked_since_by_id.pop(robot_id, None)
                continue
            untracked_since = self.untracked_since_by_id.setdefault(robot_id, current_time)
            if current_time - untracked_since >= timeout_s:
                timed_out_robot_ids.append(robot_id)
        if not timed_out_robot_ids:
            return None
        message = f"tracking_lost: untracked robots={timed_out_robot_ids}"
        self._record_safety_event(message)
        return message

    def _is_out_of_bounds(self, world_state: WorldState | None, world_config: dict) -> bool:
        # 越界先发送零命令等待宽限期，超过 out_of_bounds_fail_delay_s 后才标记 failed。
        # Out-of-bounds first sends zero commands during the grace period, then marks the task failed.
        robot_ids = out_of_bounds_robot_ids(world_state, world_config)
        if not robot_ids:
            self.out_of_bounds_since = None
            return False
        message = f"world_out_of_bounds: robots={robot_ids}"
        self._record_safety_event(message)
        current_time = monotonic_s()
        if self.out_of_bounds_since is None:
            self.out_of_bounds_since = current_time
        delay_s = float(world_config.get("out_of_bounds_fail_delay_s", 0.0))
        if current_time - self.out_of_bounds_since < delay_s:
            self.logger.warning("World bounds exceeded by robots=%s, waiting for grace period", robot_ids)
            return True
        self.task_failed = True
        self.failure_message = message
        self.running = False
        self.logger.error("World bounds exceeded by robots=%s, stopping controller", robot_ids)
        return True

    def _record_safety_event(self, message: str) -> None:
        safety_events = getattr(self, "safety_event_messages", None)
        if safety_events is None:
            self.safety_event_messages = []
            safety_events = self.safety_event_messages
        if message not in safety_events:
            safety_events.append(message)

    def _failure_message_with_safety_events(self) -> str:
        # 主失败原因保持单一；prior_events 只用于事后解释先前发生过的安全事件。
        # Keep one primary failure reason; prior_events only explains earlier safety events for post-analysis.
        message = self.failure_message or "controller task failed"
        prior_events = [event for event in getattr(self, "safety_event_messages", []) if event != message]
        if not prior_events:
            return message
        return f"{message}; prior_events={prior_events}"

    def _bounds_check_enabled(self, world_state_fresh: bool) -> bool:
        world_config = self.system_config.get("world", {})
        if not world_config.get("stop_on_out_of_bounds", True):
            return False
        if not bool(self.configs.get("supervisor", {}).get("use_optitrack", True)):
            return False
        optitrack_status = self.latest_module_status_by_name.get("optitrack")
        optitrack_running = optitrack_status is not None and optitrack_status.status in {"ready", "running"}
        return world_state_fresh or optitrack_running

    def _zero_command(self, mode: str) -> ControlCommand:
        command = ControlCommand(
            timestamp=now_s(),
            robot_mode=self.controller.robot_mode,
            commands=[RobotCommand(robot_id, 0.0, 0.0, 0.0, 0.0, 0.0, mode) for robot_id in self.robot_ids],
        )
        return self._normalize_command_for_mode(command)

    def _apply_gimbal_control(self, command: ControlCommand) -> ControlCommand:
        gimbal_config = self.system_config.get("gimbal_control", {})
        if not gimbal_config:
            return command
        return ControlCommand(
            timestamp=command.timestamp,
            robot_mode=command.robot_mode,
            commands=[self._apply_gimbal_control_to_robot(robot_command, gimbal_config) for robot_command in command.commands],
        )

    def _apply_gimbal_control_to_robot(self, command: RobotCommand, gimbal_config: dict) -> RobotCommand:
        if self._skip_gimbal_control(command):
            return command
        return RobotCommand(
            robot_id=command.robot_id,
            chassis_vx=command.chassis_vx,
            chassis_vy=command.chassis_vy,
            chassis_wz=command.chassis_wz,
            gimbal_yaw_speed=self._gimbal_yaw_speed(command, gimbal_config),
            gimbal_pitch_speed=self._gimbal_pitch_speed(command.robot_id, gimbal_config),
            controller_mode=command.controller_mode,
        )

    @staticmethod
    def _skip_gimbal_control(command: RobotCommand) -> bool:
        # 安全/停止类命令必须保持全零，不再叠加 yaw follow 或 pitch hold。
        # Safety and stop commands must stay zero and must not receive yaw-follow or pitch-hold augmentation.
        return command.controller_mode in {
            "point_untracked",
            "point_zero",
            "cvt_untracked",
            "cvt_zero",
            "tracking_lost",
            "state_timeout",
            "world_out_of_bounds",
            "shutdown",
        }

    def _gimbal_yaw_speed(self, command: RobotCommand, gimbal_config: dict) -> float:
        yaw_follow = gimbal_config.get("yaw_follow", {})
        if not yaw_follow.get("enabled", False):
            return command.gimbal_yaw_speed or 0.0
        speed = 0.0
        if yaw_follow.get("feedforward_enabled", True) and command.chassis_wz is not None:
            speed += math.degrees(command.chassis_wz)
        if yaw_follow.get("feedback_enabled", False):
            speed += self._gimbal_yaw_feedback_speed(command.robot_id, yaw_follow)
        return speed

    def _gimbal_yaw_feedback_speed(self, robot_id: str, yaw_follow: dict) -> float:
        status = self.latest_robot_status_by_id.get(robot_id)
        if status is None or status.yaw_angle is None:
            return 0.0
        if now_s() - status.timestamp > float(yaw_follow.get("feedback_timeout_s", 0.5)):
            return 0.0
        yaw_angle = float(status.yaw_angle)
        if abs(yaw_angle) < float(yaw_follow.get("feedback_deadband_deg", 0.0)):
            return 0.0
        feedback = float(yaw_follow.get("feedback_kp", 0.0)) * yaw_angle
        max_speed = float(yaw_follow.get("feedback_max_speed_deg_s", 0.0))
        if max_speed <= 0.0:
            return feedback
        return clamp(feedback, -max_speed, max_speed)

    def _gimbal_pitch_speed(self, robot_id: str, gimbal_config: dict) -> float:
        pitch_hold = gimbal_config.get("pitch_hold", {})
        if not pitch_hold.get("enabled", False):
            return 0.0
        status = self.latest_robot_status_by_id.get(robot_id)
        if status is None or status.pitch_angle is None:
            return 0.0
        if now_s() - status.timestamp > float(pitch_hold.get("feedback_timeout_s", 0.5)):
            return 0.0
        error = float(status.pitch_angle) - float(pitch_hold.get("target_deg", 0.0))
        if abs(error) < float(pitch_hold.get("deadband_deg", 0.0)):
            return 0.0
        speed = float(pitch_hold.get("kp", 0.0)) * error
        max_speed = float(pitch_hold.get("max_speed_deg_s", 0.0))
        if max_speed <= 0.0:
            return speed
        return clamp(speed, -max_speed, max_speed)

    @staticmethod
    def _normalize_command_for_mode(command: ControlCommand) -> ControlCommand:
        if command.robot_mode == "chassis_lead":
            return ControlCommand(
                timestamp=command.timestamp,
                robot_mode=command.robot_mode,
                commands=[
                    RobotCommand(
                        robot_id=robot_command.robot_id,
                        chassis_vx=robot_command.chassis_vx,
                        chassis_vy=robot_command.chassis_vy,
                        chassis_wz=robot_command.chassis_wz,
                        gimbal_yaw_speed=None,
                        gimbal_pitch_speed=None,
                        controller_mode=robot_command.controller_mode,
                    )
                    for robot_command in command.commands
                ],
            )
        if command.robot_mode == "gimbal_lead":
            return ControlCommand(
                timestamp=command.timestamp,
                robot_mode=command.robot_mode,
                commands=[
                    RobotCommand(
                        robot_id=robot_command.robot_id,
                        chassis_vx=None,
                        chassis_vy=None,
                        chassis_wz=None,
                        gimbal_yaw_speed=robot_command.gimbal_yaw_speed,
                        gimbal_pitch_speed=robot_command.gimbal_pitch_speed,
                        controller_mode=robot_command.controller_mode,
                    )
                    for robot_command in command.commands
                ],
            )
        return command

    def _publish_status(self, status: str, message: str) -> None:
        module_status = ModuleStatus("controller", status, message, now_s())
        self.publisher.publish(MODULE_STATUS, module_status)
        if self.recorder:
            self.recorder.record_module_status(module_status)

    def _controller_task_completed(self) -> bool:
        task_completed = getattr(self.controller, "task_completed", None)
        if task_completed is None:
            return False
        return bool(task_completed())

    def _handle_signal(self, signum: int, frame: Any) -> None:
        self.logger.info("Received signal %s, stopping controller", signum)
        self.interrupted = True
        self.running = False

    def shutdown(self) -> None:
        self.running = False
        shutdown_command = self._zero_command("shutdown")
        self.publisher.publish(CONTROL_COMMAND, shutdown_command)
        if self.recorder:
            # 将停机零命令写入 CSV，便于确认实验退出前发送过最终停车命令。
            # Record the shutdown zero command so the CSV shows the final stop command explicitly.
            self.recorder.record_control_command(shutdown_command)
        experiment_dir = self.recorder.experiment_dir if self.recorder else None
        self._publish_final_status()
        if self.recorder:
            flush = getattr(self.recorder, "flush", None)
            if flush is not None:
                flush()
        if experiment_dir is not None and not getattr(self, "interrupted", False):
            self._plot_after_experiment(experiment_dir)
            self._check_after_experiment(experiment_dir)
        elif experiment_dir is not None:
            self.logger.info("Skipping experiment plots/check after interrupt: %s", experiment_dir)
        if self.recorder:
            self.recorder.close()
        self.publisher.close()
        self.subscriber.close()
        self.robot_status_subscriber.close()

    def _publish_final_status(self) -> None:
        if getattr(self, "task_failed", False):
            self._publish_status("failed", self._failure_message_with_safety_events())
        elif getattr(self, "task_completed", False):
            self._publish_status("completed", "controller task completed")
        else:
            self._publish_status("stopped", "controller module stopped")

    def _plot_after_experiment(self, experiment_dir: Any) -> None:
        plot_config = self.controller_config.get("plot", {})
        if not plot_config.get("enable_after_experiment", False):
            return
        try:
            outputs = ExperimentPlotter(experiment_dir).plot_all()
            self.logger.info("Generated experiment plots: %s", outputs)
        except Exception as exc:
            self.logger.exception("Failed to generate experiment plots: %s", exc)

    def _check_after_experiment(self, experiment_dir: Any) -> None:
        try:
            from apps.check_experiment import check_experiment

            report = check_experiment(experiment_dir)
        except Exception as exc:
            self.logger.exception("Failed to check experiment output: %s", exc)
            return
        for warning in report.warnings:
            self.logger.warning("Experiment check warning: %s", warning)
        if report.ok:
            self.logger.info("Experiment check passed: %s", experiment_dir)
            return
        for error in report.errors:
            self.logger.error("Experiment check error: %s", error)

