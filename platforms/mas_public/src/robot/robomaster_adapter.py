from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import Any

from src.common.messages import RobotCommand, RobotStatus
from src.common.time_utils import now_s
from src.robot.robot_registry import RobotInfo


class RoboMasterAdapter:
    """封装 RoboMaster SDK；导入失败时保持 mock 可运行。 / Wrap the RoboMaster SDK with a mock fallback."""

    def __init__(
        self,
        logger: logging.Logger,
        drive_timeout_s: float = 0.1,
        angular_unit: str = "rad_per_s",
        sdk_z_unit: str = "deg_per_s",
        gimbal_config: dict | None = None,
    ):
        self.logger = logger
        self.drive_timeout_s = drive_timeout_s
        self.angular_unit = angular_unit
        self.sdk_z_unit = sdk_z_unit
        self.gimbal_config = gimbal_config or {}
        try:
            from robomaster import robot  # type: ignore

            self.robot_sdk = robot
            self.sdk_available = True
        except Exception as exc:
            self.robot_sdk = None
            self.sdk_available = False
            self.logger.warning("RoboMaster SDK unavailable, using mock adapter: %s", exc)
        self.instances: dict[str, Any] = {}
        self.robot_info: dict[str, RobotInfo] = {}
        self.zero_sent_by_robot: dict[str, bool] = {}
        self.latest_robot_status: dict[str, RobotStatus] = {}
        self.pending_robot_statuses: deque[RobotStatus] = deque()
        self.robot_status_lock = threading.Lock()
        self.gimbal_angle_subscribed: set[str] = set()
        self.robot_mode = "free"
        self.valid_robot_modes = {"chassis_lead", "free", "gimbal_lead"}
        self.requested_mode_by_robot: dict[str, str] = {}
        self.actual_mode_by_robot: dict[str, str | None] = {}

    def connect_all(
        self,
        robots: list[RobotInfo],
        conn_type: str = "sta",
        proto_type: str = "udp",
        retry_count: int = 2,
        retry_delay_s: float = 1.0,
        require_sn: bool = True,
    ) -> None:
        for info in robots:
            self.robot_info[info.robot_id] = info
            self._cache_robot_mode(info.robot_id, self.robot_mode, self.robot_mode)
            if not self.sdk_available:
                self.logger.info("[mock] robot %s connected", info.robot_id)
                continue
            if require_sn and not info.sn:
                raise ValueError(f"Robot {info.robot_id} requires SN in STA mode")
            last_exc: Exception | None = None
            for attempt in range(1, retry_count + 2):
                ep_robot = self.robot_sdk.Robot()
                try:
                    # 官方 STA 组网 + SN 指定连接方式；实时速度控制建议 proto_type='udp'。
                    # Official STA networking with SN targeting; UDP is recommended for real-time speed control.
                    ep_robot.initialize(conn_type=conn_type, proto_type=proto_type, sn=info.sn or None)
                    init_config = self.gimbal_config.get("init_zero_on_connect", {})
                    if info.gimbal_enabled and init_config.get("enabled", True):
                        self._move_gimbal_to_initial_position(info.robot_id, ep_robot)
                    version = ep_robot.get_version()
                    self.instances[info.robot_id] = ep_robot
                    actual_mode = self._read_robot_mode(ep_robot)
                    self._cache_robot_mode(info.robot_id, self.robot_mode, actual_mode)
                    self._subscribe_gimbal_angle_status(info, ep_robot)
                    self.logger.info(
                        "RoboMaster %s connected conn_type=%s proto_type=%s sn=%s version=%s",
                        info.robot_id,
                        conn_type,
                        proto_type,
                        info.sn,
                        version,
                    )
                    break
                except Exception as exc:
                    last_exc = exc
                    self.logger.warning(
                        "Connect robot %s failed attempt %d/%d: %s",
                        info.robot_id,
                        attempt,
                        retry_count + 1,
                        exc,
                    )
                    try:
                        ep_robot.close()
                    except Exception:
                        pass
                    if attempt <= retry_count:
                        time.sleep(retry_delay_s)
            else:
                raise RuntimeError(f"Failed to connect robot {info.robot_id}") from last_exc

    def _subscribe_gimbal_angle_status(self, info: RobotInfo, ep_robot: Any) -> None:
        angle_config = self.gimbal_config.get("angle_status", {})
        if not info.gimbal_enabled or not angle_config.get("enabled", False):
            return
        freq = int(angle_config.get("freq_hz", 10))

        def callback(angle_data: tuple[float, float, float, float], robot_id: str = info.robot_id) -> None:
            self._update_gimbal_angle_status(robot_id, angle_data)

        if ep_robot.gimbal.sub_angle(freq=freq, callback=callback):
            self.gimbal_angle_subscribed.add(info.robot_id)
            self.logger.info("RoboMaster %s gimbal angle status subscribed freq=%sHz", info.robot_id, freq)
        else:
            self.logger.warning("RoboMaster %s gimbal angle status subscription failed", info.robot_id)

    def _update_gimbal_angle_status(self, robot_id: str, angle_data: tuple[float, float, float, float]) -> None:
        pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_data
        requested_mode, actual_mode = self._mode_status_for_robot(robot_id)
        status = RobotStatus(
            robot_id=robot_id,
            status_type="gimbal_angle",
            timestamp=now_s(),
            pitch_angle=float(pitch_angle),
            yaw_angle=float(yaw_angle),
            pitch_ground_angle=float(pitch_ground_angle),
            yaw_ground_angle=float(yaw_ground_angle),
            requested_mode=requested_mode,
            actual_mode=actual_mode,
        )
        with self.robot_status_lock:
            self.latest_robot_status[robot_id] = status
            self.pending_robot_statuses.append(status)

    def get_robot_statuses(self) -> list[RobotStatus]:
        with self.robot_status_lock:
            statuses = list(self.pending_robot_statuses)
            self.pending_robot_statuses.clear()
        return statuses

    def _move_gimbal_to_initial_position(self, robot_id: str, ep_robot: Any) -> None:
        """连接成功后将云台运动到初始零角度位置。 / Move the gimbal to its configured initial zero position."""
        init_config = self.gimbal_config.get("init_zero_on_connect", {})
        pitch = float(init_config.get("pitch_deg", 0.0))
        yaw = float(init_config.get("yaw_deg", 0.0))
        pitch_speed = float(init_config.get("pitch_speed", 60.0))
        yaw_speed = float(init_config.get("yaw_speed", 60.0))
        wait_timeout_s = float(init_config.get("wait_timeout_s", 5.0))
        self.logger.info("Moving gimbal to initial position robot=%s pitch=%s yaw=%s", robot_id, pitch, yaw)
        action = ep_robot.gimbal.moveto(
            pitch=pitch,
            yaw=yaw,
            pitch_speed=pitch_speed,
            yaw_speed=yaw_speed,
        )
        action.wait_for_completed(timeout=wait_timeout_s)
        ep_robot.gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)

    def send_command(self, command: RobotCommand, force: bool = False) -> None:
        info = self.robot_info.get(command.robot_id)
        if info is None:
            self.logger.warning("Unknown robot id in command: %s", command.robot_id)
            return
        effective_command = self._effective_command(command)
        active_channels = self._active_channels(info)
        is_zero_command = self._is_active_zero_command(effective_command, active_channels)
        if not force and is_zero_command and self.zero_sent_by_robot.get(command.robot_id, False):
            self.logger.debug("Skip repeated zero command for robot %s", command.robot_id)
            return
        if not self.sdk_available:
            self.logger.debug("[mock] command %s effective=%s", command, effective_command)
            self.zero_sent_by_robot[command.robot_id] = is_zero_command
            return
        ep_robot = self.instances.get(command.robot_id)
        if ep_robot is None:
            self.logger.warning("Robot not connected: %s", command.robot_id)
            return
        self._send_command_to_sdk(info, ep_robot, effective_command, active_channels)
        self.zero_sent_by_robot[command.robot_id] = is_zero_command

    def set_robot_mode(self, mode: str) -> None:
        if mode not in self.valid_robot_modes:
            raise ValueError(f"Unsupported robot_mode: {mode}")
        if mode == self.robot_mode:
            return
        self.robot_mode = mode
        if not self.sdk_available:
            self.logger.info("[mock] robot mode set to %s", mode)
            return
        for robot_id, ep_robot in self.instances.items():
            self._set_robot_mode(robot_id, ep_robot, mode)

    def _set_robot_mode(self, robot_id: str, ep_robot: Any, mode: str) -> None:
        sdk_mode = self._sdk_robot_mode(mode)
        if sdk_mode is None:
            self.logger.warning("RoboMaster SDK mode constant unavailable for mode=%s", mode)
            return
        ep_robot.set_robot_mode(mode=sdk_mode)
        actual_mode = self._read_robot_mode(ep_robot)
        self._record_robot_mode_status(robot_id, mode, actual_mode)
        self.logger.info("RoboMaster %s robot mode set to %s", robot_id, mode)

    def _sdk_robot_mode(self, mode: str) -> Any:
        names = {
            "chassis_lead": "CHASSIS_LEAD",
            "free": "FREE",
            "gimbal_lead": "GIMBAL_LEAD",
        }
        return getattr(self.robot_sdk, names[mode], None)

    def _read_robot_mode(self, ep_robot: Any) -> str | None:
        try:
            sdk_mode = ep_robot.get_robot_mode()
        except Exception as exc:
            self.logger.warning("RoboMaster get_robot_mode failed: %s", exc)
            return None
        return self._mode_name_from_sdk(sdk_mode)

    def _mode_name_from_sdk(self, sdk_mode: Any) -> str | None:
        if sdk_mode is None:
            return None
        for mode in self.valid_robot_modes:
            if sdk_mode == self._sdk_robot_mode(mode):
                return mode
        return str(sdk_mode)

    def _record_robot_mode_status(self, robot_id: str, requested_mode: str, actual_mode: str | None) -> None:
        self._cache_robot_mode(robot_id, requested_mode, actual_mode)
        status = RobotStatus(
            robot_id=robot_id,
            status_type="robot_mode",
            timestamp=now_s(),
            requested_mode=requested_mode,
            actual_mode=actual_mode,
        )
        with self.robot_status_lock:
            self.latest_robot_status[robot_id] = status
            self.pending_robot_statuses.append(status)

    def _cache_robot_mode(self, robot_id: str, requested_mode: str, actual_mode: str | None) -> None:
        self.requested_mode_by_robot[robot_id] = requested_mode
        self.actual_mode_by_robot[robot_id] = actual_mode

    def _mode_status_for_robot(self, robot_id: str) -> tuple[str | None, str | None]:
        requested_mode = self.requested_mode_by_robot.get(robot_id, self.robot_mode)
        actual_mode = self.actual_mode_by_robot.get(robot_id)
        return requested_mode, actual_mode

    def _effective_command(self, command: RobotCommand) -> RobotCommand:
        if self.robot_mode == "chassis_lead":
            return RobotCommand(
                command.robot_id,
                command.chassis_vx,
                command.chassis_vy,
                command.chassis_wz,
                None,
                None,
                command.controller_mode,
            )
        if self.robot_mode == "gimbal_lead":
            return RobotCommand(
                command.robot_id,
                None,
                None,
                None,
                command.gimbal_yaw_speed,
                command.gimbal_pitch_speed,
                command.controller_mode,
            )
        return command

    def _active_channels(self, info: RobotInfo) -> set[str]:
        channels: set[str] = set()
        if info.chassis_enabled and self.robot_mode in {"chassis_lead", "free"}:
            channels.update({"chassis_vx", "chassis_vy", "chassis_wz"})
        if info.gimbal_enabled and self.robot_mode in {"gimbal_lead", "free"}:
            channels.update({"gimbal_yaw_speed", "gimbal_pitch_speed"})
        return channels

    def _send_command_to_sdk(
        self,
        info: RobotInfo,
        ep_robot: Any,
        command: RobotCommand,
        active_channels: set[str],
    ) -> None:
        chassis_channels = ("chassis_vx", "chassis_vy", "chassis_wz")
        if info.chassis_enabled and any(channel in active_channels for channel in chassis_channels):
            chassis_z = self._convert_chassis_wz(self._value_or_zero(command.chassis_wz))
            ep_robot.chassis.drive_speed(
                x=self._value_or_zero(command.chassis_vx),
                y=self._value_or_zero(command.chassis_vy),
                z=chassis_z,
                timeout=self.drive_timeout_s,
            )
        gimbal_channels = ("gimbal_yaw_speed", "gimbal_pitch_speed")
        if info.gimbal_enabled and any(channel in active_channels for channel in gimbal_channels):
            ep_robot.gimbal.drive_speed(
                yaw_speed=self._value_or_zero(command.gimbal_yaw_speed),
                pitch_speed=self._value_or_zero(command.gimbal_pitch_speed),
            )

    def stop_all(self) -> None:
        start = time.monotonic()
        self.logger.info("RoboMaster stop_all begin robots=%s", list(self.robot_info.keys()))
        for robot_id in self.robot_info:
            self.send_command(RobotCommand(robot_id, 0.0, 0.0, 0.0, 0.0, 0.0, "stop"), force=True)
        self.logger.info("RoboMaster stop_all end elapsed=%.3fs", time.monotonic() - start)

    def close(self) -> None:
        start = time.monotonic()
        self.logger.info("RoboMaster adapter close begin")
        self.stop_all()
        for robot_id, ep_robot in self.instances.items():
            if robot_id in self.gimbal_angle_subscribed:
                try:
                    ep_robot.gimbal.unsub_angle()
                except Exception as exc:
                    self.logger.warning("RoboMaster %s gimbal angle unsubscribe failed: %s", robot_id, exc)
            close_start = time.monotonic()
            self.logger.info("RoboMaster %s close begin", robot_id)
            ep_robot.close()
            self.logger.info("RoboMaster %s close end elapsed=%.3fs", robot_id, time.monotonic() - close_start)
        self.instances.clear()
        self.gimbal_angle_subscribed.clear()
        self.logger.info("RoboMaster adapter close end elapsed=%.3fs", time.monotonic() - start)

    def _convert_chassis_wz(self, wz: float) -> float:
        """项目消息保持 rad/s；按配置转换为 RoboMaster SDK 的 z 角速度单位。 / Convert chassis wz to the SDK unit."""
        if self.angular_unit == self.sdk_z_unit:
            return wz
        if self.angular_unit == "rad_per_s" and self.sdk_z_unit == "deg_per_s":
            return math.degrees(wz)
        if self.angular_unit == "deg_per_s" and self.sdk_z_unit == "rad_per_s":
            return math.radians(wz)
        raise ValueError(f"Unsupported angular unit conversion: {self.angular_unit} -> {self.sdk_z_unit}")

    @staticmethod
    def _is_active_zero_command(command: RobotCommand, active_channels: set[str]) -> bool:
        return all(RoboMasterAdapter._value_or_zero(getattr(command, channel)) == 0.0 for channel in active_channels)

    @staticmethod
    def _value_or_zero(value: float | None) -> float:
        return 0.0 if value is None else value
