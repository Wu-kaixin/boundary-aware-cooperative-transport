# Hardware Setup

This document records hardware preparation notes for RoboMaster S1, OptiTrack/Motive, and the experiment area.

本文档记录 RoboMaster S1、OptiTrack/Motive 和实验场地的硬件准备事项。

## RoboMaster S1 / RoboMaster S1

Before running real robot tests:

实机测试前：

- Use STA network mode unless intentionally testing another mode.
- Confirm each robot SN matches `configs/robots.yaml`.
- Confirm battery level is sufficient.
- Confirm chassis and gimbal are physically free to move.
- Confirm the physical emergency stop procedure.
- Keep the operator outside the robot motion area.

- 除非有明确测试目的，否则使用 STA 网络模式。
- 确认每台机器人 SN 与 `configs/robots.yaml` 匹配。
- 确认电量充足。
- 确认底盘和云台有足够运动空间。
- 确认物理急停流程。
- 操作人员不要站在机器人运动区域内。

Useful references:

参考资料：

- RoboMaster EP SDK documentation: <https://robomaster-dev.readthedocs.io/en/latest/python_sdk/installs.html>
- RoboMaster EP SDK Chinese documentation: <https://robomaster-dev.readthedocs.io/zh-cn/latest/>
- RoboMaster S1 root tutorial: <https://github.com/yunswj/dji_S1_hack>

## OptiTrack and Motive / OptiTrack 和 Motive

Before running OptiTrack-based tests:

基于 OptiTrack 的实验前：

- Open the Motive software.
- Calibrate cameras.
- Create rigid bodies for all expected robots.
- Make sure each Motive rigid-body ID matches `rigid_body_id` in `configs/robots.yaml`.
- Keep `motive_name` consistent with the Motive rigid-body name for readable logs and manual debugging.
- Enable NatNet streaming.
- Confirm server/client IP, data port, command port, and unicast/multicast setting match `configs/optitrack.yaml`.
- Confirm Motive shows stable tracking before starting controller motion.

- 打开 Motive 软件。
- 完成相机标定。
- 为所有预期机器人创建刚体。
- 确认 Motive 中每个刚体 ID 与 `configs/robots.yaml` 中的 `rigid_body_id` 匹配。
- 建议 `motive_name` 与 Motive 刚体名称保持一致，便于日志阅读和人工排查。
- 启用 NatNet streaming。
- 确认 unicast/multicast 设置与 `configs/optitrack.yaml` 匹配。
- 启动 controller 运动前，确认 Motive 中 tracking 稳定。

Useful reference:

参考资料：

- OptiTrack support and downloads: <https://optitrack.com/support/downloads>

## Experiment Area / 实验场地

Before real motion:

实机运动前：

- Confirm coordinate-frame definition.
- Confirm safe world bounds in `configs/system.yaml`.
- Place robots inside the configured bounds.
- Keep enough clearance around the boundary.
- Confirm robot starting positions and headings.
- Confirm operator position and emergency stop path.

- 确认坐标系定义。
- 确认 `configs/system.yaml` 中的安全边界。
- 将机器人放在配置边界内。
- 在边界附近保留足够余量。
- 确认机器人初始位置和朝向。
- 确认操作人员位置和急停路径。

## Pre-Run Checklist / 运行前清单

- `configs/controller.yaml` selects the intended controller.
- `configs/system.yaml/experiment_name` matches the intended experiment type.
- `configs/supervisor.yaml` enables the intended modules.
- OptiTrack is running if trajectory recording or bounds checking is expected.
- Robot module is connected before starting controller motion.
- Emergency stop is available.

- `configs/controller.yaml` 已选择预期控制器。
- `configs/system.yaml/experiment_name` 与预期实验类型一致。
- `configs/supervisor.yaml` 已启用预期模块。
- 如果需要轨迹记录或越界判断，OptiTrack 已运行。
- controller 运动前 robot 模块已连接。
- 急停方式可用。
