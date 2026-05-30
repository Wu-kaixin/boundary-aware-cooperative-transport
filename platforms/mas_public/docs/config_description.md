# Config Description

This document describes the current configuration files and the main parameter groups. It focuses on behavior and safety implications instead of listing every numeric field.

本文档说明当前配置文件和主要参数组，重点解释行为和安全含义，而不是逐项展开所有数值字段。

## `configs/system.yaml`

Global experiment behavior and shared runtime settings.

全局实验行为和共享运行设置。

- `experiment_name`: suffix used in experiment folder names, for example `20260520_175715_cvt`.
- `experiment.auto_stop_on_task_completed`: supervisor-managed runs stop robot/optitrack after controller completion.
- `experiment.auto_stop_on_untracked`: persistent untracked robots trigger `tracking_lost` and global stop.
- `experiment.untracked_timeout_s`: how long a robot can remain untracked before `tracking_lost`.
- `z_up_transform`: converts raw Motive/world coordinates into the control frame.
- `worldstate_smoothing`: optional EMA smoothing before coordinate transform; currently usually disabled for direct testing.
- `limits`: global chassis/gimbal command limits.
- `gimbal_control.yaw_follow`: optional gimbal yaw feedforward/feedback based on chassis yaw and gimbal status.
- `gimbal_control.pitch_hold`: optional pitch hold based on gimbal angle status.
- `robot_command_transform`: transforms project command signs/axes to RoboMaster hardware command signs/axes.
- `frequency`: OptiTrack publish target, controller loop rate, and robot command polling rate.
- `network`: ZeroMQ backend, host, and ports.
- `world`: world bounds and out-of-bounds failure grace time.

- `experiment_name`：实验文件夹名后缀，例如 `20260520_175715_cvt`。
- `experiment.auto_stop_on_task_completed`：supervisor 一键运行时，controller 完成后停止 robot/optitrack。
- `experiment.auto_stop_on_untracked`：机器人持续 untracked 后触发 `tracking_lost` 和全局停止。
- `experiment.untracked_timeout_s`：机器人允许持续 untracked 的时间。
- `z_up_transform`：把原始 Motive/world 坐标转换到控制坐标系。
- `worldstate_smoothing`：坐标转换前的可选 EMA 平滑；当前直接测试通常关闭。
- `limits`：底盘和云台命令全局限幅。
- `gimbal_control.yaw_follow`：基于底盘 yaw 和云台状态的可选云台 yaw 跟随。
- `gimbal_control.pitch_hold`：基于云台角度状态的可选 pitch 保持。
- `robot_command_transform`：把项目命令符号/坐标轴转换为 RoboMaster 硬件命令符号/坐标轴。
- `frequency`：OptiTrack 目标发布频率、controller 循环频率和 robot 命令轮询频率。
- `network`：ZeroMQ 后端、主机和端口。
- `world`：实验边界和越界失败宽限时间。

Notes:

说明：

- `world_state.csv` records the controller-sampled world state, not the full raw OptiTrack rate.

- `world_state.csv` 记录 controller 采样到的 world state，不是 OptiTrack 原始全频率数据。

## `configs/controller.yaml`

Selects the controller and controller-level runtime behavior.

选择控制器，并定义 controller 模块运行行为。

- `controller.type`: selected controller, one of `manual`, `point`, or `cvt`.
- `controller.robot_mode`: RoboMaster mode command, one of `free`, `chassis_lead`, or `gimbal_lead`.
- `input.state_timeout_ms`: world-state freshness timeout used by closed-loop controllers.
- `input.require_all_tracked_for_valid_state`: whether one untracked robot invalidates the whole `WorldState`.
- `recording.enable`: enable experiment CSV output and config snapshot.
- `recording.output_dir`: experiment output directory.
- `plot.enable_after_experiment`: natural completed/failed runs generate plots/checks after CSV flush.
- `plot.*`: plot category switches. `save_every_n_frames` is used by CVT-related plots.

- `controller.type`：当前控制器，取值为 `manual`、`point` 或 `cvt`。
- `controller.robot_mode`：RoboMaster 模式命令，取值为 `free`、`chassis_lead` 或 `gimbal_lead`。
- `input.state_timeout_ms`：闭环控制器使用的 world-state 新鲜度超时。
- `input.require_all_tracked_for_valid_state`：任意机器人 untracked 时，是否让整个 `WorldState` 失效。
- `recording.enable`：启用实验 CSV 输出和配置快照。
- `recording.output_dir`：实验输出目录。
- `plot.enable_after_experiment`：自然 completed/failed 时，CSV flush 后自动绘图和检查。
- `plot.*`：绘图类别开关。`save_every_n_frames` 用于 CVT 相关绘图。

Controller parameters are loaded automatically from `configs/controllers/{type}.yaml`. The experiment `config_snapshot/controllers/` folder stores only the selected controller parameter file.

控制器参数根据 `controller.type` 从 `configs/controllers/{type}.yaml` 自动加载。实验 `config_snapshot/controllers/` 中只保存当前选中的控制器参数文件。

Safety note: for real manual/point/cvt tests, the recommended default is:

安全说明：实机 manual/point/cvt 测试推荐默认组合：

```yaml
require_all_tracked_for_valid_state: false
auto_stop_on_untracked: true
```

This keeps tracked robots usable for bounds checking while persistent untracked robots still stop the experiment.

这样仍可用 tracked 机器人做越界判断，同时持续 untracked 的机器人仍会触发停止。

## `configs/controllers/manual.yaml`

Fixed command values for the manual controller.

manual 控制器固定命令值。

- `chassis_vx`, `chassis_vy`, `chassis_wz`: chassis velocity command.
- `gimbal_yaw_speed`, `gimbal_pitch_speed`: base gimbal speed command before optional global gimbal control.

- `chassis_vx`、`chassis_vy`、`chassis_wz`：底盘速度命令。
- `gimbal_yaw_speed`、`gimbal_pitch_speed`：全局云台控制叠加前的基础云台速度命令。

`manual` does not require OptiTrack data to produce commands. Without `WorldState`, there is no trajectory recording or bounds-check basis.

`manual` 不依赖 OptiTrack 数据生成命令。没有 `WorldState` 时，没有轨迹记录和越界判断依据。

## `configs/controllers/point.yaml`

Target-point controller parameters.

目标点控制器参数。

- PD gains for x/y/yaw.
- Position and yaw tolerances.
- Optional hold behavior after a robot reaches its target.
- Per-robot target x/y/yaw values.

- x/y/yaw 的 PD 参数。
- 位置和 yaw 容差。
- 机器人到达目标后的可选 hold 行为。
- 每台机器人的目标 x/y/yaw。

The task completes after all robots meet the completion condition. If `hold_enabled` is true, completion waits until hold finishes.

所有机器人满足完成条件后任务完成。若 `hold_enabled` 为 true，则等待 hold 完成后才算完成。

## `configs/controllers/cvt.yaml`

Centroidal Voronoi coverage controller parameters.

CVT 覆盖控制器参数。

- PD gains for centroid tracking and yaw.
- `grid_resolution`: grid approximation resolution for CVT calculation.
- `yaw.mode`: `face_velocity` or `fixed`.
- `centroid_tolerance_m`: completion tolerance.
- Optional hold behavior after completion.

- 质心跟踪和 yaw 的 PD 参数。
- `grid_resolution`：CVT 计算的网格近似分辨率。
- `yaw.mode`：`face_velocity` 或 `fixed`。
- `centroid_tolerance_m`：完成判定容差。
- 完成后的可选 hold 行为。

`yaw.mode` is validated; typos such as `face_veloci` are rejected during config loading.

`yaw.mode` 会被校验；例如 `face_veloci` 这类拼写错误会在配置加载时报错。

## `configs/robots.yaml`

RoboMaster robot identities and hardware command settings.

RoboMaster 机器人身份和硬件命令设置。

- `robots.expected_count`: optional consistency check for the robot list length.
- `robots.list`: robot_id, SN, group, rigid-body mapping, chassis/gimbal switches.
- `connection`: SDK connection type, protocol, retry policy, and SN requirement.
- `chassis`: angular units, SDK drive timeout, and stop-on-exit behavior.
- `gimbal.angle_status`: gimbal angle callback subscription.
- `gimbal.init_zero_on_connect`: optional gimbal zeroing after connection.
- `watchdog`: command timeout safety stop.

- `robots.expected_count`：机器人列表长度一致性检查。
- `robots.list`：robot_id、SN、分组、刚体映射、底盘/云台开关。
- `connection`：SDK 连接类型、协议、重试策略和 SN 要求。
- `chassis`：角速度单位、SDK drive timeout 和退出停车行为。
- `gimbal.angle_status`：云台角度 callback 订阅。
- `gimbal.init_zero_on_connect`：连接成功后的可选云台回零。
- `watchdog`：命令超时安全停车。

Notes:

说明：

- `group` is currently metadata for humans/future grouping; it does not affect control logic.
- Repeated effective zero commands are filtered in the robot adapter, while `stop_all(force=True)` always sends forced zero commands.

- `group` 当前是人工/未来分组元数据，不影响控制逻辑。
- 重复的等效零命令会在 robot adapter 中过滤；`stop_all(force=True)` 始终强制发送零命令。

## `configs/optitrack.yaml`

OptiTrack/Motive input behavior.

OptiTrack/Motive 输入行为。

- `natnet`: server/client addresses, connection type, stream type, ports, connection check timeout, and Python client path.
- `tracking_validation`: optional OptiTrack-side validation.
- `state_estimation.enable_velocity_estimation`: velocity estimation from adjacent poses.
- `publish.publish_untracked`: whether untracked rigid bodies are published when available.
- `diagnostics`: rigid-body diagnostic logging.

- `natnet`：server/client 地址、连接类型、stream type、端口、连接检查超时和 Python client 路径。
- `tracking_validation`：可选 OptiTrack 侧 tracking 校验。
- `state_estimation.enable_velocity_estimation`：根据相邻位姿估计速度。
- `publish.publish_untracked`：可用时是否发布 untracked 刚体。
- `diagnostics`：刚体诊断日志。

When `tracking_validation.enabled` is false, jump rejection and validation timeout settings are read but not applied.

当 `tracking_validation.enabled` 为 false 时，跳变过滤和 validation timeout 参数会被读取，但不会实际参与校验。

## `configs/supervisor.yaml`

One-command startup and shutdown orchestration.

一键启动和关闭编排。

- `use_optitrack`, `use_robot`, `use_controller`: module enable switches.
- `*_ready_timeout_s`: startup ready wait time.
- `shutdown_timeout_s`: graceful stop wait time before force kill.

- `use_optitrack`、`use_robot`、`use_controller`：模块启用开关。
- `*_ready_timeout_s`：启动 ready 等待时间。
- `shutdown_timeout_s`：优雅停止等待时间，超时后强制结束。

Supervisor startup is lifecycle-managed. Manual three-process startup is not lifecycle-managed; controller exit does not stop robot/optitrack.

Supervisor 一键启动是生命周期托管模式。手动三进程启动不是生命周期托管模式；controller 退出不会自动停止 robot/optitrack。

## `configs/logging.yaml`

Logging level, console/file output, and log format.

日志等级、控制台/文件输出和日志格式。

Each module writes to its own log file under `logs/` when `log_to_file` is true.

`log_to_file` 为 true 时，每个模块会在 `logs/` 下写入独立日志文件。

## Validation / 配置校验

The config loader validates important enums and booleans. Boolean fields must be real YAML booleans (`true` or `false`), not strings such as `"false"`.

配置加载器会校验重要枚举和值为 bool 的字段。bool 字段必须是真正的 YAML 布尔值（`true` 或 `false`），不能写成 `"false"` 这类字符串。
