# Usage and Debug

This document describes daily setup, startup modes, key safety switches, experiment data, plotting, and common debug cases for the MAS RoboMaster S1 project.

本文档说明 MAS RoboMaster S1 项目的日常环境准备、启动方式、关键安全开关、实验数据、绘图和常见调试情况。

## Code Setup / 代码环境

```powershell
git clone https://github.com/qiubinquan/MAS.git
cd MAS
python --version
python -m venv .venv
.\.venv\Scripts\Activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Use Python 3.10 or newer. The current development environment uses Python 3.11.

建议使用 Python 3.10 或更新版本。当前开发环境使用 Python 3.11。

## Before Running / 运行前检查

- Prepare RoboMaster S1 network and battery.
- Start Motive and confirm the expected rigid bodies are tracked.
- Confirm NatNet streaming settings match `configs/optitrack.yaml`.
- Confirm robot IDs, SNs, and rigid-body names/IDs in `configs/robots.yaml`.
- Confirm `configs/controller.yaml`, `configs/system.yaml`, and `configs/supervisor.yaml` match the intended experiment.
- For real robot motion, confirm the physical emergency stop procedure before running.

- 准备 RoboMaster S1 网络和电量。
- 启动 Motive，并确认预期刚体可被跟踪。
- 确认 NatNet streaming 设置与 `configs/optitrack.yaml` 匹配。
- 确认 `configs/robots.yaml` 中的 robot_id、SN、刚体名称/ID 正确。
- 确认 `configs/controller.yaml`、`configs/system.yaml` 和 `configs/supervisor.yaml` 符合本次实验目的。
- 实机运动前确认物理急停流程。

## Coordinate Frames / 坐标系

![Coordinate frames](assets/coordinate_frames.png)

`world_state.csv` records the raw Motive/world-frame state. When `z_up_transform.enabled` is true, `world_state_zup.csv` records the transformed control-frame state. Controller algorithms, world-bounds checks, and trajectory plots use the transformed state when available.

`world_state.csv` 记录原始 Motive/world 坐标状态。开启 `z_up_transform.enabled` 时，`world_state_zup.csv` 记录变换后的控制坐标状态。控制器算法、越界判断和轨迹绘图优先使用变换后的状态。

The experiment CSV records the world state sampled by the controller loop, usually around `controller_hz`, not the full raw OptiTrack publishing rate.

实验 CSV 记录的是 controller 循环采样到的 world state，通常接近 `controller_hz`，不是 OptiTrack 原始发布频率的全量数据。

## Default Manual Mode / 默认 Manual 模式

The current default test configuration can be set to manual mode:

当前默认测试配置可设置为 manual 模式：

```yaml
# configs/controller.yaml
controller:
  type: manual
  robot_mode: free
```

Manual controller parameters are loaded from `configs/controllers/manual.yaml`.

manual 控制器参数从 `configs/controllers/manual.yaml` 自动加载。

Important: `manual` control does not require OptiTrack data to generate commands. If OptiTrack is not running or no `WorldState` is received, `manual` can still send the fixed command from `configs/controllers/manual.yaml`. In that case, trajectory recording and world-bounds stopping do not have tracking data to work from.

注意：`manual` 控制不依赖 OptiTrack 数据生成命令。如果 OptiTrack 未运行或 controller 没收到 `WorldState`，`manual` 仍会按 `configs/controllers/manual.yaml` 发送固定命令。此时轨迹记录和越界停车没有 tracking 数据依据。

## Startup Modes / 启动方式

### Manual Module Startup / 手动分模块启动

Open separate PowerShell terminals after activating the virtual environment.

激活虚拟环境后，分别打开多个 PowerShell 终端。

Three-process real-hardware closed loop:

三进程实机闭环：

```powershell
python apps/run_optitrack.py
python apps/run_robot_comm.py
python apps/run_controller.py
```

Two-process manual robot test without OptiTrack:

不启用 OptiTrack 的两进程 manual 实机测试：

```powershell
python apps/run_robot_comm.py
python apps/run_controller.py
```

In manual module startup, controller completion or failure only exits the controller process. Robot and OptiTrack continue running until manually stopped.

手动分模块启动时，controller 完成或失败只会退出 controller 进程。Robot 和 OptiTrack 会继续运行，直到手动停止。

### One-Command Startup / 一键启动

```powershell
python apps/run_supervisor.py
```

Supervisor starts enabled modules in this order: OptiTrack, Robot, Controller. On shutdown, it records enabled and actually-started module statuses into the latest experiment folder.

Supervisor 按 OptiTrack、Robot、Controller 顺序启动已启用模块。关闭时，它会把已启用且实际启动过的模块状态写入最新实验文件夹。

## Key Switches / 关键开关

- `configs/supervisor.yaml/use_optitrack`: enable or disable the OptiTrack process.
- `configs/supervisor.yaml/use_robot`: enable or disable the RoboMaster process.
- `configs/supervisor.yaml/use_controller`: enable or disable the Controller process.
- `configs/controller.yaml/controller.type`: select `manual`, `point`, or `cvt`.
- `configs/controller.yaml/controller.robot_mode`: select `free`, `chassis_lead`, or `gimbal_lead`.
- `configs/controller.yaml/input.require_all_tracked_for_valid_state`: decide whether one untracked robot invalidates the whole state.
- `configs/system.yaml/experiment.auto_stop_on_untracked`: stop when untracked timeout is reached.
- `configs/system.yaml/experiment.auto_stop_on_task_completed`: supervisor-managed experiments stop after controller completion.
- `configs/system.yaml/world.stop_on_out_of_bounds`: enable or disable bounds stop.
- `configs/system.yaml/worldstate_smoothing.enabled`: enable or disable controller-side world-state smoothing.
- `configs/optitrack.yaml/tracking_validation.enabled`: enable or disable OptiTrack-side validation.

- `configs/supervisor.yaml/use_optitrack`：启用或关闭 OptiTrack 进程。
- `configs/supervisor.yaml/use_robot`：启用或关闭 RoboMaster 进程。
- `configs/supervisor.yaml/use_controller`：启用或关闭 Controller 进程。
- `configs/controller.yaml/controller.type`：选择 `manual`、`point` 或 `cvt`。
- `configs/controller.yaml/controller.robot_mode`：选择 `free`、`chassis_lead` 或 `gimbal_lead`。
- `configs/controller.yaml/input.require_all_tracked_for_valid_state`：决定一个机器人 untracked 时是否让整体状态失效。
- `configs/system.yaml/experiment.auto_stop_on_untracked`：untracked 超时后停止。
- `configs/system.yaml/experiment.auto_stop_on_task_completed`：supervisor 管理的一键实验在 controller 完成后停止。
- `configs/system.yaml/world.stop_on_out_of_bounds`：启用或关闭越界停车。
- `configs/system.yaml/worldstate_smoothing.enabled`：启用或关闭 controller 侧 world-state 平滑。
- `configs/optitrack.yaml/tracking_validation.enabled`：启用或关闭 OptiTrack 侧 tracking 校验。

## Tracking Safety / Tracking 安全

The following switches must be considered together:

以下开关需要一起考虑：

```yaml
# configs/controller.yaml
input:
  require_all_tracked_for_valid_state: false

# configs/system.yaml
experiment:
  auto_stop_on_untracked: true
  untracked_timeout_s: 3.0
```

`require_all_tracked_for_valid_state` controls whether a `WorldState` is considered valid when any robot is untracked. `auto_stop_on_untracked` controls whether a robot that remains untracked for `untracked_timeout_s` causes `tracking_lost` and global stop.

`require_all_tracked_for_valid_state` 控制任意机器人 untracked 时，整个 `WorldState` 是否仍被视为有效。`auto_stop_on_untracked` 控制机器人持续 untracked 超过 `untracked_timeout_s` 后是否触发 `tracking_lost` 和全局停止。

Recommended default for real point/cvt/manual motion tests:

实机 point/cvt/manual 运动测试推荐默认组合：

```yaml
require_all_tracked_for_valid_state: false
auto_stop_on_untracked: true
```

This keeps tracked robots usable for bounds checking while persistent untracked robots still stop the experiment after the timeout.

这样仍能使用 tracked 机器人做越界判断，同时持续 untracked 的机器人会在超时后停止实验。

Avoid this combination during manual motion tests:

manual 运动测试中避免以下组合：

```yaml
require_all_tracked_for_valid_state: true
auto_stop_on_untracked: false
```

Reason: if any robot becomes untracked, the controller treats the whole state as invalid. In `manual` mode, the manual controller does not require a valid `WorldState`, so it may continue sending the fixed manual command. At the same time, bounds checking may not receive a valid control-frame state, and untracked timeout is disabled.

原因：如果任意机器人变为 untracked，controller 会把整个状态视为无效。`manual` 模式不强制依赖有效 `WorldState`，因此仍可能继续发送固定 manual 命令。同时，越界判断可能拿不到有效控制坐标状态，且 untracked 超时停车被关闭。

## Shutdown and Final Commands / 退出和最终停车命令

Controller shutdown publishes a zero command and records it in `control_command.csv` with:

Controller 退出时会发布零命令，并在 `control_command.csv` 中记录：

```text
controller_mode=shutdown
```

These shutdown rows are final stop records and are not expected to have matching `world_state.csv` rows. `check_experiment.py` ignores `shutdown` rows when comparing world-state and command row counts.

这些 shutdown 行是最终停车记录，不要求在 `world_state.csv` 中有对应行。`check_experiment.py` 比较 world-state 和 command 行数时会忽略 `shutdown` 行。

Ctrl+C keeps the fast safety stop and CSV flush path, but skips automatic plotting and experiment checking. Natural completion/failure still runs plot/check when enabled.

Ctrl+C 保留快速安全停车和 CSV flush，但跳过自动绘图和实验检查。自然 completed/failed 结束时，如果配置开启，仍会自动绘图和检查。

## Experiment Data / 实验数据

Each experiment folder can contain:

每个实验文件夹可能包含：

- `world_state.csv`: raw Motive/world-frame state sampled by controller.
- `world_state_zup.csv`: transformed control-frame state.
- `control_command.csv`: controller output commands, including final `shutdown` rows.
- `system_status.csv`: module status records. Failed controller rows may include `prior_events` for earlier safety events.
- `robot_status.csv`: robot/gimbal/mode status reported by robot module callbacks.
- `plots/`: generated figures.
- `config_snapshot/`: copied configuration files. Only the selected controller parameter file is copied under `controllers/`.

- `world_state.csv`：controller 采样到的原始 Motive/world 坐标状态。
- `world_state_zup.csv`：变换后的控制坐标状态。
- `control_command.csv`：controller 输出命令，包含最终 `shutdown` 行。
- `system_status.csv`：模块状态记录。controller failed 行可能包含此前安全事件的 `prior_events`。
- `robot_status.csv`：robot 模块 callback 报告的机器人/云台/模式状态。
- `plots/`：生成的图。
- `config_snapshot/`：配置快照。`controllers/` 下只复制当前选中的控制器参数文件。

## Plotting and Checking / 绘图和检查

Manual plotting:

手动绘图：

```powershell
python apps/plot_experiment.py data/experiments/<experiment_folder>
```

Manual experiment check:

手动检查实验数据：

```powershell
python apps/check_experiment.py data/experiments/<experiment_folder>
```

`check_experiment.py` verifies file format and closure. It does not prove the experiment succeeded physically; use plots, `system_status.csv`, `control_command.csv`, and logs together.

`check_experiment.py` 检查文件格式和状态闭合。它不代表物理实验一定成功，需要结合图、`system_status.csv`、`control_command.csv` 和日志判断。

## Common Debug Cases / 常见调试情况

- Config error: check `configs/controller.yaml` and the selected `configs/controllers/{type}.yaml` first.
- No OptiTrack data: confirm Motive streaming, NatNet IP/ports, and rigid-body mapping.
- Manual keeps moving without OptiTrack: expected behavior; manual does not require `WorldState`.
- `tracking_lost`: a robot stayed untracked longer than `untracked_timeout_s`.
- `world_out_of_bounds`: a tracked robot stayed outside configured bounds longer than `out_of_bounds_fail_delay_s`.
- Robot connection failure: check STA network, SN, SDK dependency, and battery.
- No automatic plots after Ctrl+C: expected behavior; run `apps/plot_experiment.py` manually if needed.
- `robot_status.csv` is not a fixed-rate heartbeat; it records pending robot status callbacks such as gimbal angle and robot mode.

- 配置错误：优先检查 `configs/controller.yaml` 和当前控制器的 `configs/controllers/{type}.yaml`。
- 没有 OptiTrack 数据：确认 Motive streaming、NatNet IP/端口和刚体映射。
- manual 在没有 OptiTrack 时仍运动：这是预期行为，manual 不依赖 `WorldState`。
- `tracking_lost`：机器人 untracked 持续超过 `untracked_timeout_s`。
- `world_out_of_bounds`：tracked 机器人越界持续超过 `out_of_bounds_fail_delay_s`。
- 机器人连接失败：检查 STA 网络、SN、SDK 依赖和电量。
- Ctrl+C 后没有自动绘图：这是预期行为，需要时手动运行 `apps/plot_experiment.py`。
- `robot_status.csv` 不是固定频率心跳；它记录云台角度、机器人模式等 pending callback 状态。

## Extension Notes / 扩展说明

Keep module boundaries clear:

保持模块边界清楚：

When extending project code, use AI coding tools as much as possible while still checking changes against the current module structure.

扩展项目代码时，尽量利用 AI 工具辅助开发，同时注意结合当前模块结构核查改动。

- OptiTrack/Vision modules publish perception state.
- Controller consumes state and produces `control_command`.
- Robot module executes commands and publishes hardware status.
- Supervisor manages process lifecycle only.
- Recorder/checker/plotter maintain experiment data closure.

- OptiTrack/Vision 模块发布感知状态。
- Controller 消费状态并生成 `control_command`。
- Robot 模块执行命令并发布硬件状态。
- Supervisor 只管理进程生命周期。
- Recorder/checker/plotter 负责实验数据闭环。

When adding a new controller, add the controller class, `configs/controllers/{type}.yaml`, config validation, tests, and plot/check updates if the output format changes.

新增控制器时，需要添加控制器类、`configs/controllers/{type}.yaml`、配置校验、测试；如果输出格式变化，还要同步更新 plot/check。

