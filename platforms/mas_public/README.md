# MAS S1 System

MAS S1 System is a laboratory multi-robot closed-loop experiment framework for RoboMaster S1 robots with OptiTrack/Motive tracking.

MAS S1 System 是一个面向 RoboMaster S1 与 OptiTrack/Motive 的实验室多机器人闭环实验框架。

## Purpose / 项目作用

This project is designed for laboratory multi-agent and controller-algorithm verification. It integrates motion-capture state acquisition, controller algorithms, robot velocity control, experiment data recording, runtime logs, experiment checking, and offline plotting into one runnable system.

本项目主要用于实验室多智能体及相关控制算法验证，融合动捕状态获取、控制器算法、机器人速度控制、实验数据记录、运行日志、实验检查和实验绘图等多功能为一体。

Core runtime chain:

核心运行链路：

```text
Motive GUI
  -> NatNet
  -> OptiTrack Module
  -> WorldState
  -> Controller Module
  -> ControlCommand
  -> Robot Module
  -> RoboMaster S1
```

## Features / 功能

- Multi-process runtime with one-command Supervisor startup.
- Manual startup for three-process experiments: OptiTrack, Robot, and Controller.
- Manual startup for two-process experiments: Robot and Controller, when OptiTrack is disabled.
- Supports the expansion of the number of robots.
- ZeroMQ-based messaging layer with typed JSON messages.
- Implemented controllers: `manual`, `point`, and `cvt`.
- Optional OptiTrack tracking validation and velocity estimation.
- Optional Z-up coordinate transform, raw-state smoothing, robot command transform, gimbal yaw-follow, and pitch-hold logic.
- Robot command limiting, watchdog stop, shutdown stop, untracked stop, and world-bound stop logic.
- Experiment CSV recording, runtime logs, config snapshot, experiment check, and offline plotting.
- Mock OptiTrack and mock Robot scripts for no-hardware closed-loop checks.

- 支持 Supervisor 一键启动的多进程运行架构。
- 支持 OptiTrack、Robot、Controller 三进程手动启动。
- 当关闭 OptiTrack 时，支持 Robot、Controller 两进程手动启动。
- 支持拓展机器人数量。
- 基于 ZeroMQ 的 messaging 通信层，使用统一 JSON 消息。
- 已实现 `manual`、`point`、`cvt` 三类控制器。
- 支持可选 OptiTrack tracking validation 和速度估计。
- 支持可选 Z-up 坐标转换、原始状态平滑、机器人命令转换、云台 yaw 跟随和 pitch 保持。
- 支持机器人命令限幅、watchdog 停车、shutdown 停车、untracked 停车和世界边界停车。
- 支持实验 CSV 记录、运行日志、配置快照、实验检查和离线绘图。
- 提供 mock OptiTrack 和 mock Robot 脚本，可在无硬件时检查闭环链路。

## Hardware / 硬件依赖

Required for real experiments:

实机实验需要：

- RoboMaster S1 robots.
- OptiTrack cameras and Motive.
- A Windows computer running this project. The current project has been tested on Windows 11.
- RoboMaster S1 and the computer must be on the same local network.
- The camera/Motive computer should use wired network communication for OptiTrack.
- If Motive and this project run on the same computer, use the matching NatNet unicast setup.
- A prepared physical emergency-stop method. `Ctrl+C` can stop software processes, but it is not a replacement for physical safety.

- RoboMaster S1 机器人。
- OptiTrack 相机系统和 Motive 软件。
- 运行本项目的 Windows 电脑。当前项目已在 Windows 11 环境中测试。
- RoboMaster S1 需要和电脑处于同一局域网。
- OptiTrack 相机/Motive 电脑建议使用有线网络通信。
- 如果 Motive 和本项目运行在同一台电脑上，需要使用匹配的 NatNet unicast 设置。
- 需要准备物理急停或人工安全停止手段。`Ctrl+C` 可停止软件进程，但不能替代物理安全措施。

The project can also run mock closed-loop checks without real hardware.

无真实硬件时，仍可运行 mock 闭环检查。

## Documentation / 文档说明

- [docs/config_description.md](docs/config_description.md): configuration file guide.
- [docs/usage_and_debug.md](docs/usage_and_debug.md): usage, startup modes, safety switches, data recording, plotting, checking, and debugging workflow.
- [docs/hardware_setup.md](docs/hardware_setup.md): hardware setup checklist.

- [docs/config_description.md](docs/config_description.md)：配置文件说明。
- [docs/usage_and_debug.md](docs/usage_and_debug.md)：代码使用、启动方式、安全开关、数据记录、绘图、检查和调试流程说明。
- [docs/hardware_setup.md](docs/hardware_setup.md)：硬件配置检查清单。

## Project Structure / 项目结构

```text
apps/
  run_optitrack.py       Start OptiTrack Module / 启动 OptiTrack 模块
  run_robot_comm.py      Start Robot Module / 启动 Robot 模块
  run_controller.py      Start Controller Module / 启动 Controller 模块
  run_supervisor.py      One-command orchestration / 一键启动编排
  plot_experiment.py     Offline plotting / 离线绘图
  check_experiment.py    Experiment output check / 实验结果检查
  manual_tests/          Manual debug scripts / 手动联调脚本
  pytest_tests/          Automated tests / 自动化测试

configs/
  system.yaml            System behavior, frequency, bounds, limits, transforms / 系统行为、频率、边界、限幅、转换
  controller.yaml        Controller selection and runtime options / 控制器选择和运行选项
  controllers/           Per-controller algorithm parameters / 各控制器算法参数
  robots.yaml            Robot list, SN, rigid-body mapping, SDK settings / 机器人、SN、刚体映射、SDK 设置
  optitrack.yaml         NatNet and tracking validation / NatNet 与 tracking validation
  supervisor.yaml        Module switches and timeouts / 模块开关和超时
  logging.yaml           Logging config / 日志配置

src/
  common/                Shared messages, config, time, math, logging / 公共消息、配置、时间、数学、日志
  messaging/             Messaging abstraction and ZeroMQ backend / 通信抽象和 ZeroMQ 实现
  optitrack/             NatNet to WorldState / NatNet 到 WorldState
  controller/            Controllers, recording, plotting, bounds / 控制器、记录、绘图、边界
  robot/                 RoboMaster adapter and command handling / RoboMaster 适配和命令处理
  supervisor/            Multi-process orchestration / 多进程编排

docs/
  assets/coordinate_frames.png
  config_description.md
  usage_and_debug.md
  hardware_setup.md

data/
  experiments/           Experiment outputs / 实验输出

logs/
  Runtime logs / 运行日志
```

## Quick Commands / 快速命令

The following commands assume a Windows PowerShell environment. Run them from the project root.

以下命令默认在 Windows PowerShell 中执行，并且需要在项目根目录下运行。

### 1. Download or Copy / 下载或复制代码

```powershell
git clone https://github.com/qiubinquan/MAS
cd MAS
```

If the project has already been copied to the computer:

如果项目已经复制到电脑上：

```powershell
cd path\to\MAS
```

### 2. Create Environment / 创建环境

Recommended Python version: `3.11`
Tested Python version: `3.11.9`

推荐 Python 版本：`3.11`
实测 Python 版本：`3.11.9`

```powershell
python --version
python -m venv .venv
.\.venv\Scripts\Activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

`requirements.txt` includes runtime dependencies, RoboMaster hardware dependencies, and release-check tools.

`requirements.txt` 同时包含运行依赖、RoboMaster 实机依赖和发布检查工具。

### 3. Verify Installation / 检查安装

```powershell
python -m pytest -q
python -m ruff check .
python -m compileall -q src apps
python -m pip check
```

### 4. Import NatNet Python Client / 导入 NatNet Python Client

Download the NatNet SDK from the OptiTrack downloads page:

请先从 OptiTrack 下载页面下载 NatNet SDK：

```text
https://optitrack.com/support/downloads
```

After installing or extracting the SDK, copy the Python client `.py` files from:

安装或解压 SDK 后，将 Python client 的 `.py` 文件从以下目录复制到本项目：

```text
NatNetSDK\Samples\PythonClient
```

Target folder:

目标文件夹：

```text
MAS\third_party\natnet_client
```

Copy all `.py` files provided by your SDK version. The files required by MAS include `NatNetClient.py`, `DataDescriptions.py`, and `MoCapData.py`; if your SDK also provides a sample entry file such as `PythonSample.py`, copy it as well.

请复制当前 SDK 版本提供的所有 `.py` 文件。MAS 运行所需文件包括 `NatNetClient.py`、`DataDescriptions.py` 和 `MoCapData.py`；如果你的 SDK 还提供 `PythonSample.py` 等示例入口文件，也一并复制。

### 5. Configure Before Running / 运行前配置

Before real experiments, check these files:

实机运行前，请先检查以下配置文件：

```text
configs/controller.yaml      # controller type, robot mode, recording, plotting
configs/controllers/*.yaml   # controller algorithm parameters
configs/supervisor.yaml      # enable/disable OptiTrack, Robot, Controller
configs/robots.yaml          # robot IDs, SNs, rigid-body mapping, SDK settings
configs/optitrack.yaml       # Motive/NatNet IPs and tracking validation
configs/system.yaml          # frequency, bounds, limits, coordinate and command transforms
```

Default manual controller selection:

默认 manual 控制器选择：

```yaml
controller:
  type: manual
  robot_mode: free
```

Controller parameters are loaded automatically from:

控制器参数会自动从以下文件加载：

```text
configs/controllers/manual.yaml
configs/controllers/point.yaml
configs/controllers/cvt.yaml
```

### 6. Mock Closed Loop / 无硬件闭环测试

Use this when RoboMaster or OptiTrack hardware is unavailable.

无 RoboMaster 或 OptiTrack 硬件时，可用以下方式测试通信闭环。

```powershell
# Terminal 1
python apps/manual_tests/mock_optitrack.py
```

```powershell
# Terminal 2
python apps/run_controller.py
```

```powershell
# Terminal 3
python apps/manual_tests/mock_robot.py
```

Expected chain:

预期链路：

```text
mock_optitrack -> controller -> mock_robot
```

### 7. Manual Real-Hardware Startup / 实机手动启动

Before running these commands, prepare RoboMaster robots, Motive, OptiTrack rigid bodies, network settings, and a physical emergency-stop method.

运行以下命令前，请先准备 RoboMaster 机器人、Motive、OptiTrack 刚体、网络设置和物理急停手段。

```powershell
# Terminal 1
python apps/run_optitrack.py
```

```powershell
# Terminal 2
python apps/run_robot_comm.py
```

```powershell
# Terminal 3
python apps/run_controller.py
```

Recommended startup order:

推荐启动顺序：

```text
OptiTrack -> Robot -> Controller
```

In manual module startup, controller completion does not automatically stop Robot or OptiTrack.

手动分模块启动时，controller 完成不会自动停止 Robot 或 OptiTrack。

### 8. One-Command Startup / 一键启动

After the manual startup workflow is verified, use Supervisor for one-command startup. Supervisor is recommended for regular experiments because it provides managed startup/shutdown and complete status recording.

手动启动流程确认无误后，可使用 Supervisor 一键启动。常规实验推荐使用 Supervisor 一键启动，因为它具备托管启动/关闭和完整状态记录。

```powershell
python apps/run_supervisor.py
```

Supervisor starts enabled modules according to `configs/supervisor.yaml`.

Supervisor 会根据 `configs/supervisor.yaml` 启动已启用的模块。

### 9. Check and Plot Experiment / 检查和绘制实验结果

Manual experiment check:

手动检查实验：

```powershell
python apps/check_experiment.py data/experiments/<experiment_dir>
```

Manual plotting:

手动绘图：

```powershell
python apps/plot_experiment.py data/experiments/<experiment_dir>
```

Example:

示例：

```powershell
python apps/check_experiment.py data/experiments/20260520_175715_cvt
python apps/plot_experiment.py data/experiments/20260520_175715_cvt
```

Plots are saved under:

绘图结果会保存到：

```text
data/experiments/<experiment_dir>/plots/
```

### 10. Experiment Outputs / 实验输出

When `recording.enable: true`, experiment data is saved under:

当 `recording.enable: true` 时，实验数据会保存到：

```text
data/experiments/YYYYMMDD_HHMMSS_experiment_name/
```

Main output files:

主要输出文件：

```text
world_state.csv       # raw Motive/controller-sampled state / 原始 Motive/controller 采样状态
world_state_zup.csv   # Z-up/control-frame state / Z-up 控制坐标系状态
control_command.csv   # published controller commands, including shutdown / 已发布控制命令，包含 shutdown
system_status.csv     # module lifecycle status / 模块生命周期状态
robot_status.csv      # robot/gimbal feedback status / 机器人和云台反馈状态
config_snapshot/      # copied configs for this experiment / 本次实验配置快照
plots/                # generated figures / 生成的图
```

## Safety Notes / 安全说明

- `manual` mode can send commands without OptiTrack data. If bounds checking is required, make sure OptiTrack is running and `WorldState` is received.
- `Ctrl+C` triggers software shutdown and stop commands, but it is not a replacement for physical safety.
- `control_command.csv` includes final `controller_mode=shutdown` rows to record the final stop command.
- `system_status.csv` records the primary final status; failed rows may include `prior_events` for earlier safety events.

- `manual` 模式即使没有 OptiTrack 数据也能发送命令。如果需要越界判断，必须确认 OptiTrack 正在运行且 controller 收到 `WorldState`。
- `Ctrl+C` 会触发软件关闭和停车命令，但不能替代物理安全措施。
- `control_command.csv` 会包含最终 `controller_mode=shutdown` 行，用于记录最终停车命令。
- `system_status.csv` 记录主要最终状态；failed 行可能包含此前安全事件的 `prior_events`。

## Acknowledgements / 致谢

Thanks to the author of [jeguzzi/RoboMaster-SDK](https://github.com/jeguzzi/RoboMaster-SDK) for technical support and reference work around RoboMaster SDK usage.

感谢 [jeguzzi/RoboMaster-SDK](https://github.com/jeguzzi/RoboMaster-SDK) 项目作者在 RoboMaster SDK 使用方面提供的技术支持和参考工作。
