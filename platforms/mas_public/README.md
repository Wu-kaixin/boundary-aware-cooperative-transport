# MAS S1 平台与 DBACT 接口

此目录是随 DBACT 仓库保留的 MAS 实验平台，面向 RoboMaster S1 与 OptiTrack/Motive。
平台包含消息传输、控制器、机器人通信、记录和多进程管理；论文仿真使用仓库根目录的
`dbact_sim`，两者入口和配置目录不同。

## 从哪里运行

从本仓库根目录进入平台，不需要另外克隆 MAS：

```powershell
cd platforms/mas_public
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python -m pip install -e ../..
```

Python 要求见本目录 [pyproject.toml](pyproject.toml)。
平台依赖还包含 RoboMaster SDK；根目录的仿真依赖不能替代它们。

## 配置与程序

| 路径 | 作用 |
| --- | --- |
| `configs/controller.yaml` | 当前选择 `dtransport`；控制器与记录开关 |
| `configs/controllers/` | `manual`、`point`、`cvt`、`dtransport` 参数 |
| `configs/robots.yaml` | 机器人身份、刚体映射和 SDK 设置 |
| `configs/optitrack.yaml` | NatNet 地址及跟踪设置 |
| `configs/system.yaml` | 坐标系、速度限幅、工作区域与超时 |
| `configs/supervisor.yaml` | 子进程开关与生命周期 |
| `apps/run_supervisor.py` | 多进程启动与关闭 |
| `apps/run_controller.py` | 单独运行控制器 |
| `apps/run_optitrack.py`、`apps/run_robot_comm.py` | 动捕与机器人通信进程 |
| `src/controller/decentralized_transport_controller.py` | DBACT 适配入口 |

实机 NatNet Python 客户端放在 `third_party/natnet_client/`，其准备方式见
[硬件配置](docs/hardware_setup.md)。DBACT 对象观测与适配背景见
[MAS 集成说明](../../docs/MAS_INTEGRATION.md)；该指南包含移植到独立 MAS 副本的步骤，
本目录已集成的文件应先检查现状再操作。

## 无硬件通信检查

从平台目录在三个终端分别运行：

```powershell
python apps/manual_tests/mock_optitrack.py
python apps/run_controller.py
python apps/manual_tests/mock_robot.py
```

这是三个独立进程的命令，不能在同一终端顺序等待完成。
Mock 只验证消息与控制链路，不能验证真实物体观测或论文的搬运性能。
实际控制器类型及虚拟物体设置以 `configs/` 为准。

## 检查、记录与硬件运行

```powershell
python -m pytest -q
python -m ruff check .
python -m pip check
```

以上是平台自己的检查命令，不能与根目录 DBACT 的测试计数混为一谈。
运行记录写入 `data/experiments/<experiment_dir>/`，包含状态、控制命令、配置快照和图表。
使用 `apps/check_experiment.py` 与 `apps/plot_experiment.py` 检查或绘制指定回合。

实机启动前按[操作与调试指南](docs/usage_and_debug.md)检查网络、跟踪、坐标、限幅与停止链路。
准备好物理急停；`Ctrl+C` 和软件 watchdog 不能替代它。
配置核实后从平台目录运行 `python apps/run_supervisor.py`。
本轮 README 整理没有执行机器人硬件动作，也没有新增实机验证结果。

## 资料与来源

- [配置说明](docs/config_description.md)
- [操作与调试](docs/usage_and_debug.md)
- [硬件配置](docs/hardware_setup.md)
- [上游 MAS](https://github.com/qiubinquan/MAS)
- [RoboMaster SDK 参考](https://github.com/jeguzzi/RoboMaster-SDK)
- [返回 DBACT](../../README.md)
