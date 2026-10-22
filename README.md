# DBACT：边界感知部署与协作搬运

研究机器人如何从局部边界观测形成安全部署配置，并在接触后协作搬运未知形状物体。当前开发框架、历史实验、可靠性诊断、文献原件与论文草稿已于 **2026-10-23** 整合在此项目目录。

## 环境与快速开始

**一个项目、一个根目录 `.venv`，MAS 平台也共用它。** 使用独立 Python 3.12 创建环境。迁移到另一台电脑后重新创建 `.venv`，依赖和数据随项目保存。

从项目根目录运行 PowerShell：

```powershell
.\scripts\setup_env.ps1
.\.venv\Scripts\python.exe -m dbact_sim --help
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --dry-run
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/my_smoke
.\.venv\Scripts\python.exe -m pytest -q
```

已有独立 Python 但未被找到时：`.\scripts\setup_env.ps1 -Python C:\path\to\python.exe`。
脚本按 `requirements.lock.txt` 安装已锁定版本，再将本项目安装为可编辑包。主动更新依赖使用 `-UpdateDependencies`，之后重新生成锁文件并验证。

Linux / macOS：

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock.txt
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m dbact_sim --help
.venv/bin/python -m pytest -q
```

锁文件覆盖仿真、QP 交叉核验、独立物理引擎、分析、视频、文献读取和 MAS 消息依赖。真实 RoboMaster SDK 另外安装到同一 `.venv`，见 [MAS 入口](platforms/mas_public/README.md)。本机已建立并验证 Windows Python 3.12 环境；其他系统需在各自机器上创建环境并验证。

五个 smoke 案例检查执行管线，不用于判断收敛或论文性能。每次新运行使用新输出目录；只有源码、环境、协议和产品身份一致时才能 `--resume`。

## 项目布局

| 目录 / 文件 | 内容 |
| --- | --- |
| `src/dbact/` | `sensing` 感知、`deployment` 部署、`safety` 安全约束、`control` 控制与阶段逻辑 |
| `src/dbact_sim/` | 仿真、独立评价、实验运行、统计和可视化 |
| `src/mas_adapter/`、`platforms/mas_public/` | MAS / RoboMaster / OptiTrack 桥接及平台 |
| `configs/`、`experiments/ccta2027/` | 仿真与平台配置、实验设计、会议要求快照 |
| `scripts/`、`tests/` | 命令入口、维护和文献辅助脚本、回归测试 |
| `runs/` | 完整本地运行数据，包括原项目数据和恢复的 `reliability/` |
| `docs/results/` | 保留版本边界的精简研究证据和清单 |
| `docs/literature_review_2026-10-22/` | 71 份阅读记录、能力矩阵与研究定位工作稿 |
| `data/literature/2026-10-22/` | 70 个 PDF、1 个 DOCX、提取全文、分块及原页图像 |
| `manuscript/ccta2027/`、`theory/static_deployment/` | 英文论文 TeX/PDF 草稿、静态推导和适用范围 |
| `artifacts/`、`tools/tectonic/` | 图表、历史渲染和验证记录、迁移备份、随转移保留的 TeX 工具 |
| `.venv/`、`requirements.lock.txt` | 当前机器独立环境、锁定的第三方依赖 |

核心控制不导入仿真评价模块；旧模块路径保留兼容别名。[架构说明](docs/ARCHITECTURE.md)与[数据索引](docs/DATA_INDEX.md)给出职责和位置。

## 常用工作流

以下命令均使用 `.\.venv\Scripts\python.exe -m dbact_sim <命令>`：

| 命令 | 用途 |
| --- | --- |
| `simulate` | 单次仿真、回放与调试 |
| `deployment` | 配对静态部署或搬运案例 |
| `study` | 协议校验、多进程实验、结果收据与恢复 |
| `evidence` | 从已保存结果生成统计和图表 |
| `campaign` | 冻结、初始化预检与完整矩阵审计 |

```powershell
.\.venv\Scripts\python.exe -m dbact_sim simulate --config configs/sim/d/l_shape_closed_loop.yaml --seed 2 --steps 500 --no-figures --output runs/my_example
.\.venv\Scripts\python.exe -m dbact_sim evidence --batch smoke=runs/my_smoke --out runs/my_smoke_evidence
```

完整实验步骤见[实验入口](experiments/ccta2027/README.md)。大型原始数据、参考全文、备份与环境保存在本地并由 Git 忽略；仅克隆代码仓库不会得到这些本地文件。跨电脑完整迁移需复制整个项目的数据目录，或按迁移清单恢复。

## 已保存的研究证据

本次整合采用开发提交 `568b1fc`，它继承原项目 `e4b0a19`，再加入项目内 `.venv`、数据恢复和仓库健康检查。`main` 与 `paper/boundary-aware-safe-deployment` 统一使用重建后的最新框架；旧分支保存在归档标签中。同步规则、CI 与一致性检查见[仓库健康说明](docs/REPOSITORY_HEALTH.md)，历史来源见[重建记录](docs/REBUILD_2026-10-23.md)。

- **2026-10-21 冻结实验：169/169 案例、零执行错误。** 历史实现回归为 616 通过、2 跳过。
- **2026-10-22 可靠性修订：**保留 140 个搬运案例分类、失败候选与默认修复；默认 CVT 开发完成数仍为 5/9。其历史回归为 629 通过、2 跳过，832 案例前瞻设计尚未执行。
- 本次环境与工作区验证单独记录在[重建验证收据](docs/migrations/2026-10-23/workspace_verification.json)，不会改写历史实验归属。

| 冻结实验 | Boundary CVT / 分段候选 | 对照 |
| --- | --- | --- |
| 静态路径安全 | 分段候选 9/9，最小间距 66.04 mm | 原局部样本 1/9；oracle 9/9；要求 65 mm |
| 开发集搬运完成 | CVT 5/9 | 最近观测目标 5/9；局部间距 4/9 |
| 新种子泛化搬运完成 | CVT 5/12 | 最近观测目标 3/12；局部间距 1/12 |
| 扰动鲁棒性搬运完成 | CVT 1/18 | 最近观测目标 2/18；局部间距 1/18 |

静态结论限于固定多边形与完整控制保持段，不能推出运动接触安全。重复使用种子的汇总仅作描述；鲁棒搬运、独立引擎一致性和实机性能仍待解决。[冻结实验报告](docs/CCTA2027_EXPERIMENTS.md)和[可靠性修订报告](docs/RELIABILITY_REVISION_2026-10-22.md)保留失败及适用范围。

## 阅读入口与追溯

- [完整结果索引](docs/results/README.md)、[数据位置与完整性](docs/DATA_INDEX.md)
- [论文草稿](manuscript/ccta2027/README.md)、[文献阅读与研究定位](docs/literature_review_2026-10-22/README.md)
- [静态分析](theory/static_deployment/main_theorem.md)、[适用范围补充](theory/static_deployment/applicability_and_residual.md)
- [2026-10-23 重建记录](docs/REBUILD_2026-10-23.md)、[逐文件迁移清单](docs/migrations/2026-10-23/transfer_mapping.jsonl)
- [环境约定](AGENTS.md)、[环境安装脚本](scripts/setup_env.ps1)、[MIT License](LICENSE)
- [仓库同步与健康检查](docs/REPOSITORY_HEALTH.md)、[远程 CI](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml)

<!-- CONSOLIDATION_RESULTS_START -->
历史 Gate 0–6 汇总见 [consolidation](docs/results/consolidation/results.md)，更早机制探索见[历史闭环说明](docs/HISTORICAL_CLOSED_LOOP_README.md)；它们对应当时源码。
<!-- CONSOLIDATION_RESULTS_END -->
