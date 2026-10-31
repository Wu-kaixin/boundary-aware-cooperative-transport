# DBACT：边界感知部署与协作搬运

**Decentralized Boundary-Aware Cooperative Transportation** 研究多机器人如何利用局部边界观测，围绕未知形状物体完成部署，并在接触后协作搬运。本仓库提供感知与局部地图、边界密度/CVT 部署、安全 QP、控制阶段、仿真与独立评价，以及可追溯的实验运行和分析工具。

这是持续研究中的科研软件。静态部署分析有明确适用条件；动态接触、鲁棒搬运及实机性能仍有未解决问题。程序执行完成与科研任务成功分别记录，失败案例保留在结果中。

当前研究与维护入口为 [`paper/boundary-aware-safe-deployment`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment)；`main` 保留已有基线。历史分支成果通过原归档标签和本地备份追溯。两个活动分支不要求指向同一提交，具体状态以 Git 为准。

**阅读入口：** [架构](docs/ARCHITECTURE.md) · [实验协议](experiments/ccta2027/README.md) · [冻结结果](docs/results/README.md) · [数据索引](docs/DATA_INDEX.md) · [论文稿件](manuscript/ccta2027/README.md) · [仓库指南](docs/REPOSITORY_GUIDE.md)

## 环境与快速开始

**一个项目、一个根目录 `.venv`，MAS 平台共用。** 可复现环境使用独立 Python 3.12 和 `requirements.lock.txt`；本机维护验证使用 Python 3.12.14。包元数据中的最低 Python 版本不是对所有版本均已完成验证的声明。

以下命令从现有 Git 根目录执行。已有可用 `.venv` 时直接使用；首次安装或在新机器上建立环境时运行安装脚本，不复制旧机器的虚拟环境，不借用其他项目的依赖。

从项目根目录运行 PowerShell：

```powershell
.\scripts\setup_env.ps1
.\.venv\Scripts\python.exe -m dbact_sim --help
.\.venv\Scripts\python.exe -m pip check
```

已有独立 Python 但未被找到时：`.\scripts\setup_env.ps1 -Python C:\path\to\python.exe`。
脚本按原锁文件安装依赖，再以 `--no-deps -e .` 安装本项目。正常复现不使用 `-UpdateDependencies`；依赖升级应作为独立变更审查和验证。

Linux / macOS：

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock.txt
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m dbact_sim --help
.venv/bin/python -m pytest -q
```

锁文件覆盖仿真、QP 交叉核验、独立接触引擎、分析、视频、文献读取和 MAS 消息依赖。Linux/macOS 也必须使用各自新建的根环境，不能直接运行 Windows 环境。真实 RoboMaster SDK 与硬件配置另见 [MAS 平台](platforms/mas_public/README.md)；下列流程均为离线操作。

### 五例短 smoke 与结果分析

使用新的输出目录，逐条执行，任何非零退出码都应先排查：

```powershell
$repo = (Get-Location).Path
$py = Join-Path $repo '.venv\Scripts\python.exe'
$attempt = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
$out = Join-Path $repo "runs/maintenance/readme-$attempt"
if (Test-Path -LiteralPath $out) { throw 'Choose a new output directory.' }

& $py -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --dry-run
if ($LASTEXITCODE -ne 0) { throw 'Protocol validation failed.' }
& $py -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out "$out/smoke"
if ($LASTEXITCODE -ne 0) { throw 'Smoke execution failed; retain its receipts.' }
& $py -m dbact_sim evidence --batch "smoke=$out/smoke" --out "$out/evidence"
if ($LASTEXITCODE -ne 0) { throw 'Evidence generation failed.' }
```

五例覆盖 oracle/local 静态执行与三种搬运方法，帧数只有 4 或 12，用于验证执行和分析管线，不评价收敛或方法优越性。短 smoke 中运输任务未完成也是正常的测量结果，不能改写为成功。

### 核心、MAS 与工程回归

可用同一薄入口将 Git、依赖、健康检查、核心测试、MAS 无硬件测试、smoke、分析及同身份 resume 分别留存命令、退出码和日志：

```powershell
$check = "runs/maintenance/readme-check-$(Get-Date -Format 'yyyyMMdd_HHmmss_fff')"
.\.venv\Scripts\python.exe scripts/maintenance/validate_repository.py --output $check --extended
if ($LASTEXITCODE -ne 0) { throw 'Inspect commands.json and the failing check log.' }
```

输出目录须为新目录。`commands.json` 汇总结果，`core-tests.xml` 与 `mas-tests.xml` 分开保存。该入口不运行完整科研矩阵或硬件。只检查代码仓库完整性时可运行 `scripts/check_repository_health.py --output <新的本地JSON路径>`，同样使用根解释器。

[GitHub Actions](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml) 在 Windows/Ubuntu 上安装锁定依赖并运行离线检查。是否通过须核对相应提交的实际 CI 状态；历史测试数量不是当前提交的自动保证。

## 项目布局

| 目录 / 文件 | 内容 |
| --- | --- |
| `src/dbact/` | `sensing` 感知、`deployment` 部署、`safety` 安全约束、`control` 控制与阶段逻辑 |
| `src/dbact_sim/` | 仿真、独立评价、实验运行、统计和可视化 |
| `src/mas_adapter/`、`platforms/mas_public/` | MAS / RoboMaster / OptiTrack 桥接及平台 |
| `configs/`、`experiments/ccta2027/` | 运行配置与实验协议分别维护；会议信息保留核对日期 |
| `scripts/`、`tests/` | 命令入口、维护和文献辅助脚本、回归测试 |
| `runs/` | 本地原始批次与新运行；`runs/maintenance/` 单列工程验证 |
| `docs/results/` | 保留版本边界的精简研究证据和清单 |
| `docs/literature_review_2026-10-22/` | 71 份阅读记录、能力矩阵与研究定位工作稿 |
| `data/literature/2026-10-22/` | 70 个 PDF、1 个 DOCX、提取全文、分块及原页图像 |
| `manuscript/ccta2027/`、`theory/static_deployment/` | 英文论文 TeX/PDF 草稿、静态推导和适用范围 |
| `artifacts/`、`tools/` | 图表、渲染、恢复材料及有来源的本地工具 |
| `.github/`、`docs/` | CI、架构、方法说明、索引和有日期的历史记录 |
| `.venv/`、`requirements.lock.txt` | 当前机器独立环境、锁定的第三方依赖 |

核心控制不导入仿真评价模块或 MAS 网络层；旧模块路径保留同一模块对象的兼容别名。[架构说明](docs/ARCHITECTURE.md)给出依赖方向、观测权限和公共模型边界。

```text
仿真真值 ──> 传感器 ──> 局部观测/地图 ──> 密度与CVT ──> 阶段/名义控制
   │                                         └──────────> 安全QP ──> 物理
   └────────────────────────> 独立评价 ──> 指标与收据
```

局部控制只读取声明的观测。oracle 模式单独标识；物理和评价使用真值并不授权局部控制读取真值。

## 常用工作流

以下命令均使用 `.\.venv\Scripts\python.exe -m dbact_sim <命令>`：

| 命令 | 用途 |
| --- | --- |
| `simulate` | 单次仿真、可选图像输出与调试 |
| `deployment` | 配对静态部署或搬运案例 |
| `study` | 协议校验、多进程实验、结果收据与恢复 |
| `evidence` | 从已保存结果生成统计和图表 |
| `campaign` | 冻结、初始化预检与完整矩阵审计 |

各子命令支持 `--help`。完整实验设计、冻结执行及历史重建见[实验入口](experiments/ccta2027/README.md)；首页 smoke 不会触发完整矩阵。

study 输出包含 `manifest.json`、`resolved_protocol.json`、`source_snapshot/`、每案例的 `resolved_case.json`/`case_result.json`、产品及 `summary.json`。收据区分执行状态、测量结果、源码和环境身份。运行错误会保留记录并返回非零；测量到的任务失败仍是有效样本。

`--resume` 仅用于同一次、源码/环境/协议/产品身份一致的运行；先验证保存的源码快照，再恢复执行。输入变化或产品损坏时使用新的输出目录，不能改收据、删失败记录或覆写快照来绕过检查。分析版本与执行版本分别记录。

## 数据可得性与来源恢复

Git 中包含活动源码、配置、实验设计、回归测试、理论/稿件源文件和精简冻结证据。**完整轨迹、文献全文、视频、环境与恢复备份不随 Git 克隆下载**；`runs/`、`data/literature/`、大型 `artifacts/` 等按[数据索引](docs/DATA_INDEX.md)在本仓库内部保存。

历史批次维持原路径及字节。迁移映射中的 `archive.zip::member` 表示可按 SHA-256 核验的原件；不能将同名不同内容当作等价版本。2026-10-31 的 PC 来源整理及后续容器核查记录在本地 `artifacts/migrations/2026-10-31-single-repository/`，不作为新增科研样本或论文证据。恢复工具只处理显式登记的项目内文件；损坏原始字节仍受保护，不从其他项目补齐身份未知的数据。

在新机器上恢复必要数据后重新创建根 `.venv`，再执行工程检查。同盘备份提供误操作回滚能力，不代表整盘故障备份。完整清单、旧机器路径和恢复材料不上传到公共仓库。

## 已保存的研究证据

2026-10-23 整合采用开发提交 `568b1fc`，继承原项目 `e4b0a19`。以下数字归属于当时冻结实现，不能自动归属于当前维护提交。历史来源见[重建记录](docs/REBUILD_2026-10-23.md)，当前验证入口及分支规则见[健康说明](docs/REPOSITORY_HEALTH.md)。

- **2026-10-21 冻结实验：169/169 案例、零执行错误。** 历史实现回归为 616 通过、2 跳过。
- **2026-10-22 可靠性修订：**保留 140 个搬运案例分类、失败候选与默认修复；默认 CVT 开发完成数仍为 5/9。其历史回归为 629 通过、2 跳过，832 案例前瞻设计尚未执行。
- 2026-10-23 的环境与工作区验证记录在[重建验证收据](docs/migrations/2026-10-23/workspace_verification.json)。当前维护验证保存在 `runs/maintenance/`，不会改写历史实验归属。

| 冻结实验 | Boundary CVT / 分段候选 | 对照 |
| --- | --- | --- |
| 静态路径安全 | 分段候选 9/9，最小间距 66.04 mm | 原局部样本 1/9；oracle 9/9；要求 65 mm |
| 开发集搬运完成 | CVT 5/9 | 最近观测目标 5/9；局部间距 4/9 |
| 新种子泛化搬运完成 | CVT 5/12 | 最近观测目标 3/12；局部间距 1/12 |
| 扰动鲁棒性搬运完成 | CVT 1/18 | 最近观测目标 2/18；局部间距 1/18 |

静态结论限于固定多边形与完整控制保持段，不能推出运动接触安全。重复使用种子的汇总仅作描述；鲁棒搬运、独立引擎一致性和实机性能仍待解决。[冻结实验报告](docs/CCTA2027_EXPERIMENTS.md)和[可靠性修订报告](docs/RELIABILITY_REVISION_2026-10-22.md)保留失败及适用范围。

| 记录概念 | 含义与边界 |
| --- | --- |
| `CONTACT_READY` | 控制阶段的准备状态，不直接证明已发生物理接触 |
| 真实物理接触 | 按接触模型/相应测量记录，不能由阶段名称代替 |
| 收定 | 原协议定义的稳定/停止条件 |
| `G500` | 原预算内的阶段、安全与运动合同，单独报告 |
| 搬运任务完成 | 原收定规则及终点欧氏误差容差同时满足，独立于G500 |

本次维护不修改控制律、CVT定义、安全阈值、接触判据、动力学或成功标准；新增工程回归不冒充新增物理实验。

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
