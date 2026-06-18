<div align="center">

# DBACT: Boundary-Aware Cooperative Transport

**面向未知形状物体的去中心化边界感知多机器人协同围捕与搬运。**

<p>
  <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License: MIT">
  <img src="https://img.shields.io/badge/python-%3E%3D3.9-3776AB.svg" alt="Python >= 3.9">
  <img src="https://img.shields.io/badge/tests-pytest%20passing-brightgreen.svg" alt="pytest passing">
  <img src="https://img.shields.io/badge/status-research%20prototype-orange.svg" alt="Research prototype">
</p>

</div>

---

## 项目概览

**项目名称**：DBACT, Boundary-Aware Cooperative Transport

**项目简介**：DBACT 是一个多机器人协同搬运研究软件栈。它让机器人在不知道物体完整几何、中心、半径和预设分工的情况下，仅依靠局部边界观测、邻居通信、局部 CVT 和安全过滤，自组织形成围捕结构，并进一步支持模拟搬运和 MAS/RoboMaster S1 集成验证。

**核心技术栈**：Python 3.9+, NumPy, Matplotlib, PyYAML, pytest, 局部 CVT, CBF-style safety filter, MAS adapter, OptiTrack read-only bridge, RoboMaster S1 command smoke path。

**现有可视化素材**：

| 类型 | 当前路径 | 说明 |
| --- | --- | --- |
| 动态演示 GIF | `runs/paper_like_irregular_moving_cargo/animation.gif` | 最适合作为 README 首屏展示的移动物体模拟动画。 |
| 最终状态截图 | `runs/*/final_snapshot.png` | 展示机器人最终围捕构型和物体位置。 |
| 轨迹图 | `runs/*/trajectory.png` | 展示多机器人运动轨迹、物体方向和最终分布。 |
| 覆盖率曲线 | `runs/*/coverage_rate_curve.png` | 展示边界覆盖率随迭代过程的变化。 |
| 论文风格帧图 | `runs/*/figures/FIG_*.png` | 同屏展示工作空间、局部 Voronoi/CVT 结构和密度场。 |
| MAS 干运行轨迹 | `platforms/mas_public/data/dry_runs/*/trajectory.png` | 展示 MAS controller/module 级干运行输出。 |

> 说明：`runs/` 和 `platforms/mas_public/data/` 下的图像、GIF、CSV 是本地生成的实验产物，默认被 `.gitignore` 排除。若要让 GitHub 首页长期显示这些图片，建议将精选素材复制到 `docs/assets/`，并将 README 中的展示路径同步指向 `docs/assets/...`。

---

## Visual Showcase

![DBACT moving cargo demo](runs/paper_like_irregular_moving_cargo/animation.gif)

<p align="center">
  <sub>DBACT 在不直接使用物体完整先验的条件下，通过局部边界感知与局部密度驱动控制，形成围捕并推动不规则物体移动。</sub>
</p>

| 论文风格过程帧 | 覆盖率曲线 |
| --- | --- |
| ![paper style density and local CVT frame](runs/paper_like_irregular_moving_cargo/figures/FIG_520.png) | ![coverage curve](runs/paper_like_irregular_moving_cargo/coverage_rate_curve.png) |

---

## Features

- 🚀 **未知物体几何友好**：控制器不依赖物体中心、半径或完整多边形，只使用局部边界观测生成 cage target。
- 🧪 **模拟实验覆盖丰富**：内置圆形、矩形、L-shape、非凸多边形、多物体和移动不规则物体等场景。
- 📊 **数据产出完整**：每次仿真会输出 `trajectories.csv`、`coverage_rates.csv`、`final_snapshot.png`、`trajectory.png`、`coverage_rate_curve.png` 和论文风格帧图。
- 🛡️ **安全约束清晰**：局部 CBF-style 安全过滤维护机器人间最小距离，并支持速度限制和投影 fallback。
- 🔌 **面向真实平台扩展**：提供 MAS controller adapter、OptiTrack 只读日志链路，以及 RoboMaster S1 低速命令 smoke test。

---

## Current Status

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| Core DBACT simulation | Working | 未知多边形围捕、局部感知、边界密度、CVT 目标分配、安全过滤和指标计算已实现。 |
| MAS controller adapter | Working | `dtransport` 控制器可从 `WorldState` 生成 MAS-compatible `ControlCommand`。 |
| MAS dry-runs | Working | Controller 级和 ControllerModule 级干运行不依赖 OptiTrack、RoboMaster 硬件或网络运行时。 |
| OptiTrack read-only bridge | Partially validated | mock logging 和 NatNet client import 可用，真实 Motive 流需要稳定刚体配置。 |
| Seven-S1 command smoke path | Working in mock mode | 支持 mock 执行和可选的低速真实 S1 命令流。 |
| Full physical experiment | Not complete | 真实物体观测、闭环 OptiTrack 到控制器集成和低速实物搬运仍需硬件验证。 |

最新本地健康检查：**2026-06-18**，Conda 环境 `dbact`，Python `3.10.20`。根测试、`compileall`、YAML 解析和 MAS platform pytest 均通过。

---

## Results & Visualizations

### Stage 1: Unknown Polygon Caging

Stage 1 验证了一个关键主张：在控制器不直接读取 `cargo.center`、`cargo.radius`、`cargo.vertices` 或 `cargo.closest_boundary()` 的情况下，仅使用局部边界观测、边界感知密度、局部 CVT 和机器人间安全过滤，也能形成稳定围捕结构。

#### 原始基线结果

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

#### Tight baseline 结果

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

#### 关键结论

| 对比项 | 原始基线 | Tight baseline | 结论 |
| --- | ---: | ---: | --- |
| arbitrary polygon coverage | 0.7625 | 0.95625 | 更紧凑的 cage offset 和 density sigma 显著提升边界覆盖。 |
| rectangle polygon coverage | 0.7000 | 0.99375 | 矩形物体在 tight 参数下几乎完成全边界围捕。 |
| nonconvex polygon coverage | 0.90625 | 0.9750 | 非凸形状仍能保持高覆盖率。 |
| minimum distance | > 0.33 m | > 0.33 m | 围捕更紧密的同时，机器人间安全距离仍保持有效。 |

### 图表占位

将精选图表复制到 `docs/assets/` 后，可使用以下占位直接发布论文风格结果：

![实验覆盖率对比折线图](docs/assets/coverage_comparison.png)

该图建议对比 `baseline_unknown_polygon_caging`、`one_rectangle_polygon_caging`、`one_nonconvex_polygon_caging` 以及对应 tight 场景的 coverage 曲线，用来证明参数收紧后边界覆盖速度和最终覆盖率均提升。

![不同物体形状最终围捕截图](docs/assets/final_snapshots_grid.png)

该图建议拼接圆形、矩形、L-shape、非凸多边形和多物体场景的 `final_snapshot.png`，用来展示 DBACT 对不同物体几何的适应能力。

![轨迹与密度场联合可视化](docs/assets/density_voronoi_frame.png)

该图建议使用 `runs/*/figures/FIG_*.png`，展示机器人局部 Voronoi/CVT、边界感知密度场和未知物体围捕结构之间的关系。

视频占位：[DBACT full demo video](docs/assets/dbact_demo.mp4)

---

## Quick Start

### 1. 克隆仓库

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
```

### 2. 创建并激活环境

推荐使用 Conda：

```bash
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
```

或者使用标准虚拟环境：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

Windows PowerShell 激活方式：

```powershell
.\.venv\Scripts\Activate
```

### 3. 安装依赖

```bash
pip install -r requirements.txt
pip install -e .[dev]
```

### 4. 一行代码跑通模拟实验

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

运行完成后会生成：

```text
runs/paper_like_irregular_moving_cargo/
|-- animation.gif
|-- final_snapshot.png
|-- trajectory.png
|-- coverage_rate_curve.png
|-- trajectories.csv
|-- coverage_rates.csv
`-- figures/
    |-- FIG_0.png
    |-- FIG_130.png
    |-- FIG_260.png
    |-- FIG_390.png
    `-- FIG_520.png
```

### 5. 批量运行标准场景

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

### 6. 验证项目健康状态

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

MAS platform 测试：

```bash
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

---

## Repository Structure

```text
boundary-aware-cooperative-transport/
|-- README.md                         # 项目首页和快速复现实验入口
|-- README_DBACT.md                   # 早期 DBACT 研究说明草稿
|-- pyproject.toml                    # Python 包配置: dbact
|-- requirements.txt                  # 运行时依赖
|-- configs/
|   |-- sim/                          # 仿真场景: circle, rectangle, l_shape, nonconvex, multi_object 等
|   `-- mas/                          # MAS dtransport/controller 配置
|-- src/
|   |-- dbact/                        # 核心算法: sensing, density, CVT, safety, metrics, command policies
|   |-- dbact_sim/                    # 仿真环境、场景加载、绘图、GIF 导出、CLI
|   `-- mas_adapter/                  # 根层 MAS-compatible 控制器适配
|-- scripts/
|   |-- run_all_scenarios.py          # 批量运行标准仿真场景
|   |-- run_mock_mas_pipeline.py      # 不依赖真实硬件的 MAS mock pipeline
|   `-- run_seven_s1_cvt_test.py      # 七台 RoboMaster S1 命令 smoke test
|-- tests/                            # 根层单元测试与 smoke tests
|-- docs/
|   |-- ARCHITECTURE.md               # 软件分层和数据流
|   |-- ALGORITHM.md                  # 核心算法说明
|   |-- MAS_INTEGRATION.md            # MAS 集成说明
|   |-- stage1_results.md             # Unknown polygon caging 结果
|   `-- daily_health_2026-06-18.md    # 最新健康检查记录
|-- runs/                             # 本地生成实验输出: GIF、PNG、CSV, 默认不提交 Git
|   |-- paper_like_irregular_moving_cargo/
|   |-- l_shape/
|   |-- nonconvex/
|   `-- multi_object/
`-- platforms/mas_public/             # Vendored MAS, OptiTrack, RoboMaster, app/config/test 代码
```

---

## How It Works

1. **场景加载**
   仿真入口 `dbact_sim.run_sim` 读取 `configs/sim/*.yaml`，初始化工作空间、机器人初始位置、物体形状、控制参数和输出目录。

2. **局部边界观测**
   每个机器人只在传感器半径内观察物体边界，得到局部 `BoundaryObservation`。控制器不直接读取完整物体多边形，完整几何只用于仿真端生成观测和离线评估。

3. **生成 cage target**
   对每个局部边界点 `b` 估计外法向 `n_out`，并在物体外侧生成目标点：

```text
q_target = b + d_cage * n_out
```

4. **构造边界感知密度场**
   机器人把 cage target 转成高斯密度峰值，让局部 CVT 被吸引到物体边界外侧，而不是被吸引到一个已知物体中心。

5. **局部 CVT 分配目标**
   每个机器人只使用自己和通信范围内邻居的位置，计算局部 weighted centroid。这个目标驱动机器人补齐边界空缺，形成更稳定的围捕结构。

6. **安全过滤**
   名义速度进入局部 CBF-style safety filter。过滤器维护机器人间最小距离，并把速度限制在安全范围内。

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

7. **输出模拟数据和图表**
   仿真结束后自动保存轨迹 CSV、覆盖率 CSV、最终截图、轨迹图、覆盖率曲线、论文风格帧图。若启用 `--animate`，还会导出 GIF。

8. **接入 MAS 或硬件验证**
   MAS adapter 将 DBACT 控制器包装为 `WorldState -> ControlCommand`。真实硬件前应先完成 mock pipeline、ControllerModule dry-run、OptiTrack read-only logging 和低速 S1 smoke test。

---

## MAS And Hardware Validation

### Mock MAS pipeline

```bash
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

预期输出：

```text
runs/mock_mas_pipeline/
|-- states.csv
|-- commands.csv
`-- mock_trajectory.png
```

### MAS platform dry-run

```bash
cd platforms/mas_public
python apps/dbact/run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/dtransport_auto_init --clamp-to-world-bounds
```

ControllerModule 级 dry-run：

```bash
cd platforms/mas_public
python apps/dbact/run_controller_module_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/controller_module_dtransport
```

### OptiTrack read-only path

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

真实 Motive/NatNet 只读检查：

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --frames 500 --hz 100 --print-every 50 --output data/optitrack_readonly/real_world_states.csv
```

### Safety first

- 先跑只读 OptiTrack 日志，再启用控制器输出。
- 逐台移动机器人确认 robot ID 与 rigid body 映射。
- 真实机器人第一次运行时使用极低速度限制。
- 在物理实验中保持可触达的急停手段，键盘中断不能替代物理安全系统。
- 每次硬件运行后检查 `commands.csv`、`states.csv` 和事件日志。

---

## Documentation

| 文件 | 内容 |
| --- | --- |
| `docs/ARCHITECTURE.md` | 软件分层、核心模块和数据流。 |
| `docs/ALGORITHM.md` | 边界感知 cage target、局部 CVT、局部安全过滤和运输模型说明。 |
| `docs/MAS_INTEGRATION.md` | MAS 集成路径和控制器接入说明。 |
| `docs/ROADMAP.md` | 分阶段开发路线。 |
| `docs/stage1_results.md` | Unknown polygon caging baseline 的实验数据和结论。 |
| `docs/stage2_mas_virtual_object.md` | MAS virtual-object integration notes。 |
| `docs/stage3_mas_dry_run.md` | MAS dry-run notes。 |
| `docs/stage4_optitrack_readonly.md` | OptiTrack read-only bridge notes。 |
| `docs/daily_health_2026-06-18.md` | 最新分支与工作区健康检查报告。 |
| `platforms/mas_public/docs/*.md` | MAS platform 的配置、硬件、使用和调试说明。 |

---

## Contributing

欢迎提交 issue、实验复现实录、场景配置、图表、文档修订和平台集成改进。建议贡献流程：

1. Fork 本仓库并创建功能分支。
2. 针对新场景或新控制逻辑补充最小测试。
3. 运行根层测试和必要的 MAS platform 测试。
4. 在 PR 中说明实验配置、输出路径、关键指标和安全边界。

推荐提交前检查：

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

---

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.
