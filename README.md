# DBACT：边界感知部署与协作搬运

DBACT 研究机器人如何从局部边界观测形成部署配置，并在接触后协作搬运物体。
项目面向 [IEEE CCTA 2027](https://ccta2027.ieeecss.org/)，现阶段的论文方向是
静态安全部署与有明确适用范围的搬运准备实验。

## 分支与阅读入口

| 远端分支 | 用途 | 推荐入口 |
| --- | --- | --- |
| [`main`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/main) | 模块化重构基线 | 架构、安装与历史证据 |
| [`paper/boundary-aware-safe-deployment`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment) | 当前开发与 CCTA 实验 | 最新修复、169 案例与论文图表 |

**本页对应 `paper/boundary-aware-safe-deployment` 分支。** 各分支 README 只描述该分支已有的代码和结果。

2026-10-22 后续开发见 [可靠性修订报告](docs/RELIABILITY_REVISION_2026-10-22.md)：
140 个搬运案例的失败分类、默认控制约束修复、保留的失败候选、理论适用范围补充，
以及已编译的 [英文论文初稿](manuscript/ccta2027/main.tex)。
本轮回归为 629 通过、2 跳过；新控制律的失败候选没有替换下列冻结结果。

## 当前实验结果

2026-10-21 冻结的 CCTA 修订实验已完成 **169/169** 个案例，执行错误为零，完整性审计通过。
对应实现的回归测试为 **616 通过、2 跳过**。这些是执行与验证结果，任务成功率单独报告。

| 实验 | Boundary CVT / 分段候选 | 对照 |
| --- | --- | --- |
| 静态路径安全 | 分段候选 9/9，最小间距 66.04 mm | 原局部样本 1/9；oracle 9/9；要求 65 mm |
| 开发集搬运完成 | CVT 5/9 | 最近观测目标 5/9；局部间距 4/9 |
| 新种子泛化搬运完成 | CVT 5/12 | 最近观测目标 3/12；局部间距 1/12 |
| 扰动鲁棒性搬运完成 | CVT 1/18 | 最近观测目标 2/18；局部间距 1/18 |

静态安全证据对应固定多边形和完整控制保持段，不能推出运动物体的连续安全性。
鲁棒搬运和独立引擎一致性仍未解决；Pymunk 案例执行结束但未完成搬运任务。
开发集与鲁棒性实验重复使用种子，其汇总率仅作描述，不按独立重复计算置信区间。

[完整实验报告](docs/CCTA2027_EXPERIMENTS.md) · [169 案例与复现包](docs/results/ccta2027_draft_v2/README.md)

## 安装与快速检查

需要 Python 3.10+；CI 使用 Python 3.11。以下命令从仓库根目录运行：

```sh
git clone --branch paper/boundary-aware-safe-deployment https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
python -m pip install -e ".[dev,qp,sim,analysis,publication]"
python -m dbact_sim --help
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --dry-run
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/readme_smoke
python -m pytest -q
```

五个 smoke 案例只检查执行管线，不能用于判断收敛或论文性能。
重复运行使用新的输出目录；只有恢复同一源码、环境、协议和产品时才使用 `--resume`。
安装后也可用 `dbact` 替代 `python -m dbact_sim`。

## 常用工作流

| 命令 | 用途 |
| --- | --- |
| `simulate` | 单次仿真、回放与调试 |
| `deployment` | 配对静态部署或搬运案例 |
| `study` | 协议校验、多进程实验、结果收据与恢复 |
| `evidence` | 从既有结果生成统计和图表 |
| `campaign` | 冻结、预检与审计完整 CCTA 实验矩阵 |

```sh
python -m dbact_sim simulate --config configs/sim/d/l_shape_closed_loop.yaml --seed 2 --steps 500 --no-figures --output runs/readme_example
python -m dbact_sim evidence --batch smoke=runs/readme_smoke --out runs/readme_smoke_evidence
```

完整实验的输入、输出、冻结与复现步骤见 [实验入口](experiments/ccta2027/README.md)。
所有新运行写入 `runs/`，精简且需要公开的证据放入 `docs/results/`。

## 代码结构

```text
src/dbact/
  sensing/              射线感知、局部地图与边界特征
  deployment/           边界密度、局部 CVT、对照方法与静态参考
  safety/               约束构造、硬安全滤波与二维 QP
  control/              控制器编排、阶段守卫与搬运命令
src/dbact_sim/          环境、独立评价、实验与可视化
src/mas_adapter/        MAS / RoboMaster 适配
experiments/ccta2027/   协议与会议要求快照
manuscript/ccta2027/    论文结构与证据映射
theory/static_deployment/ 静态分析及其假设
docs/results/          当前与历史证据索引
platforms/mas_public/  随仓库保留的 MAS 平台
tests/                 回归、接口与依赖边界检查
```

核心控制不导入仿真和评价模块；真值用于传感器、物理与独立评价，局部控制通过观测获取信息。
旧 `dbact.controller` 等路径保留兼容别名。详见 [架构说明](docs/ARCHITECTURE.md)。

## 论文与证据

- [会议要求快照](docs/CCTA2027.md)：页面上次核对信息见 `experiments/ccta2027/conference.yaml`。
- [论文结构](manuscript/ccta2027/README.md)：当前是章节与证据映射，尚无投稿正文。
- [结果索引](docs/results/README.md)：按实验版本阅读，保留失败及分母。
- [静态分析](theory/static_deployment/)：区分 oracle、局部样本及其几何假设。
- [MAS 平台说明](platforms/mas_public/README.md)：硬件桥接流程独立于论文仿真。
- [历史素材说明](docs/assets/README.md)：旧动画不能作为当前实现的验证。

<!-- CONSOLIDATION_RESULTS_START -->
历史 Gate 0–6 汇总见 [consolidation](docs/results/consolidation/results.md)；其数字对应旧源码。
更早的机制探索见 [历史闭环说明](docs/HISTORICAL_CLOSED_LOOP_README.md)。
<!-- CONSOLIDATION_RESULTS_END -->

[CI 运行记录](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml?query=branch%3Apaper%2Fboundary-aware-safe-deployment) · [MIT License](LICENSE)
