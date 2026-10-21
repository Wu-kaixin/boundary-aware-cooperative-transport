# DBACT：边界感知部署与协作搬运

DBACT 研究机器人如何从局部边界观测形成部署配置，并在接触后协作搬运物体。
项目面向 [IEEE CCTA 2027](https://ccta2027.ieeecss.org/)，现阶段的论文方向是
静态安全部署与有明确适用范围的搬运准备实验。

## 分支与阅读入口

| 远端分支 | 用途 | 推荐入口 |
| --- | --- | --- |
| [`main`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/main) | 模块化重构基线 | 架构、安装与历史证据 |
| [`paper/boundary-aware-safe-deployment`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment) | 当前开发与 CCTA 实验 | 最新修复、169 案例与论文图表 |

**本页对应 `main` 分支。** 各分支 README 只描述该分支已有的代码和结果。

## 本分支状态

`main` 保存 2026-10-21 的模块化重构基线，包含控制、感知、部署、安全和实验运行器。
该基线的历史验证记录为 **602 通过、2 跳过**，详见
[重构记录](docs/REFACTOR_2026-10-21.md)；这不是本次 README 修改重新运行的测试计数。

后续控制器修复、完整 169 案例实验、`campaign` 命令和论文图表位于
[开发分支](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment)。这些代码和结果尚未合并到 `main`。
当前分支中的历史实验不能用来替代开发分支的新验证。

## 安装与快速检查

需要 Python 3.10+；CI 使用 Python 3.11。以下命令从仓库根目录运行：

```sh
git clone --branch main https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
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

[CI 运行记录](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml?query=branch%3Amain) · [MIT License](LICENSE)
