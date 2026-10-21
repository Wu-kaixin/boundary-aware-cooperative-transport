# DBACT：边界感知安全部署与协作搬运

目标会议：[IEEE CCTA 2027](https://ccta2027.ieeecss.org/)。当前工作分支：`paper/boundary-aware-safe-deployment`。

本项目研究机器人如何从局部边界观测形成部署配置，为协作搬运做准备。静态 oracle 几何下的理论验证、未知边界下的经验部署，以及动态接触搬运采用独立的实验与结论。未知几何搬运的普遍安全性和性能优势目前没有获得证明；结论以[主张与证据矩阵](docs/paper_claim_evidence_matrix.md)为准。

## 安装与运行

Python 3.10+。建议使用独立环境并安装完整复现依赖：

```sh
python -m pip install -e ".[dev,qp,sim,analysis,publication]"
python -m dbact_sim --help
```

统一入口提供 `simulate`、`deployment`、`study`、`evidence`；安装后也可用 `dbact`。原有 `scripts/` 命令继续兼容。

先检查实验协议，再运行五个短案例，验证静态与动态管线：

```sh
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --dry-run
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2
```

这只是执行检查，短时窗不用于评价收敛、任务完成率或论文优越性。输出位于 `runs/ccta2027/smoke/smoke/`。完整性检查后恢复同一次运行时，加 `--resume`；改变代码、环境或协议后使用新输出目录。

运行单个闭环仿真：

```sh
python -m dbact_sim simulate --config configs/sim/d/l_shape_closed_loop.yaml --seed 2 --steps 500 --no-figures --output runs/ccta2027/example
```

从已有运行生成统计和图表：

```sh
python -m dbact_sim evidence --batch smoke=runs/ccta2027/smoke/smoke --out runs/ccta2027/smoke_evidence
```

完整测试：

```sh
python -m pytest -q
```

## 项目结构

```text
src/
  dbact/
    sensing/                 射线感知、局部地图、经验边界特征
    deployment/              边界密度、有限范围 CVT、对照方法、静态定理模式
    safety/                  硬约束滤波与二维 QP
    control/                 控制器编排、阶段守卫、搬运命令
    types.py, geometry.py    共享数据类型与几何
    cargo.py, *_dynamics.py  物体与接触模型
    contracts.py             配置与评价契约
  dbact_sim/
    experiments/             成对实验、协议校验、恢复、统计证据
    environment.py           仿真环境；真值评价与控制输入分离
    paper_evaluation.py      独立评价器
    visualization/           回放、调试与论文图
    cli.py                   统一命令入口
  mas_adapter/               MAS / RoboMaster 转接
experiments/ccta2027/         会议要求快照、运行入口、短案例
configs/                     仿真与平台参数
manuscript/ccta2027/          论文结构与证据映射
theory/static_deployment/   静态理论、假设、推导和历史证据
docs/results/               已归档的可追溯结果
tests/                      行为、接口与依赖边界检查
runs/                       本地生成结果（不提交）
```

原 `dbact.controller` 等路径是兼容别名，指向同一实现模块；新代码使用子包路径。[详细架构与数据流](docs/ARCHITECTURE.md)。

## CCTA 2027 与当前证据

[官方征稿](https://ccta2027.ieeecss.org/authors/call-for-papers)规定 IEEE 双栏格式，初稿最多 8 页；录用稿以 6 页为限，超页收费。[官网日期](https://ccta2027.ieeecss.org/)列出的全文截止日为 2027-02-07。核对日期：2026-10-21。未公布的时区、费用金额和最终页数上限保留待确认。

项目以 6 页正文组织方法、限定的理论结论和可复现实验；这是项目选择。[会议要求与证据边界](docs/CCTA2027.md)及[实验说明](experiments/ccta2027/README.md)列出具体分工。

- 理论：静态 oracle 几何的适用条件见 [theory/static_deployment](theory/static_deployment/)。
- 方法与主张：[论文主线](docs/paper_storyline.md)、[证据矩阵](docs/paper_claim_evidence_matrix.md)。
- 历史完整比较：[2026-10-07 证据归档](docs/results/evidence_closure/)。它们对应当时的源码。
- 当前探索：局部 gap guard 和 `push_cone` 保留为研究代码，尚不能作为新的论文结论。
- [重构记录](docs/REFACTOR_2026-10-21.md)记录远程/本地比较、迁移与验证范围。

<!-- CONSOLIDATION_RESULTS_START -->
较早的 Gate 0–6 汇总见 [consolidation](docs/results/consolidation/results.md)；其中数字是历史运行结果。[重构前 README](docs/archive/README_before_ccta2027.md)保留原有叙述。
<!-- CONSOLIDATION_RESULTS_END -->

平台和硬件流程见 [MAS 集成说明](docs/MAS_INTEGRATION.md)；旧的闭环开发记录见 [CLOSED_LOOP_D](docs/CLOSED_LOOP_D.md)。

[MIT License](LICENSE)。
