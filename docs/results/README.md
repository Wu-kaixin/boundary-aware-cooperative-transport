# 结果与证据索引

本目录保存 Git 跟踪的精简研究证据。完整逐帧数据通常在本地 `runs/`，是否包含在仓库中以
各批次清单为准。不同源码、实验契约、种子和模型的结果应分别报告。


当前 169 案例修订结果仅在[开发分支的证据包](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment/docs/results/ccta2027_draft_v2)中，
本分支不包含该目录或对应控制器修复。

## 批次入口

| 目录 | 内容与适用范围 |
| --- | --- |

| [refactor_2026-10-21](refactor_2026-10-21/) | 模块迁移、串并行一致性与历史 602 项测试验证 |
| [evidence_closure](evidence_closure/) | 2026-10-07 的三方法比较及历史验收，使用当时源码 |
| [consolidation](consolidation/results.md) | 较早 Gate 0–6 汇总，不能替代当前重跑 |
| [v2_shape_matrix](v2_shape_matrix/) | 历史 180 案例形状 / 距离矩阵 |
| [v2_control_explore_gain0](v2_control_explore_gain0/) | 历史 45 案例探索增益对照 |
| [v2_baseline_12seed](v2_baseline_12seed/) | 早期 12 种子 L 形基线 |
| [se2](se2/)、[t3](t3/)、[t4](t4/) | 估计器、控制权限与失败机制诊断 |
| [t5](t5/)、[t6](t6/)、[t7](t7/) | 历史退化、距离与运行开销分析 |
| [representative](representative/) | 历史代表回合及回放产品 |
| [issf](issf/)、[progress](progress/) | 早期安全与进程诊断；阅读目录内原始收据 |

原结果 README 中的形状矩阵数字、解释和失败分析完整保存在
[历史 v2 报告](legacy_v2_report.md)。这里的“历史 v2”与 `ccta2027_draft_v2` 是不同实验版本，
不能因为名称相近就合并统计或引用。

## 使用规则

1. 从批次清单确定源码、环境、协议和成功定义，再引用数字。
2. 执行完成、接触就绪、物理接触、收定、G500 与任务完成是不同测量。
3. 保留执行异常、删失事件、任务失败和全部计划案例的分母。
4. 静态 oracle 结论、未知边界经验结果与动态接触搬运分别论述。
5. 生成新分析或重跑时写入新目录，不能覆盖历史收据或将旧数据标成新验证。

返回[项目首页](../../README.md)。
