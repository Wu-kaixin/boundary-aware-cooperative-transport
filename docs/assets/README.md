# 展示素材与来源

此目录是 Git 跟踪的**历史展示素材**，源于早期闭环和诊断运行。
动画展示机制与行为，不能作为当前开发分支的成功率、安全性或硬件验证证据。

| 素材 | 原始来源 | 说明 |
| --- | --- | --- |
| `closed_loop_d_seed2.gif` | `runs/readme/near_seed_2/closed_loop.gif` | 历史近场闭环，seed 2 |
| `closed_loop_d_seed4.gif` | `runs/readme/near_seed_4/closed_loop.gif` | 历史近场闭环，seed 4 |
| `closed_loop_d_seed8.gif` | `runs/readme/near_seed_8/closed_loop.gif` | 历史近场闭环，seed 8 |
| `search_d_seed7.gif` | `runs/readme/search_seed7/closed_loop.gif` | 历史远场搜索，seed 7 |
| `d10-post-detection-stages.png` | `runs/readme/d10_diag/figA_segments.png` | 检测后阶段诊断 |
| `d10-coverage-and-gap.png` | `runs/readme/d10_diag/figB_coverage.png` | 地图覆盖与未观测弧 |
| `d10-gate-tradeoff.png` | `runs/readme/d10_enc/figF_gate_tradeoff.png` | 旧阶段守卫对比 |
| `static-deployment-comparison.png` | `runs/paper/jeh_matrix` 经 `scripts/build_consolidation_report.py` | 旧 oracle 静态比较 |

当前 CCTA 图表来自 [169 案例结果包](../results/ccta2027_draft_v2/README.md)，使用其 PNG/PDF 和对应图注。
历史素材的来源路径可能仅存在于原实验机器；不据此宣称当前 checkout 包含原始逐帧数据。
历史限制见[归档闭环说明](../HISTORICAL_CLOSED_LOOP_README.md)。

新论文图必须绑定确定的实验批次、源码、单位和评价定义。
`paper_like_irregular_moving_cargo.yaml` 是非论文配置，不用于论文展示。
已撤回素材的限制保留在[历史重构审计](../REFACTOR_2026-08-08.md)，旧文件可从 Git 历史追溯。
