# 冻结后有限支持：执行与审计检查点

八个物理回合均已按原协议执行结束，执行错误0，源码完整性PASS。
任务收据显示：**对照安全完成4/4，有限支持候选完成0/4；四个候选均在600秒预算截尾。**
候选继续默认关闭、未采用。完整逐帧配对、桥接和有限支持约束重放仍待完成，
尚未出具正式审计门槛结果。

| 条件 | 对照终点误差 | 候选终点误差 | 候选耗时 / 结果 |
| --- | ---: | ---: | --- |
| 近场圆形 | 15.599mm | 1041.299mm | 600s / 预算截尾 |
| 近场L形 | 17.949mm | 1141.581mm | 600s / 预算截尾 |
| 远场C形 | 17.445mm | 1197.246mm | 600s / 预算截尾 |
| 远场斜箱 | 16.457mm | 1195.318mm | 600s / 预算截尾 |

G500为对照2/4、候选0/4。八例原硬安全指标违规和错误HOLD均0；
这不代替尚未完成的新约束逐帧执行审计。
[全部八回合表](all_eight_receipts.csv)、[任务检查和失败原因](terminal_receipt_summary.json)、
[最终执行收据](execution_outcome.json)及[产品清单](receipt_publication.json)完整保留。
终点误差另由保存的真实终点与目标坐标核对，原G500门槛按既有检查器重新计算。

四条件×两臂保持冻结源码、最佳数值参数、原初态和每例600秒预算。
较早的[7/8执行快照](execution_snapshot.json)、[部分收据](partial_receipt_summary.json)
与[全部计划行快照](all_eight_scheduled_snapshot.csv)保留原时间和待完成行；
上表使用后取得的八份最终收据，不改写早期状态。

## 对照与参数保持

四个对照的全部25组回放数组、config、paired_metrics、physics_substeps、summary
和结果科学字段均精确复现旧桥接候选，见[比较](control_reproduction.json)。
旧共享比较器把预声明的实验臂名称变化也当作科学变化，产生
[原始不一致记录](original_label_mismatch.json)。分析适配层只允许
`map_bridge_candidate → map_bridge_control` 这一精确更名，核对条件与case ID，
保留真实标签；终点或其他科学字段差异仍会失败，不修改原数据。

冻结源码为 `e6b7456f5f0a4d71ddd4b33159c058e215115aae35c01b6c5c9eae7e68f32767`。
保护配置SHA-256为 `a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112`。
运行源码、22个冻结辅助/测试文件、物理、数值参数及成功/安全判据保持不变。

## 审计工程验证

原冻结审计器在比较含NumPy数组的扫描包时会触发数组真值异常。
独立分析适配层使用现有精确列表转换，保留每个字段与全部原断言，退出后恢复原转换器。
单个浮点值改变一个ULP仍会被拒绝。冻结审计器和原错误证据保留。

- 发布前核心回归：**1079通过、2跳过**；适配层针对性检查5通过。
- 冻结前核心：1074通过、2跳过；物理协议针对性检查7通过，保留原身份。
- 用前80步旧圆形对照姿态驱动候选控制器，**不做物理积分**；新生成的命令、诊断与地图
  经过同一审计器重放。原比较器按预期失败，适配层验证80步和412个有限支持准入状态。
- 以上是合成控制重放的工程预检，**新增物理回合0**，不代表候选真实轨迹或任务通过。

[工程收据](engineering_checks.json)绑定测试、预检源码及原始结果哈希。
本机joblib/PyYAML为1.5.1/6.0.2，锁文件为1.6.0/6.0.3；运行保留原环境身份。

## 下一步

八个终端收据已收齐。下一步运行
`scripts/analyze_post_acquisition_support_v1.py` 完成原协议要求的完整审计。
原 `post_acquisition_support_study.py analyze` 入口保留冻结身份，需通过上述适配层执行。
审计失败时保留原尝试，另立分析恢复身份，不能重跑或改写本次物理结果。

[接触兼容性诊断](../post_support_contact_compatibility_v1/README.md)在零新增物理回合下发现，
11个实际正向施力的声明推手在新过滤器下均得到目标向分量为负的反事实速度。
这支持优先检查推动接触与边界排斥的兼容性；不能据此独自证明候选闭环失败的完整因果链。

协议、设计及分析更正分别见
[物理协议](../../../experiments/ccta2027/post_acquisition_finite_support_v1.json)、
[物理设计](../../../experiments/ccta2027/post_acquisition_finite_support_v1_physical_design.md)、
[执行记录](../../../experiments/ccta2027/post_acquisition_finite_support_v1_physical_plan.md)和
[分析更正](../../../experiments/ccta2027/post_acquisition_finite_support_v1_analysis_addendum.md)。
原件在 `runs/research/2026-11-03-post-acquisition-support-v1/`；
[检查点产品清单](publication.json)与[任务收据产品清单](partial_receipt_publication.json)分别核验。
