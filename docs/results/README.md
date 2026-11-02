# 结果与证据索引

[匹配支撑固定筛选](reference_support_v1/README.md)：8条同输入轨迹已完成；局部大误差减少但未通过团队误差门槛，新增物理回合0，候选默认关闭。


[最新参考运动与论文证据核验](reference_motion_audit_v1/README.md)：同输入参考运动核验完成：24条旧轨迹、6条远场原配准器精确重放，新增物理回合0；区分初始化、参考点旋转和匹配误差，并核查论文主张与验证缺口。


[参考初始化消融](reference_origin_v1/README.md)：参考初始化固定消融完成：24回合（8个已知开发条件×3臂），三臂均7/8完成；取消初始化放大救回斜箱但损失远场C形，提前参考未救回斜箱且G500下降。两项均未采用，原判据和默认方法不变。


[投票与停止消融](voter_continuity_v1/README.md)：投票集合与停止确认固定消融：32回合（8个已知开发条件×4臂），分别检验换票读数连续化和1秒到达确认，保留全部失败与截尾；默认关闭、原成功安全判据不变。


[运动估计先验复验](transport_validation_v1/README.md)：97个新回合与41条核验后的历史记录分别登记。旧33条件CVT安全完成31/33→33/33，G500 27/33→31/33；12个新初态下CVT安全完成仍11/12、G500仍9/12。共同候选参数的CVT/spacing/nearest完成数相同，G500为9/12、6/12、8/12，保留停止偏差和全部负结果。默认不变，不合并为任意形状总体成功率。

[不规则货物与远场比较](cargo_discovery_v1/README.md)：固定近场132回合（33条件×4方法），安全完成为边界 CVT 31/33；spacing 31/33；nearest 31/33；检测点密度 CVT 1/33；远场12回合中未知位置 3/6 完成、6/6 检测；初始位置提示 3/6 完成、6/6 检测；硬安全失败：未知位置 0/6、提示 1/6。两组分别冻结，完整负结果保留；未达到相对全部对照的预先开发优势门槛，不能宣称整体算法优越；应按具体条件与指标报告得失。

[坐标一致修复固定比较](contact_consistency_v1/README.md)：固定 18 回合，九个开发条件、同一 seed 2、每例最多 600 秒。共同参考配置下，旧聚合公式完成 5/9，坐标一致修正完成 8/9；G500 分别 2/9、7/9，修正臂原硬安全检查违规 0/9。C 背向仍失败且出现越过目标后的错误停止；8/9 不是总体成功概率。本轮是公共安全计算修复的配对比较，不是原 CVT 对 spacing 的优越性检验。

[几何读出固定消融](contact_geometry_v1/README.md)：固定 12 回合已完成：原参考 0/3、参考点修正 0/3、接触距离拟合 0/3、组合 0/3。没有候选达到预设任务救回门槛。保留有支持的诊断与实验开关，默认关闭；不追加回合或事后调参。

本目录保存 Git 跟踪的精简研究证据。完整逐帧数据通常在本地 `runs/`，是否包含在仓库中以
各批次清单为准。不同源码、实验契约、种子和模型的结果应分别报告。


## 批次入口

| 目录 | 内容与适用范围 |
| --- | --- |
| [contact_reinforcement_v1](contact_reinforcement_v1/README.md) | 三个失败条件的恢复/增援消融，9回合、600秒；原参考 0/3、仅修恢复 0/3、修复加增援 0/3；全部负结果、安全与参考精确复现 |
| [feedback_components_v1](feedback_components_v1/README.md) | 位置/速度共同准入四臂36个新回合，另列27个历史参照；3/9、6/9、4/9、6/9完成，保留G500失败、接触恢复循环问题及C背向未完成 |
| [displacement_stop_v1](displacement_stop_v1/README.md) | 旧C背向12000帧精确重建、固定观测参考读出与36回合比较；5/9至6/9仅为开发结果，保留C背向退化和G500失败 |
| [boundary_navigation_v1](boundary_navigation_v1/README.md) | 六方法×九条件，54回合、150/600秒观察；完成数未提升，保留误差改善、停止失败及穿透预算超限 |
| [contact_handoff_v1](contact_handoff_v1/README.md) | 压力/换位隔离、确认与回退交接、双方 600 秒观察；150 秒三方法仍为 5/9，无持续接管确认，保留全部失败 |
| [dynamic_contact_v1](dynamic_contact_v1/README.md) | 无附着接触分工与侧向换位；27 回合比较为 5/9、4/9、2/9 完成，保留退化，候选开关关闭 |
| [multishape_census_v1](multishape_census_v1/README.md) | 五形状、九朝向条件、三方法、固定开发 seed 的 27 回合完整任务普查；含失败、阶段图和同步视频索引 |
| [progress_local_v2](progress_local_v2/README.md) | 实际配准、影子证书和 anchored 读出诊断；候选未救回失败，停止扩展 |
| [readiness_diagnostic_v1](readiness_diagnostic_v1/README.md) | 历史审查、仅进度 oracle 和合成信息试验；不与多形状普查混合统计 |
| [reliability_2026-10-22](reliability_2026-10-22/README.md) | 后续可靠性分类、失败候选、默认修复和未执行的前瞻设计；完整原始数据已恢复到 `runs/reliability/` |
| [ccta2027_draft_v2](ccta2027_draft_v2/README.md) | 历史 CCTA 修订：169 案例、零执行错误、审计 PASS；含失败、四组图和原始源码归档 |
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

## 最新固定参数开发

[transport_tuning_v1](transport_tuning_v1/README.md)记录48回合筛选与24回合迁移检查：CVT筛选安全完成4/8→7/8，G500 2/8→6/8；另四条件CVT安全完成2/4→4/4，spacing、nearest各2/4→3/4。参数选择、约束缩放的相反变化和全部失败同时报告；这是开发证据，不是任意形状可靠性验证。

## 使用规则

1. 从批次清单确定源码、环境、协议和成功定义，再引用数字。
2. 执行完成、接触就绪、物理接触、收定、G500 与任务完成是不同测量。
3. 保留执行异常、删失事件、任务失败和全部计划案例的分母。
4. 静态 oracle 结论、未知边界经验结果与动态接触搬运分别论述。
5. 生成新分析或重跑时写入新目录，不能覆盖历史收据或将旧数据标成新验证。

返回[项目首页](../../README.md)。

2026-10-23 重建只整合源码、原始文件与运行环境。本次验证和所有文件去向见[重建记录](../REBUILD_2026-10-23.md)，本地数据范围见[数据索引](../DATA_INDEX.md)。
