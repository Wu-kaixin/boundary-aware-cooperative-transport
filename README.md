# DBACT：边界感知部署与协作搬运

## 最新成果与保护基线（2026-11-03）

**本轮结论：有限支持候选出现明显搬运退化，继续默认关闭。** 最新[接触几何与方向能力诊断](docs/results/post_support_contact_geometry_capability_v1/README.md)已完成，进一步确认主要瓶颈在约束本身；[独立八回合检查点](docs/results/post_support_validation_checkpoint_v1/README.md)公开冻结协议、四个对照的精确复现和审计工具验证。八个物理回合均已执行结束、执行错误0；任务收据为对照4/4完成、候选0/4完成，四个候选均在600秒预算截尾。完整约束重放已启动，正式门槛审计仍待完成；[早期审计进度快照](docs/results/post_support_contact_row_decomposition_v1/analysis_checkpoint.json)记录核对时点和执行身份。成果沿 [`paper/boundary-aware-safe-deployment`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment) 发布。

最终目标：**利用局部感知与邻域通信，实现未知形状物体的安全部署、无附着协作搬运和可靠停止，单独证明边界部署的贡献，并完成符合 CCTA2027 要求的论文初稿。** 当前仍在完善公共搬运链，尚未达到最终验收。

最新扫描包追踪重放1749个原控制帧，保留128状态和3813物体行，新增物理0。固定17个原正向力状态中，原聚合约束均允许正向速度；新截断约束仅6个允许，10个强制目标反向、1个无解，全部具有可复算证书。限制最强的行有10个来自自身、7个来自邻居，不能只归咎于邻居粗采样。另有3个状态允许正向却被QP选成反向，其中2个是声明推手。**约束兼容是主要问题，优化目标另有次要问题；正向速度能力不等于接触力或任务成功。** [完整336系统、证书与证据边界](docs/results/post_support_contact_geometry_capability_v1/README.md)保留全部分母。

本次只读诊断对四个已完成对照，分别预先选择参考冻结后、制动前的首次和峰值正向接触力时刻。精确重放 **1749帧**，保留8个事件的全部128个机器人状态，新增物理回合 **0**：

| 接触兼容性指标 | 结果 |
| --- | ---: |
| 全部选定状态 / 有限支持准入 | 128 / 112 |
| 实际产生正向力的状态 / 其中准入 | 17 / 17 |
| 原有效命令违反新截断 / 未截断物体约束 | 17/17 / 17/17 |
| 反事实新速度的目标向分量变负 | 13/17 |
| 声明推手且产生正向力的状态，其新目标向速度变负 | 11/11 |

这些是同状态重新过滤后的**机器人速度**，不代表新物体受力或新闭环失败。四条件反复用于开发，事件与机器人状态也不是独立统计样本。完整[128状态表](docs/results/post_support_contact_compatibility_v1/all_128_states.csv)和[17个正向力状态表](docs/results/post_support_contact_compatibility_v1/all_17_positive_force_states.csv)保留全部分母。

随后对同一固定状态组做两项代数诊断，新增物理回合仍为 **0**。移除非负运动需求项后，17/17个正向力状态的原指令仍违反新物体行；允许原0.2m/s速度界内任意有符号运动时，其中 **9/17**（实际正向施力的声明推手中 **8/11**）仍被至少一行排除。单独更换运动项无法恢复全部原指令，需继续拆解有限支持的法向、几何余量和接触距离。该结论只针对这些固定原指令，**不表示不存在其他安全推动指令**；[全部128状态、公式、条件分母与证据限制](docs/results/post_support_contact_row_decomposition_v1/README.md)均已保留。[独立审查](docs/results/post_support_contact_row_decomposition_v1/final_review.md)复核全部3813条物体行和112个运动上界，未发现需要修正的问题。

此前[有限边界冲突诊断](docs/results/map_bridge_conflicts_v1/README.md)精确重放517帧，覆盖18个缩放状态及226项比较。局部有限扫描与接收速度模长，使17个冻结后冲突由0/17变为17/17未截断可行；同17帧全队272状态中，181个获准状态全部可行。新接触诊断说明：**QP可行与维持推动能力必须同时验证。** 接收速度模长不是物体真实速度的安全证书，完整0.2m/s需求仍曾新增6个未截断冲突。

| 新物理批次的执行与任务收据 | 状态 |
| --- | --- |
| 冻结计划 | 四条件×两臂，8回合，每例600秒；最佳数值参数、初态及原判据不变 |
| 四个桥接对照 | 已完成；全部25组数组和科学字段精确复现 |
| 四个有限支持候选 | 全部执行结束，任务完成0/4、G500 0/4；均在600秒预算截尾 |
| 候选完整逐帧约束审计与正式门槛 | 审计执行中；正式结论待完成，任务回退已确认，候选未采用 |
| 审计工程预检 | 80步旧姿态驱动的合成控制重放，412个准入状态；无物理积分 |

[全部八回合任务收据](docs/results/post_support_validation_checkpoint_v1/all_eight_receipts.csv)保留四个候选终点误差：圆形1.041m、L形1.142m、C形1.197m、斜箱1.195m。

[冻结协议](experiments/ccta2027/post_acquisition_finite_support_v1.json)、[执行记录](experiments/ccta2027/post_acquisition_finite_support_v1_physical_plan.md)和[最终执行收据](docs/results/post_support_validation_checkpoint_v1/execution_outcome.json)分别保留身份；较早的7/8回合快照也完整保留。分析适配层仅处理预定实验臂更名和扫描数组精确比较；原错误比较、序列化失败及修复预检均保留，运行源码与冻结辅助文件未改。候选完整审计正沿原分析尝试执行；合成重放和局部审计进度均不代替完整物理验收。

最近**完整发布并审计**的闭环批次仍是[共同投票者增量八回合](docs/results/map_reference_bridge_v1/README.md)：两臂安全完成均4/4、G500均2/4，错误HOLD、原硬安全违规和预算截尾均0；但机制与完整门槛均FAIL。斜箱冻结后缩放1→17，C形最大转角4.942°→5.839°，且未正式进入TRANSPORT。全部失败和回退仍在原报告中。

最佳配置 [best_known_transport_v1.yaml](configs/research/best_known_transport_v1.yaml) 的字节和全部数值参数保持不变。冻结前核心1074通过/2跳过；本地最新核心1087通过/2跳过，见[几何与方向能力工程收据](docs/results/post_support_contact_geometry_capability_v1/engineering_checks.json)。此前1084项回归见[跨平台检查记录](docs/results/post_support_validation_checkpoint_v1/ci_portability.md)；审查修正后的1081项回归见[修正核验](docs/results/post_support_validation_checkpoint_v1/review_checks.json)；1079项回归和合成预检保留在[工程收据](docs/results/post_support_validation_checkpoint_v1/engineering_checks.json)，[Windows / Ubuntu CI](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml)按各自提交核验。测试通过与研究门槛通过分别记录；大体积原件保留本地，位置见[数据索引](docs/DATA_INDEX.md)。

## 目前遗留的问题与下一步

| 优先级 | 遗留问题及证据 | 下一项工作与验收方向 |
| --- | --- | --- |
| P0 | **有限支持与正常接触的兼容性未解决。** 新候选任务完成0/4；17个原正向力状态中，新截断约束仅6个允许正向速度，10个强制反向、1个无解。另有2个声明推手有正向可行速度却被QP选成反向。完整约束审计仍待完成。 | 完成全部逐帧约束审计，联合处理几何不确定性与有符号相对运动，设计兼容推动的边界约束，再处理优化目标。不能只删裕量、改目标或以QP可行为成功。 |
| P0 | **采集期残余冲突及上游参考偏差仍在。** 402帧agent_05的采集冲突未被本候选覆盖；旧451.596mm换票跳变虽已追清，局部配准为何接受大偏差仍待解释。 | 追踪参考准入、匹配支撑及偏差累积，分别核验地图、导航和停止读数；共同来源增量不能消除时变偏差。 |
| P0 | **C形协作激活、施力与停止尚未联合达标。** 上轮C形完成但未正式TRANSPORT、旋转回退；两臂G500均2/4，完整门槛FAIL。 | 核验部署运动、推手邻域确认、正式施力与制动的先后关系，保留原四台门槛和停止规则；共同检查安全完成、错误HOLD、硬违规、截尾、终点误差、G500和转角。 |
| P1 | **独立部署贡献未证明最终完成率优势。** [36条历史匹配轨迹](docs/results/deployment_contribution_v1/README.md)中，CVT覆盖率相对spacing/nearest增加1.52/4.85个百分点，三方法完成均11/12。 | 公共搬运链稳定后，用相同参数、初态与信息权限冻结匹配对照，分别评估覆盖、最大空缺、路径、耗时和最终搬运。 |
| P1 | **泛化与动态安全证据不足。** 开发条件反复使用；有限边界延伸、运动估计、旋转与接触模型仍有假设。 | 开发验收后再冻结未使用初态、形状、偏置接触、噪声和摩擦条件；不将保存状态可行率或四条件比例当成总体可靠率。 |
| P1 | **论文正文落后于最新证据。** [当前初稿](manuscript/ccta2027/README.md)仍主要基于旧27回合普查。 | 更新主张—证据对应、图表和局限，区分开发、复现与新验证，保留全部负面结果。 |
| P2 | **全系统去中心化、实机及环境一致性仍待验证。** 现有全队进度聚合和共同阶段监督仍在；MAS离线测试不是实机验证。joblib/PyYAML本机为1.5.1/6.0.2，锁文件为1.6.0/6.0.3。 | 先完成仿真可靠性验收，再单独验证通信限制、硬件和锁环境；历史数据保持原环境身份。 |

后续顺序：**完成当前八回合的逐帧审计 → 处理推动接触、采集冲突、参考偏差和C形激活 → 四条件完整搬运验收 → 更广原成功保持与未用条件验证 → 部署贡献匹配对照 → 更新论文。** 新方案继续默认关闭，最佳参数和成功/安全门槛保持不变。

## 前序进展与证据入口

2026-11-03：[初始任务参考](docs/results/task_reference_v1/README.md)完成固定12回合：错误 HOLD 1/4→0/4，但完整任务完成3/4→1/4，候选未采用、默认关闭。[部署贡献审计](docs/results/deployment_contribution_v1/README.md)核验36条既有匹配轨迹：CVT改善共同部署时段覆盖，三方法完整搬运均11/12，不能宣称完成率优势。

[匹配支撑固定筛选](docs/results/reference_support_v1/README.md)：8条同输入轨迹已完成；局部大误差减少但未通过团队误差门槛，新增物理回合0，候选默认关闭。


[最新参考运动与论文证据核验](docs/results/reference_motion_audit_v1/README.md)：同输入参考运动核验完成：24条旧轨迹、6条远场原配准器精确重放，新增物理回合0；区分初始化、参考点旋转和匹配误差，并核查论文主张与验证缺口。


[最新参考初始化消融](docs/results/reference_origin_v1/README.md)：参考初始化固定消融完成：24回合（8个已知开发条件×3臂），三臂均7/8完成；取消初始化放大救回斜箱但损失远场C形，提前参考未救回斜箱且G500下降。两项均未采用，原判据和默认方法不变。


[最新投票与停止消融](docs/results/voter_continuity_v1/README.md)：投票集合与停止确认固定消融：32回合（8个已知开发条件×4臂），分别检验换票读数连续化和1秒到达确认，保留全部失败与截尾；默认关闭、原成功安全判据不变。


**Decentralized Boundary-Aware Cooperative Transport** 研究多机器人如何利用局部边界观测，围绕未知形状物体完成部署，并在接触后协作搬运。本仓库提供感知与局部地图、边界密度/CVT 部署、安全 QP、控制阶段、仿真与独立评价，以及可追溯的实验运行和分析工具。

这是持续研究中的科研软件。静态部署分析有明确适用条件；动态接触、鲁棒搬运及实机性能仍有未解决问题。程序执行完成与科研任务成功分别记录，失败案例保留在结果中。

两个活动分支是 `main` 和 [`paper/boundary-aware-safe-deployment`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment)。本轮诊断、默认关闭的有限支持实现及README沿现有研究分支更新。此前地图点源及桥接实现、完整八回合发布、独立审查分别记录于 `1f50d8d`、`2a09f3f`、`bceb1c4`。最佳数值参数与历史结果均保留；新旧运行源码身份分别记录，不把旧轨迹重标为新实现验证。大体积原始轨迹、源码快照和环境保留本地，Git发布实现、设计与完整分母的精简证据。

**阅读入口：** [架构](docs/ARCHITECTURE.md) · [实验协议](experiments/ccta2027/README.md) · [冻结结果](docs/results/README.md) · [数据索引](docs/DATA_INDEX.md) · [论文稿件](manuscript/ccta2027/README.md) · [仓库指南](docs/REPOSITORY_GUIDE.md)

## 当前研究：从局部边界部署到实际搬运和停止

[运动估计先验复验](docs/results/transport_validation_v1/README.md)：固定新增97回合，核验后复用41条历史比较记录。旧33个近场条件中，CVT安全完成31/33→33/33、G500 27/33→31/33；冻结后的12个新初态中，CVT仍为11/12、G500仍9/12，未通过“新增完成数”的预定门槛。共同候选参数下，CVT、spacing、nearest均11/12完成，G500分别9/12、6/12、8/12，完成率优势未成立。所有回合无硬安全违规；远场斜箱候选仍在0.135米误差处停止，超过原0.12米容差。保留失败与停止时估计偏差；默认值和论文题目不变，不把有限条件比例视为任意形状成功概率。

[可靠运输与G500参数开发](docs/results/transport_tuning_v1/README.md)：固定完成48回合筛选＋24回合迁移检查，原成功与安全判据不变。提高运动估计速度先验的CVT候选在八个筛选条件上安全完成4/8→7/8、G500 2/8→6/8；另四个迁移条件中，CVT安全完成2/4→4/4，spacing、nearest各2/4→3/4。CVT迁移约束缩放24→0次，但spacing为25→178次，不能笼统宣称全部方法更安全。原参数回合精确复现，失败保留；未更改全局默认值，不把开发计数解释为任意形状可靠概率。[G500定义、失败机制与后续对照](docs/TRANSPORT_TUNING_RESEARCH_2026-11-02.md)。

[不规则货物与远场比较](docs/results/cargo_discovery_v1/README.md)：固定近场132回合（33条件×4方法），安全完成为边界 CVT 31/33；spacing 31/33；nearest 31/33；检测点密度 CVT 1/33；远场12回合中未知位置 3/6 完成、6/6 检测；初始位置提示 3/6 完成、6/6 检测；硬安全失败：未知位置 0/6、提示 1/6。两组分别冻结，完整负结果保留；未达到相对全部对照的预先开发优势门槛，不能宣称整体算法优越；应按具体条件与指标报告得失。

[最新坐标一致修复](docs/results/contact_consistency_v1/README.md)：固定 18 回合，九个开发条件、同一 seed 2、每例最多 600 秒。共同参考配置下，旧聚合公式完成 5/9，坐标一致修正完成 8/9；G500 分别 2/9、7/9，修正臂原硬安全检查违规 0/9。C 背向仍失败且出现越过目标后的错误停止；8/9 不是总体成功概率。本轮是公共安全计算修复的配对比较，不是原 CVT 对 spacing 的优越性检验。

[最新几何读出消融](docs/results/contact_geometry_v1/README.md)：固定 12 回合已完成：原参考 0/3、参考点修正 0/3、接触距离拟合 0/3、组合 0/3。没有候选达到预设任务救回门槛。保留有支持的诊断与实验开关，默认关闭；不追加回合或事后调参。 圆形位移误差与接触能力、椭圆结束确认、C形接触可达性分别核查，全部结果及原判据保留。

[最新恢复与增援比较](docs/results/contact_reinforcement_v1/README.md)按固定预算完成三条件×三臂九回合，每例最多600秒：原参考 0/3、仅修恢复 0/3、修复加增援 0/3。未达到本轮开发门槛。停止这一冻结候选的继续调参，保留实验开关，默认关闭。原参考三例全轨迹和历史字段精确复现；全部失败、G500与安全指标见[九回合能力表](docs/results/contact_reinforcement_v1/capability_table.md)。无附着、局部信息和原判据保持，论文题目未改；不将这三个失败条件与历史成功拼成新总体成功率。

[上一轮位置/速度分量比较](docs/results/feedback_components_v1/README.md)新增36个冻结回合，并明确复用27个历史任务参照。共同准入下的旧通道、只换位置、只换速度、双通道分别完成3/9、6/9、4/9、6/9，G500为2/9、2/9、1/9、3/9；原CVT历史参照为完成5/9、G500 5/9。位置反馈的收益与速度反馈的取舍分开记录，圆形、椭圆平行和C背向仍全部未完成。现有暂停/恢复存在计数循环依赖，但影子资格也不足，不能宣称修复计数就能救回C形。默认方法不提升，见[完整能力表](docs/results/feedback_components_v1/capability_table.md)与[研究判断](docs/results/feedback_components_v1/DECISION.md)。仅仿真，无附着，原物理、成功和安全判据及论文题目不变。

[上一轮位移估计与停止比较](docs/results/displacement_stop_v1/README.md)先从旧 C 背向的12000帧记录精确重建约0.5米的估计偏差，再比较一个固定观测参考候选。相同历史输入上的平均位移误差从0.282米降至0.012米；新冻结36回合中，原CVT、spacing、旧二维CVT均完成5/9，候选在600秒内完成6/9（150秒仍5/9）。新增椭圆垂直方向完成，但其25次屏障缩放使G500仍失败；新C背向即使估计较准，仍因推进不足而失败、终点误差退化到0.784米。候选保持默认关闭，不据这一个反复使用的开发seed宣称可靠性或算法优越。见[全部结果](docs/results/displacement_stop_v1/capability_table.md)和[诊断与局限](docs/DISPLACEMENT_STOP_RESEARCH_2026-11-01.md)。当前只做仿真，保留MAS接口，论文题目不变。

[上一轮六方法比较](docs/results/boundary_navigation_v1/README.md)完成54回合固定开发试验（每例最多600秒）：原CVT、spacing、push-cone、侧向推手、二维反馈＋CVT、二维反馈＋spacing分别完成5/9、5/9、4/9、4/9、5/9、4/9，150秒与600秒完成数相同。二维CVT降低四个原失败条件的终点误差，但没有救回完成；部分候选还出现穿透预算超限，全部保持实验选项，不替换默认方法。参见[完整能力表](docs/results/boundary_navigation_v1/capability_table.md)及[研究解释](docs/BOUNDARY_NAVIGATION_RESEARCH_2026-11-01.md)。

[多形状完整任务普查](docs/results/multishape_census_v1/README.md)恢复圆形、椭圆、矩形、L 形和 C 形：九个形状—朝向条件，三种既有部署方法，一个固定开发 seed，共 27 回合。各方法共用感知、反馈、物理、安全和完成判据。几何就绪、真实接触、收定、二维任务完成和 G500 分别记录；全部失败与 watchdog 保留。这不是独立泛化验证。

- [完整能力表](docs/results/multishape_census_v1/capability_table.md)与[全部 27 回合 CSV](docs/results/multishape_census_v1/all_27_episodes.csv)。
- [五形状阶段图](docs/results/multishape_census_v1/five_shape_stages.png)及[本地完整过程/同步视频索引](docs/results/multishape_census_v1/MEDIA.md)。
- [CCTA2027 初稿](manuscript/ccta2027/main.pdf)：*Boundary-Aware Deployment for Cooperative Transport of Unknown Objects*。
- [v2 否定结果](docs/results/progress_local_v2/README.md)保留为诊断支线；anchored 候选不再扩展。已有[累计误差分解](docs/results/multishape_census_v1/README.md#已有-v2-日志的离线分析)基于冻结日志离线生成。
- [无附着动态接触 v1](docs/results/dynamic_contact_v1/README.md)：新增固定 27 回合，原行为/安全反馈修复/动态接触分别完成 5/9、4/9、2/9。候选退化，实验开关保持关闭；代码修复、换位失败和全部原始结果均可追溯。
- [确认与回退交接 v1](docs/results/contact_handoff_v1/README.md)：先隔离压力分配与换位，再比较单个交接候选。150 秒主比较三方法仍为 5/9；35 次提议没有形成持续接管确认，候选不推广。单列双方最多 600 秒观察、全部失败、同步视频与前缀一致性检查。[研究判断和标题选项](docs/CONTACT_HANDOFF_RESEARCH_2026-11-01.md)说明下一项应检验的能力。

局部部署使用自身/邻居边界信息，但闭环还包含全队进度中位数与共同阶段监督器，因此不宣称整个搬运系统完全去中心化。G500 是旧执行契约：本配置按投影进度判到达，部分轨迹质量仅告警；它与二维完成不是严格包含关系。

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

2026-10-23仓库整合没有修改控制律、CVT定义、安全阈值、接触判据、动力学或成功标准。后续控制实验分别记录实现与运行身份，新增工程回归不冒充新增物理实验。

## 阅读入口与追溯

2026-10-31 [进度反馈跟进](docs/results/progress_local_v2/README.md)已完成实际匹配日志及固定三臂比较，更新为[6 页 CCTA 初稿](manuscript/ccta2027/README.md)。仅证书与原轨迹精确一致；无真值读出未救回失败且加重 C 形误差，停止扩大此候选。此前[oracle 诊断](docs/results/readiness_diagnostic_v1/README.md)及不利结果保留，尚不构成一般性能优势。

- [完整结果索引](docs/results/README.md)、[数据位置与完整性](docs/DATA_INDEX.md)
- [论文草稿](manuscript/ccta2027/README.md)、[文献阅读与研究定位](docs/literature_review_2026-10-22/README.md)
- [静态分析](theory/static_deployment/main_theorem.md)、[适用范围补充](theory/static_deployment/applicability_and_residual.md)
- [2026-10-23 重建记录](docs/REBUILD_2026-10-23.md)、[逐文件迁移清单](docs/migrations/2026-10-23/transfer_mapping.jsonl)
- [环境约定](AGENTS.md)、[环境安装脚本](scripts/setup_env.ps1)、[MIT License](LICENSE)
- [仓库同步与健康检查](docs/REPOSITORY_HEALTH.md)、[远程 CI](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml)

<!-- CONSOLIDATION_RESULTS_START -->
历史 Gate 0–6 汇总见 [consolidation](docs/results/consolidation/results.md)，更早机制探索见[历史闭环说明](docs/HISTORICAL_CLOSED_LOOP_README.md)；它们对应当时源码。
<!-- CONSOLIDATION_RESULTS_END -->

[独立发布审查与修正](docs/results/post_support_validation_checkpoint_v1/final_review.md)记录发布工具的异常路径修复；原实验数据和待完成的约束审计边界保持不变。
