# DBACT：边界感知部署与协作搬运

## 最新成果与保护基线（2026-11-03）

本项目的目标是利用局部边界感知和邻域通信，对未知形状物体安全部署，完成无附着协作搬运和可靠停止，并通过匹配对照实验单独验证边界部署的贡献，最终形成符合 CCTA2027 要求的论文初稿。当前保留既有共享阶段监督器，先建立可复现的仿真证据。

**最佳公共配置已完整保存，本轮未改既有数值参数、物理模型或成功/安全判据。** [可运行配置](configs/research/best_known_transport_v1.yaml)以近场 L 形为例，显式保存原控制参数；[使用说明](configs/research/README.md)和[参数来源及 SHA-256](docs/results/reference_surface_v1/protected_baseline.json)用于核验。其余形状及初态仍由原实验协议提供。全局历史默认值保持兼容，新参考面候选默认关闭。

**最新诊断：[联合安全 QP 完整重放](docs/results/qp_conflict_v1/README.md)。** 四条已有远场前缀、**3,411 帧**逐帧精确复现，新增物理回合 **0**。953 个缩放机器人步的联合约束均不可行；去掉 `rho` 后，机器人约束和物体约束各自可行、联合仍不可行；再将估计物体/边界点速度置零，仅 205 个恢复可行。11 个不同见证经独立求解器核验；其中 9 个即使取消速度上限仍有线性矛盾。C 形还发现一个地图点距真实边界至少 **0.590 m**。下一步追踪安全地图的参考和观测有效性，并单独处理邻域协作退让；尚未宣称运行修复。[独立最终审查](docs/results/qp_conflict_v1/final_review.md)已通过，无未解决审查项。

前序[采集几何一致性八回合比较](docs/results/acquisition_geometry_v1/README.md)未通过冻结门槛，候选不采用、默认关闭。原最佳数值参数不变，两臂安全完成均 **4/4**、错误 HOLD 和硬安全失败均为 **0**、G500 均 **2/4**；但斜箱采集期缩放 **240→689**，C 形冻结后缩放 **7→16**。四个平移参照的全部回放数组和科学字段精确复现；几何目标一致不代表联合安全约束可行。

前序[平移任务参考首次闭环比较](docs/results/task_translation_physical_v2/README.md)固定新增 8 回合：安全完成 **3/4→4/4**、错误 HOLD **1→0**，远场斜箱终点误差 **135.106→18.126 mm**。这项改善保留为研究参照；四个复用开发条件不足以证明普遍可靠性，所有新机制仍默认关闭。

前序[四条件×四臂同输入比较](docs/results/task_translation_completion_v1/README.md)的原冻结筛选仍为 **FAIL**。新物理试验是另行冻结的推进与收定问题，没有修改旧连续反馈门槛。[原 3/4 时间戳快照](docs/results/task_translation_checkpoint_v1/README.md)、预算截尾、汇总恢复及全部负结果均保留。

前序[自由射线信息诊断](docs/results/free_ray_enclosure_v1/README.md)定位了观测覆盖限制：四个采集冻结快照中，64 份单机和 64 份一跳信息均不能缩小未知中心范围；全队信息只在两个近场形成包围，远场仍缺少观测。全部 132 份集合均包含真实物体，但该诊断中的圆形虚假旋转阻止了可靠停止。该诊断新增物理回合为 0，组件未接入控制器，没有形成误停修复；后续平移试验的结果另行记录。

| 证据范围 | 已核验结果 | 适用范围 |
| --- | --- | --- |
| 联合 QP 诊断：四条既有远场前缀 | 3,411 帧精确重放；953 个缩放步，去 `rho` 后再将估计物体/边界点速度置零，仅 205 个恢复可行；11 个见证独立核验一致 | 新增物理回合 0；已定位约束矛盾及边界图失真，尚未验证控制修复 |
| 采集几何：四条件×两臂 | 两臂安全完成均 4/4；斜箱采集缩放 240→689，C 形冻结后缩放 7→16 | 原数值参数不变；冻结门槛 FAIL、候选未采用，全部退化保留 |
| 首次平移闭环：四条件×两臂 | 安全完成 3/4→4/4，误停 1→0；C/斜箱终点误差 12.925/18.126 mm | 四个复用开发条件；G500 仍 2/4，斜箱采集缩放 240 次，候选未采用 |
| 平移参考完整比较：四条件×四臂 | C/斜箱组合 MAE 为 14.270/3.802 mm；四票帧数分别由 4,020/11,549、1,296/11,189 提升到全部帧 | 三项旧输出核验后复用、一项补算；原筛选 FAIL，无新闭环成功 |
| 最佳公共配置的历史结果 | 旧近场安全完成 33/33、G500 31/33；后续初态安全完成 11/12、G500 9/12 | 两批分别报告，不能合并为总体可靠率 |
| 前序完整基线复现：近场 L 形 | 19.65 s 完成，终点误差 8.433 mm，G500 通过；硬安全失败和屏障缩放均为 0 | 一个新执行的旧条件回合；25 个 replay 数组及全部旧科学字段精确复现 |
| 前序八条同输入参考面试验 | 斜箱位移 MAE 125.735→111.533 mm，改善 11.30%；八块团队反馈可用率相对原方法均为 100% | 未达预定 25% 改善门槛；候选未采用，新增候选闭环回合为 0 |
| 边界部署独立贡献审计 | CVT 相对 spacing / nearest，共同时段平均覆盖率增加 1.52 / 4.85 个百分点 | 使用 36 条既有匹配轨迹；三方法完整搬运均 11/12，完成率优势未成立 |
| 自由射线包围诊断 | 局部中心集合收缩 0/128；全队条件集合收缩 2/4；真实物体排除 0/132 | 四个复用开发快照；16,687 个旧基本停止候选帧无新增上界及票数同时通过，不能计作新成功 |

[完整试验报告与图表](docs/results/reference_surface_v1/README.md) · [全部八块数据](docs/results/reference_surface_v1/all_eight_blocks.csv) · [基线复现证据](docs/results/reference_surface_v1/baseline_reproduction.json) · [部署贡献审计](docs/results/deployment_contribution_v1/README.md)

现行控制代码的[本地完整回归](docs/results/acquisition_geometry_v1/engineering_checks.json)为 **892 项通过、2 项跳过**，针对性检查 37 项通过；[Windows / Ubuntu CI](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml)按各自提交记录核心、MAS 和 smoke 检查。前序采集几何八回合的四个参照精确复现，产品身份通过核验；最新 QP 诊断沿用同一运行源码，新增物理回合为 0。单元测试不等同于研究成功。Git 跟踪代码、配置、协议和精简证据；大体积原始轨迹及源码快照仍按[数据索引](docs/DATA_INDEX.md)保留在本地。

## 目前遗留的问题与下一步

| 优先级 | 遗留问题及证据 | 下一项工作与验收方向 |
| --- | --- | --- |
| P0 | **联合安全约束和地图有效性尚未修复。** 已精确复原 953 个缩放步：去掉 `rho` 后，两个约束子集各自可行但联合不可行；9 个独立见证有不依赖速度上限的线性矛盾。C 形部分地图点已偏离真实边界至少 0.590 m。 | 追踪地图运动参考与陈旧点的观测支持，验证一个共同参考/观测有效性修复；另行处理邻域协作退让。保留原参照和已否决候选，不放宽安全规则。 |
| P0 | **远场误停修复仅在四个开发条件中成立。** 新候选已救回斜箱，四例无错误 HOLD，但尚未覆盖全部旧 33 条件、12 个初态及未用条件。 | 完成采集安全机制检查后，冻结更广的原成功保持和新条件验证；不按形状混用控制器，不把 4/4 当成总体可靠率。 |
| P1 | **真实旋转和信息不足仍限制目标参考。** 平移参考中的零拟合转角是假设；C 形最大真实旋转从前序平移参照 2.574° 增至本轮几何候选 3.575°，自由射线诊断的远场观测仍不能包围未知中心。 | 明确旋转适用边界，检验更大旋转、偏置接触和不完整观测；不得把条件上界称作配准证书。旧重放 FAIL 和初始参考 1/4 完成的负结果均保留。 |
| P1 | **独立部署贡献仍限于覆盖等过程指标。** CVT、spacing、nearest 的完成数同为 11/12；公共估计/安全修复不能归作 CVT 的独立贡献。 | 在公共搬运链稳定后，以相同参数、初态和信息权限做匹配对照，分别评价部署覆盖、最大空缺、路径/时间和最终搬运。 |
| P1 | **泛化、旋转适用范围与完整动态安全证据不足。** 当前条件已反复用于开发；[真实旋转背景核验](docs/results/task_translation_rotation_context_v1/README.md)发现最佳参数 12 条轨迹的最大转角为 3.504°，不能将平移模型中的零拟合转角当成真实零旋转。静态部署分析也不能直接推广为动态接触安全保证。 | 开发验收通过后，再冻结未使用初态、形状、旋转/偏置接触、噪声与摩擦条件；分开报告硬安全、G500、任务完成及失败原因。 |
| P1 | **论文正文与最新研究进展尚未统一。** [当前初稿](manuscript/ccta2027/README.md)仍以旧 27 回合普查为主；[CCTA2027 官方要求复核](docs/CCTA2027_REQUIREMENTS_CHECK_2026-11-03.md)仅核对交付格式，并非研究完成验收。 | 更新主张—证据对应关系与图表，区分开发、复现和新验证；不能用新结果覆盖旧失败，也不能把部署覆盖改善写成完成率优势。 |
| P2 | **全系统去中心化和实机尚未验证。** 当前搬运仍使用全队进度聚合及共同阶段监督，MAS 接口与离线测试不代表硬件验证完成。 | 明确信息边界，在仿真可靠性成立后另行设计通信受限和硬件验证，不提前扩大论文主张。 |
| P2 | **本地环境与锁文件存在差异。** 当前 joblib / PyYAML 为 1.5.1 / 6.0.2，锁文件为 1.6.0 / 6.0.3。 | 保留原环境与运行身份；未来环境一致性验证单独冻结，不能把历史数据重标为锁环境复跑。 |

下一阶段顺序：**追踪安全地图参考与观测有效性 → 验证对应修复及邻域退让可行性 → 更广的原成功保持与未用条件验证 → 公共搬运链下的部署贡献对照 → 更新论文。** 主指标是安全完整完成、错误 HOLD、硬安全失败和预算截尾；辅指标包括终点误差、成功耗时、G500、间距/穿透、分阶段屏障缩放、估计误差及反馈可用率。[最新诊断](docs/results/qp_conflict_v1/README.md)与[原物理结果](docs/results/acquisition_geometry_v1/README.md)分别保存，不混淆数据类型。

## 前序进展与证据入口

2026-11-03：[初始任务参考](docs/results/task_reference_v1/README.md)完成固定12回合：错误 HOLD 1/4→0/4，但完整任务完成3/4→1/4，候选未采用、默认关闭。[部署贡献审计](docs/results/deployment_contribution_v1/README.md)核验36条既有匹配轨迹：CVT改善共同部署时段覆盖，三方法完整搬运均11/12，不能宣称完成率优势。

[匹配支撑固定筛选](docs/results/reference_support_v1/README.md)：8条同输入轨迹已完成；局部大误差减少但未通过团队误差门槛，新增物理回合0，候选默认关闭。


[最新参考运动与论文证据核验](docs/results/reference_motion_audit_v1/README.md)：同输入参考运动核验完成：24条旧轨迹、6条远场原配准器精确重放，新增物理回合0；区分初始化、参考点旋转和匹配误差，并核查论文主张与验证缺口。


[最新参考初始化消融](docs/results/reference_origin_v1/README.md)：参考初始化固定消融完成：24回合（8个已知开发条件×3臂），三臂均7/8完成；取消初始化放大救回斜箱但损失远场C形，提前参考未救回斜箱且G500下降。两项均未采用，原判据和默认方法不变。


[最新投票与停止消融](docs/results/voter_continuity_v1/README.md)：投票集合与停止确认固定消融：32回合（8个已知开发条件×4臂），分别检验换票读数连续化和1秒到达确认，保留全部失败与截尾；默认关闭、原成功安全判据不变。


**Decentralized Boundary-Aware Cooperative Transport** 研究多机器人如何利用局部边界观测，围绕未知形状物体完成部署，并在接触后协作搬运。本仓库提供感知与局部地图、边界密度/CVT 部署、安全 QP、控制阶段、仿真与独立评价，以及可追溯的实验运行和分析工具。

这是持续研究中的科研软件。静态部署分析有明确适用条件；动态接触、鲁棒搬运及实机性能仍有未解决问题。程序执行完成与科研任务成功分别记录，失败案例保留在结果中。

两个活动分支是 `main` 和 [`paper/boundary-aware-safe-deployment`](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment)。本轮成果与 README 沿现有研究分支更新；既有研究于 `27d17ad` 同步，跨平台分析导入修复于 `6e7958c` 同步，初始任务参考与部署审计于 `ff585b7` 同步，最佳参数保护与参考面试验于 `bee74ee` 同步。实验运行身份仍以各自冻结源码快照和收据为准，不能因后续提交而重标记旧结果。

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

本次维护不修改控制律、CVT定义、安全阈值、接触判据、动力学或成功标准；新增工程回归不冒充新增物理实验。

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
