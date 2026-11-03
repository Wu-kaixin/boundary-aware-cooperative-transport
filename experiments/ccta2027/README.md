# CCTA 2027 实验入口

2026-11-03：[接触激活诊断设计](contact_activation_v1_design.md)已完成十二回合保存谓词审计与 4,317 帧精确重放，[证据](../../docs/results/contact_activation_v1/README.md)不包含新物理样本。[观测恢复设计](reference_reacquisition_v1_design.md)对应默认关闭的新开关，工程回归 928 通过/2 跳过、针对性 57 通过；拟议四条件×两臂八回合仍待冻结执行，不能把[当前实现检查点](../../docs/results/reference_reacquisition_v1/README.md)当作闭环成功。

2026-11-03：[acquisition_circulation_v1.json](acquisition_circulation_v1.json)冻结四条件×三臂12回合，现已全部完成。原平移/参考地图/主动采集安全完成4/4、3/4、2/4，G500均2/4；八个旧臂精确复现，全部数值参数及判据不变。[完整结果](../../docs/results/acquisition_circulation_v1/README.md)保留新候选两项远场600秒截尾和约束缩放，门槛FAIL、候选未采用。[前置采集追踪](../../docs/results/acquisition_trace_v1/README.md)精确复现1,992旧帧、没有新增物理回合；[机制设计](acquisition_circulation_v1_design.md)的原验收门槛未改。

2026-11-03：[reference_map_v1.json](reference_map_v1.json)冻结原平移参照与参考驱动地图候选，四条件×两臂，最佳数值参数不变。[完整结果](../../docs/results/reference_map_v1/README.md)为4/4→3/4安全完成、门槛FAIL，候选未采用；保留斜箱600秒截尾及全部缩放。[来源追踪设计](map_provenance_v1_design.md)与[933帧离线证据](../../docs/results/map_provenance_v1/README.md)单独记账，后者没有新物理回合。

2026-11-03：[qp_conflict_v1.json](qp_conflict_v1.json)冻结四条已有轨迹的完整控制器重放与约束子集诊断，共 3,411 帧、953 个缩放步，新物理回合 0。[全部结果和独立见证](../../docs/results/qp_conflict_v1/README.md)区分联合约束矛盾、速度影响及地图失真；运行代码和最佳数值参数未改，控制修复仍待验证。

2026-11-03：[acquisition_geometry_v1.json](acquisition_geometry_v1.json)冻结平移参照与仅调整采集几何的候选，四条件×两臂，全部数值参数和判据不变。[完整八回合结果](../../docs/results/acquisition_geometry_v1/README.md)：完成均 4/4、G500 均 2/4，但斜箱采集缩放 240→689、C 形冻结后缩放 7→16，门槛 FAIL。默认关闭，不追加条件或事后放宽门槛。

2026-11-03：[task_translation_physical_v2.json](task_translation_physical_v2.json)固定四个原最佳配置回合＋四个平移组合回合，数值参数不变。[完整结果](../../docs/results/task_translation_physical_v2/README.md)：安全完成 3/4→4/4、误停 1→0、硬违规均 0、G500 均 2/4，扩展门槛 PASS、未采用。斜箱采集缩放 240 次需先处理。[v1](task_translation_physical_v1.json)仅冻结、未执行，修正预算标签计数后另冻 v2；全部原件保留。

2026-11-03：[task_translation_completion_v1.json](task_translation_completion_v1.json)为 v2 预算截尾后的独立资源补全：核验复用三项，只补算缺失斜箱，算法和阈值不变。四项数据已完整；补算末尾汇总路径错误通过独立零重放汇总恢复，原失败身份保留。[完整结果](../../docs/results/task_translation_completion_v1/README.md)仍为原筛选 FAIL，新增候选物理回合 0；[默认关闭的实验接口](task_translation_wiring_plan.md)已实现，[后续闭环草案](task_translation_physical_pilot_draft.md)尚未冻结或执行。

2026-11-03：[task_translation_v2.json](task_translation_v2.json)冻结四条旧任务参考轨迹×四个估计臂，固定比较平移模型和全部既有采集模板的准入，不改变最佳数值参数；[原发布快照](../../docs/results/task_translation_checkpoint_v1/README.md)记录当时 3/4 条件完成、整体 INCOMPLETE，新增物理回合 0。L/C 已未通过原连续反馈筛选。无效的 [v1 原协议](task_translation_v1.json)和本地运行原件保留；当时候选尚未接入控制器。

2026-11-03：[reference_surface_v1.json](reference_surface_v1.json)固定八条历史轨迹的同输入估计试验，数值参数全部保持；斜箱 MAE 改善 11.30%，未过 25% 门槛，不进入候选闭环比较。[protected_baseline_v1_reproduction.json](protected_baseline_v1_reproduction.json)另执行一个完整基线复现回合，原物理轨迹和科学字段精确一致。[结果、指标与下一步](../../docs/results/reference_surface_v1/README.md)。

2026-11-03：[task_reference_v1.json](task_reference_v1.json)固定执行12回合，三组完整完成3/4、3/4、1/4，新参考未通过采用门槛，默认关闭；[结果与停止诊断](../../docs/results/task_reference_v1/README.md)。前置[获取协议](task_reference_acquisition_v1.json)另执行8回合。既有36回合的[部署贡献审计](../../docs/results/deployment_contribution_v1/README.md)没有新增物理回合或新验证集。

[匹配支撑固定筛选](../../docs/results/reference_support_v1/README.md)：8条同输入轨迹已完成；局部大误差减少但未通过团队误差门槛，新增物理回合0，候选默认关闭。


[最新参考运动与论文证据核验](../../docs/results/reference_motion_audit_v1/README.md)：同输入参考运动核验完成：24条旧轨迹、6条远场原配准器精确重放，新增物理回合0；区分初始化、参考点旋转和匹配误差，并核查论文主张与验证缺口。


[固定24回合结果](../../docs/results/reference_origin_v1/README.md)：参考初始化固定消融完成：24回合（8个已知开发条件×3臂），三臂均7/8完成；取消初始化放大救回斜箱但损失远场C形，提前参考未救回斜箱且G500下降。两项均未采用，原判据和默认方法不变。


[固定32回合结果](../../docs/results/voter_continuity_v1/README.md)：投票集合与停止确认固定消融：32回合（8个已知开发条件×4臂），分别检验换票读数连续化和1秒到达确认，保留全部失败与截尾；默认关闭、原成功安全判据不变。


## 运动估计先验：旧条件与新初态复验

[transport_validation_v1_regression.json](transport_validation_v1_regression.json)补跑25个旧条件候选，其余33个原参数与8个候选结果核验后复用；[transport_validation_v1_new_starts.json](transport_validation_v1_new_starts.json)执行12条件×3部署模块×2参数的72回合。两协议在运行前同时冻结，仅比较运动估计先验0.25/0.45；原成功、安全、G500与600秒预算不变。新seed为20261102，近场沿用scatter规则，远场显式改变机器人起点且初始扫描全部为空。

[完整结果与失败审计](../../docs/results/transport_validation_v1/README.md)。入口为 `scripts/validate_transport_prior.py`，分析/图表入口为 `scripts/analyze_transport_validation.py` 和 `scripts/publish_transport_validation.py`。两阶段均已执行；不能重复冻结、覆盖或追加调参。只读检查可用：

```powershell
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/transport_validation_v1_new_starts.json --split validation --dry-run
```

## 坐标一致修复固定比较

[contact_consistency_v1.json](contact_consistency_v1.json) 已冻结执行。固定 18 回合，九个开发条件、同一 seed 2、每例最多 600 秒。共同参考配置下，旧聚合公式完成 5/9，坐标一致修正完成 8/9；G500 分别 2/9、7/9，修正臂原硬安全检查违规 0/9。C 背向仍失败且出现越过目标后的错误停止；8/9 不是总体成功概率。本轮是公共安全计算修复的配对比较，不是原 CVT 对 spacing 的优越性检验。 [完整结果](../../docs/results/contact_consistency_v1/README.md)。旧公式兼容保留，新开关默认关闭，本协议显式开启；不重新冻结或追加实验。

## 几何读出独立消融

[contact_geometry_v1.json](contact_geometry_v1.json)已冻结执行12回合，三条件×四臂，每例600秒、seed 2、8进程、内部单线程。原参考 0/3、参考点修正 0/3、接触距离拟合 0/3、组合 0/3。不追加调参；[完整结果](../../docs/results/contact_geometry_v1/README.md)。

## 恢复与增援固定比较

[contact_reinforcement_v1.json](contact_reinforcement_v1.json)已冻结并执行九回合：圆形、椭圆平行、C背向，各比较原双参考CVT、仅修恢复、恢复加保留推手与增援。每例600秒，seed 2，最多8案例进程、内部单线程。原参考 0/3、仅修恢复 0/3、修复加增援 0/3。结果和全部失败见[报告](../../docs/results/contact_reinforcement_v1/README.md)。不重新冻结或追加调参，原成功和安全判据、物理、估计器及稿件题目不变。

本页对应 `paper/boundary-aware-safe-deployment`。依赖安装见[项目首页](../../README.md)。
[conference.yaml](conference.yaml) 保存会议要求及上次核对日期；投稿前仍需核对官方页面。

## 最新位置/速度分量比较

[feedback_components_v1.json](feedback_components_v1.json) 已冻结执行36回合：九条件、同一开发seed 2、四个共同准入组合、每例最多600秒，8个案例进程、内部单线程。位置与速度独立切换；共同准入＋旧数值是新的诊断对照，不能称为原CVT。原CVT/spacing/旧二维CVT的27个参照保持历史身份。

四臂完成数为3/9、6/9、4/9、6/9，G500为2/9、2/9、1/9、3/9。圆形、椭圆平行和C背向均未救回；不推广默认方法。九个双通道回合用于复跑兼容核验，全部失败和来源见[完整结果](../../docs/results/feedback_components_v1/README.md)、[预先设计](../../docs/FEEDBACK_COMPONENTS_RESEARCH_2026-11-01.md)与[下一步研究判断](../../docs/results/feedback_components_v1/DECISION.md)。暂停/恢复循环问题未混入本冻结干预，须单独修复验证。

已有原始数据在 `runs/research/2026-11-01-feedback-components-v1/run/`。不重新冻结或追加运行。只读协议校验：

```powershell
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/feedback_components_v1.json --split development --dry-run
```

离线生成入口为 `scripts/analyze_feedback_components.py`，输出必须是该任务分析目录下尚不存在的子目录。报告区分任务完成、安全硬限制、G500、收定与观察截止；物理与判据不变，论文题目不变。

## 上一轮位移估计与停止比较

[displacement_stop_v1.json](displacement_stop_v1.json) 已冻结并执行36回合：九条件×原CVT/spacing/旧二维CVT/固定观测参考CVT，seed 2、每例最多600秒，8个案例进程、内部单线程。候选以CONTACT_READY后允许的自身/一跳边界扫描建立固定参考；安全地图、物理、完成判据及停止门控不变，默认仍使用旧累计读出。

新候选在600秒内完成6/9，其余三臂5/9；150秒均为5/9。新增椭圆垂直方向完成，但G500仍失败；C背向估计变准而实际推进退化。全部36回合、27个旧行为精确一致性检查及原安全指标见[完整结果](../../docs/results/displacement_stop_v1/README.md)。本协议不重新冻结、不追加调参运行。已有输出在 `runs/research/2026-11-01-displacement-stop-v1/run/`。

只读核验协议，不启动新回合：

```powershell
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/displacement_stop_v1.json --split development --dry-run
```

离线生成入口及输出身份见[研究记录](../../docs/DISPLACEMENT_STOP_RESEARCH_2026-11-01.md)与[图表清单](../../docs/results/displacement_stop_v1/FIGURES.md)。已有结果不因当前工作区后续改变而重新标记为新版本证据。

## 上一轮六方法边界反馈比较

[boundary_navigation_v1.json](boundary_navigation_v1.json) 冻结54回合：九条件×六方法、seed=2、每例最多12000帧（600秒），同一轨迹分别评价150秒和600秒。新加入已有push-cone部署对照、侧向推手筛选、二维反馈＋CVT、二维反馈＋spacing。原CVT/spacing保持不变，原安全约束、物理和独立成功判据不变；二维控制到达策略作为方法改动明确记录。

全部执行结束、0执行错误；完成数依次为5/9、5/9、4/9、4/9、5/9、4/9，未提高成功数。候选出现的退化、穿透超限和估计—物理到达差异见[完整结果](../../docs/results/boundary_navigation_v1/README.md)。不重新冻结此协议，不在原输出中追加调参运行；原始数据留在 `runs/research/2026-11-01-boundary-navigation-v1/run/`。

## 当前多形状完整任务普查

[multishape_census_v1.json](multishape_census_v1.json) 是独立冻结协议：九个形状—朝向条件 × 三种部署方法 × seed 2，27 回合，每例最多 3000 帧、dt=0.05 s，沿用原收定规则。全部世界目标方向为 +x、距离 1.2 m；形状归一化到同面积和同质量，惯量和周长不强行统一。不是原 169 例批次的扩充或复现。

圆为 64 边形，椭圆为 24 顶点；当前矩形/L/C 轮廓来源保留，C 原开口 +y。旋转、实际物理元数据、初态哈希和完整配置均在冻结文件内。三种方法仅改变首次 CONTACT_READY 前的部署目标。结果、全部失败与媒体见[普查报告](../../docs/results/multishape_census_v1/README.md)。

只读解析，不新增实验：

```powershell
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/multishape_census_v1.json --split development --dry-run
```

已有原始数据的离线分析入口为 `scripts/analyze_multishape_census.py`；`--videos` 输出共同物理时间轴的回放。冻结生成脚本 `scripts/freeze_multishape_census.py` 拒绝覆盖已有协议；本次不再启动第二轮候选搜索或新种子矩阵。

## 快速验证管线

从仓库根目录运行，使用根 `.venv`，输出目录首次使用时应不存在。Windows PowerShell：

```powershell
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --dry-run
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/ccta2027/smoke_parallel
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 1 --out runs/ccta2027/smoke_serial
.\.venv\Scripts\python.exe -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/ccta2027/smoke_parallel --resume
.\.venv\Scripts\python.exe -m dbact_sim evidence --batch smoke=runs/ccta2027/smoke_parallel --out runs/ccta2027/smoke_evidence
```

五例覆盖 oracle/local 静态执行和 CVT/最近目标/局部间距搬运。
4 或 12 帧只用于检查程序，不评价收敛、任务完成或优越性。

## 已冻结的 169 例 CCTA 矩阵

[revision.yaml](revision.yaml) 定义 169 案例：静态 27、开发 27、消融 15、主动探索诊断 6、
新种子泛化 36、扰动鲁棒性 54、数值参考 2、接触引擎参考 2。
该历史批次结果见[实验报告](../../docs/CCTA2027_EXPERIMENTS.md)与
[归档包](../../docs/results/ccta2027_draft_v2/README.md)。审计 PASS 只说明约定的完整性和实现检查通过。

### 冻结与运行新修订

以下为 Windows PowerShell，从仓库根目录运行。选择尚不存在的目录：

```powershell
.\.venv\Scripts\python.exe -m dbact_sim campaign freeze --plan experiments/ccta2027/revision.yaml --out runs/ccta2027/next_revision/frozen_protocol.yaml
$campaignRoot = (Resolve-Path runs/ccta2027/next_revision).Path
$env:PYTHONPATH = Join-Path $campaignRoot 'execution_source/src'
$env:DBACT_ROOT = Join-Path $campaignRoot 'execution_source'
.\.venv\Scripts\python.exe -m dbact_sim study --protocol "$campaignRoot/frozen_protocol.yaml" --split all --workers 4 --out "$campaignRoot/run"
.\.venv\Scripts\python.exe -m dbact_sim campaign audit --protocol "$campaignRoot/frozen_protocol.yaml" --run "$campaignRoot/run" --out "$campaignRoot/acceptance.json"
```

冻结后必须从 `execution_source` 执行，协议与输出传绝对路径，避免工作目录中的修改混入。
工人数按机器资源调整；原 v2 使用 16 工人，另有两例串行复核。

分析在另一个 PowerShell 会话中从仓库根目录运行，使用当前分析代码：

```powershell
.\.venv\Scripts\python.exe -m dbact_sim evidence --batch revision=runs/ccta2027/next_revision/run --by-split --out runs/ccta2027/next_revision/evidence
.\.venv\Scripts\python.exe -m dbact_sim.experiments.report --run runs/ccta2027/next_revision/run --out runs/ccta2027/next_revision/figures
```

复现已经发布的 v2 时，应使用归档包内的原始模型源码及独立的 `analysis_source`，
而不是用当前 checkout 给历史冻结重新归属。无需运行物理也可从精简结果重建统计和四组论文图；
具体步骤见归档包 README。完整接触时序图还需要本地原始诊断。

## 输出和恢复

```text
manifest.json               协议、源码、环境与 Git 身份
resolved_protocol.json      展开的案例配置
source_snapshot/            运行时源码快照
resources.csv              资源采样
<case_id>/resolved_case.json 精确输入
<case_id>/case_result.json   结果、身份及产品哈希
<case_id>/...               回放与诊断
summary.json                全部已记录案例、分母和执行状态
```

`--resume` 只恢复身份和所有产品一致的同一次实验。
变更源码、协议或环境后必须另建目录。执行或进程错误保留记录并返回非零；
测量到的任务失败保留在统计中，不能作为坏数据删除。

## 历史协议

2026-11-01 接触交接采用三个独立冻结协议：
[消融](contact_handoff_v1_ablation.json) 12 回合、[主比较](contact_handoff_v1_main.json) 27 回合、
[延长观察](contact_handoff_v1_extended.json) 18 回合。[候选定义](contact_handoff_v1_design.json)
记录局部消息权限、确认代理、有限尝试、逐步卸载与回退。原物理、安全和完成判据不变；
150 秒与 600 秒结果分别报告，不能延长候选却仅给基线原预算。
[结果与失败边界](../../docs/results/contact_handoff_v1/README.md)明确区分程序完成、任务完成和实际接管。
冻结之后不修改本协议或重复冻结来追求成功；`confirmed_contact_handoff` 默认关闭。

2026-11-01 [dynamic_contact_v1.json](dynamic_contact_v1.json) 已完成固定 27 回合：原行为、安全反馈修复、修复加动态接触三个版本，九个形状/方向条件，seed 2。没有加入附着或真值控制，未改变物理、安全与成功阈值。[结果](../../docs/results/dynamic_contact_v1/README.md)为 5/9、4/9、2/9 完成；候选未通过事前门槛，两个新控制开关保持关闭。该协议不同于旧的三部署方法普查和仅设计未执行的 `decision_data_plan_v1`，不重新归属历史数据。

```powershell
.\.venv\Scripts\python.exe -m dbact_sim study --protocol docs/paper_simulation_protocol.test_frozen_v1.yaml --split test --dry-run
```

此命令只解析历史 36 例。真正执行必须使用对应冻结源码和环境；
当前重构源码不能冒充旧源码，也不能把原测试集重新标为新的泛化验证。
历史定义见 [paper_simulation_protocol.md](../../docs/paper_simulation_protocol.md)。

## 不规则货物与位置权限的分离开发比较

[cargo_near_v1.json](cargo_near_v1.json)冻结33条件×四方法=132回合；[cargo_far_v1.json](cargo_far_v1.json)冻结六物理初态×两位置权限臂=12回合。每例最多600秒、seed 2、同物理/评价；[结果和边界](../../docs/results/cargo_discovery_v1/README.md)保留失败。协议冻结后不得按当前新代码直接重标历史身份；复现需对应执行快照。

## 可靠运输与G500的有限参数开发

[筛选协议](transport_tuning_v1_screen.json)事先固定8条件×6设置=48回合；按其中选择规则确定一个候选后，[迁移协议](transport_tuning_v1_transfer.json)执行4条件×3部署方法×2设置=24回合。全部已完成，每例上限600秒，原成功与安全判据不变。参数组合、初态、失败均不可事后替换。[完整结果](../../docs/results/transport_tuning_v1/README.md)区分任务、G500与安全代价；候选不自动覆盖全局默认。

只读复核可使用根环境执行 `scripts/transport_tuning.py run --stage screen --dry-run` 或 `--stage transfer --dry-run`。真实执行输出已存在，不得覆盖；同身份恢复须先检查当前进程和冻结源码，再使用 `--resume`，不会把旧结果重标为新环境的验证。
