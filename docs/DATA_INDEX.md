# 数据与原始文件索引

2026-11-03：[参考驱动地图八回合](results/reference_map_v1/README.md)的原始数据在 `runs/research/2026-11-03-reference-map-v1/run/`，源码/脚本/协议冻结在同级 `freeze/`；全部8例均执行完成，候选3/4成功，斜箱600秒截尾，门槛FAIL。核心903通过/2跳过、针对性71通过及后验C形阶段审计在 `runs/maintenance/2026-11-03-reference-map-v1/`。[地图来源追踪](results/map_provenance_v1/README.md)的933帧逐阶段数组、独立祖先分析在 `runs/research/2026-11-03-map-provenance-v1/`，首次失败与第二次通过预检另保留在同名maintenance目录。旧数据和冻结源码均未覆盖。

2026-11-03：[联合 QP 诊断](results/qp_conflict_v1/README.md)的四条精确重放、每步可行性和选定完整矩阵在 `runs/research/2026-11-03-qp-conflict-v1/`，执行前冻结在同级 `freeze/`；原始物理数据仍在 acquisition-geometry-v1。预检首轮失败、原脚本及第二轮通过分别保留在 `runs/maintenance/2026-11-03-qp-conflict-v1/preflight1/`、`preflight2/`。953 步精简 CSV、代数见证和图表已入 Git；新增物理回合 0。

2026-11-03：[采集几何八回合](results/acquisition_geometry_v1/README.md)的完整原始结果位于 `runs/research/2026-11-03-acquisition-geometry-v1/run/`，执行前源码/脚本/协议/环境在同级 `freeze/`。全部门槛及逐帧缩放索引在 `analysis.json`；冻结门槛 FAIL，不修改旧平移试验。工程 RED、首轮测试消息不匹配、最终 37 项针对性通过和完整 892 通过/2 跳过日志分别保存在 `runs/maintenance/2026-11-03-acquisition-geometry-v1/`。

2026-11-03：[首次平移闭环八回合](results/task_translation_physical_v2/README.md)的原始产品在 `runs/research/2026-11-03-task-translation-physical-v2/run/`，执行前源码/脚本/协议在同级 `freeze/`，屏障事件明细在 `barrier_diagnostic_v1/`。v1 只冻结、未执行，原件与退役收据保留在 `2026-11-03-task-translation-physical-v1/`；v2 在任何新物理数据前修正预算标签统计。八个新回合及全部原失败分别记账，不与旧重放混算。

重放完成后的控制器接口检查在 `runs/maintenance/2026-11-03-task-translation-completion-v1/`：`wiring-red.*` 保留实现前失败，`wiring-green.*` 保留 33 项针对性通过；`core-wiring.*` 是默认沙箱临时目录权限导致的失败，`core-wiring-v2.*` 使用新的仓库内临时目录。它们均为工程检查，不是物理试验；原重放及环境身份不变。

2026-11-03：[四条件完整结果](results/task_translation_completion_v1/README.md)由前三项旧完整输出及新补算斜箱构成；补算数组/源码在 `runs/research/2026-11-03-task-translation-completion-v1/`。该次逐帧计算完成后，汇总路径错误导致执行退出，原失败收据保留；独立汇总恢复在 `runs/research/2026-11-03-task-translation-finalization-v1/`，没有重跑估计器或物理。所有原始 v1/v2/补算/快照均保留，不能把汇总恢复当成新的闭环验证。

2026-11-03：[平移任务参考进展](results/task_translation_checkpoint_v1/README.md)是带时间戳的 3/4 条件快照，不是完整结论。原始重放、模板、逐帧数组和源码在 `runs/research/2026-11-03-task-translation-v2/`；v1 严格重建检查失败的原件在同级 `2026-11-03-task-translation-v1/`，未被覆盖。旋转背景与停止筛选校准分别在 `runs/maintenance/2026-11-03-task-translation-rotation-context/` 和 `runs/maintenance/2026-11-03-task-translation-v2/`。Git 只保存协议与精简证据，新增物理回合为 0。

2026-11-03：[自由射线包围诊断](results/free_ray_enclosure_v1/README.md)使用四个原采集冻结快照，新增物理回合 0；清理后的测量包、完整 132 份集合、旧停止帧反事实统计和独立源码快照保存在 `runs/research/2026-11-03-free-ray-enclosure-v1/`。局部收缩 0/128，全队收缩 2/4，未作为运行修复采用。

2026-11-03：[保护最优参数的参考面试验](results/reference_surface_v1/README.md)保存八条同输入重放于 `runs/research/2026-11-03-reference-surface-v1/`；一个完整基线复现于 `runs/research/2026-11-03-protected-baseline-reproduction-v1/`。未改旧数据；最优公共参数在 `configs/research/best_known_transport_v1.yaml`，完整来源与 SHA-256 在结果目录。

2026-11-03：新获取8回合在 `runs/research/2026-11-02-task-reference-acquisition-v1/`；新闭环12回合在 `runs/research/2026-11-03-task-reference-v1/`，各自保留原始产品、冻结源码和失败。[新参考报告](results/task_reference_v1/README.md)与[部署贡献审计](results/deployment_contribution_v1/README.md)分别记录新实验和既有36回合的重新分析。轮廓探测原次已标 INVALIDATED，修正次在 `runs/maintenance/2026-11-03-initial-contour-probe-v2/`；没有覆盖原结果。

[匹配支撑固定筛选](results/reference_support_v1/README.md)：8条同输入轨迹已完成；局部大误差减少但未通过团队误差门槛，新增物理回合0，候选默认关闭。


[最新参考运动与论文证据核验](results/reference_motion_audit_v1/README.md)：同输入参考运动核验完成：24条旧轨迹、6条远场原配准器精确重放，新增物理回合0；区分初始化、参考点旋转和匹配误差，并核查论文主张与验证缺口。


[参考初始化数据](results/reference_origin_v1/README.md)：参考初始化固定消融完成：24回合（8个已知开发条件×3臂），三臂均7/8完成；取消初始化放大救回斜箱但损失远场C形，提前参考未救回斜箱且G500下降。两项均未采用，原判据和默认方法不变。


[投票与停止数据](results/voter_continuity_v1/README.md)：投票集合与停止确认固定消融：32回合（8个已知开发条件×4臂），分别检验换票读数连续化和1秒到达确认，保留全部失败与截尾；默认关闭、原成功安全判据不变。


2026-11-02 运动先验复验：新数据在 `runs/research/2026-11-02-transport-validation-v1/{regression,new_starts}/`，分别25和72回合；另核验复用33个原参数、8个候选旧回合，路径与原执行身份在本任务 `artifacts/migrations/` 下的 `design.json` 登记，未复制或重标记旧结果。两阶段结果、全部失败、停止事件审计及图表见[复验报告](results/transport_validation_v1/README.md)。33条件旧初态与12条件新初态分开统计，保留全部原始产物及哈希。

坐标一致修复批次原始 18 回合在 `runs/research/2026-11-02-contact-consistency-v1/run/`，执行源码与原始失败完整保留；分析/审计/工程测试分别在同身份 `artifacts/research/`、`artifacts/migrations/`、`runs/maintenance/`。见[结果与身份](results/contact_consistency_v1/README.md)。固定 18 回合，九个开发条件、同一 seed 2、每例最多 600 秒。共同参考配置下，旧聚合公式完成 5/9，坐标一致修正完成 8/9；G500 分别 2/9、7/9，修正臂原硬安全检查违规 0/9。C 背向仍失败且出现越过目标后的错误停止；8/9 不是总体成功概率。本轮是公共安全计算修复的配对比较，不是原 CVT 对 spacing 的优越性检验。

最新几何读出批次：`runs/research/2026-11-02-contact-geometry-v1/run/`，12个开发回合及原始失败均保留；[结果与身份](results/contact_geometry_v1/README.md)。独立评价与控制器估计分开，论文未改。

2026-11-01 恢复与增援固定九回合：原始产品及执行源码在 `runs/research/2026-11-01-contact-reinforcement-v1/run/`，分析和旧日志诊断在 `artifacts/research/2026-11-01-contact-reinforcement-v1/`，恢复备份/审计在同身份 `artifacts/migrations/`，工程测试在 `runs/maintenance/`。完整九行结果和矢量图见[结果入口](results/contact_reinforcement_v1/README.md)。全部失败保留，默认候选关闭，稿件题目不变。

本索引区分历史迁移记录与当前机器的本地文件。大体积文件在本地完整项目中，不等同于 Git 克隆内容。

2026-11-01 位置/速度分量比较：`runs/research/2026-11-01-feedback-components-v1/run/` 保存36个新回合、源码快照、协议和产品收据；27个原CVT/spacing/二维CVT参照仍来自上一轮原路径，未重标为新实验。旧日志接触丢失审查、恢复循环核验及新闭环分析在 `artifacts/research/2026-11-01-feedback-components-v1/`。精简证据、全部63行身份明确的结果、矢量图及未解决项见[分量比较](results/feedback_components_v1/README.md)。恢复和验证材料分别在同身份 `artifacts/migrations/`、`runs/maintenance/`；大体积逐帧数据与备份不自动加入Git。所有失败保留，默认控制与论文未替换。

2026-11-01 位移估计与可靠停止开发比较：36回合原始数据、执行源码和收据在 `runs/research/2026-11-01-displacement-stop-v1/run/`；旧C背向的完整重建扫描、同输入估计和闭环分析在 `artifacts/research/2026-11-01-displacement-stop-v1/`。重建输入经12000帧输出精确核验，但身份仍是“验证过的重建”，不是原始扫描录像。精简证据见[位移与停止结果](results/displacement_stop_v1/README.md)。四方法共用同一开发seed和600秒上限；全部失败、G500不通过及C背向退化保留。审计/恢复材料在同身份的 `artifacts/migrations/`，工程验证在 `runs/maintenance/`，大体积数据不自动加入Git。

2026-11-01 六方法边界反馈开发比较：54回合原始数据和执行源码在 `runs/research/2026-11-01-boundary-navigation-v1/run/`，分析在 `artifacts/research/2026-11-01-boundary-navigation-v1/`，恢复材料在同身份的 `artifacts/migrations/`。精简证据见[六方法结果](results/boundary_navigation_v1/README.md)。同一开发seed、最多600秒；全部失败和安全超限保留，原CVT/spacing历史前缀精确复现。这里的原始日志、详细机器记录和备份是本地数据，不自动加入Git。

2026-10-31 原地整理从 PC2 恢复本地运行、文献和旧备份，保留其原相对路径、批次结构和字节。
PC3 中重复错放的稿件图片按哈希映射到已有规范副本；40种损坏 ZIP/bundle 的原始字节逐项保全在
本次 `backup/source-originals-PC3.zip` 的独立成员中。续查确认10个文件由有效开发归档前缀和周期填充组成，
另30个完全由周期填充组成。只有在全字节比较、原始归档成员哈希及有效备份验证后，才清理原路径的重复副本。
有效的2026-10-23恢复包继续保留，开发归档可由原Git提交再生成相同SHA-256；有效bundle的独有旧对象也完成隔离恢复。
这些结果不能证明PC3文件损坏前的完整身份；要证明它们当时没有独有改动，仍需损坏前清单/哈希或可读原件。
损坏原始字节没有被丢弃，也不作为有效恢复包或活动代码。
逐项位置、校验结果及完整性限制见本地
`artifacts/migrations/2026-10-31-single-repository/migration_manifest.csv` 与 `FINAL_REPORT.md`。
后续40项决定及证据在同目录 `pc3-recovery-001/recovery_decisions.csv` 与验证收据中；不是改写原历史映射。
这些是维护记录，不是新增科研样本。历史来源清单中的未知执行/分析版本保持 unknown，不从文件夹名补造身份。

| 位置 | 数据与归属 |
| --- | --- |
| `runs/ccta2027/` | 原项目既有 CCTA 原始执行产品，保持原字节 |
| `runs/evidence_closure/`、`runs/consolidation/` 等 | 原项目历史运行、诊断与归档 |
| `runs/reliability/2026-10-22*` | 恢复的基线、失败候选、默认修复与逐案例记录 |
| `runs/reliability/validation_2026-10-22*` | 前瞻设计、冻结协议和初始化预检；不是已执行的 832 个实验 |
| `docs/results/` | 按源码和实验版本分组的精简公开证据 |
| `data/literature/2026-10-22/` | 本地原件、逐页全文、分块及原图 |
| `docs/literature_review_2026-10-22/` | 71 份阅读笔记、索引、能力矩阵和研究定位工作稿 |
| `manuscript/ccta2027/` | 既有论文 TeX/PDF 草稿与引文依据 |
| `artifacts/manuscript/ccta2027/2026-10-22/` | 原始论文渲染和编译日志 |
| `artifacts/validation/2026-10-22/` | 原始测试 XML 与测试临时产品 |
| `artifacts/validation/2026-10-23/`、`runs/rebuild_20261023/` | 本次重建测试和五案例 smoke；独立于历史科研结果 |
| `artifacts/migrations/2026-10-23/` | 原版本、隐藏 Git/环境文件及开发分支的可恢复备份 |
| `runs/maintenance/2026-10-31-single-repository/` | 当前机器基线、回归测试及短 smoke；独立于论文实验 |
| `artifacts/migrations/2026-10-31-single-repository/` | 本次清单、操作日志、来源原件、隔离记录及恢复说明 |

2026-10-23 收据记录恢复了 **2,825 个原始运行文件，共 2,112,477,082 字节**。该次迁移前可读运行文件 8,478 个，逐文件保留。500 个文献数据产品已归位，其中 70 个 PDF 和 1 个 DOCX 按原清单 SHA-256 核验；其余是提取、分块与原页图像。上述历史数量不等于本次输入来源或新增产物数量。

[逐文件迁移清单](migrations/2026-10-23/transfer_mapping.jsonl)给出每个转移文件的原路径、字节数、SHA-256 和当前路径。`zip::member` 表示原件保留在对应 ZIP 成员内；同名差异版本不会互相覆盖丢失。[保全验证](migrations/2026-10-23/preservation_verification.json)记录最终核对数量。

2026-10-23 原项目记录了 14 个不可读历史测试缓存目录和 `.worktrees/` 遗留目录，详见当时的[访问边界](migrations/2026-10-23/local_access_boundaries.json)。这是旧机器状态，不能代替当前逐文件清单。该次转移目录的全部 18,355 个文件由历史映射定义其身份。

历史清单保留旧机器绝对路径和 `tmp/literature/` 等原路径。新工具使用当前目录；历史文件身份、环境归属和科研结论不重写。文献全文不提交到 Git。

完整迁移时复制 `runs/`、`data/literature/`、`artifacts/`、`tools/` 及代码文档；在新电脑重新创建 `.venv`。随后从[README](../README.md)的流程验证。参考环境和数据目录不应从其他项目借用。

## 2026-10-31 多形状完整任务普查

- `runs/research/2026-10-31-multishape-census-v1/run/`：27 回合原始数据、真实 replay、逐帧/子步记录、执行源快照、环境和逐项哈希收据。
- `runs/research/2026-10-31-multishape-census-v1/analysis/`：全部任务能力表和阶段图；派生质量告警单独保存，不改原 summary。
- `runs/research/2026-10-31-multishape-census-v1/v2-offline-final/`：仅从原 v2 日志计算的带符号累计误差，非新增实验。
- `artifacts/research/2026-10-31-multishape-census-v1/`：修改前备份、状态、媒体、源码补充和验证报告；视频及大体积回放不自动加入 Git。
- `runs/maintenance/2026-10-31-multishape-census-v1/`：基线与修改后工程验证，不是论文样本。
- [精简公开结果](results/multishape_census_v1/README.md)：源身份、完整 CSV、图和本地媒体路径。一个 seed 的九条件是开发普查，未知泛化范围不补写为成功率。

旧 v1/v2 原始日志、配置、失败、哈希和历史形状矩阵保持原字节。新数据没有替代或重标旧数据。

## 2026-11-01 无附着动态接触比较

- `runs/research/2026-11-01-dynamic-contact-v1/run/`：新冻结 27 回合，旧行为、反馈修复及局部动态接触三臂；完整失败、回放、观测法向、名义命令、角色和换位事件。不同于前次部署方法普查，不能合并为同一实验。
- `artifacts/research/2026-11-01-dynamic-contact-v1/analysis/`：全表、配对比较、逐帧旧行为一致性及图—数据—分析哈希映射。
- [精简结果](results/dynamic_contact_v1/README.md)：任务完成 5/9、4/9、2/9，候选不推广；图表是完整派生产品的校验副本。
- `runs/maintenance/2026-11-01-dynamic-contact-v1/` 与 `artifacts/migrations/2026-11-01-dynamic-contact-v1/`：工程测试、备份和恢复记录，非新增科研样本。

## 2026-11-01 确认与回退接触交接

- `runs/research/2026-11-01-contact-handoff-v1/ablation/`：12 回合，三条件四臂，隔离归一化压力分配和换位；参考结果精确复现。
- 同批 `main/`：九条件三方法共 27 回合，150 秒固定预算，原 CVT/spacing/交接候选均完成 5/9；无持续接管确认，不能把位置判定当成完成换位或物理接管。
- 同批 `extended/`：九条件两方法共 18 回合，用户指定最多 600 秒物理时间；原停止规则保留，预算耗尽为右删失，已结束的 HOLD 不重启。
- `artifacts/research/2026-11-01-contact-handoff-v1/`：离线失败审查、三批全表、矢量图、按物理时间同步的说明视频、GIF 与来源哈希。
- [精简证据](results/contact_handoff_v1/README.md)是上述派生文件的哈希核验副本；完整数据与视频留在本仓库本地目录。
- `artifacts/migrations/2026-11-01-contact-handoff-v1/` 保存原工作区补丁、被改文件备份和状态；`runs/maintenance/2026-11-01-contact-handoff-v1/` 保存测试与工程 smoke，不计入科研样本。

## 2026-11-02-cargo-discovery-v1

- `runs/research/2026-11-02-cargo-discovery-v1/near/`：132个固定近场回合。
- `runs/research/2026-11-02-cargo-discovery-v1/far/`：12个未知位置/初始位置提示配对回合。
- `artifacts/research/2026-11-02-cargo-discovery-v1/`：C背向离线否定结果、核验分析、矢量图及同步视频。
- `artifacts/migrations/2026-11-02-cargo-discovery-v1/`：保护记录、状态、命令、哈希、验证与最终报告。
- [精简证据](results/cargo_discovery_v1/README.md)：新旧条件分层，失败与G500单列；不代表独立90%可靠性。

## 2026-11-02-transport-tuning-v1

- `runs/research/2026-11-02-transport-tuning-v1/screen/`：48个固定参数开发回合；`transfer/`：另24个跨条件、跨部署方法回合。每例最多600秒，保留全部失败、配置、轨迹、执行快照与收据。
- `artifacts/research/2026-11-02-transport-tuning-v1/`：G500原因审查、六个旧案例的参考点误差分解、两阶段核验结果、全表和部署安全统计。离线反事实不是新增物理试验。
- `artifacts/migrations/2026-11-02-transport-tuning-v1/`：修改前原件、工作树/暂存区补丁、状态及恢复说明；`runs/maintenance/2026-11-02-transport-tuning-v1/`：测试、命令、退出码及资源日志。
- [精简结果和负结果](results/transport_tuning_v1/README.md)：两阶段不混为总体成功率。候选仅为开发版本，全局默认、原判据、论文标题和历史数据不变。
