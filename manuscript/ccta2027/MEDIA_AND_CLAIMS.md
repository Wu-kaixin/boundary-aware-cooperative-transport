# CCTA 初稿：论证与媒体包（2026-11-01）

本次只分析和渲染冻结记录，没有新仿真、调参或追加样本。论文是具有明确范围的系统研究初稿，尚未建立新算法优越性。旧稿及本地修改已保存在 `artifacts/research/2026-11-01-paper-media-v1/backup/ccta2027/`。

## 三句话定位

1. **问题：**没有完整物体几何先验时，局部边界部署能否形成实际接触，并在固定资源和时间内完成二维搬运？
2. **系统贡献：**将局部边界地图、部署、接触运输、公共阶段监督与独立任务评价连接成可复现系统，明确各层的信息权限与任务契约。
3. **已有结论：**一个开发随机块的九条件普查揭示部署、接触、投影到达与二维完成的差距；CVT与spacing均完成5/9，尚不支持CVT优越。

CVT/CBF及其组合有明确先例；可复现工具是工程贡献。任务方向可辨识命题是限制模型内的标准线性代数结果，不是新的搬运完成定理。v1/v2适合作为机制诊断和失败边界，不能迫使全文围绕已失败的anchored候选展开。

## 科学问题与图表

| 科学问题 | 图型/文件 | 所需字段 | 统计单位 | 支持主张 | 主张边界 | 位置 |
|---|---|---|---|---|---|---|
| 部署后是否真的接触、推进并完成？ | `full_process`事件图 | 位姿、融合地图、阶段、实际接触数、终止 | 固定circle/CVT/seed2个案 | 几何就绪与任务完成可分离 | 不证明严格caging；列按事件分类，接触可能早于ready | 正文图1 |
| 三方法改变哪些完整任务结果？ | `method_comparison`配对点图 | 全27回合完成、二维误差、实际耗时、路径 | 9固定条件、1共享随机块 | 各条件结果及实际资源代价 | 无可靠性CI、无文献SOTA结论；失败时间不是完成时间 | 正文图2 |
| 进度、横偏、接触与停止如何同时演化？ | `aligned_mechanism`时间序列 | 实际team读出、控制前真值、接触、阶段 | 3事后选择开发个案 | 圆横偏与C背向短距停滞是不同现象 | 团队读出不是每台局部读出；非因果反事实 | 正文图3 |
| 五形状是否具有完整行为证据？ | `five_shape_stages`五行事件图 | 五个预选CVT轨迹的实际帧 | 5参考方向个案 | 可追溯地图、ready、接触、搬运和最终状态 | 不是全部朝向；全部九条件在结果表/视频中 | 补充S1 |
| 成本与安全情况如何？ | `safety_resources` | 最小间距、最大穿透、命令、QP计数 | 全27回合；1块 | 离散安全与路径/努力描述 | 不能证明连续安全；命令平方积分不是电池能量 | 补充S2 |
| 被拒绝的候选是否有效？ | `v2_negative` | v2九臂原始派生表、身份 | 3开发条件配对 | 两失败未救回、C8退化，证书与原方法一致 | 与census不同身份，不合并统计；不支持普遍无效 | 补充S3 |
| 如何观看成功和失败的完整过程？ | 总览＋9组三方法MP4 | 全轨迹、物理时间和状态 | 同条件配对 | 真实执行过程，失败和提前终止保留 | 视觉展示不是统计证明 | 补充视频 |
| 噪声与资源鲁棒性是否成立？ | **不作结果图** | 尚缺相应冻结执行数据 | 未执行 | 无 | 设计文件不是结果，不能画占位实验曲线 | 缺口 |

六组图均生成PDF/SVG/PNG。双栏原始宽7.1英寸，单栏v2图3.4英寸；PNG 220dpi仅预览，投稿优先矢量PDF。方法用蓝圆、橙方、绿三角；实心完成、空心失败。所有定量图无平滑，时间序列保留每个控制样本，不计算虚假的逐帧置信区间。图中不同案例的真实终止时间保留，不按回合长度归一化。

观测限制：回放只有agent_00保存的融合地图，不含全部机器人原始激光射线。因此不制作伪造射线。实际接触法向不是控制器选中的观测法向；完整名义命令、私有holding状态等缺失项不补造。

时间对齐：回放第k+1行的进度来自控制前更新，和第k行物体真值比较；阶段线标控制决策时刻。接触报告来自机器人移动后、物体积分前，按其报告边界另标。视频显示后积分物体位姿及上一控制时刻估计，HUD同时给出两个时刻；不把它们相减成同步误差。

## 英文caption草稿与选例

**Fig. 1 — Full process.** Event-indexed snapshots of the circle/CVT development case: first saved local map, geometric readiness, first physical contact, transport midpoint and actual termination. Orange geometry is evaluation truth; teal points are agent 00's fused map. Event columns are not necessarily chronological: physical contact precedes geometric readiness here. The task ends outside the spatial tolerance despite passing G500. Rings indicate geometric readiness, not physical contact.

**Fig. 2 — Complete comparison.** All 27 tasks across nine fixed shape–orientation conditions and three deployment methods, sharing development seed 2 and initial states. Marker shape identifies the method; filled/open markers denote spatial completion/failure. The dotted line is the unchanged 0.12 m endpoint tolerance. Duration is the actual termination time, with W marking watchdogs; failed runs have no successful completion time. Total robot path includes every recorded displacement. B=1 shared randomization block; no inferential confidence intervals are reported.

**Fig. 3 — Aligned mechanisms.** Pre-physics true progress and actual shared team-progress estimates, signed progress error and true cross-track displacement, and physical contact reports for circle, rectangle-parallel and C-away CVT cases. These cases were selected after the census to show distinct outcomes. All control samples are retained without smoothing. Vertical lines mark recorded supervisor events; absent events are not imputed. Different horizons are real termination differences, not rescaled time.

**Fig. S1 — Five-shape events.** Five preselected reference conditions under boundary CVT: circle, ellipse-parallel, rectangle-parallel, reference L and C-toward. Columns retain first saved map, geometric readiness, actual first contact, transport midpoint and final state. Common spatial limits include all selected robot trajectories and targets. The early contact in these cases precedes readiness; no missing stage is replaced with a successful frame. All other orientations remain in the full results and videos.

**Fig. S2 — Safety and resources.** Discrete minimum robot separation, maximum physical penetration, command-squared integral and barrier-scaling counts for all 27 census tasks. Condition IDs follow the complete table; marker shapes and colors identify methods. These measurements do not constitute continuous-time safety guarantees or physical energy estimates. Different episode durations remain part of the observed cost.

**Fig. S3 — Negative feedback ablation.** Endpoint errors in the frozen v2 baseline, certificate-only and anchored-readout comparison on R2, L8 and C8. Baseline and passive certificate trajectories coincide. The candidate does not rescue either selected failure and increases C8 error; the dotted line marks 120 mm. These selected diagnostic conditions are not independent generalization trials and are not pooled with the normalized census.

预选沿用原协议的五参考条件总览；九条件的三方法视频全部交付。新增机制图明确为事后选择，不伪称事前冻结。C背向失败在机制图及独立完整视频中保留。视频从t=0到最长真实终止，按共同物理时间检索最后已记录样本，不插值、不伪造终止后的运动。20Hz控制数据每0.25s取样，10fps播放，名义2.5倍速；不足整步的真实末帧另保留。GIF为同一总览的轻量预览，不能代替读数用MP4。

## 主张、近邻工作和缺口

| 主张 | 最接近原文 | 具体差异 | 所需证据 | 已有证据 | 剩余缺口 |
|---|---|---|---|---|---|
| 未知几何的局部部署服务于搬运 | [Song等，§III–IV](https://arxiv.org/html/2602.19070v2)：检测密度、CVT、CBF及圆形运输 | 本系统显式评价部分边界地图、五形状接触与终止 | 逐式差异、忠实对照、任务收益 | 27回合内部机制对照 | 尚无对该文的等权限复现；原文接触/停止细节不足，不能自行补全后宣称战胜原方法 |
| 部署进入有效接触运输 | [Sequential caging，§3–4](https://arxiv.org/pdf/2106.03132)：顺序部署、推手、接触保持、共享waypoint及实机 | 本系统地图、部署目标与独立二维评价不同 | 保留各自初始化/定位/通信假设的对照 | 仿真完整轨迹 | 不能宣称首次局部观测→运输；无实机，凸性合围理论不能移植给当前非凸场景 |
| 任务相关接触可能改善失败 | [Tang等RSS2024，§II–III](https://arxiv.org/html/2405.07908v2)：已知几何/力学参数的接触模式与wrench优化 | 潜在差异是更弱先验下的局部机制 | 候选实现、同预算作用证据和独立收益 | 当前只有失败机制线索 | 候选未实现；方向选择/接触重配置本身不是新概念 |
| 可复现完整任务画像 | 当前源码、收据、全27轨迹、v1/v2 | 独立记录就绪、接触、G500与二维完成 | 全分母、身份、负结果 | 已有 | 一个随机块，不足以支撑可靠性或算法普遍优越 |

本轮重新查阅以上三篇最接近工作的相关全文段落；其余引文沿用已有阅读深度登记，不宣称本轮重新精读全部20篇。未为维持引用数量添加文献。

## 三个标题

| 英文标题／中文 | 承诺与证据 | 使用时机 |
|---|---|---|
| **Boundary-Aware Deployment for Cooperative Transport of Unknown Objects**／面向未知物体协同搬运的边界感知部署 | 保留完整任务主线，正文限定局部算法和公共监督；不承诺优越或全闭环去中心化 | **当前推荐，已写入稿件** |
| From Local Boundary Deployment to Cooperative Transport: Capabilities and Failure Boundaries／从局部边界部署到协同搬运：能力与失败边界 | 与现有能力画像一致，更突出系统评价 | 当前备选 |
| Decentralized Cooperative Transport of Unknown Objects via Task-Aware Boundary Deployment／基于任务感知边界部署的未知物体去中心化协同搬运 | 承诺任务相关方法及闭环信息结构 | 未来：需新机制独立收益和公共监督的分布式实现证据 |

仅增加独立seed而无方法收益，不足以采用更强标题；独立任务收益可强化方法贡献；有明确通信协议与延迟条件的分布式闭环才支持标题前置Decentralized。噪声/参数/资源范围和连续安全证据未补齐前不使用robust、general或guaranteed。

## 正文结构与实际修改

1. 引言：定义部署到可操作运输问题，直接说明系统贡献与未成立的优越性。
2. 相关工作：逐项说明CVT/CBF、顺序合围与模型化推动先例，不靠标题推断差异。
3. 系统与契约：局部地图、邻居通信、全队中位数与公共监督；真值只用于传感器/物理/评价和另列oracle。公式给出部署质心、投影进度及二维误差；线速度收定不等于刚体完全静止。
4. 冻结设计：五形状九方向、等面积质量、不同惯量周长、一个开发块，固定27回合。
5. 任务能力：全分母结果表、图1–3；失败、C背向和横偏不省略。
6. 反馈诊断：保留v1/v2与带符号记账，限制纯平移/固定配准假设；不把线性代数或控制努力界延伸为完成保证。
7. 局限与结论：无新矩阵、无噪声/资源/独立泛化/实机证据；后续6回合设计仍是未执行计划。

关键改动已经落实于摘要相关叙述、题目、贡献段、近邻比较、统计单位、收定定义、图注、对齐机制段和局限。五形状全事件图与安全/资源/v2图放补充，避免细节挤占完整任务主线。

## 格式、生成和身份

2026-11-01重新核对：[CCTA2027 CFP](https://ccta2027.ieeecss.org/authors/call-for-papers)要求标准IEEE双栏、初投至多8页、录用6页基准超页收费；[CSS官方模板](https://css.paperplaza.net/conferences/support/tex.php)支持现有ieeeconf、Letter、10pt。当前CFP没有明确普通论文视频大小/时长/编码或匿名细则；这些MP4/GIF是科研补充材料，不宣称通过尚未核验的上传规则。

生成入口（根环境；输出必须全新）：

```powershell
.\.venv\Scripts\python.exe scripts/build_ccta_media.py --batch runs/research/2026-10-31-multishape-census-v1/run --out artifacts/research/<new-media-id>/assets --dry-run
.\.venv\Scripts\python.exe scripts/build_ccta_media.py --batch runs/research/2026-10-31-multishape-census-v1/run --out artifacts/research/<new-media-id>/assets
.\.venv\Scripts\python.exe scripts/build_ccta_media.py --batch runs/research/2026-10-31-multishape-census-v1/run --out artifacts/research/<new-media-id>/videos --videos-only
```

实际产物在 `artifacts/research/2026-11-01-paper-media-v1/`。`assets/provenance.json`记录输入和生成器hash，`assets/keyframes.json`记录帧，`assets/alignment.json`记录时间语义，`videos-final/video_receipts.json`记录编码/解码与实际吞吐，最终输出映射在`deliverables.json`。Matplotlib绘图和FFmpeg软件编码无需GPU/3D；未做Inkscape手工改图，全部排版可由脚本重建。
