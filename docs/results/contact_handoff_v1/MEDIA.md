# 图表、视频与主张边界

| 科学问题 | 产品与字段 | 统计单位 | 可支持与不可支持 |
|---|---|---|---|
| 压力分配还是换位导致退化？ | 消融全表、终点图；四个机制开关、完成、误差 | 3 个配对条件，一个 seed | 矩形反例与机制隔离；不支持跨种子可靠性 |
| 是否救回完整任务失败？ | 主比较终点图、全轨迹；全部 27 回合 | 9 个配对条件，一个 seed | 描述完成与配对变化；不以帧数作样本，不做显著性/等价结论 |
| 更多时间是否足够？ | 延长观察全表及轨迹；18 回合，600 s | 9 个配对条件 | 有限观察结果；删失不等于永远失败，HOLD 不重启 |
| 是否真的发生接管？ | `main/analysis.json` 的事件及独立接触核验 | 机器人事件，非独立实验 | 本轮没有通过确认/卸载；不能把位置到达当作接触能力 |
| 完整过程如何发生？ | 三方法同步视频、真实物体轨迹和已保存地图 | 两个事后说明案例 | 展示成功与失败，不替代全表，不声称这些是独立验证 |

English caption — endpoint comparison:
“Final two-dimensional endpoint errors for every frozen development condition.
Filled markers denote completion under the original settling and 0.12 m endpoint
criterion; open markers denote noncompletion at the observed end. Conditions are
paired within one seed-2 block. No confidence intervals or independent-sample
inference are asserted. G500 is reported separately.”

English caption — complete trajectories:
“Unsmoothed evaluation trajectories from start to actual termination for all
conditions. Crosses mark observed endpoints; dashed circles show the unchanged
target tolerance. Axes use metric coordinates with equal aspect within each panel;
panel ranges differ. These are object-truth evaluation paths, not information
available to local controllers.”

English caption — extended observation:
“Matched original and candidate episodes under a separate 600 s physical-time
observation budget. Original termination rules are retained. Unfinished episodes
at the budget are right-censored; previously terminated off-target HOLD episodes
are not restarted or imputed as later successes.”

English caption — synchronized videos:
“Post-hoc circle-failure and rectangle-parallel retained-completion illustrations,
shown on the same physical timeline, camera and scale. Ended episodes retain their
actual final frame with a terminal label. Orange geometry is evaluation truth;
teal points show agent 00's saved fused map, not raw laser rays. Geometric readiness
rings are not contact certificates. The candidate did not achieve a confirmed
handoff in these experiments.”

本地媒体目录：`artifacts/research/2026-11-01-contact-handoff-v1/media/`。

- `circle.mp4`、`circle.gif`：三方法圆形失败，完整起止过程。
- `rectangle_parallel.mp4`、`rectangle_parallel.gif`：三方法矩形保留完成，完整起止过程。
- `receipts.json`、`manifest.json`：输入与输出哈希、实际渲染吞吐和生成版本。

控制频率 20 Hz；视频采样间隔 0.25 s、播放 10 fps，名义 2.5 倍速，末尾补实际最后一帧。
不插值、不按数组比例强行同步；视频 HUD 区分物理时间和前一控制时刻的团队估计。
GIF 为 4 fps、720 像素宽预览。所有定量曲线不平滑、保留全部样本；视频下采样不替代原始数据。
PDF/SVG 为矢量，PNG 仅作预览。图—数据—分析版本映射见各阶段 `provenance.json` 与本目录清单。
