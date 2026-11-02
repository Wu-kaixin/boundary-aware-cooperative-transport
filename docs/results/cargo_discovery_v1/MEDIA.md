# 图表与真实轨迹视频

所有图来自冻结配置或已核验的真实记录。下采样、时间对齐、统计单位和主张范围见图注。

生成入口：`scripts/plot_cargo_discovery.py preflight|near|far`；视频入口：`scripts/render_cargo_discovery.py near|far`。已有产物拒绝覆盖。

| 科学问题 | 图或视频 | 必需字段 | 统计单位 | 支持与边界 | 建议位置 |
| --- | --- | --- | --- | --- | --- |
| 边界部署表示是否改变最终任务能力？ | near_all_outcomes、near_paired_metrics | 完成、安全、终点误差、物理时间、首次部署边界间隙 | 33 个设计条件的四臂配对 | 固定开发条件下的得失；不是可靠性概率或完整文献基线排名 | 配对指标正文，完整矩阵补充 |
| 未知位置是否额外限制完整任务？ | far_location_pairs、far_full_process、far 视频 | 首次检测、接触、阶段、真实位姿、实际终止 | 6 个物理初态的两臂配对 | 区分发现和搬运；没有任意环境搜索保证 | 正文及视频补充 |
| C 背向的单次估计改动能否解决大误差？ | reference_negative_result | 同输入原读数、候选读数、评价真值和准入 | 1 条历史轨迹，帧是重复测量 | 候选门槛失败；不能从离线读数推出闭环改善 | 局限或补充 |
| 新增货物覆盖了哪些几何？ | cargo_geometry_atlas | 冻结顶点、面积、质量、姿态 | 12 个几何实例、24 个方向条件 | 展示设计范围；轮廓图没有运输成功含义 | 补充 |
| 方法怎样从部署走到实际终止？ | near 同步视频 | 真实 replay、估计读数、阶段和终止状态 | 1 个预先选定条件的四臂 | 说明完整过程及失败，不能代替全部结果表 | 视频补充 |

## cargo_geometry_atlas
[PDF](cargo_geometry_atlas.pdf) · [SVG](cargo_geometry_atlas.svg) · [来源与哈希](cargo_geometry_atlas.json)

Predeclared development cargo outlines. Each has polygon area 2.05875 m2 and mass 4.1175 kg. All twelve are tested in two orientations under four deployment rules. The arrows mark the +x task direction. These are protocol geometries, not transport outcomes or draws from an arbitrary-shape distribution.

## reference_negative_result
[PDF](reference_negative_result.pdf) · [SVG](reference_negative_result.svg) · [来源与哈希](reference_negative_result.json)

Historical C-away trajectory with two readouts on identical local scan inputs. Gaps denote no admitted reading. No smoothing, extrapolation or new physics. The candidate reduced paired vector MAE by 7.48%, but retained 0.715 m final vector error and failed the predeclared gate. Frames are repeated measurements of one development trajectory.

## near_all_outcomes
[PDF](near_all_outcomes.pdf) · [SVG](near_all_outcomes.svg) · [来源与哈希](near_all_outcomes.json)

All 33 paired near-field development conditions under the same repaired transport pipeline. The first nine retain historical initial states; the remaining 24 use a common new-geometry annulus. S denotes task completion with no original hard-safety failure, F a task failure, H a hard-safety failure, and E an execution exception. Entries are final Euclidean goal errors, not population success probabilities.

## near_paired_metrics
[PDF](near_paired_metrics.pdf) · [SVG](near_paired_metrics.svg) · [来源与哈希](near_paired_metrics.json)

Paired task error, physical duration and sampled uncovered boundary arc at first CONTACT_READY. Every planned condition is retained. Failed or censored durations use x markers; missing readiness measurements are not filled with zeros. Arc measurements use 360 equal-arclength samples, a 0.42 m observer radius and only external robot centers. They are independent deployment diagnostics, not caging or transport guarantees.

## far_location_pairs
[PDF](far_location_pairs.pdf) · [SVG](far_location_pairs.svg) · [来源与哈希](far_location_pairs.json)

Far-field location-information ablation on three geometries and two positions. Both arms have zero initial sensor returns and identical physical states; the hint arm alone receives a one-shot initial location token with unchanged TTL. Detection, task completion and final error are reported separately. No geometry prior or ongoing pose oracle is supplied.

## far_full_process
[PDF](far_full_process.pdf) · [SVG](far_full_process.svg) · [来源与哈希](far_full_process.json)

Position-0 cases selected before outcomes: circle, skew box and C opening away. Columns show the initial state, first recorded actual contact and actual terminal state. Cargo geometry and its black trajectory are evaluation truth used only for visualization. Missing contact is explicitly labelled; terminal failures are not replaced by successful frames.

## near 同步视频
[MP4](../../../artifacts/research/2026-11-02-cargo-discovery-v1/media/near_synchronized.mp4) · [GIF](../../../artifacts/research/2026-11-02-cargo-discovery-v1/media/near_preview.gif) · [身份](near_media.json)
完整时间范围按共同物理时钟显示，每物理秒取一真实状态、10 fps（约10倍速）；不插值、不补齐轨迹。已结束方法冻结并明确标注终止时刻及结果。估计标注为上一控制步读数；货物轮廓/轨迹是评价真值。选例与结果无关。大视频留本地，不自动加入 Git。

## far 同步视频
[MP4](../../../artifacts/research/2026-11-02-cargo-discovery-v1/media/far_synchronized.mp4) · [GIF](../../../artifacts/research/2026-11-02-cargo-discovery-v1/media/far_preview.gif) · [身份](far_media.json)
完整时间范围按共同物理时钟显示，每物理秒取一真实状态、10 fps（约10倍速）；不插值、不补齐轨迹。已结束方法冻结并明确标注终止时刻及结果。估计标注为上一控制步读数；货物轮廓/轨迹是评价真值。选例与结果无关。大视频留本地，不自动加入 Git。
