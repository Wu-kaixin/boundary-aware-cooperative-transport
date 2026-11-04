# 媒体、选例和主张边界（2026-11-04）

本页对应当前八页主稿及 `supplementary.pdf`。上一版说明完整保存在本轮 `manuscript_before/`，旧结果和媒体不变。

| 图/材料 | 科研用途与来源 | 统计单位与边界 |
|---|---|---|
| 主图 1 `media_v1/aligned_mechanism.pdf` | 历史 circle、rectangle-parallel、C-away 的真实进度、实际团队读出、横偏、接触和停止事件 | 三个事后选择的开发个案；控制前估计与控制前真值对齐，不用帧数作样本量 |
| 主图 2 `latest_evidence/deployment_tradeoff.pdf` | 全部 36 条匹配轨迹的共同部署区间覆盖差及终点误差 | 12 设计条件、一个共享种子；三方法均 11/12，不能从覆盖改善推断完成优势 |
| 主图 3 `latest_evidence/contact_incompatibility.pdf` | 全部四个物理对照对及 17 个原正向力状态的约束能力 | 4 对重复使用开发条件；17 依赖状态 × 3 系统，不混同物理与代数单位 |
| 主图 4 `resource_noise/resource_noise_outcomes.pdf` | 新种子下全部 60 回合的四个资源/噪声条件 | 实心表示空间完成且无原硬安全失败；空心也包括容差内未收定和安全违规，不隐藏退化 |
| 补充阶段图 `resource_noise/five_shape_stages.pdf` | 事前选择的五形状 CVT reference 实际事件帧 | 三列为几何就绪、搬运区间中点、真实末态；不把 CONTACT_READY 当物理接触 |
| 补充 `reference_navigation.pdf` | 五个预选 reference 案例的向量导航投影、真值、空间误差、力与停止事件 | 实际导航读出与控制前真值对齐；录像的辅助 scalar 不能替代导航输入 |
| 补充 `c8_failure_diagnostics.pdf` | 新验证的 C-away/8 机器人三方法全部轨迹 | 事后诊断选择；保留 571.55、600、48.65 s 各自时长；不宣称独立因果干预 |
| 补充历史方法比较与 anchored 负结果 | 完整 27 条 census 和另一个 9 回合候选比较 | 不与当前源码的新验证合并 |

所有量化图使用原记录、明确单位、无平滑和无伪造置信区间。论文默认使用 Matplotlib 矢量 PDF，SVG 与 PNG 同时保留。Inkscape 1.4.4 对接触机制 SVG 作独立矢量导出，产物及哈希在本轮 draft-refresh 目录，未更改数据点。无显示器环境的 GTK 警告不等于导出失败，导出文件另经渲染查看。

## 当前完整视频

路径均相对仓库根目录。

| 文件 | 内容 | 选择和时钟 |
|---|---|---|
| `runs/research/2026-11-04-resource-noise-validation-v1/media_v1/five_shape_overview.mp4` | 五形状 CVT reference 全过程 | 事前选择；81 帧，10 fps，0.25 s 取样，名义 2.5× |
| 同目录 `five_shape_overview.gif` | 同一视频的轻量预览 | 全 81 帧解码核对；不是定量读数替代品 |
| 同目录 `rectangle_parallel_three_methods.mp4` | 相同条件三方法 | 事前选择；70 帧，2.5× |
| 同目录 `c_open_away_three_methods.mp4` | 相同 C-away reference 三方法 | 事前选择；81 帧，2.5× |
| `runs/research/2026-11-04-resource-noise-validation-v1/diagnostics_v1/c_away_8_robots_full_comparison.mp4` | 长时 CVT 完成、spacing 超时、nearest 停错位置 | 事后失败诊断；覆盖全部 600 s，601 帧、10 fps、1 s 取样、10× |

视频按共同物理时间同步；结束案例保持实际末帧，HUD 标实际帧时间和终态。橙色形状为离线评价真值，teal 为 agent 00 的融合地图，环形标记为几何就绪。未保存原始射线，因此未伪造射线。终端 G500 是最终评价，明确标 `terminal`，不是控制器当时已知的信号。辅助 scalar 与物体位姿来自不同更新边界，HUD 已说明；误差分析使用正确对齐的向量导航。

各包的 `media_manifest.json` / `diagnostics_manifest.json` 记录生成器、回放和视频哈希、完整解码、帧数与时钟。旧 census 的九个方向比较视频仍在 `artifacts/research/2026-11-01-paper-media-v1/videos-final/`；[旧索引](../../docs/results/multishape_census_v1/MEDIA.md)保留其历史身份。

## 贡献与尚缺证据

最接近工作是 Song 等的检测密度/CVT/CBF协作运输、Vardharajan 等的 sequential caging 与有效推动、Tang 等的模型化接触模式和力可行性推动，原文和阅读范围见 [sources.md](sources.md)。当前两个部署控制是内部机制对照，不是这些论文的忠实复现。

本稿贡献判断是完整任务契约、匹配部署贡献与接触约束反例的可复现分析。CVT、CBF、方向可辨识线性代数以及凸优化对偶界本身均有先例；没有把它们写成新理论。需要新增独立方法收益、等信息/作用预算文献基线、分布式监督和实机证据后，才能提出更强的算法与应用主张。独立种子验证不能补足尚未存在的方法创新。
