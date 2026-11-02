# 图、数据与主张

| 科学问题 | 图 | 字段与统计单位 | 支持与边界 | 位置 |
| --- | --- | --- | --- | --- |
| 修复是否改变实际物体路径？ | complete_trajectories | 保存的 cargo center、start、goal；9 条件配对 | 展示全部 18 条路径与失败；不是全形状可靠性或机器人部署视频 | 正文候选 |
| 是否救回任务、是否有退化？ | paired_task_results | 完成/误差、运行时长、屏障缩放；9 条件配对 | 全部启动任务；失败运行时长不当作完成时长 | 正文候选 |
| 给定同一输入，平面计算误差是否减少？ | same_input_plane_error | 独立几何距离与旧/新聚合平面；两例末10秒的相关机器人/帧 | 同输入反事实计算，不能替代真实闭环或独立统计验证 | 补充/机制图 |

所有图源数据为 analysis.json、all_18_episodes.csv、paired_comparison.csv、same_input_screen.json 和本地 plot_data.json；逐项输入 SHA-256 与分析代码 SHA-256 在 analysis.json。无平滑、插值或外推，无置信区间（没有适合可靠性推断的独立样本）。蓝色虚线/方形为旧公式，橙色实线/圆形为修正；灰色圈/线为原目标容差。图宽 7.15 inch，矢量 PDF/SVG 与 220 dpi PNG，已核验嵌入字体和渲染。

**Trajectory caption.** Complete recorded object-center paths for nine paired development conditions (seed 2). Both arms share sensing, geometry readouts, physical parameters and evaluation; only the safety-plane normalization differs. Crosses mark terminal positions and the circle denotes the unchanged 0.12 m goal tolerance. Failed trials are retained. These reused conditions do not estimate population reliability.

**Task caption.** Paired endpoint error, observed duration and barrier-scaling events for all eighteen started episodes. F denotes task failure; a cross in the duration panel denotes an unfinished or off-target trial, whose duration is not a completion time. G500 and hard safety checks are reported separately in the accompanying table; they are not interchangeable with task completion.

**Mechanism caption.** Mean absolute error of the aggregate plane gap relative to the independent geometric gap under identical saved inputs. The audit covers nominal pushers during the final ten seconds of two historical failures (211 and 1272 repeated agent/frame records). These correlated records are not independent experiments. Corrected maximum errors remain 108.5 and 85.5 mm; the calculation is not a geometric safety certificate.

生成入口为 `scripts/analyze_contact_consistency.py`。图表不声称展示接触力时序、逐机器人边界观测或完整过程视频；原始轨迹仍可供后续按统一物理时间离线渲染。
