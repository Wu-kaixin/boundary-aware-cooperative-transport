# 完整过程与同步回放

媒体来自实际 replay，未重跑物理、未根据 summary 补造轨迹。大视频仅保存在本地仓库，Git 克隆不一定包含。完整启动/运行/停止过程使用每五控制帧一帧、10 fps，名义2.5倍速；最后一帧单独保留。三方法按共同物理时间及同一视角播放，结束较早的回合明确标为 ENDED 并保持最后实测状态。

HUD 的真值来自物理积分后位姿；估计量来自前一控制更新，不能将两者直接相减当作对齐误差。物理接触数来自引擎记录；机器人外圈是几何就绪标记。显示的 teal 点是 agent_00 融合地图，不是全部机器人原始激光。

五形状总览预选CVT的 circle、ellipse_parallel、rectangle_parallel、l_reference、c_open_toward；含失败。九个条件全部有三方法同步视频。

| 视频 | 内容 | 物理时长 s | 输出帧数 |
|---|---|---:|---:|
| [五形状完整过程总览](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/multishape_overview.mp4) | 5 条完整轨迹 | 26.60 | 108 |
| [circle：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/circle.mp4) | 3 条完整轨迹 | 13.85 | 57 |
| [ellipse_parallel：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/ellipse_parallel.mp4) | 3 条完整轨迹 | 150.00 | 601 |
| [ellipse_perpendicular：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/ellipse_perpendicular.mp4) | 3 条完整轨迹 | 13.40 | 55 |
| [rectangle_parallel：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/rectangle_parallel.mp4) | 3 条完整轨迹 | 30.15 | 122 |
| [rectangle_perpendicular：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/rectangle_perpendicular.mp4) | 3 条完整轨迹 | 23.35 | 95 |
| [l_reference：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/l_reference.mp4) | 3 条完整轨迹 | 29.40 | 119 |
| [l_rotated：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/l_rotated.mp4) | 3 条完整轨迹 | 150.00 | 601 |
| [c_open_toward：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/c_open_toward.mp4) | 3 条完整轨迹 | 17.35 | 71 |
| [c_open_away：三方法同步](../../../artifacts/research/2026-10-31-multishape-census-v1/videos-final/c_open_away.mp4) | 3 条完整轨迹 | 150.00 | 601 |

[全部27回合能力表](capability_table.md) · [渲染收据](video_receipts.json)

初次并行渲染在最后一条实验回放尚未写完时停止，前九段已完成。续作验证其回放哈希与完整解码帧数后复用，补齐最后一段。原先未记录输出哈希的九段在本次完整解码核验后建立哈希，收据明确标识，不冒称先前已拥有这些哈希。

阶段图遵循首次CONTACT_READY、运输区间中点、实际最终帧；控制事件k对齐回放k+1。若阶段不存在则标未到达。额外圆形过程图是从固定参考条件中选出的失败示例，不宣称其五阶段图布局另有事前冻结。

最终交付目录为 videos-final/。视频末帧目视检查发现旧三方法版本HUD底部裁切，已仅从原回放重新编码九段；最终十段均通过完整解码和帧数/哈希校验，全部末帧已目视复查。旧编码只保留在本地审计区，不作为交付入口。未宣称逐帧人工审阅。
