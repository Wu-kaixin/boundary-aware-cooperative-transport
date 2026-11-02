# 多形状完整任务开发普查 v1

本轮主问题是：机器人不知道物体完整几何时，局部边界部署能否接上真实接触、搬运及准确停止？

## 冻结设计与身份

九条件 × 三方法 × 一个固定开发 seed 2，共 27 回合。圆形；椭圆长轴平行/垂直；矩形长边平行/垂直；L 参考/旋转 180°；C 开口朝向/背向目标。目标方向统一为世界 +x，距离 1.2 m。三方法为 boundary_cvt、nearest_observed_target、local_spacing，只改变首次 CONTACT_READY 前部署目标。

圆形 64 顶点、椭圆 24 顶点源自旧 MC 轮廓；矩形/L/C 使用当前 CCTA 几何，C 原开口 +y。统一质心、面积 2.05875 m²、面密度 2 kg/m²、质量 4.1175 kg。实际惯量分别约 1.349、1.550、1.472、2.087、2.547 kg·m²，周长与外形跨度也不同。不是同惯量比较。冻结前逐条件构造环境，确认初态合法、未发生环带自动调整；所有条件的机器人初始位置完全相同，组内配置、任务和初态哈希一致。

机器人在近场开始，部分机器人起始已在传感范围内；不是未知远处目标搜索。沿用原 0.01 m 距离噪声、感知、反馈、接触物理、安全阈值、0.05 s 步长、最多 3000 帧和 HOLD+低速连续40帧收定规则。没有换 seed、失败重试、加时或调参。

- [冻结协议](../../../experiments/ccta2027/multishape_census_v1.json)；源码哈希与环境以原始运行 manifest 为准。
- 原始运行：`runs/research/2026-10-31-multishape-census-v1/run/`。
- [全部回合 CSV](all_27_episodes.csv)、[部署—接触—完成能力表](capability_table.md)、[派生分析](analysis.json)、[身份索引](identity.json)。
- [五形状阶段图](five_shape_stages.png)、[同条件方法比较](method_comparison.png)、[完整过程与视频](MEDIA.md)。

## 结果如何解释

这是一个 seed 下的开发普查，不能把27条当成27个独立随机样本，更不能称作普适可靠成功率。任务完成要求收定且二维终点误差 ≤0.12 m；CONTACT_READY、真实接触、物理接触quorum、收定、G500和完成各自保留。

G500 是旧执行契约，不是二维任务完成的更严格版本。本配置按投影进度 |J−L|≤0.12 m 判到达；效率、方向误差、横向偏差和 yaw 超限只形成质量告警。四个事件截止帧为空，但事件仍须发生。名称不表示500帧上限。圆形CVT就是反例：G500通过而二维误差1.1772 m；其横向偏差1.1748 m、方向误差46.25°、效率0.6915。

原 summary 序列化没有保留已计算的 quality_reasons。本轮用**相同 ClosedLoopContract 和存下的 metrics/gates**离线重算，并确认原G500判定及原因完全一致，才把质量告警写入派生表。未改原summary、冻结配置或门槛。

安全列报告离散观测的间距、穿透、QP缩放与故障；接触中允许的穿透不等于物体穿越，离散检查通过也不等于连续安全证明。原始数据含控制帧及四个物理子步，未声称逐连续时刻检查。

## 已有 v2 日志的离线分析

原 v1/v2 数据和 anchored 候选选项保留；v2 未救回R2/L8失败且令C8终点误差变差，停止候选扩展。本轮没有新反馈算法。

[带符号误差报告](signed_error_analysis.json)把实际读出写成原始地图量、固定增益增量、保持项、候选差值；团队中位数引入单独的权重变化项。累计恒等式与 registration+fusion 地图增量逐帧验证，不能把代数自洽当成物理准确性。中位数同值按agent ID选权重，分项不是唯一因果贡献。

- [R2分解](v2_rectangle_signed_error.png)
- [L8分解](v2_l_signed_error.png)
- [C8分解](v2_c_signed_error.png)

基线最终对齐控制时刻的带符号误差分别约+171、+155、+36 mm。R2/L8首次共享HOLD时实际速度为零，但仍在原二维完成容差外；C8已达投影目标。个体transport loop私有holding状态没有记录，不能据输入补造准确转换时刻。没有确认新的实现错误；累计偏差、增益/保持/权重影响保留为待检验假说，不启动v3/v4搜索。全部九条G500原因保存在离线报告中；C8完成而G500失败继续保留。

## 复现与权限边界

局部地图、邻居反馈之外，还有全队进度中位数与共同阶段监督器。无物体真值输入不等于整个系统完全去中心化。真值只用于传感器/物理、独立评价与标注清楚的旧oracle诊断。

本批完整执行配置与src/configs源码已冻结。生成脚本补充副本在运行开始后保存，标记为补充来源记录，不伪称原manifest字段。分析与渲染脚本、关键函数哈希和产物哈希见本地最终报告。运行目录若缺失，不能凭精简表重建轨迹。

从仓库根目录使用根 `.venv`，只分析已有记录：

```powershell
.\.venv\Scripts\python.exe scripts/analyze_multishape_census.py --batch runs/research/2026-10-31-multishape-census-v1/run --out runs/research/census-independent-analysis
.\.venv\Scripts\python.exe scripts/analyze_cumulative_progress.py --batch runs/research/2026-10-31-progress-local-v2/comparison --out runs/research/census-independent-v2-analysis
```

视频使用同一脚本加 `--videos` 并指定新的输出目录，拒绝覆盖已有视频。科学执行身份、分析版本和新输出身份分别记录，不覆盖原件。完整工程验证与恢复记录见 `artifacts/research/2026-10-31-multishape-census-v1/FINAL_REPORT.md`。

## 全部27回合的统计

| 方法 | 二维完成 | G500 | 收定 | 质量诊断无告警 |
|---|---:|---:|---:|---:|
| boundary_cvt | 5/9 | 5/9 | 8/9 | 6/9 |
| nearest_observed_target | 2/9 | 1/9 | 7/9 | 6/9 |
| local_spacing | 5/9 | 3/9 | 7/9 | 4/9 |

CVT与局部间距完成数相同，本轮没有支持CVT优于两种对照。圆及两种椭圆朝向的三方法均未完成；C开口朝向目标时三方法完成，背向时三方法均到3000帧watchdog且失败。全部27回合都有几何就绪、实际接触和正向运动记录；这些中间事件不充分。22回合收定、5回合watchdog。没有删去任何失败。
