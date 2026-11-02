# Readiness / progress diagnostic v1（2026-10-31）

这是 **开发阶段诊断** 的紧凑结果包，包含历史审查、新 oracle 干预和合成观测机制三个不同身份。不是新泛化实验或新控制器性能声明。

## 文件与身份

| 文件 | 内容 |
|---|---|
| `historical_cases.csv` | 九例旧 CVT development 的派生指标、原轨迹路径与 SHA-256 |
| `paired_results.csv` | 本轮六回合摘要（误差、完成、G500、帧数、配对哈希） |
| `paired_results_original.json` | 本轮 `paired/results.json` 的逐字节副本；保留失败和元数据瑕疵 |
| `observation_conditions.csv` | 18 个配对设置 × 3 臂，54 行；不是 54 个独立随机样本 |
| `observation_summary.json` | 该固定网格的描述汇总 |
| `execution_identity.json` | 执行源码哈希、后续分析源码哈希、环境、元数据勘误和原始数据完整性清单哈希 |

大轨迹、每帧日志、完整执行源快照留在 `runs/research/2026-10-31-readiness-v1/paired/`，没有提交到 Git。
本地 `artifacts/research/2026-10-31-readiness-v1/raw_integrity.json` 为 137 个原始执行/快照文件逐项登记哈希。
执行前冻结的 84 个源码/输入文件均复核一致。分析源码在执行后单独保存于 `runs/research/2026-10-31-readiness-v1/analysis_snapshot/`，不冒充事前注册。

## 结果

历史 development CVT 九例：5 完成，全部收定、几何就绪；物理接触 quorum+dwell 事件 0/9。这里并不是“没有物理接触”。

| 新配对条件 | 原反馈误差 m | 进度 oracle 误差 m | 完成（原/oracle） | G500（原/oracle） |
|---|---:|---:|---|---|
| Rectangle seed2 | 0.162137 | 0.071958 | 否/是 | 否/是 |
| L-shape seed8 | 0.151645 | 0.048475 | 否/是 | 否/是 |
| C-shape seed8 | 0.008288 | 0.013513 | 是/是 | 否/否 |

这三条件从历史 development 事后选择。每对配置与初态哈希一致，原始最大 3000 帧、原停止条件、最多两个进程，总计只运行 6 回合。
没有换种子、重试、调整阈值或延长预算。六回合程序成功执行不等于六回合科研任务成功。
**C8 的误差退化满足协议原 falsifier 的 adverse-control 项。整体无退化改善未成立。**

新 R2 baseline 的结果不同于旧历史 R2，二者不混作对照。新实验保留独立源码与 Python 环境身份。
Oracle 从回合开始执行原更新后覆盖进度反馈数值，不更改地图、速度与质量计算；不是局部可部署的算法。

Synthetic：9 种几何/任务方向组合 × 2 噪声水平，复用 seed101 的同一个噪声向量和同一个真位移。
每臂 12 个观测。repeat / certificate-only / targeted 的 ridge 平均绝对误差分别为 8.724 / 8.724 / 0.156 mm；带固定 1.24 增益后为 10.610 / 10.610 / 6.951 mm。
均值仅描述这个固定网格；没有安全运动、真实扫描可达性或一般统计优势证据。

## 原始元数据勘误（保留原字节）

原 `transport_arm` 返回的 `changed_mechanism` 描述的是此前 centroid-target 消融，不能用于解释本轮进度 oracle。
六条原始结果均保留此旧字段。**本轮干预身份以外层 `mode`、`information`、执行快照和 `oracle_events.json` 为准**；勘误见 `execution_identity.json`。
当前 runner 已修正未来诊断收据的标签；通用 helper 的原 summary 仍仅描述部署变体。没有改写已冻结收据来消除该瑕疵。

## 实现、验证与局限

新增 `task_observability.py` 提供固定对应、纯平移的任务方向证书和候选法向选择，尚未接入在线控制。
`readiness_diagnostic.py` 在仿真层隔离真值权限，拒绝覆盖既有输出，保存失败和源码快照。
回归：完整核心 **661 passed, 2 skipped**（本分支没有两个历史脚本，沿用原测试跳过）；专项 **19 passed**。新增文件 Ruff 检查通过。
5 页 Letter/IEEE 双栏 PDF 已编译，26 个字体全部嵌入且无 Type3；20 个引用键全部对应正文；无溢出框、缺失引用或字体替换。少量 underfull 警告已目视检查。
未运行硬件、MAS 网络、独立保留集或完整科研矩阵。未更改已有核心控制器、动力学、成功定义、阈值和历史证据。

[英文稿与复现入口](../../../manuscript/ccta2027/README.md) · [研究决策](../../READINESS_RESEARCH_DECISION_2026-10-31.md) · [文献审查](../../../manuscript/ccta2027/sources.md)
