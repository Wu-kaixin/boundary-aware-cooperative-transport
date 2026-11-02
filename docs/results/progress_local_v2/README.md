# 实际匹配日志与局部进度读出：progress-local-v2

状态：固定研究执行完成；候选未通过无退化要求，停止扩大和调参。保留为实验选项，不更改默认控制。
这是同三个事后选定 development 条件的机制研究，不是新的独立泛化验证。

## 结果与判断

| 条件 | Baseline / certificate endpoint m | Anchored endpoint m | 改变量 | 完成（原/候选） |
|---|---:|---:|---:|---|
| Rectangle seed2 | 0.1621367 | 0.1591551 | −2.982 mm | 否/否 |
| L-shape seed8 | 0.1516452 | 0.1522133 | +0.568 mm | 否/否 |
| C-shape seed8 | 0.0082882 | 0.0187094 | +10.421 mm | 是/是 |

九回合都收定、没有watchdog，G500全部false。候选终止帧为677/385/393，原方法和仅证书为679/376/393。
矩形有小幅终点改善，所以协议“无任何失败条件改善”的字面反证未触发；成功对照退化的反证触发，两个失败均未变为成功。
不能宣称候选普遍有效，也不能说三个条件都变差。

原方法与仅证书：每对全部25个replay数组、完整逐帧科学记录精确一致，包含接触明细、命令、进度、阶段、积分、求解器状态/回退与安全统计；没有忽略某些科学字段或放宽容差。

## 诊断与机制

先执行3个baseline记录回合，再同源冻结3×3比较，总12回合，最多2进程、原3000帧上限与原停止规则。没有追加回合、换seed、调整参数或重试。
三个前置诊断不能与后九回合作独立样本合并。

捕获的是 `_solve_translation` 收到的实际匹配、MAD trim后的 A=n 和 b=residual。融合部分另记融合后的法向和返回修正；不把它冒充配准A。
本帧epoch防止旧last_registration被重复使用；累计感知dt原样记录。真值仅由仿真评价在进度更新后、物理推进前记录，与估计同一时刻。

22,812次配准中22,797在固定容差下可辨识；其余15次全部被原进度质量逻辑拒绝。原合成rank反例不能直接解释这些失败，但近退化、错配、旋转和历史地图误差未被排除。
配准+融合记账恒等式最大残差1.12e-16 m。

候选为 `g*(d@m_pre + ell@b)`：保留g=1.24，不再次积分修正。准入用本帧fresh、二维可辨识、原质量可用、原注册未clamp、有限数值和原速度×感知dt限制；拒绝回退，不截断输入或负进度。
候选准入后缓存本臂选定标量以延续原unavailable hold规则；地图、velocity、quality、median、安全、物理和成功判据均未更改。
保持缓存的一致性修复发生于comparison之前，不依据比较结果选参；诊断baseline不受影响。两个阶段各保留独立源码快照。

在相同诊断准入样本上，原值/候选局部MAE（mm）为139.654/139.928、77.890/78.174、58.333/58.546，候选三例均略差。
正式候选启用6019/10832、3765/6160、3820/6288个机器人控制更新，平均绝对改变量仅2.130/1.558/1.457 mm。完整拒绝原因在analysis JSON中，没有把几乎全回退当成有效机制。
全回合team MAE使用各臂自己的轨迹和终止时刻，是策略相关描述量，不能当作同输入估计精度比较；跨臂准入集合也不同。

## 文件、身份和复现

- `diagnostic_results.csv` / `diagnostic_analysis.json`：3前置回合派生数据。
- `comparison_results.csv` / `comparison_analysis.json`：9固定比较、全部等价检查、准入覆盖与误差。
- `diagnostic_results_original.json` / `comparison_results_original.json`：原始聚合收据逐字节副本，失败保留。
- `identity.json`：两个执行快照、分析源码与输入/原始产物哈希、实施变更说明。
- 原始gzip法向日志、轨迹、执行快照在 `runs/research/2026-10-31-progress-local-v2/{diagnostic,comparison}/`，本地保留、不加入Git。
- [冻结协议](../../../experiments/ccta2027/progress_local_v2.json)；[论文入口](../../../manuscript/ccta2027/README.md)。

从根目录分析已有记录：

```powershell
.\.venv\Scripts\python.exe scripts/analyze_progress_feedback.py --phase diagnostic
.\.venv\Scripts\python.exe scripts/analyze_progress_feedback.py --phase comparison
```

实验入口为根环境的 `-m dbact_sim.experiments.progress_feedback --help`，任何独立复现必须使用新的输出目录。
本轮没有自动提交或push；没有硬件、真实网络、完整实验矩阵或独立保留集。

元数据说明：底层deployment helper的旧 `changed_mechanism` 仅描述部署变体，不能代表本轮反馈干预。原记录保留，外层arm与执行快照为权威身份；comparison外层结果已按臂正确标注。诊断外层早期宽泛描述也按baseline解释，不改写旧字节。

验证：完整核心679 passed、2个原有历史脚本缺失skip；冻结前专项68 passed。新增代码Ruff通过；PDF 6页，保留20篇文献，逐页检查。
下一步必须提出新的累计误差假设与独立干预，不能继续微调本候选直到成功；本轮结果不指定增益、融合、聚合中哪项为根因。
