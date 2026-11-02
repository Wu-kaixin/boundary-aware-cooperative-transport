# Dynamic contact v1：完成一轮比较，候选不推广

2026-11-01，27/27 回合执行完成、0 执行异常。目标“让所有形状正常到达指定点”**尚未实现**。
本轮保持无附着、只推接触；未加入吸引力、连接器或真值控制。新开关均保持关闭。

| 版本 | 二维任务完成 | watchdog | G500 |
|---|---:|---:|---:|
| 原行为 `legacy` | 5/9 | 1/9 | 5/9 |
| 恢复安全反馈 `feedback_fix` | 4/9 | 2/9 | 3/9 |
| 反馈修复＋动态接触 `dynamic_contact` | 2/9 | 7/9 | 1/9 |

这是一个 seed 的九个配对开发条件，不是独立可靠性验证。G500 与二维完成分别报告。
候选相对原行为丢失矩形平行、矩形垂直、L 形旋转三个原成功条件，没有新增成功；未通过事前继续门槛。
反馈修复也损失 L 形旋转这一条件，不能将“代码错误得到修复”直接写成搬运改进。
全部结果见 [逐回合表](all_27_episodes.csv) 和 [能力表](capability_table.md)。

## 实际修改及归因边界

1. **确认了一个数据流错误。** 控制器原来先清空 `self.diagnostics`，再查询当前机器人的上一帧安全修改，导致 `_blocked_along` 恒为 false。新回归测试在修复前失败；改为完成整帧后发布新诊断，修复后通过。`transport_safety_feedback=true` 才启用修复通道；其已有“QP 修改量大于 1e-3 就冻结积分”的策略没有进一步修改。恢复反馈后仍有退化，因此不自动修改科研默认行为。
2. **实现一个独立候选模块。** `src/dbact/control/contact_roles.py` 只接收本机融合地图、自己/已接收邻居位置、已有任务方向及本机运动估计。利用观测法向的非负组合分配相对压入权重，在持续缺少纠偏方向或估计停滞时寻找附近更有利且未占用的已观测边界。
3. **允许接触带内换位。** 与原覆盖重部署不同，接触带内机器人也可先退出压入，再沿局部切向移动。每机器人最多 3 次，单次租期 `2*local_radius/max_speed`，超时/占用/地图丢失均保留原因。总任务时钟不暂停；安全 QP 始终保留。未改物理、进度增益、感知、阶段和成功判据。
4. **补足直接日志。** 保存选中的观测法向、请求方向、局部横向估计、完整名义命令、安全受阻标志、局部停止锁存和角色/换位事件，弥补原来的方向诊断缺口。

这是一项“角色分配＋局部换位”的联合候选，不能用这轮比较将全部退化单独归因于换位。
其余层保持固定；原行为的九个条件中，25 个回放数组及所有原有逐帧字段都与旧冻结数据精确一致。
新增诊断字段不参与旧字段的一致性比较，未忽略任何旧科学字段。

## 数学含义及观察到的困难

机器人以自己的地图估计自己和邻居附近的内向单位法向，构成矩阵 A。
对已有的局部转向需求 d，求非负最小二乘

`min_(a >= 0) ||A a - d||² + 1e-4 ||a||²`，再将权重按最大值归一化至 [0,1]。

这是方向分配启发式，**不是物理接触证书**：它没有实际接触力上限、地面静摩擦、接触力矩或可达性证明；不同机器人还可能基于不同地图得出不同分配。

- 候选发起 124 次换位，56 次到达选定目标，31 次租期到期、21 次目标被占用、2 次地图丢失。其余 14 次没有上述明确结束标记，包含权限失效或回合结束等未细分中断，不计为到达；这些计数不是独立实验样本。到达几何目标也不等于形成有效推力。
- 圆形的横向偏移减少，但实际前进仅约 0.088 m（目标 1.2 m），最后超时。这不是完成能力改善。
- 矩形平行和椭圆平行出现 `contact_recovery_failed`；候选在 L 形旋转和 C 形背向条件下停留于 CONTACT_READY，未进入 TRANSPORT。能发起换位不等于维持推动接触组。
- C 形朝向仍完成，终点误差由 0.1015 m 降至 0.0708 m；L 形参考也仍完成。不能据两个有利条件掩盖整体退化。
- 记录的硬分离/穿入预算及求解失败没有出现新的违规；但候选圆形有 5 次 barrier scaling，L 形参考有 2 次。没有连续动态安全保证，不能写成“无安全问题”。

另完成一次事后探索性的安全修改方向分解；它仅用于提出后续问题，没有改变协议、参数或执行。不得将未干预的相关性写成根因证明。

下一步最有价值的问题应收窄为：**能否在保留现有有效推进接触的条件下，以局部信息完成有确认、有回退的接触交接？**
先区分“仅重新分配压力”和“仅换位”的影响，检验替补接触能否接管任务，再决定是否允许原推手退出。
所需确认不能偷用真实接触力；可用局部运动响应是否足够，仍需独立验证。本轮不启动新的候选搜索。

## 身份、复现与图表

- 协议：[dynamic_contact_v1.json](../../../experiments/ccta2027/dynamic_contact_v1.json)。9 条件 × 3 臂，seed=2，最多 3000 帧，dt=0.05 s。
- 感知/物理/初态来自原等面积普查；每组仅两个声明的开关不同。8 个案例进程、内部线程 1；实际运行约 438.6 s。该耗时不含测试及离线分析。
- 本地原始数据：`runs/research/2026-11-01-dynamic-contact-v1/run/`。执行源码和环境由 manifest、source_snapshot、逐案例产品 SHA-256 收据界定，不能只用旧 HEAD 归属。
- 本地分析：`artifacts/research/2026-11-01-dynamic-contact-v1/analysis/`；`provenance.json` 连接图、输入及分析脚本身份。
- [终点误差图](endpoint_comparison.pdf)（[SVG](endpoint_comparison.svg)）和 [全部物体轨迹](cargo_paths.pdf)（[SVG](cargo_paths.svg)）。未平滑、未补轨迹；方法在同一条件内共用坐标轴。失败终止时间不作为完成时间。
- 基线核心测试 682 通过、2 跳过；最终 692 通过、2 项历史跳过；MAS 106 通过（无硬件）。
- 恢复与完整维护记录：`artifacts/migrations/2026-11-01-dynamic-contact-v1/`。

复核已有结果（不运行新物理）：

```powershell
.\.venv\Scripts\python.exe scripts/analyze_dynamic_contact_study.py
```

已有批次只能在源码、环境、协议及产品完全匹配时由 `study --resume` 复用。
不要重新冻结同一路径，也不要覆盖原运行。真正推—拉协作仍是需要连接能力的独立研究路线。

English captions:

**Endpoint comparison.** Final Euclidean endpoint error for all nine paired development conditions and three controller arms (one fixed seed). Filled markers denote completion under the unchanged settled-state and 0.12 m endpoint rule; open markers denote failures. The repair and dynamic-contact arms are not promoted to the default controller.

**Object trajectories.** Recorded object-centroid paths from start to actual termination for every paired development condition. Crosses mark the terminal states, and the shaded disk is the unchanged target tolerance. Reduced cross-track displacement can accompany loss of forward motion; no simulated continuation or missing trajectory is synthesized.
