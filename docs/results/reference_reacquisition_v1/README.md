# 缺参考时的观测恢复候选：实现检查点

2026-11-03：实现 `reference_map_reacquisition=False` 候选开关，默认关闭、尚未采用。**目前只有工程验证，新增闭环物理回合为 0。** 本次按用户要求先发布已有成果；八回合比较仍为[前瞻设计](../../../experiments/ccta2027/reference_reacquisition_v1_design.md)，尚未冻结可执行协议或运行。

[接触激活诊断](../contact_activation_v1/README.md)发现，缺少局部位移参考的远处机器人连原搜索、召回、接近指令也被归零。候选仅在初始参考已经有效冻结、局部参考不可用、阶段早于 TRANSPORT 时，允许 `search` / `recall` / `approach` 的原名义指令继续进入原安全 QP，并要求 `push_side=False`、`effort=0`。输出模式增加 `map_reference_reacquire_` 前缀以便追踪。

初始采集失败、redeploy/cage/push、非零按压力以及 TRANSPORT/BRAKE/HOLD 仍保持原缺参考抑制。地图仍只使用已有的新鲜单机/一跳观测，保留缺测时最大物体速度界。没有改变 QP、安全距离、速度上限、物理模型、阶段门槛、成功判据或最佳配置数值。

最佳配置 SHA-256 保持 `a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112`。新增 16 项回归先观察到缺少接口的 RED，再通过包含相关模块的 57 项针对性检查；完整回归及源码身份见[工程收据](engineering_checks.json)。原 RED、GREEN、完整回归日志分别保留在 `runs/maintenance/2026-11-03-reference-reacquisition-v1/`，不覆盖旧实验。

下一步按原四个开发条件执行原主动采集与本候选的匹配八回合比较，验证旧四例精确复现、候选采集前缀不变、安全完整完成、误停、硬违规、G500 与冻结后缩放。候选只在初始参考冻结后起作用，因此**不会解决之前已发生的 C/斜箱各 13 次采集缩放**。即使后续恢复运动，也需要分别验证接触分工、联合安全可行性和可靠停止，不能据此采用候选或宣布完整目标完成。

本地环境保留原 joblib 1.5.1 / PyYAML 6.0.2，锁文件对应 1.6.0 / 6.0.3；工程 CI 与未来物理试验分别记录，不能把旧物理结果重标为锁环境复跑。

[实施计划与状态](../../../experiments/ccta2027/reference_reacquisition_v1_plan.md) · [回归测试](../../../tests/test_reference_reacquisition.py) · [控制器](../../../src/dbact/control/controller.py)
