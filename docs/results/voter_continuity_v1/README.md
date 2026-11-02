# 投票集合连续性与停止确认：固定预算开发消融

状态：32回合固定比较完成；预注册组合候选采用门槛 **FAIL**。所有结果见[完整逐回合表](episodes.csv)和[分析JSON](analysis.json)。默认仍关闭，论文题目未修改。

| 消融臂 | 安全完成 | 错误HOLD | G500 | 硬安全失败 | 约束缩放 | 截尾 |
| --- | --- | --- | --- | --- | --- | --- |
| Original | 7/8 | 1 | 6/8 | 0 | 5 | 0 |
| Continuity | 6/8 | 1 | 6/8 | 0 | 0 | 1 |
| Confirm 1 s | 6/8 | 0 | 6/8 | 0 | 13 | 2 |
| Combined | 6/8 | 1 | 6/8 | 0 | 0 | 1 |

这里的“错误HOLD”严格指首次进入HOLD时，独立评价的真实目标误差超过原任务容差0.12m；它不由控制器读取。没有进入HOLD的截尾回合不能算可靠停止。G500、硬安全、实际完成分别报告。

## 比较身份

8个已经消费过的开发条件 × 4臂，固定seed 20261102，每回合至多600秒，16机器人、dt=0.05秒。参考是上一轮0.45运动估计先验的开发候选，**不是全局默认0.25**。四臂共用该设置、相同初态、同一感知/物理/推手控制、安全及任务判据。8条原方法重跑的全部replay数组及原有逐帧科学字段均精确复现上一轮；新增诊断字段不参加旧字段比较。32条均为新执行，未把旧数据重复充作新增样本。

身份核验明确区分配置哈希与数值：显式增加False/1两个默认开关会改变config_sha256和config_hash。已核对配置仅有这两项增加，分别用原SHA-256及BLAKE2算法重算两个哈希；这两个身份字段单列，所有其余旧字段仍要求精确相等。首版分析把身份哈希也当作科学数值比较而触发断言，原分析及日志已保留在本地审计；只修正离线核验，没有重跑仿真、修改结果或放宽数值容差。

连续化仅在相邻时刻共有的有效投票者上计算位移变化，累加抵消换票贡献；每个机器人维护自己的本地实例，只接收自身和一跳消息。全队实例只供原有公共阶段监督器。无共有成员时不假装连续：该帧拒绝读数并以当前原始值建立新段，下一帧有共有成员才恢复。切换地图/参考估计来源时分段。

连续化作用于已有局部导航和团队导航的整个读数链，因此也可能改变接触前的转向和部署轨迹，不能把该臂的所有终点差异归因于最后一次停止跳变。相反，“仅停止确认”在原方法首次HOLD以前的轨迹及科学字段逐帧对比见analysis.json的stop_only_prefix_equivalence；它是区分停止许可与早期闭环改变的对照。

停止确认仅将原到达、低速、可用估计条件要求持续20帧（1秒），对照为原1帧。窗口事先借用已有contact_dwell长度，没有试多个窗口。制动、局部可撤销卸载及独立物理停稳规则保持原样；没有修改0.02m控制到达带、0.12m任务容差或安全阈值。

## 区分瞬时跳变与历史偏差

上一轮远场斜箱首次HOLD处，物体没有移动，移除agent_03贡献−49.25mm，共有成员变化−3.83mm，总跳变−53.08mm。这让估计误差进入20mm到达带，但真实误差仍135.11mm。

从同轮首次有效参考读数累计到HOLD，斜箱x误差分解为：初值+51.02mm、换票累计−22.06mm、共有成员相对真位移的误差+92.08mm，合计+121.04mm。远场C形则为+152.22−103.37+26.63=+75.48mm。因而部分换票恰好抵消历史偏差；消除跳变并不等于估计无偏，也不能把全部误差归因于换票。

公式为 e_k = e_0 + Σ(roster_delta) + Σ(common_delta − true_delta)，仅在连续有效段内成立。共有成员误差仍混合配准、旋转参考点、历史估计和成员选择效应，不能直接指定某一模块为根因。数值恒等式最大残差≤1.12e−16m，不是精度证明。

[全部回合图](all_episodes.pdf) · [停止与偏差时间序列](stop_and_bias.pdf) · [旧日志累计误差分解](prior_error_accounting.pdf) · [首次HOLD与同输入误差](stop_audit.json)。

## 本轮研究判断

**不采用这两个设置替换原方法；停止扩展本轮“全程抵消换票偏移＋固定1秒确认”的组合。** 六个近场条件四臂都安全完成；原本完成的远场C形在三个修改臂均未完成，斜箱也没有救回。所有臂G500仍6/8；硬安全失败均为0，但约束缩放另计，不能把它们抹去。这里只否定本轮具体方案，并不否定继续研究可撤销的停止确认。

斜箱连续化臂的同输入原始/使用读数MAE为44.30/305.83mm；组合臂为41.29/331.89mm。组合进入HOLD时，原始x估计误差约+30.50mm，累积连续化偏移约−389.71mm，实际使用的x估计误差变成−359.21mm；即使连续20帧满足原停止条件，仍在真实误差360.16mm处停止。这是候选自己引入并保留大幅聚合偏置的直接数值证据，不能只归咎于原有估计。连续桥接恒等式残差很小，也不代表读数正确。

仅确认臂在原HOLD之前逐帧一致，随后阻止了原斜箱错误HOLD，但到600秒仍在BRAKE、终点误差741.60mm；C形误差140.85mm，且未完成停稳终止。这说明“没有锁定错误HOLD”和“可靠完成任务”是两个指标。连续化C形与组合C形的终点误差均778.34mm；该轨迹没有一次满足全部原停止条件，不能通过解释为确认窗太长来掩盖反馈本身的问题。

下一项有价值的核验应围绕一致的位移参考与偏差来源：分别量化参考初始化误差、共有读数的后续偏差、成员切换的短暂影响，避免把所有成员变化永久积成校正量。可以保留可撤销的到达候选状态作为后续设计，但本轮不再缩短窗口、调增益或换seed追求成功。

## 全部任务或G500不通过项

| 条件 | 臂 | 终点误差/m | 物理秒 | 状态 | G500原因 |
| --- | --- | --- | --- | --- | --- |
| far_skew_box_s0_r0_pos0 | confirmation | 0.741596 | 600.0 | CENSORED_600S_BRAKE | G500: transport activation never happened within the 12000-frame run  /  G500: J=0.8501 m stopped 0.3499 m short of L=1.2000 m, outside the declared arrival tolerance 0.1200 m  /  G500: the run did not end in HOLD  /  G500: 10 scaled-barrier event(s), smallest factor 0.285; the contract allows 0 |
| far_skew_box_s0_r0_pos0 | continuity | 0.452629 | 63.400000000000006 | SETTLED_OFF_TARGET | G500: J=1.6514 m stopped 0.4514 m past L=1.2000 m, outside the declared arrival tolerance 0.1200 m |
| far_c_open_away_pos0 | continuity | 0.778344 | 600.0 | CENSORED_600S_BRAKE | G500: transport activation never happened within the 12000-frame run  /  G500: J=0.4593 m stopped 0.7407 m short of L=1.2000 m, outside the declared arrival tolerance 0.1200 m  /  G500: the run did not end in HOLD |
| far_c_open_away_pos0 | reference | 0.077409 | 75.4 | COMPLETED | G500: transport activation never happened within the 1508-frame run  /  G500: 3 scaled-barrier event(s), smallest factor 0.477; the contract allows 0 |
| far_c_open_away_pos0 | combined | 0.778344 | 600.0 | CENSORED_600S_BRAKE | G500: transport activation never happened within the 12000-frame run  /  G500: J=0.4593 m stopped 0.7407 m short of L=1.2000 m, outside the declared arrival tolerance 0.1200 m  /  G500: the run did not end in HOLD |
| far_c_open_away_pos0 | confirmation | 0.140850 | 600.0 | CENSORED_600S_BRAKE | G500: transport activation never happened within the 12000-frame run  /  G500: J=1.3342 m stopped 0.1342 m past L=1.2000 m, outside the declared arrival tolerance 0.1200 m  /  G500: the run did not end in HOLD  /  G500: 3 scaled-barrier event(s), smallest factor 0.477; the contract allows 0 |
| far_skew_box_s0_r0_pos0 | reference | 0.135106 | 89.25 | SETTLED_OFF_TARGET | G500: transport activation never happened within the 1785-frame run  /  G500: target reached never happened within the 1785-frame run  /  G500: J=1.0651 m stopped 0.1349 m short of L=1.2000 m, outside the declared arrival tolerance 0.1200 m  /  G500: 2 scaled-barrier event(s), smallest factor 0.926; the contract allows 0 |
| far_skew_box_s0_r0_pos0 | combined | 0.360159 | 83.5 | SETTLED_OFF_TARGET | G500: J=1.5596 m stopped 0.3596 m past L=1.2000 m, outside the declared arrival tolerance 0.1200 m |

## 结论边界与复现

这些是事后选定的8个开发条件，不支持任意形状、90%可靠性或对其他部署算法的优势结论。同一臂内部的raw/used MAE使用完全相同的输入与物理时刻；跨臂MAE依赖不同轨迹及终止时刻，不能当作同输入精度排名。确认持续到达只能检验估计的时间持续性，无法证明绝对位移正确。全部失败、截尾及负向变化保留；本轮停止，不追加调参。

实际环境记录于analysis.json，PyYAML仍为本轮开始前已有的6.0.2，与锁文件6.0.3不一致；未安装或升级依赖。本轮所有臂同环境。

- [冻结协议](../../../experiments/ccta2027/voter_continuity_v1.json)
- [执行入口](../../../scripts/voter_continuity_study.py)
- [旧日志审计](../../../scripts/audit_voter_continuity.py)
- [结果分析](../../../scripts/analyze_voter_continuity.py)
- [图表与报告生成](../../../scripts/publish_voter_continuity.py)
- [图—数据—脚本哈希及英文caption](figure_manifest.json)

原始结果、源代码快照和收据：runs/research/2026-11-02-voter-continuity-v1/ablation/。保护、状态及恢复记录：artifacts/migrations/2026-11-02-voter-continuity-v1/。工程验证及逐帧分析CSV：runs/maintenance/2026-11-02-voter-continuity-v1/。大体积原始轨迹和备份留在本仓库内，不自动加入Git。
