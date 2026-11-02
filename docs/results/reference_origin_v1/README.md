# 参考初始化：固定24回合消融

**两项修改均未通过事先冻结的采用门槛，不替换默认方法。** 原方法、提前参考和取消初始化放大均安全完成7/8；取消放大救回远场斜箱，却使远场C形失败。全部程序正常结束，没有用满600秒的截尾回合，没有追加调参。论文题目不变。

| 消融臂 | 安全完成 | 错误HOLD | G500 | 硬安全失败 | 约束缩放 | 截尾 |
| --- | --- | --- | --- | --- | --- | --- |
| Original | 7/8 | 1 | 6/8 | 0 | 5 | 0 |
| Earlier reference | 7/8 | 1 | 5/8 | 0 | 4 | 0 |
| Unscaled bootstrap | 7/8 | 1 | 6/8 | 0 | 7 | 0 |

“错误HOLD”指首次进入HOLD时，评价真值目标误差大于原0.12m任务容差；评价真值不进入控制。安全完成使用原任务完成与硬安全判据。G500另有激活、到达和零约束缩放等要求，不能与任务完成或硬安全互换。完整字段见[逐回合CSV](episodes.csv)、[分析JSON](analysis.json)。

## 固定身份与实际改动

8个已消费开发条件 × 3臂，固定seed 20261102，16机器人、dt=0.05s，每回合上限600s。六个近场条件包含圆、椭圆两朝向、矩形、L和C；两个远场条件为开口背向C形和斜箱。所有部署模块固定boundary_cvt，本轮不比较spacing或nearest优越性。

参考配置沿用上一轮max_object_speed=0.45的开发版本，**不是全局默认0.25**；关闭上一轮失败的投票连续化，保持原1帧HOLD许可。控制到达带0.02m、任务容差0.12m、40帧物理停稳、物理、安全与接触模型均未改动。各条件三臂初态完全相同。

- Original：原CONTACT_READY后建立不可变参考，初始偏移为1.24倍既有地图位移。
- Earlier reference：最早已有可信地图及可观测自身/一跳观测时建立参考，仍继承原1.24倍地图位移；不假设物体静止、不把偏移置零。CONTACT_READY前仅更新观测器，不进入导航反馈，之后不重建参考。
- Unscaled bootstrap：建立时机保持原样，**仅参考初始化偏移**使用地图位移原值1.0倍；后续配准、投票、速度、其他1.24进度增益均不改动。

代码改动集中在controller.py的两个默认关闭/继承设置、observed_reference.py的实际初始化元数据，以及deployment.py的收据输出。初始化记录保存实际时间、观测锚点、偏移、径向支点和模板点/法向，不含评价真值。运行中没有加入额外观测、未知完整几何或附着能力。局部读数仅用自身和一跳信息；原全队公共阶段监督仍存在，不能写成完全去中心化闭环。

8条原方法replay所有数组和原有逐帧科学字段精确复现上一轮；显式新增False/None配置项引起的两个身份哈希独立重算后单列。16条修改臂在CONTACT_READY控制区间结束前的物理轨迹、命令、阶段及导航记录均与配对原方法精确一致；早期观察器诊断记录按设计不同。之后是不同闭环轨迹，不把跨臂MAE当作同输入估计比较。

## 结果与反证

取消初始化放大将斜箱终点误差从135.11mm降至31.17mm，失败变为完成；但C形从77.41mm升至150.00mm，完成变为错误停止。总完成数仍7/8，原方案中的失败被转移，没有普遍解决。

提前参考将C形误差降至17.21mm；斜箱仍在125.53mm处错误停止。它曾距目标25.65mm，随后越过目标，因此“曾经接近目标”不等于最后完成。矩形仍完成任务，但新增两次约束缩放，导致G500从6/8降至5/8。冻结门槛要求救回斜箱、不能损失原完成、没有新增硬安全失败且G500不下降；两臂均FAIL。

[全部24回合图](all_episodes.pdf) · [实际与控制读数](stop_and_bias.pdf) · [首次HOLD误差来源](hold_error_components.pdf)。

## 误差究竟从哪里来

逐机器人令参考创建时刻为tau_i，继承偏移为o_i，锚点加有效径向支点为p_i。离线评价的物体中心为c，转角为theta：

```text
b_i = o_i - (c(tau_i) - c(0))
r_i(k) = [R(theta(k)-theta(tau_i)) - I] [p_i - c(tau_i)]
q_i(k) = reference_i(k) - (c(k)-c(0)) - b_i - r_i(k)
```

原聚合是坐标中位数，不能错误地把三组中位数直接相加。对当时实际投票集合V，定义B=median(b)、Rpart=median(b+r)-median(b)、Q=median(reference)-true-median(b+r)，因此总误差=B+Rpart+Q。真值只在离线计算。这里Q包含配准读数与真实刚体变换之间的余项；B、Rpart、Q相关且分解次序固定，数值恒等式不等于独立因果归因。

| 条件 | 臂 | 初始化 x/mm | 参考点旋转 x/mm | 配准余项 x/mm | 总 x 误差/mm |
| --- | --- | --- | --- | --- | --- |
| far_c_open_away_pos0 | reference | 40.12 | 31.07 | 4.29 | 75.48 |
| far_c_open_away_pos0 | early_reference | -5.00 | 20.77 | 4.83 | 20.59 |
| far_c_open_away_pos0 | unscaled_bootstrap | 69.89 | 106.33 | -14.89 | 161.33 |
| far_skew_box_s0_r0_pos0 | reference | 45.42 | -17.28 | 92.90 | 121.04 |
| far_skew_box_s0_r0_pos0 | early_reference | -6.39 | -24.37 | -78.30 | -109.06 |
| far_skew_box_s0_r0_pos0 | unscaled_bootstrap | 58.00 | -44.11 | -20.98 | -7.08 |

两项关键发现：

1. 原斜箱首次HOLD的x误差中，初始化项+45.42mm，旋转项−17.28mm，配准余项+92.90mm。提前建立参考后，初始化项降至−6.39mm，但配准余项变成−78.30mm，仍错误停止。**只把参考建立得早，不能保证后续配准正确。**
2. 取消放大的C形，创建时各机器人偏移误差平均范数从171.04降至138.75mm，但HOLD时x方向参考点旋转项增至+106.33mm，加上+69.89mm初始化项与−14.89mm配准余项，最终读数比真实中心位移多161.33mm。终点估计目标误差仅14.38mm，真实误差却150.00mm。**局部边界参考点不是已知质心；旋转会使这两种位移不同。**

第二点不是发现配准程序必然写错，也不能直接用真值质心修控制器。它说明当前读数语义与评价中心位移之间存在可量化差异。缩放偏移可改变之后的转动、接触和有效投票集合，因而局部初始化误差下降不保证任务改善。也不能按形状从两种干预里事后挑最优臂、拼成8/8成绩。

新结果使用实际记录的不可变参考元数据。之前8条轨迹的初始化审计则只从既有姿态离线重建创建时的传感扫描，核对返回点数、支点状态与残差；它不是原始全扫描重放，也不算新增物理实验。原审计代码版本与输出保存在本地恢复区。

## 下一步判断（未执行）

本轮停止，保留两个选项作为消融，不组合、不搜索中间增益。后续最小核验应先利用已保存的模板和轨迹，在**相同输入**上区分：真实刚体变换下的参考点旋转，实际匹配的平移/转角误差，以及投票者各自参考系的不一致。主问题应转为“局部观测能否产生与任务目标一致的位移读数，并辨别何时不能可靠判断已到达”，而不是继续永久抵消换票或延长等待。

只有当候选能用允许的局部观测建立一致任务参考，或给出不依赖真值的可检验误差界/拒绝策略，才值得冻结新闭环比较。禁止直接读取物体中心/完整轮廓来消除旋转项。若局部信息不足以辨认中心位移，应明确记录不可辨识条件；不能悄悄把评价改成任意边界点到达。新的任务/信息假设必须单独研究，本轮判据不变。

## 全部逐回合结果

| 条件 | 臂 | 终点误差/m | 物理秒 | 状态 | G500 |
| --- | --- | --- | --- | --- | --- |
| near_circle | reference | 0.019148 | 15.95 | COMPLETED | True |
| near_circle | early_reference | 0.032002 | 15.70 | COMPLETED | True |
| near_circle | unscaled_bootstrap | 0.027219 | 15.10 | COMPLETED | True |
| near_ellipse_parallel | reference | 0.016572 | 14.05 | COMPLETED | True |
| near_ellipse_parallel | early_reference | 0.018764 | 17.20 | COMPLETED | True |
| near_ellipse_parallel | unscaled_bootstrap | 0.017441 | 13.75 | COMPLETED | True |
| near_ellipse_perpendicular | reference | 0.006598 | 15.55 | COMPLETED | True |
| near_ellipse_perpendicular | early_reference | 0.029783 | 15.70 | COMPLETED | True |
| near_ellipse_perpendicular | unscaled_bootstrap | 0.017173 | 14.15 | COMPLETED | True |
| near_rectangle_parallel | reference | 0.012817 | 13.35 | COMPLETED | True |
| near_rectangle_parallel | early_reference | 0.027514 | 13.70 | COMPLETED | False |
| near_rectangle_parallel | unscaled_bootstrap | 0.007849 | 12.65 | COMPLETED | True |
| near_l_reference | reference | 0.008433 | 19.65 | COMPLETED | True |
| near_l_reference | early_reference | 0.009044 | 17.25 | COMPLETED | True |
| near_l_reference | unscaled_bootstrap | 0.019411 | 13.90 | COMPLETED | True |
| near_c_open_away | reference | 0.038804 | 11.90 | COMPLETED | True |
| near_c_open_away | early_reference | 0.015115 | 11.70 | COMPLETED | True |
| near_c_open_away | unscaled_bootstrap | 0.038290 | 12.25 | COMPLETED | True |
| far_c_open_away_pos0 | reference | 0.077409 | 75.40 | COMPLETED | False |
| far_c_open_away_pos0 | early_reference | 0.017213 | 60.25 | COMPLETED | False |
| far_c_open_away_pos0 | unscaled_bootstrap | 0.149998 | 64.90 | SETTLED_OFF_TARGET | False |
| far_skew_box_s0_r0_pos0 | reference | 0.135106 | 89.25 | SETTLED_OFF_TARGET | False |
| far_skew_box_s0_r0_pos0 | early_reference | 0.125527 | 60.50 | SETTLED_OFF_TARGET | False |
| far_skew_box_s0_r0_pos0 | unscaled_bootstrap | 0.031170 | 56.70 | COMPLETED | False |

## 全部G500不通过项

| 条件 | 臂 | 原G500不通过原因 |
| --- | --- | --- |
| far_c_open_away_pos0 | reference | G500: transport activation never happened within the 1508-frame run  /  G500: 3 scaled-barrier event(s), smallest factor 0.477; the contract allows 0 |
| far_skew_box_s0_r0_pos0 | reference | G500: transport activation never happened within the 1785-frame run  /  G500: target reached never happened within the 1785-frame run  /  G500: J=1.0651 m stopped 0.1349 m short of L=1.2000 m, outside the declared arrival tolerance 0.1200 m  /  G500: 2 scaled-barrier event(s), smallest factor 0.926; the contract allows 0 |
| near_rectangle_parallel | early_reference | G500: 2 scaled-barrier event(s), smallest factor 0.559; the contract allows 0 |
| far_c_open_away_pos0 | early_reference | G500: transport activation never happened within the 1205-frame run  /  G500: 2 scaled-barrier event(s), smallest factor 0.473; the contract allows 0 |
| far_skew_box_s0_r0_pos0 | unscaled_bootstrap | G500: 7 scaled-barrier event(s), smallest factor 0.160; the contract allows 0 |
| far_skew_box_s0_r0_pos0 | early_reference | G500: J=1.3237 m stopped 0.1237 m past L=1.2000 m, outside the declared arrival tolerance 0.1200 m |
| far_c_open_away_pos0 | unscaled_bootstrap | G500: target reached never happened within the 1298-frame run  /  G500: J=1.0504 m stopped 0.1496 m short of L=1.2000 m, outside the declared arrival tolerance 0.1200 m |

G500中“transport activation never happened”是原事件条件未触发，不表示物体从未发生物理接触或推进。所有24条均检测到物体、达到CONTACT_READY并发生实际接触；硬安全失败为0，缩放事件分别5/4/7次另行保留。

## 工程验证与复现

基线40项通过；修改后核心818项通过、2项历史跳过。原环境与冻结源码保持一致，pip check与diff check通过。PyYAML实际6.0.2、锁文件6.0.3的既有差异仍保留，没有安装或升级。没有实机测试或论文改写。

4个独立案例worker、内部线程1，本批墙钟381.32s，采样进程树峰值RSS约1.44GiB；仅是本批实测，不是并行加速倍数。Windows磁盘等待、CPU频率等计数器不可用，记录为缺失，没有虚构。

8个复用开发条件不是独立泛化验证，不支持任意形状或90%总体可靠性。帧、匹配数和16机器人均不充当独立实验单位。本轮有24条新执行，但只有8个配对条件块。

- [冻结协议](../../../experiments/ccta2027/reference_origin_v1.json)
- [执行入口](../../../scripts/reference_origin_study.py)
- [初始化审计](../../../scripts/audit_reference_origin.py)
- [结果分析](../../../scripts/analyze_reference_origin.py)
- [图表生成](../../../scripts/publish_reference_origin.py)
- [图—数据—版本与英文caption](figure_manifest.json)

原始结果及源快照：runs/research/2026-11-02-reference-origin-v1/comparison/。工程记录及逐帧分析：runs/maintenance/2026-11-02-reference-origin-v1/。状态、备份及最终核验：artifacts/migrations/2026-11-02-reference-origin-v1/。本轮新增产物留在原仓库，未提交、推送或移动分支，未删除历史内容。
