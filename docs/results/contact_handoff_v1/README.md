# 确认与回退接触交接 v1：固定预算结果

三个批次分别冻结：12 回合压力/换位消融、27 回合 150 秒主比较，以及用户指定的 18 回合最多 600 秒观察。
同一开发 seed 2；原始初态、感知、物理、安全约束、阶段/停稳和二维完成判据保留。全部失败和执行身份保留。
这不是独立泛化验证，也不是硬件实验。

| 比较 | 原 CVT | spacing | 交接候选 |
|---|---:|---:|---:|
| 150 秒任务完成 | 5/9 | 5/9 | 5/9 |
| 150 秒 G500 | 5/9 | 3/9 | 6/9 |
| 600 秒观察任务完成 | 5/9 | 未在本批执行 | 5/9 |

600 秒只是独立观察预算：已经按原规则进入 HOLD 并停稳的回合在原时刻结束；不会用真值重启。
尚未完成且到达 600 秒的回合记为右删失，不能推断永远不可能完成。

## 本轮结论

压力分配单独启用即可重现矩形成功丢失：反馈修复/仅换位分别完成 1/3，归一化分配/联合版本分别 0/3。
这支持保留原有推压能力，不支持把上一版退化全部归因于换位。消融中的反馈修复开关与主比较的默认控制明确分开。

主比较记录 35 次提议、10 次接纳、7 次位置到达判定，
但持续局部确认 0 次，试探卸载 0 次，原推手退出 0 次。
到达判定的前一状态计数为 `{'offer': 7}`：这些都是已在目标邻域的接纳，不能当作完成侧向换位。
因此个别终点误差下降不能归因于成功交接。候选没有通过事前完成能力门槛，默认开关仍关闭。

局部间隙、推压命令和运动响应仅构成操作性代理判据，既非力传感器确认，也不能辨识替补的独立贡献。
当前候选还要求单个替补不产生反向纵向分量，这比无附着物理更严格；其拒绝不证明对应形状不可搬运。

## 可追溯结果

- [消融全表](ablation/capability_table.md) · [CSV](ablation/all_episodes.csv)。
- [150 秒九条件全表](main/capability_table.md) · [CSV](main/all_episodes.csv)。
- [600 秒观察全表](extended/capability_table.md) · [CSV](extended/all_episodes.csv)。
- [主比较终点图 PDF](main/endpoint_comparison.pdf) · [全轨迹图 PDF](main/complete_trajectories.pdf)。
- [研究判断、文献对照与标题选项](../../CONTACT_HANDOFF_RESEARCH_2026-11-01.md)。
- [图表含义、英文图注与本地视频](MEDIA.md)。

六个消融参考、十八个主比较对照的回放/原日志字段精确一致；十八个延长观察的 150 秒前缀检查通过。
记录中的硬安全异常与 barrier scaling 分别报告，未用测试通过代替动态安全证明。

## 复现与身份

完整原始数据位于本地 `runs/research/2026-11-01-contact-handoff-v1/ablation|main|extended/`，每回合有独立收据和产品哈希。
规范派生结果位于 `artifacts/research/2026-11-01-contact-handoff-v1/`。本目录是经 SHA-256 核对的小型公开证据副本。
完整视频、逐帧数据和环境不随 Git 克隆提供；不自动提交或上传。

```powershell
.\.venv\Scripts\python.exe scripts/analyze_contact_handoff_study.py ablation
.\.venv\Scripts\python.exe scripts/analyze_contact_handoff_study.py main
.\.venv\Scripts\python.exe scripts/analyze_contact_handoff_study.py extended
```

重新执行必须使用相应冻结源码/环境和新输出目录；同身份中断使用原目录 `--resume`。
不能把当前 checkout 的提交号当作脏工作树实验源码身份，也不能修改旧协议后继续同一收据。
