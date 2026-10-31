# DBACT 单仓库使用指南

从当前 Git 根目录运行。使用根 `.venv`，MAS 共用这个环境；不要启用迁入副本或归档中的环境。
代码导航见[架构](ARCHITECTURE.md)，科研结果边界见[结果索引](results/README.md)，
原始资料与旧机器路径见[数据索引](DATA_INDEX.md)。

| 工作 | 入口与位置 |
| --- | --- |
| 仿真、实验及分析 | `.venv/Scripts/python.exe -m dbact_sim --help`；实现位于 `src/dbact_sim/` |
| 局部观测、部署和控制 | `src/dbact/`；旧模块路径保留同一模块对象的兼容别名 |
| 平台适配 | `src/mas_adapter/`、`platforms/mas_public/`；硬件运行须另行授权 |
| 配置与实验设计 | `configs/`、`experiments/`；保持历史协议及 seed 身份 |
| 论文、证明与文献 | `manuscript/`、`theory/`、`data/literature/` 与现有阅读笔记目录 |
| 当前工程验证 | `runs/maintenance/<mission>/<attempt>/`，每次使用新目录 |
| 本地审计及恢复 | `artifacts/migrations/<mission>/`；完整清单及源 Git、旧环境原件不提交 |

2026-10-31 维护记录使用任务标识 `2026-10-31-single-repository`。
`inventory.csv` 记录进入任务时的文件身份；`migration_manifest.csv` 记录每项最终保留位置；
`operations.jsonl` 记录实际动作，`FINAL_REPORT.md` 说明完成状态及原有损坏内容。
`archive.zip::member` 表示 ZIP 内的原字节。恢复前先核验清单 SHA-256，再将单个成员写入新的仓库内位置；
禁止覆盖根 `.git` 或活动代码。源环境只供追溯，不可直接运行。

本地的同盘备份用于误操作回滚，不能防止整盘故障。历史 ZIP 若损坏，保全其字节并不意味着修复了内容。
本次记录区分有效 PC2 原备份与 PC3 中损坏、错放的副本。
后续PC3核查逐字节确认：10个文件保留有效开发归档前缀，30个没有周期填充以外的内容。
40种损坏原始字节仍保存在已验证的外层ZIP成员中；原路径的重复副本经条件清理移除。
有效历史备份、原Git历史与独有旧对象保留；不能据此宣称PC3的损坏前身份已完全恢复。
本地 `pc3-recovery-001/recovery_decisions.csv` 逐项记录替代备份、原件归档及尚缺的损坏前身份依据。

当前活动分支为 `main` 和 `paper/boundary-aware-safe-deployment`。前者保留已有基线，后者承载本次经验证的维护。
已被paper分支包含的本地临时整理分支退役；原标签与归档内部历史不删除。发布只针对明确审查的代码/文档，
不上传本地完整数据、文献或恢复材料。

已有入口的验证可以统一记录到新目录：

```powershell
.\.venv\Scripts\python.exe scripts/maintenance/validate_repository.py --output runs/maintenance/my_task/after-001 --extended
```

该命令只运行健康检查、核心/MAS 无硬件测试、5 例短 smoke、分析与同身份 resume，
不运行完整科研矩阵或实机通信。smoke 的执行完成不等于 CONTACT_READY、物理接触、收定、G500 或运输完成。

历史迁移工具的 `verify` 默认只读；需要保存当前检查时，指定新的 `--output`。
`prepare`、`integrate` 与 `source-check` 属于 2026-10-23 的一次性流程，不作为当前整理入口。
活动仓库不采用外部工作区或三分区方案。根目录包管理文件保持标准位置；不为目录整齐制造第二套代码。
