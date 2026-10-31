# 数据与原始文件索引

本索引区分历史迁移记录与当前机器的本地文件。大体积文件在本地完整项目中，不等同于 Git 克隆内容。

2026-10-31 原地整理从 PC2 恢复本地运行、文献和旧备份，保留其原相对路径、批次结构和字节。
PC3 中重复错放的稿件图片按哈希映射到已有规范副本；40种损坏 ZIP/bundle 的原始字节逐项保全在
本次 `backup/source-originals-PC3.zip` 的独立成员中。续查确认10个文件由有效开发归档前缀和周期填充组成，
另30个完全由周期填充组成。只有在全字节比较、原始归档成员哈希及有效备份验证后，才清理原路径的重复副本。
有效的2026-10-23恢复包继续保留，开发归档可由原Git提交再生成相同SHA-256；有效bundle的独有旧对象也完成隔离恢复。
这些结果不能证明PC3文件损坏前的完整身份；要证明它们当时没有独有改动，仍需损坏前清单/哈希或可读原件。
损坏原始字节没有被丢弃，也不作为有效恢复包或活动代码。
逐项位置、校验结果及完整性限制见本地
`artifacts/migrations/2026-10-31-single-repository/migration_manifest.csv` 与 `FINAL_REPORT.md`。
后续40项决定及证据在同目录 `pc3-recovery-001/recovery_decisions.csv` 与验证收据中；不是改写原历史映射。
这些是维护记录，不是新增科研样本。历史来源清单中的未知执行/分析版本保持 unknown，不从文件夹名补造身份。

| 位置 | 数据与归属 |
| --- | --- |
| `runs/ccta2027/` | 原项目既有 CCTA 原始执行产品，保持原字节 |
| `runs/evidence_closure/`、`runs/consolidation/` 等 | 原项目历史运行、诊断与归档 |
| `runs/reliability/2026-10-22*` | 恢复的基线、失败候选、默认修复与逐案例记录 |
| `runs/reliability/validation_2026-10-22*` | 前瞻设计、冻结协议和初始化预检；不是已执行的 832 个实验 |
| `docs/results/` | 按源码和实验版本分组的精简公开证据 |
| `data/literature/2026-10-22/` | 本地原件、逐页全文、分块及原图 |
| `docs/literature_review_2026-10-22/` | 71 份阅读笔记、索引、能力矩阵和研究定位工作稿 |
| `manuscript/ccta2027/` | 既有论文 TeX/PDF 草稿与引文依据 |
| `artifacts/manuscript/ccta2027/2026-10-22/` | 原始论文渲染和编译日志 |
| `artifacts/validation/2026-10-22/` | 原始测试 XML 与测试临时产品 |
| `artifacts/validation/2026-10-23/`、`runs/rebuild_20261023/` | 本次重建测试和五案例 smoke；独立于历史科研结果 |
| `artifacts/migrations/2026-10-23/` | 原版本、隐藏 Git/环境文件及开发分支的可恢复备份 |
| `runs/maintenance/2026-10-31-single-repository/` | 当前机器基线、回归测试及短 smoke；独立于论文实验 |
| `artifacts/migrations/2026-10-31-single-repository/` | 本次清单、操作日志、来源原件、隔离记录及恢复说明 |

2026-10-23 收据记录恢复了 **2,825 个原始运行文件，共 2,112,477,082 字节**。该次迁移前可读运行文件 8,478 个，逐文件保留。500 个文献数据产品已归位，其中 70 个 PDF 和 1 个 DOCX 按原清单 SHA-256 核验；其余是提取、分块与原页图像。上述历史数量不等于本次输入来源或新增产物数量。

[逐文件迁移清单](migrations/2026-10-23/transfer_mapping.jsonl)给出每个转移文件的原路径、字节数、SHA-256 和当前路径。`zip::member` 表示原件保留在对应 ZIP 成员内；同名差异版本不会互相覆盖丢失。[保全验证](migrations/2026-10-23/preservation_verification.json)记录最终核对数量。

2026-10-23 原项目记录了 14 个不可读历史测试缓存目录和 `.worktrees/` 遗留目录，详见当时的[访问边界](migrations/2026-10-23/local_access_boundaries.json)。这是旧机器状态，不能代替当前逐文件清单。该次转移目录的全部 18,355 个文件由历史映射定义其身份。

历史清单保留旧机器绝对路径和 `tmp/literature/` 等原路径。新工具使用当前目录；历史文件身份、环境归属和科研结论不重写。文献全文不提交到 Git。

完整迁移时复制 `runs/`、`data/literature/`、`artifacts/`、`tools/` 及代码文档；在新电脑重新创建 `.venv`。随后从[README](../README.md)的流程验证。参考环境和数据目录不应从其他项目借用。
