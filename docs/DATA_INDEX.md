# 数据与原始文件索引

此目录同时保留原项目数据与 2026-10-23 恢复的转移数据。大体积文件在本地完整项目中，不等同于 Git 克隆内容。

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

恢复了 **2,825 个原始运行文件，共 2,112,477,082 字节**。原项目迁移前可读运行文件 8,478 个，逐文件保留。500 个文献数据产品已归位，其中 70 个 PDF 和 1 个 DOCX 按原清单 SHA-256 核验；其余是提取、分块与原页图像。

[逐文件迁移清单](migrations/2026-10-23/transfer_mapping.jsonl)给出每个转移文件的原路径、字节数、SHA-256 和当前路径。`zip::member` 表示原件保留在对应 ZIP 成员内；同名差异版本不会互相覆盖丢失。[保全验证](migrations/2026-10-23/preservation_verification.json)记录最终核对数量。

原项目有 14 个历史测试缓存目录在提升读取权限后仍不可读，另有仅剩旧 pytest 缓存的 `.worktrees/` 遗留目录。它们均保持原状，不能声称其内部已逐文件核验；详见[访问边界](migrations/2026-10-23/local_access_boundaries.json)。转移目录的全部 18,355 个文件已读取和保留。

历史清单保留旧机器绝对路径和 `tmp/literature/` 等原路径。新工具使用当前目录；历史文件身份、环境归属和科研结论不重写。文献全文不提交到 Git。

完整迁移时复制 `runs/`、`data/literature/`、`artifacts/`、`tools/` 及代码文档；在新电脑重新创建 `.venv`。随后从[README](../README.md)的流程验证。参考环境和数据目录不应从其他项目借用。
