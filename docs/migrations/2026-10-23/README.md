# 2026-10-23 迁移审计文件

此处保存可版本管理的清单和验证收据。大体积原件位于 `artifacts/migrations/2026-10-23/`，仍属于本地项目完整数据的一部分。

| 文件 | 用途 |
| --- | --- |
| [source_history.json](source_history.json) | 原项目 HEAD、转移检出 HEAD、开发提交与全部原 refs |
| [transfer_inventory.jsonl](transfer_inventory.jsonl) | 转移目录全部 18,355 文件的原路径、字节数与 SHA-256 |
| [transfer_directories.json](transfer_directories.json) | 转移目录的原始子目录列表，包括空目录 |
| [transfer_mapping.jsonl](transfer_mapping.jsonl) | 每个原文件的保留位置；`zip::member` 指 ZIP 内原件 |
| [local_inventory_before.jsonl](local_inventory_before.jsonl) | 重建前原项目可读取文件的哈希清单 |
| [local_access_boundaries.json](local_access_boundaries.json) | 原有不可读测试缓存；未清理或更改权限 |
| [development_mapping.jsonl](development_mapping.jsonl) | Git 开发提交的 646 文件及导入时字节身份 |
| [integration_summary.json](integration_summary.json) | 合并动作数量 |
| [data_summary.json](data_summary.json) | 原运行与恢复数据的范围 |
| [preservation_verification.json](preservation_verification.json) | 全部转移文件、原项目备份及可读数据的哈希检查 |
| [workspace_verification.json](workspace_verification.json) | 新 .venv、依赖、两套测试、smoke、原始文献与精简证据核验 |
| [source_before_removal_verification.json](source_before_removal_verification.json) | 删除前重新完整扫描源目录，确认无新增或改动 |
| [transfer_removal.json](transfer_removal.json) | 原转移目录删除前的验证身份与删除后检查 |

原始保全 ZIP：

- `local-tracked-before.zip`：原项目被 Git 跟踪文件的原始字节。
- `development-568b1fc.zip`：开发提交的完整文件树；保留导出时 Git 原始文本字节。
- `transfer-originals.zip`：旧 checkout 的不同版本、原 `.git`、旧 `.venv`、缓存、配置及工具原件。恢复 Git 历史时在独立目录提取其中 `.git/`，避免覆盖本项目当前 Git 元数据。

当前代码和文档经过重建，导入时的 `installed_sha256` 不代表后续有意修改后的文件哈希；历史代码原字节由对应 ZIP 与清单认证。运行 `.venv` 只由当前机器创建，不能直接启用归档里的旧环境。

验证命令：

```powershell
.\.venv\Scripts\python.exe scripts/maintenance/reconcile_transfer.py verify
.\.venv\Scripts\python.exe scripts/maintenance/verify_workspace.py
```

第一条会读取历史项目文件；部分旧 pytest 产品在受限会话下需要额外读取权限。两条命令都不会删除目录或重写历史实验产品。完整决策见[重建记录](../../REBUILD_2026-10-23.md)。
