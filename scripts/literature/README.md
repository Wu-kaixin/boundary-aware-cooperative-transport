# 文献辅助脚本

这些脚本来自 2026-10-22 阅读阶段，原件已归档。所有命令从项目根目录使用根 `.venv` 运行；参考文件路径已调整为 `data/literature/2026-10-22/`。

只读示例：

```powershell
.\.venv\Scripts\python.exe scripts/literature/read.py L01 1 2
.\.venv\Scripts\python.exe scripts/literature/prose.py L01 1 2
```

`extract.py`、`make_chunks.py`、`build_index.py`、`build_integration.py`、`check_delivery.py` 是历史提取/交付工具，会写入缓存或阅读记录。仅在明确准备重建相应产品并保留旧收据时使用。本次迁移未重新执行它们，也未将历史阅读状态视为本次重新验证。

已完成使命的 `update_builder.py` 和 `update_visual_checks.py` 补丁脚本已从工作代码移除；原件仍保存在本地迁移归档中。
