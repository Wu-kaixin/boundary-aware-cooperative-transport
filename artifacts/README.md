# 生成产品与迁移原件

- `publication/`：既有论文图表与产品清单。
- `manuscript/ccta2027/2026-10-22/`：转移来的论文渲染页面和编译日志。
- `validation/2026-10-22/`：转移来的测试 XML 与临时测试产品，保留原始子目录。
- `validation/2026-10-23/`：本次重建的测试、安装与执行日志。
- `migrations/2026-10-23/local-tracked-before.zip`：改动前本项目被 Git 跟踪文件的原始字节。
- `migrations/2026-10-23/development-568b1fc.zip`：从转移 Git 导出的完整开发提交。
- `migrations/2026-10-23/transfer-originals.zip`：冲突版本、原 `.git`、旧 `.venv`、缓存与原配置。仅用于历史恢复。

除既有公开图表外，这些大体积产品由 Git 忽略，保存在本地完整项目内。哈希、来源与每个文件的去向记录于 [迁移目录](../docs/migrations/2026-10-23/README.md)。
