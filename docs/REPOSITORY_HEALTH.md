# 仓库同步与健康检查

## 2026-10-23 同步范围

同步前 GitHub 只有两个活动分支：`main` 为 `c3da546`，`paper/boundary-aware-safe-deployment` 为 `568b1fc`。本地开发分支仍为 `e4b0a19`，但工作区已恢复 `568b1fc` 的代码，并完成本地数据整合和 `.venv` 重建。

按“以最新进展为准、更新各分支”的要求，两个活动分支统一到本次重建后的最新版本。保留原 main 与开发分支的提交历史，使用普通快进推送；不强制覆盖历史。

四个旧本地分支已停止作为活动分支：`DBACT-publication-v3`、`DBACT-research-v3`、`feat/sampled-theorem-mode`、`research/simulation-evidence-closure`。它们各自保存在 `archive/2026-10-23/` 前缀的 Git 标签下，斜杠改为连字符；完整同步前 Git bundle 另存于本地 `artifacts/migrations/2026-10-23/repository-before-sync.bundle`。两个指向已不存在路径的工作树登记已移除。历史实验与原始数据不删除。

清理了已失效的一次性文献补丁脚本 `update_builder.py` 与 `update_visual_checks.py`；其原字节仍在本地迁移归档中。活动依赖入口统一使用根 `.venv`，原 Conda 清单已经从项目根移除。

## 可重复执行的检查

从根目录使用当前项目环境：

```powershell
.\.venv\Scripts\python.exe scripts/check_repository_health.py --output runs/health/current.json
.\.venv\Scripts\python.exe -m pytest -q
```

`check_repository_health.py` 检查 Git 对象完整性、被跟踪文件存在性、Python 语法、46 个冻结证据文件的原始 SHA-256、主要导航链接、依赖一致性、误跟踪的本地环境/大数据、大文件以及明显私钥/token 模式。它可以在全新克隆中执行，不要求忽略的本地数据存在；简单凭据模式筛查不等于完整安全审计或依赖漏洞审计。

GitHub Actions 在 Ubuntu 和 Windows 中分别创建根 `.venv`，安装同一锁文件，执行仓库健康检查、主项目测试、MAS 无硬件测试、协议 dry-run 与五案例双 worker smoke。每次运行保留健康 JSON、测试 XML 与 smoke 摘要，见 [Actions](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/actions/workflows/tests.yml)。通过状态应以相应分支最终提交的实际 Actions 结果为准。

## 一致性的定义

- 两个活动分支应指向同一最终提交；本地对应分支与 GitHub 的提交 SHA、Git tree、路径及 blob 身份一致。
- 本地 Git 工作区应无未提交的版本管理改动；被忽略的 `.venv`、原始实验、参考全文与备份继续保留在本地。
- GitHub 克隆包含代码、精简证据与数据清单；[数据索引](DATA_INDEX.md)明确说明本地完整数据与 Git 内容的边界。不将 Git 一致说成全部本地文件已上传。
- [重建记录](REBUILD_2026-10-23.md)与[迁移保全收据](migrations/2026-10-23/preservation_verification.json)记录原始数据身份及不可读历史缓存的访问边界。

本次推送后的精确分支 SHA、远程镜像完整性检查、CI run URL 与最终一致性结果保存在本地 `runs/repository_sync_20261023/`，用于避免把包含自身提交号的验证报告反复提交造成循环。
