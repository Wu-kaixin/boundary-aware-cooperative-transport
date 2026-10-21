# CCTA 2027 实验入口

本页对应 `paper/boundary-aware-safe-deployment`。依赖安装见[项目首页](../../README.md)。
[conference.yaml](conference.yaml) 保存会议要求及上次核对日期；投稿前仍需核对官方页面。

## 快速验证管线

从仓库根目录运行，输出目录首次使用时应不存在：

```sh
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --dry-run
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/ccta2027/smoke_parallel
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 1 --out runs/ccta2027/smoke_serial
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/ccta2027/smoke_parallel --resume
python -m dbact_sim evidence --batch smoke=runs/ccta2027/smoke_parallel --out runs/ccta2027/smoke_evidence
```

五例覆盖 oracle/local 静态执行和 CVT/最近目标/局部间距搬运。
4 或 12 帧只用于检查程序，不评价收敛、任务完成或优越性。

## 完整 CCTA 矩阵

[revision.yaml](revision.yaml) 定义 169 案例：静态 27、开发 27、消融 15、主动探索诊断 6、
新种子泛化 36、扰动鲁棒性 54、数值参考 2、接触引擎参考 2。
当前结果见[实验报告](../../docs/CCTA2027_EXPERIMENTS.md)与
[归档包](../../docs/results/ccta2027_draft_v2/README.md)。审计 PASS 只说明约定的完整性和实现检查通过。

### 冻结与运行新修订

以下为 Windows PowerShell，从仓库根目录运行。选择尚不存在的目录：

```powershell
python -m dbact_sim campaign freeze --plan experiments/ccta2027/revision.yaml --out runs/ccta2027/next_revision/frozen_protocol.yaml
$campaignRoot = (Resolve-Path runs/ccta2027/next_revision).Path
$env:PYTHONPATH = Join-Path $campaignRoot 'execution_source/src'
$env:DBACT_ROOT = Join-Path $campaignRoot 'execution_source'
python -m dbact_sim study --protocol "$campaignRoot/frozen_protocol.yaml" --split all --workers 4 --out "$campaignRoot/run"
python -m dbact_sim campaign audit --protocol "$campaignRoot/frozen_protocol.yaml" --run "$campaignRoot/run" --out "$campaignRoot/acceptance.json"
```

冻结后必须从 `execution_source` 执行，协议与输出传绝对路径，避免工作目录中的修改混入。
工人数按机器资源调整；原 v2 使用 16 工人，另有两例串行复核。

分析在另一个 PowerShell 会话中从仓库根目录运行，使用当前分析代码：

```powershell
python -m dbact_sim evidence --batch revision=runs/ccta2027/next_revision/run --by-split --out runs/ccta2027/next_revision/evidence
python -m dbact_sim.experiments.report --run runs/ccta2027/next_revision/run --out runs/ccta2027/next_revision/figures
```

复现已经发布的 v2 时，应使用归档包内的原始模型源码及独立的 `analysis_source`，
而不是用当前 checkout 给历史冻结重新归属。无需运行物理也可从精简结果重建统计和四组论文图；
具体步骤见归档包 README。完整接触时序图还需要本地原始诊断。

## 输出和恢复

```text
manifest.json               协议、源码、环境与 Git 身份
resolved_protocol.json      展开的案例配置
source_snapshot/            运行时源码快照
resources.csv              资源采样
<case_id>/resolved_case.json 精确输入
<case_id>/case_result.json   结果、身份及产品哈希
<case_id>/...               回放与诊断
summary.json                全部已记录案例、分母和执行状态
```

`--resume` 只恢复身份和所有产品一致的同一次实验。
变更源码、协议或环境后必须另建目录。执行或进程错误保留记录并返回非零；
测量到的任务失败保留在统计中，不能作为坏数据删除。

## 历史协议

```sh
python -m dbact_sim study --protocol docs/paper_simulation_protocol.test_frozen_v1.yaml --split test --dry-run
```

此命令只解析历史 36 例。真正执行必须使用对应冻结源码和环境；
当前重构源码不能冒充旧源码，也不能把原测试集重新标为新的泛化验证。
历史定义见 [paper_simulation_protocol.md](../../docs/paper_simulation_protocol.md)。
