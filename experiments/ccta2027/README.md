# CCTA 2027 实验入口

本页对应 `main`。依赖安装见[项目首页](../../README.md)。
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

## 完整矩阵在哪里

本分支包含短协议和历史协议读取支持，尚未包含 `campaign`、`revision.yaml`、
169 案例结果和最终论文图表模块。完整实验请使用[开发分支](https://github.com/Wu-kaixin/boundary-aware-cooperative-transport/tree/paper/boundary-aware-safe-deployment/experiments/ccta2027)。
不要在此基线中执行仅存在于开发分支的命令。

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
