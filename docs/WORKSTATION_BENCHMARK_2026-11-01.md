# DBACT 工作站工程基准（2026-11-01）

任务身份：`2026-11-01-workstation-benchmark-v1`。本记录对应本机实际执行，所有复跑都是工程验证，不增加科研样本分母。完整本地证据在 `artifacts/migrations/2026-11-01-workstation-benchmark-v1/`，原始测量在 `runs/maintenance/2026-11-01-workstation-benchmark-v1/`；这两个目录按仓库规则不加入 Git。

## 已完成的并发比较

当前推荐 **8 个案例进程，每进程 BLAS/OpenMP/CVT 内部线程为 1**。4 进程适合同时进行其他计算。12、16、24、32、48 进程本轮未测，8 不是经验证的全局最优值。

相同的 8 个完整任务、同一 seed=2、相同初态及源码，按 1→4→8 顺序各执行一次。它们包含圆、两种椭圆方向、两种矩形方向、两种 L 形姿态和 C 形开口朝向目标。保持 3000 帧上限及原收定规则，实际自然终止于 268–859 帧；这不是人为截短的 smoke。长达 3000 帧的 C 背向失败单独用于 profile，不混入下表吞吐分母。

| workers | 8 案例总耗时，秒 | 案例/小时 | 相对本轮单进程 | 单例最小/中位/最大，秒 | 峰值采样进程树 RSS，GiB | 执行异常 |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 360.60 | 79.87 | 1.00× | 24.41 / 40.20 / 91.99 | 0.311 | 0/8 |
| 4 | 96.97 | 297.00 | 3.72× | 19.36 / 29.55 / 60.90 | 0.613 | 0/8 |
| 8 | 69.31 | 415.54 | 5.20× | 18.70 / 28.58 / 65.34 | 0.968 | 0/8 |

总耗时包含启动、实际计算、原始日志和回放写入、收据生成、池退出，另加最多约 1 秒的轮询分辨率；跨档科学比对不计入单档耗时，但计入全任务预算。8 相对 4 的吞吐提高约 39.9%。三档实际工作进程的两个 OpenBLAS 库均确认单线程。

这些是一次顺序测量，没有重复试验或置信区间。单进程累计 worker CPU 时间 351.63 秒，4/8 进程分别 254.42/252.09 秒，说明相同科学工作量的单核执行速度也发生变化。平衡电源方案、频率、缓存和后台活动可能影响结果，不能把 5.20× 全部归因于纯并行，也不能把 415.54 案例/小时用于每例都跑满 3000 帧的负载。

## 完整任务热点与离线媒体

C 背向案例完整执行 3000 帧，profile 档端到端 483.64 秒，剖析函数计时合计约 478.64 秒。该回合仍为未收定、任务未完成、G500 未通过，且与历史逐条一致。下表为累计函数时间，有嵌套关系，**不能相加为总耗时**；cProfile 本身也会扰动时间，此比例不能直接推广到所有形状和阶段。

| 环节 | 累计剖析秒 | 解释 |
|---|---:|---|
| 感知 `sense_view` | 48.31 | 48,000 次传感器调用 |
| 边界配准 `register` | 76.76 | 包含匹配；`_match_rows` 本身累计 31.96 秒 |
| 地图更新 `update` / `carve` | 69.37 / 9.76 | 融合、去重与地图处理 |
| CVT `compute` | 78.29 | 其中密度 `_eval_offset` 累计 55.32 秒 |
| 安全过滤 `filter_velocity` | 38.74 | 包含约束构造和求解 |
| 上行内部的二维 QP | 17.45 | 48,000 次；约占本例剖析总量 3.65% |
| 接触物理 `step` | 12.83 | 3000 控制帧、原 4 子步规则 |
| 状态记录 `_record` | 18.42 | 包括原有评价检查 |
| 独立评价 observer | 1.08 | 原记录范围完整保留 |
| 诊断规范化与 JSON 序列化 `dump` | 35.40 | Python 序列化成本，不等于磁盘等待 |
| 保存环境输出 `save_outputs` | 1.17 | replay 与其他原始产物 |

最值得优先检查的下一项工程优化是在**一次 CVT 调用内**复用相同 query/targets 的距离与高斯核矩阵。当前同一核分别用于总密度和 gap 权重；任何实现必须保留两次原矩阵向量乘法的顺序、全部科学字段与 FP64 精度，并先做固定输入及整轨迹等价检查。尚未实施，也不承诺加速比例。地图的 `_cell_keys`/`unique` 也是热点，但不能为提速改变观测去重、顺序或边界质量定义。

按本例 QP 占比粗算，即使将求解时间降为零，整个被剖析任务也仅约 1.04×；这只是受 profile 扰动的局部 Amdahl 估计，足以说明本轮没有把 QP 搬上 GPU 的优先依据。当前 CPU 案例并行已经有实测收益，CUDA/GPU 仿真迁移收益仍未测。

用冻结的圆形失败轨迹离线生成 55 帧（帧索引 0,5,…,270），960×640，包含最终状态。原仿真 20 Hz，每 5 帧抽取一次，视频 10 fps，约 2.5 倍物理时间播放；这是编码工程样片，不是成功案例展示或完整论文 HUD 视频。

| 媒体操作 | 实测 |
|---|---:|
| 55 帧 Matplotlib Agg 渲染及 RGB 写入 | 4.20 秒，13.09 帧/秒 |
| 最终矢量 PDF 输出 | 0.306 秒 |
| libx264 veryfast / CRF20 / 1 线程 | 0.434 秒，122,619 字节 |
| 相同帧及编码设置 / 4 线程 | 0.331 秒，122,619 字节 |

两份视频完整解码通过，55 帧解码 YUV420P 哈希精确相同；对相同 RGB 转换参考的平均 PSNR 均 52.52 dB。已查看原始与编码后的最终帧，图形、坐标和图例可读。编码差异只有约 0.1 秒，本例主要媒体成本是渲染，不能声称 GPU 编码会明显改善整个流程。FFmpeg 列出 NVENC 编码器，但设备实际编码 **NOT_RUN**，不把编译支持写成硬件实测。

## 环境与可测边界

| 项目 | 本机核验 |
|---|---|
| 主机/系统 | Supermicro Super Server，Windows 11 Pro，10.0.26300 |
| CPU | Xeon Silver 4516Y+，24 物理核、48 逻辑处理器；进程亲和性允许全部 48 个 |
| RAM | 4×32 GiB，操作系统可见约 127.71 GiB；并发阶梯最低可用 98.44 GiB |
| GPU | NVIDIA RTX A400，4094 MiB，驱动 552.86；不是 A4000/A100 |
| 输出磁盘 | 三星 MZ1L2960HCJR-00A07，NVMe SSD；C 卷约 892.14 GiB，开始时可用约 641.08 GiB |
| Python | 根 `.venv`，Python 3.12.14；两个包都从本仓库 `src/` 导入 |
| 依赖 | 锁文件 41 个固定版本全部匹配；NumPy 2.5.4、SciPy 1.18.1、threadpoolctl 3.7.0 |
| BLAS | NumPy OpenBLAS 0.3.34.106.0；SciPy OpenBLAS 0.3.31.dev；环境未限制时两者默认 24 线程 |
| QP | 当前配置 `qp` 使用项目二维枚举投影；cvxpy 1.9.3 / OSQP 1.1.3 / Clarabel 0.11.1 / SCS 3.3.1 是已安装版本，不是本轮实际求解后端 |
| 编码器 | 根环境 imageio-ffmpeg 0.6.0 所带 FFmpeg 7.1；软件 libx264 |

本轮没有修改依赖、控制器、物理、成功判据、冻结协议或历史结果。既有 `ResourceSampler` 因 `psutil` 未安装而无法提供 CPU/RSS；新工程采集器使用 Windows 原生 API，按本次启动的进程树记录，不为此新增依赖。

每秒采样进程树 RSS、各进程 CPU/IO、可用 RAM、主机 CPU、磁盘延迟/队列和主机聚合频率。进程树 RSS 会重复计入共享页，1 秒采样可能漏掉瞬时峰值；另保留 worker 的操作系统生命周期峰值。PDH 磁盘计数是整机数据，**不是进程独占 I/O wait**。CPU 温度没有可靠测量；主机聚合频率不能当作繁忙核心的精确频率。GPU 温度仅有现场快照。

三档磁盘读延迟 p95 分别为 0.322/0.291/0.372 ms，写延迟 p95 为 0.167/0.510/1.047 ms；队列 p95 都为 0。未观察到持续磁盘排队，不能因此排除所有瞬时 IO 开销。64 MiB 顺序写入加 fsync 用时 0.150 秒，缓存读取 0.051 秒；这是文件系统小探针，不能宣称为设备峰值带宽。

## 冻结、数据和恢复规则

运行前冻结源码清单、完整 resolved case、历史产品 SHA-256、工程脚本、线程设置和预算。上限 25 次原任务工程复跑、1800 秒总预算；分档上限为 600/300/300 秒，profile 900 秒，离线媒体 180 秒，受总预算约束。没有增加种子、放宽容差或追跑到成功。

科学一致性采用回放数组精确相等（含 NaN 位置），以及完整 `paired_metrics.json`、`physics_substeps.json`、`summary.json`、`config.json` 的解析值精确相等。仅工程耗时、路径身份等外部收据元数据不作为科学值比较。w1 对历史冻结结果，w4/w8 对 w1，全部通过。三档各保留 3 个任务未完成案例；程序执行完成与科研任务完成分别记录。

原始科学日志与失败不删除。批次不绘图、不逐帧输出 PNG；保留 replay、逐帧指标和物理子步，离线渲染。运行器是 Windows spawn 可序列化顶层 worker，每个案例独立目录；同一案例跨并发档位使用相同随机输入。

本轮最终完成 25/25 工程复跑、625 个回放数组配对比较及完整逐帧/子步记录比较，总基准耗时 1038.06 秒（17.30 分钟），未超 30 分钟预算。原 9 个案例的 72 个产品哈希、原收据和源码快照复核通过。同身份 resume 复用了全部 8 个 w8 案例，没有新增仿真；只更新父目录 manifest/summary，更新前版本已备份。未带 resume 的覆盖尝试按预期退出 1，并核实目录所有文件字节不变。

协议与恢复相关测试 24 项通过；`pip check`、`git diff --check` 通过。没有把这些针对性验证报告为全仓库测试重跑。

已完成案例可凭相同身份和产品哈希复用；中断在案例中途的目录不能被冒充完整检查点，原运行器会保留并拒绝它。GitHub 代码仓库不包含这些原始输出，不能当作原始数据备份。源码快照、协议和同盘恢复材料只保障本地误操作恢复。

## 可执行入口

从仓库根目录调用根解释器。本轮已存在的身份只检查或复用，不能再次运行 `freeze`/`run` 覆盖计时试验：

```powershell
$repo = (Get-Location).Path
$py = Join-Path $repo '.venv\Scripts\python.exe'
& $py scripts/benchmark_workstation.py check
& $py scripts/analyze_workstation_benchmark.py
```

后续使用**另行冻结的新协议和独立输出目录**。以下函数使用现有 CLI，默认只校验；显式 `-Execute` 才运行。`-Resume` 只复用同身份的完整案例，不执行任意中途轨迹恢复。

```powershell
function Invoke-DbactFrozenStudy {
    param(
        [Parameter(Mandatory)][string]$Protocol,
        [Parameter(Mandatory)][string]$Split,
        [Parameter(Mandatory)][string]$Out,
        [switch]$Execute,
        [switch]$Resume
    )
    $repoPath = (Resolve-Path -LiteralPath '.').Path
    $pythonPath = Join-Path $repoPath '.venv\Scripts\python.exe'
    $outputPath = [System.IO.Path]::GetFullPath($Out, $repoPath)
    $allowedRoot = [System.IO.Path]::GetFullPath((Join-Path $repoPath 'runs')) + [System.IO.Path]::DirectorySeparatorChar
    if (-not $outputPath.StartsWith($allowedRoot, [StringComparison]::OrdinalIgnoreCase)) { throw 'Output must be a dedicated directory under this repository/runs' }
    $inspectPath = $outputPath
    while ($inspectPath -and $inspectPath -ne $repoPath) {
        if (Test-Path -LiteralPath $inspectPath) {
            if ((Get-Item -LiteralPath $inspectPath -Force).Attributes -band [System.IO.FileAttributes]::ReparsePoint) { throw 'Output path must not traverse a link or junction' }
        }
        $inspectPath = Split-Path -Parent $inspectPath
    }
    foreach ($name in 'OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS','DBACT_CVT_WORKERS') {
        [Environment]::SetEnvironmentVariable($name, '1', 'Process')
    }
    $env:MPLBACKEND = 'Agg'
    $studyArgs = @('-m','dbact_sim','study','--protocol',$Protocol,'--split',$Split,'--workers','8','--out',$outputPath)
    if (-not $Execute) { $studyArgs += '--dry-run' }
    elseif ($Resume) { $studyArgs += '--resume' }
    & $pythonPath @studyArgs
    if ($LASTEXITCODE -ne 0) { throw "study exited with code $LASTEXITCODE; preserve all outputs" }
}
```

该命令块是后续调度入口，不自动运行新科研矩阵。已在 PowerShell 7.6.5 验证其默认 dry-run。优先沿用上述 8×1 配置，为交互、分析和操作系统保留资源。本轮监控脚本采用并发上限 8、进程树 RSS 守卫 32 GiB、可用内存下限 16 GiB、可用盘下限 20 GiB；上述原生 CLI 命令本身不附带监控。后续正式调度应保留同样监控。工程保护线不是对不同规模场景内存需求的数学保证。更高并发应另立冻结基准，至少有足够案例填满对应档位。

## 正式批次成本估计

以未来同规模 27 案例为例，不自动重跑已完成的科研普查。读取已有 27 案例逐例耗时，用 8 worker FIFO 模型得到 411.46 秒；再乘本轮匹配案例最大的单进程/历史耗时比 1.424 和工程余量 1.5，约 **14.65 分钟**。

如果 27 例全部跑满 3000 帧，以已有未 profile 任务的最大每帧成本 0.1071 秒、4 波次和 1.5 余量估计，约 **32.13 分钟**。可暂按 **15–35 分钟纯仿真**安排类似负载；这是规划包络，不是实测 27 例八进程时间、置信区间或保证上界。新机制、机器人数量或日志密度变化时要重新估计。

原始输出最大观测密度约 44,394 字节/帧，27×3000 帧再乘 1.5 余量约 5.02 GiB，可预留 **6 GiB 原始输出**；若另保留完整副本则相应翻倍，媒体中间文件另计。新批次不应为了节省容量删除失败、截断诊断或改变冻结记录范围。

未测项：12+ 并发、不同内部线程数对照、CPU 温度、GPU 仿真和 NVENC 实际设备编码；没有用虚构收益补齐它们。所有原始数据、日志、执行命令、退出码、profile、编码收据、备份和分析脚本身份保存在本轮本地审计区。
