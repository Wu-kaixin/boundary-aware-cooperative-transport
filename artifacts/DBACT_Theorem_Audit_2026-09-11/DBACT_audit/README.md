# DBACT proof audit: reproduction

本包对应 2026-09-11 对提交证明和远程代码的核查。

主要文件：

- `DBACT_教授反馈核查与后续方案_2026-09-11.md`：中文报告、推导、文献映射和后续验收标准。
- `audit_theorem.py`：只读诊断脚本，调用原仓库控制器，并用独立观察器计算参考残差。
- `remote_branch_manifest.json`：全部现存远程分支 SHA、文件树和核心 blob 身份。
- `results/constants.json`：理论常数、静态表达式及几何上界。
- `results/counterexamples.json`：实际网格不连续和执行裁剪反例。
- `results/theorem_vs_observed.png`：三个种子残差瞬时值、累计时间平均与界的尺度比较。
- `results/seed_*/residuals.csv`：每 0.25 s 的参考观测量。
- `results/seed_*/trajectory.npz`：每个控制时刻的机器人位置、真实物体顶点及最终命令。
- `results/seed_*/safety_diagnostics.json`：每个控制步的半约束残差、持有段间距、裁剪和饱和记录。
- `results/seed_*/observer_refinement.json`：四个预定快照的积分加密比较。
- `results/seed_*/simulation_summary.json`：原仓库生成的仿真总结。
- `results/provenance.json`：源代码、配置和诊断脚本身份。

## 复算方法

要求 Python 3.10 或更新版本，以及原仓库 pyproject.toml 指定的依赖。默认 planar QP 不需要 cvxpy。

在本包解压目录执行，使用独立新克隆：

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git dbact-proof-audit-source
git -C dbact-proof-audit-source checkout 98ba28e835a8c45fa8375b2a5ae6130c17e547ad
python -m pip install -e ./dbact-proof-audit-source
python audit_theorem.py --repo ./dbact-proof-audit-source --out ./reproduced_results --frames 600 --seeds 2 5 8 --observer-stride 5
```

为限制线性代数线程和提高可复现性，Linux 可在运行前设置 `OPENBLAS_NUM_THREADS=1` 和 `OMP_NUM_THREADS=1`。Windows PowerShell 对应为：

```powershell
$env:OPENBLAS_NUM_THREADS="1"
$env:OMP_NUM_THREADS="1"
```

脚本不会编辑仓库文件或参数，但会开启已有的只读 trace 功能并包装命令执行方法，以观察其前后状态。

## 解读限制

1. 这是原始 sampled/clipped 控制器的诊断，不是对 Theorem 1 全部假设的认证。
2. `static bound` 是将运动项设为零的渐近表达式，用于判断粗界的尺度；实际动态轨迹的正式界尚未获得。
3. 瞬时残差和时间平均来自独立数值积分。快照加密比较不是统一误差证书。
4. 只跑了同一 L 形场景的三个种子；没有把这些数据写成已完成多形状统计实验。
5. `local_cvt_centroid2_partial` 只覆盖该时刻确实计算了 CVT centroid 的机器人，由 `local_cvt_agents` 给出数量；不能拿它与完整 N 机器人参考残差直接比较。
6. `disturbance2` 是观察时刻的实际最终速度与参考未饱和 CVT 速度之差的平方范数；它不是全局扰动上界。最后时刻没有额外计算控制命令，该列记 NaN。
7. 每个 trajectory 的最后一个位置样本没有对应的下一段速度，故 positions/vertices 有 601 帧、velocities 有 600 帧。
8. 修改绘图时可直接读取已保存 CSV，无需重新运行仿真。

不包含用户论文或参考文献全文，也不包含远程 Git 仓库副本。所有新增复算代码和结果保存在本包中。
