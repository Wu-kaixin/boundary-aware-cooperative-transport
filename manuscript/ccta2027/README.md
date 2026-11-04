# CCTA2027 英文研究初稿

**Boundary Deployment and Contact Feasibility in Cooperative Transport without Prior Object Geometry**

[8 页主稿 PDF](main.pdf) · [LaTeX](main.tex) · [补充材料 PDF](supplementary.pdf) · [结果与复现索引](RESULTS_INDEX.md) · [研究总结](RESEARCH_SUMMARY.md) · [媒体与主张](MEDIA_AND_CLAIMS.md) · [文献核验](sources.md)

2026-11-04 更新。当前交付是完整研究初稿，作者信息待填写。论文主线是局部边界部署与实际接触、搬运和停止之间的差距，以及约束允许的运动方向与 QP 选择方向之间的区别。

## 本稿成立的结论

- 历史九条件普查中，CVT、spacing、nearest 的空间完成数分别为 5/9、5/9、2/9；圆形 CVT 通过 G500 仍有 1.177 m 二维误差。
- 后续 12 条件匹配比较中，CVT 的共同部署区间覆盖比 spacing/nearest 高 1.52/4.85 个百分点，三方法空间完成均为 11/12。覆盖收益没有证明完成优势。
- 有限支持候选在四个对照均完成的条件中全部失败；固定状态证书表明，原 17 个正向力状态中有 11 个被新截断约束排除正向速度能力。另有正向可行却由 QP 选择反向的状态。它们不是完整轨迹因果证明。
- 本次新种子、当前源码的 60 回合验证全部执行结束：58 个空间完成，56 个同时未触发原硬安全失败。8 机器人 C 形出现未收定及停错位置；24 机器人出现两例穿透预算超限。所有失败保留。

各批源码、参数、条件与统计单位分开记录，不能合并成同一方法的总体成功率。未建立新算法总体优越性、完全去中心化闭环、连续动态安全或任意形状鲁棒性。局部多边形准入仍是条件设计；失败的 anchored、有限支持及射线方向候选未被重新推广。

## 实验、视频与复现

完整入口见 [RESULTS_INDEX.md](RESULTS_INDEX.md)。新增 [60 回合结果表](../../docs/results/resource_noise_validation_v1/all_60_episodes.csv)、[配对差值](../../docs/results/resource_noise_validation_v1/paired_differences.csv) 与 [报告](../../docs/results/resource_noise_validation_v1/README.md)均可追溯到冻结协议和原始收据。

当前五形状总览、三方法比较和 GIF 在 `runs/research/2026-11-04-resource-noise-validation-v1/media_v1/`。额外的 C 形 8 机器人失败比较在同级 `diagnostics_v1/`，保留全部 600 s 实际过程并明确标注事后选例。所有 MP4 已完整解码并核对帧数；结束后的案例显示实际末帧。地图是 agent 00 的融合观测，不伪造未记录的原始激光射线。

本次使用根 `.venv`；核心回归 1113 passed、2 skipped，MAS 回归 106 passed。原始临时目录权限失败日志保留，受控目录重跑成功。源码与保护 YAML 未改变。环境实际清单已冻结；joblib、PyYAML 与锁文件的两个既有差异仍在，不能用锁文件冒充本次运行环境。

固定工程前缀的 1/4/8 进程耗时为 92.374/34.126/22.108 s，所有科学输出精确一致，因此采用 8 进程。60 回合执行耗时 779.104 s。工程前缀和 profiling 不计入科学分母。Matplotlib 绘图、FFmpeg 编码/解码、Inkscape 1.4.4 矢量导出均实际执行；没有宣称 GPU 加速仿真。

## 格式与编译

2026-11-04 核对 [CCTA2027 官方 CFP](https://ccta2027.ieeecss.org/authors/call-for-papers)：IEEE 双栏，初投最多 8 页；录用稿以 6 页为基准，超页另有费用。主稿使用 [CSS PaperPlaza 原始模板](https://css.paperplaza.net/conferences/support/tex.php)，Letter、10pt、conference，模板来源与哈希在 [template/](template/)。补充 PDF 为本地阅读材料，不声称已满足会议附件要求。

从仓库根目录编译：

```powershell
$env:TECTONIC_CACHE_DIR = Join-Path (Get-Location).Path 'artifacts/research/2026-10-31-readiness-v1/tex-cache'
& tools/tectonic/tectonic.exe --only-cached --keep-logs manuscript/ccta2027/main.tex
& tools/tectonic/tectonic.exe --only-cached --keep-logs manuscript/ccta2027/supplementary.tex
```

编译、字体、引用、逐页检查和最终文件哈希见 [FINAL_AUDIT.md](FINAL_AUDIT.md)。历史稿件已备份于 `runs/research/2026-11-04-ccta2027-draft-refresh-v1/manuscript_before/`。没有提交、推送或正式投稿。

本稿使用 AI 辅助审查、执行、分析、绘图和英文写作；作者须自行核阅科研主张及署名。工作流参考 Kassis 等的 [Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents](https://arxiv.org/abs/2609.00065v2)（2026）。该条仅说明软件工作流来源，不是机器人学结论的证据。
