# CCTA2027 初稿：未知物体的边界部署与完整搬运

工作标题：**Boundary-Aware Deployment for Cooperative Transport of Unknown Objects**。

[LaTeX 源码](main.tex) · [6 页 PDF](main.pdf) · [20 篇相关文献及阅读深度](sources.md) · [新图表、英文图注与主张证据表](MEDIA_AND_CLAIMS.md) · [完整结果与能力表](../../docs/results/multishape_census_v1/README.md)

本稿是研究初稿，作者信息待填写，不是已投稿或已建立新算法优越性的论文。局部部署算法和共享阶段监督器的信息权限已明确区分，不宣称整个闭环完全去中心化。

## 当前主线与实际结果

九个形状—朝向条件覆盖 circle64、ellipse24、矩形、L、C。三种现有部署方法只改变首次 CONTACT_READY 前目标；共同 seed 2、初态、感知/控制/物理、安全与停止判据，固定27回合。所有失败保留，无调参、换种子或延长预算。

| 方法 | 二维完成 | G500 | 收定 |
|---|---:|---:|---:|
| boundary_cvt | 5/9 | 5/9 | 8/9 |
| nearest_observed_target | 2/9 | 1/9 | 7/9 |
| local_spacing | 5/9 | 3/9 | 7/9 |

这是一种子开发普查，不是泛化成功率。圆和两种椭圆朝向均没有二维完成；C开口朝向目标时三方法完成，背向时三方法均watchdog失败。CVT没有优于局部间距的完成数。圆形CVT通过G500却有1.177 m二维终点误差；投影到达、质量告警与空间完成不能混用。

[五形状完整阶段图](../../docs/results/multishape_census_v1/five_shape_stages.png)、
[全部27回合表](../../docs/results/multishape_census_v1/capability_table.md)、
[10段完整过程/同步视频索引](../../docs/results/multishape_census_v1/MEDIA.md)
都来自真实回放。2026-11-01新版正文主图为完整过程、全方法完成/误差/时间/路径比较、时间对齐机制图。五形状事件全图、安全资源及v2负结果图放补充。旧冻结媒体保持原样；新交付见 [媒体与论证说明](MEDIA_AND_CLAIMS.md)。

## 保留的诊断与数学边界

- [v1](../../docs/results/readiness_diagnostic_v1/README.md)：仅进度oracle在两个选定失败中有效，但令成功C8误差增加；不是可部署算法。
- [v2](../../docs/results/progress_local_v2/README.md)：实际配准绝大多数可辨识；证书未改变轨迹；anchored候选没有救回失败且C8更差，停止扩展。
- 本轮只从已有v2日志作带符号累计误差分析，保留原始累计、固定增益、保持值、候选项及中位数权重变化。恒等式不等于物理准确性，分量不是唯一因果贡献。
- 静态信息模型、采样安全检查和数值通过都不扩展为动态接触或鲁棒搬运保证。

## 生成关系与复现

完整数据在本地 runs/research/2026-10-31-multishape-census-v1/。

analysis/analysis.json 是表格/图的共同派生结果。运行 scripts/analyze_multishape_census.py 会生成 CSV、图及 census_numbers.tex、census_rows.tex、census_table.tex；它们与稿件中的数值输入已逐字核对。generated/census_interpretation.tex 是依据同一表撰写的结果解释。generated/cumulative_interpretation.tex 的数字来自 v2-offline-final/signed_error_analysis.json。旧诊断图和表继续保留，不与当前27回合合并计数。

分析和渲染读取现有数据，不重跑科研实验。应选择独立输出目录；视频入口支持经过回放哈希和完整解码帧数验证的 --resume-videos。完整源码和结果身份见[公开索引](../../docs/results/multishape_census_v1/identity.json)。

## 官方格式与编译

2026-11-01 再次核对 [CCTA2027 官方 CFP](https://ccta2027.ieeecss.org/authors/call-for-papers)：标准IEEE双栏，初投至多8页，录用稿6页基准。采用 [CSS PaperPlaza 官方 LaTeX 模板](https://css.paperplaza.net/conferences/support/tex.php) 的原始 ieeeconf.cls，Letter、10pt、conference及官方margin指令；模板来源保存在 [template/](template/)。

从仓库根目录使用已有本地Tectonic缓存编译：

~~~powershell
$env:TECTONIC_CACHE_DIR = Join-Path (Get-Location).Path 'artifacts/research/2026-10-31-readiness-v1/tex-cache'
& tools/tectonic/tectonic.exe --only-cached --keep-logs manuscript/ccta2027/main.tex
if ($LASTEXITCODE -ne 0) { throw 'LaTeX build failed.' }
~~~

当前验证：6页Letter；20篇引文全部解析；18个字体全部嵌入、无Type3；20项引用全部解析，无overfull；3个underfull为间距提示，最终六页已逐页目视核验。没有上传稿件，也没有编造作者/单位、导师意见或投稿状态。

本次恢复与验证记录：artifacts/research/2026-11-01-paper-media-v1/。旧稿与用户原有修改逐文件备份；本次没有新增物理回合或远端提交。

新媒体包入口：`artifacts/research/2026-11-01-paper-media-v1/INDEX.md`。生成脚本为 `scripts/build_ccta_media.py`，大视频与详细机器清单留在本地，不自动加入Git。官方CFP尚未给出普通论文视频格式/时长/大小细则，本地编码成功不等于投稿附件合规。
