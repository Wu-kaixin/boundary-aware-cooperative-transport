# 文献、模板及科研主张核验（2026-10-31）

对应当前 `main.tex` 和 20 条 `references.tex`。引用数量不是创新性证据。
本轮检索优先论文全文、作者/机构页和出版社页面。**没有宣称逐页精读全部 20 篇。**
A：本轮检查一手全文相关段落；B：一手题录/摘要与仓库既有全文阅读记录复核；C：一手题录/摘要核验及背景引用。
既有全文与提取物保留在 `data/literature/`，不加入 Git；既有笔记在 `docs/literature_review_2026-10-22/`。

## 最接近的八篇及反例审查

| 引用键 | 深度 | 已有结果/条件 | 对当前稿件的限制 |
|---|---|---|---|
| song2026 | A | 自适应边界密度、CVT 自分配、CBF 协调；直接相关框架 | 不把继承框架包装为新贡献，几何配置不自动给动态成功 |
| vardharajan2021 | A | 未知形状 sequential caging、有效推手与运输；通信及动力学假设 | 不能称既有方法只有围捕、不会操作 |
| kobayashi2012 | B | 局部观测、接触构型调整、平移闭合；对象/邻居假设，力矩范围受限 | 不能声称首次连接局部观测与操作能力 |
| tang2024 | B | 多机器人推动、力可行性接触重配置；已知对象与集中感知条件 | 不能声称首次考虑接触重配置或推动可行性 |
| yan2021 | B | 任务相关能力、异构执行器、固定抓取与参考调整 | 方向相关能力不是新概念，单边接触问题有不同条件 |
| censi2007 | A | ICP 不可观方向、可观子流形上的误差；局部条件 | 秩判定和伪逆不构成新理论；相关观测不能假装独立 |
| bonnabel2016 | A | ICP 重匹配与协方差，point-to-plane 的局部讨论 | 固定对应公式不能直接等同实际 ICP 协方差 |
| gelfand2003 | A（摘要/方法相关部分） | 特征方向稳定性与补充弱方向的采样 | 追加法向改善信息几何并非新的采样思想 |

## 全部引用的一手入口

| Key | 题目 / 正式题录要点 | 一手入口 | 深度 |
|---|---|---|---|
| song2026 | Cooperative transportation without prior object knowledge via adaptive self-allocation and coordination; arXiv:2602.19070 | [全文](https://arxiv.org/html/2602.19070v2) | A |
| vardharajan2021 | Collective transport via sequential caging; arXiv:2106.03132 | [原文](https://arxiv.org/abs/2106.03132) | A |
| kobayashi2012 | Cooperative enclosing and grasping of an object by decentralized mobile robots using local observation; IJSR 4:19–32 | [出版社](https://doi.org/10.1007/s12369-011-0118-7) | B |
| tang2024 | Collaborative planar pushing of polytopic objects with multiple robots in complex scenes; RSS XX | [RSS](https://roboticsproceedings.org/rss20/p063.html) | B |
| yan2021 | Decentralized ability-aware adaptive control for multi-robot collaborative manipulation; RAL 6(2):2311–2318 | [作者机构](https://www.research.ed.ac.uk/en/publications/decentralized-ability-aware-adaptive-control-for-multi-robot-coll/) | B |
| pereira2004 | Decentralized algorithms for multi-robot manipulation via caging; IJRR 23(7–8):783–795 | [出版社](https://doi.org/10.1177/0278364904045477) | B |
| culbertson2021 | Decentralized adaptive control for collaborative manipulation of rigid bodies; TRO 37(6):1906–1920 | [原文](https://arxiv.org/abs/2005.03153) | C |
| wang2016force | Force-amplifying N-robot transport system (Force-ANTS) for cooperative planar manipulation without communication; IJRR 35(13):1564–1586 | [出版社](https://doi.org/10.1177/0278364916667473) | C |
| chai2022 | Object rearrangement through planar pushing: A theoretical analysis and validation; TRO 38(5):2703–2719 | [出版社](https://doi.org/10.1109/TRO.2022.3153785) | B |
| wang2025caging | Caging in time: A framework for robust object manipulation under uncertainties and limited robot perception; IJRR 44(14):2365–2389 | [出版社](https://doi.org/10.1177/02783649251343926) | C |
| censi2007 | An accurate closed-form estimate of ICP's covariance; ICRA 3167–3172 | [作者全文](https://censi.science/pub/research/2007-icra-icpcov.pdf) | A |
| bonnabel2016 | On the covariance of ICP-based scan-matching techniques; ACC 5498–5503 | [原文](https://arxiv.org/pdf/1410.7632) | A |
| gelfand2003 | Geometrically stable sampling for the ICP algorithm; Fourth 3DIM | [作者全文](https://graphics.stanford.edu/papers/stabicp/stabicp.pdf) | A |
| du1999 | Centroidal Voronoi tessellations: Applications and algorithms; SIAM Review 41(4):637–676 | [出版社](https://doi.org/10.1137/S0036144599352836) | C |
| cortes2004 | Coverage control for mobile sensing networks; TRA 20(2):243–255 | [原文](https://arxiv.org/abs/math/0212212) | B |
| ames2017 | Control barrier function based quadratic programs for safety critical systems; TAC 62(8):3861–3876 | [出版社](https://doi.org/10.1109/TAC.2016.2638961) | B |
| wang2017safety | Safety barrier certificates for collisions-free multirobot systems; TRO 33(3):661–674 | [出版社](https://doi.org/10.1109/TRO.2017.2659727) | B |
| tuci2018 | Cooperative object transport in multi-robot systems: A review of the state-of-the-art; Frontiers 5:59 | [出版社](https://doi.org/10.3389/frobt.2018.00059) | C |
| stuber2020 | Let's push things forward: A survey on robot pushing; Frontiers 7:8 | [出版社](https://doi.org/10.3389/frobt.2020.00008) | C |
| ferrari1992 | Planning optimal grasps; ICRA 2290–2295 | [作者全文](https://people.eecs.berkeley.edu/~jfc/papers/92/FCicra92.pdf) | C |

这些条目全部在正文中有明确相关用途。未确定的少量 DOI/页码省略，未猜填。
题录核验的易错项：Chai 末页 2719；Ferrari 页码 2290–2295；Force-ANTS 正式年份 2016。
本轮不把此前材料中的所有论断视为已验证事实；阅读深度栏是检查范围，不是全文认证。

## 主张与证据映射

- 几何就绪不等于成功：完整九例历史开发子集及独立结果字段；没有修改事件定义。
- 物理就绪事件不是该子集成功的必要条件：5 成功/9 案例、物理 quorum 0/9；不否认真实接触的物理必要性。
- 进度反馈影响选定失败：只使用新生成的 3 对结果，不能把旧矩形结果充当新 baseline。
- 非退化主张被反证：C8 endpoint 增加，协议原 falsifier 保留。
- 方向可辨识命题：标准线性代数，固定对应/纯平移；与 ICP 可观测性文献一致，不作新颖性声明。
- 法向选择：仅合成观测机会，候选假定可得；没有机器人探测、硬件、安全运动或现场联系。
- 本轮不比较其他运输方法的总体优劣；已有前期框架/近邻工作不是刻意削弱的经验基线。

## 官方格式来源

2026-10-31 核对 [官方 CFP](https://ccta2027.ieeecss.org/authors/call-for-papers) 和 [CSS PaperPlaza LaTeX 支持](https://css.paperplaza.net/conferences/support/tex.php)。
使用后者下载的官方通用 `ieeeconf.cls`，来源、下载包哈希及适用范围见 `template/provenance.json`。
官方 CFP 规定 IEEE 标准双栏，初投最多 8 页、录用 6 页基准；不是项目自创页数规则。未提交稿件或调用投稿系统检查。

## progress-local-v2 跟进证据

20 篇文献及阅读深度未因本轮实验而扩称。新增论断来自 [实际匹配与三臂结果](../../docs/results/progress_local_v2/README.md)：22,797/22,812 次诊断配准可辨识；其余未被原质量门控接受。候选未救回两个失败，C8 继续退化。
新增锚定公式是条件模型的代数推论，保留增益及地图历史误差，不主张新可观测理论。全回合 MAE 按各臂自身轨迹/时长统计，同样本误差对照单独报告，不能混用两种口径。


## 2026-11-01 最近工作复核

重新阅读Song等v2的§III–IV、Sequential caging的§3–4、Tang等RSS2024的§II–III，并核查CCTA2027 CFP与CSS模板。具体对照、原文入口和未核验部分见[主张与媒体清单](MEDIA_AND_CLAIMS.md)。不能由这三篇的相关段落阅读推称本轮重新精读了全部20篇。现有内部对照不是这三篇的忠实复现。

## 2026-11-04 主线更新与来源复核

当前稿件新增匹配部署比较、有限支持负结果、336 系统方向能力分析和实际执行的 60 回合资源/噪声验证。最新数字见 [RESULTS_INDEX.md](RESULTS_INDEX.md)，不再把资源/噪声实验列为未执行。既有 20 篇文献的历史阅读深度未扩大为本轮全部精读。

本轮检查了 Song 等 v2 的检测密度/CVT/协调相关原文段落、Sequential caging 的原始 PDF 与通信/推动条件、Tang 等 RSS 的题录及本地全文提取中的已知对象几何/物性与准静态假设。信息权限和物理模型不同，因此正文只作有依据的机制比较，不宣称复现或战胜这些方法。Tang 全文在本地 `data/literature/2026-10-22/text/L06.txt`；原件及映射沿用原文献清单，不加入 Git。

- [Song et al., arXiv v2 原文](https://arxiv.org/html/2602.19070v2)：检测位置驱动的密度、CVT 与 CBF 已有先例。
- [Vardharajan et al., 原始论文](https://arxiv.org/pdf/2106.03132)：未知形状 sequential caging 包含有效推动和接触维护，也有机器人实验证据；不能说前人只做静态包围。
- [Tang et al., RSS2024 官方入口](https://roboticsproceedings.org/rss20/p063.html)：模型化接触模式与可行推动是既有研究，当前潜在差异须限定在信息权限和执行模型。

2026-11-04 再次核对 [官方 CFP](https://ccta2027.ieeecss.org/authors/call-for-papers) 与 [PaperPlaza 模板说明](https://css.paperplaza.net/conferences/support/tex.php)。主稿使用 8 页 Letter 双栏官方模板，补充 PDF 仅是本地审阅材料。没有上传 PDF 做服务器检查，未宣称通过正式投稿系统验收。

工作流工具出处单独列于 README，不加入机器人学理论证据。Kassis、Agarwal、He、Patel、Brueckner 的 [Scientific Agent Skills](https://arxiv.org/abs/2609.00065v2) 题录已核查；其工具说明不能替代本项目数据和人类科研审阅。
