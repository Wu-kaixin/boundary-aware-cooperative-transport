# Phase 2 路线评估：定向文献核查

检索与复核日期：2026-11-04（日本时间）。这是围绕四条候选机制的有限新颖性检查，
不是系统综述，也不是“没有其他相似研究”的证明。先独立列出 R1–R4，再核查反例与
最近工作；只将作者论文、会议页面、作者项目页和机构存档用于实质判断。
旧文献笔记及原文保持不变，未新下载全文到仓库。

检索主题包括 unknown geometry cooperative transport / sequential caging、
contact-compatible CBF / force-constrained contact、task-consistent CBF / deadlock /
feasibility certification、ICP covariance，以及 unknown-object contact configuration。
下表的阅读深度限定了本轮能够支持的结论。检索结果中的更新时间不作为出版日期。

| ID / 已核对版本 | 原始来源与本轮阅读范围 | 对本项目的具体约束 |
| --- | --- | --- |
| S1，Song, Bai & Wakamiya，2026，arXiv v2 | [Cooperative Transportation Without Prior Object Knowledge via Adaptive Self-Allocation and Coordination](https://arxiv.org/html/2602.19070v2)，摘要、机制入口；结合既有 [L10](notes/L10.md) | 密度/CVT/CBF 与未知物体协作已存在。L10 保存用户确认：这是本团队前作；不能列为独立外部对照，也不能将继承的组件组合重新声明原创。 |
| S2，Vardharajan, Soma & Beltrame，2021，DARS 接收稿 | [Collective transport via sequential caging](https://arxiv.org/abs/2106.03132)，元数据与摘要 | 无形状先验、顺序边界分配、推动角色选择和集体运输已有工作及硬件示例。“未知形状+合围+搬运”本身不足以构成新颖性；本轮未复核其全部信息假设或证明。 |
| S3，Tang, Feng & Guo，RSS 2024，arXiv v2 | [正文](https://arxiv.org/html/2405.07908v2)，摘要、§II-A/B；[会议记录](https://roboticsproceedings.org/rss20/p063.html)，DOI 10.15607/RSS.2024.XX.063；既有 [L06](notes/L06.md) | 已有多方向接触可行性、模式选择、混合规划和跟踪。模型给定多边形及质量、惯性、压力分布和摩擦参数。DBACT 可研究在更少信息下完成哪一项具体能力，不能声称首次考虑接触能力。未在本轮重审其全部完备性证明。 |
| S4，Doshi, Taylor & Rodriguez，ICRA 2022 | [作者稿](https://nddoshi.github.io/assets/pdfs/doshi_icra2022.pdf)，摘要、引言与方法范围；[MIT 记录](https://dspace.mit.edu/entities/publication/e1f6b2b7-61a4-4f60-ba10-144182db74ae)，DOI 10.1109/ICRA46639.2022.9811713 | 已有未知物体接触配置、力旋量及运动约束的联合估计控制；使用机器人本体信息和腕部力/力矩感知。不能把该信息条件直接移植为 DBACT 现有局部扫描能力。 |
| S5，Wang et al.，2024，arXiv v1 | [Guarding Force: Safety-Critical Compliant Control for Robot-Environment Interaction](https://arxiv.org/abs/2405.04859)，元数据与摘要 | 接触力约束 CBF 已存在，摘要明确使用环境先验刚度/平衡位置及误差估计。它限制“首次把安全过滤用于接触”的主张；不替 DBACT 提供未知环境力界证明。未复核全文定理。 |
| S6，Morton & Pavone，IROS 2025，arXiv v3 | [正文](https://arxiv.org/html/2503.06736v3)，§III–IV，特别是任务一致性与式 (23)–(27)；[作者页](https://stanfordasl.github.io/oscbf/) 核对出版信息 | 安全集内的目标度量会影响任务表现，任务层次和输出空间目标已有明确方法。R2 的一般思想不是新理论；移动机器人接触中任务输出如何可观测、是否改善完整任务仍是本项目问题。该文式 (28) 后也提醒加入输入界不自动给出前向不变性。 |
| S7，Grover, Liu & Sycara，IJRR 2022 接收稿 | [The Before, During, and After of Multi-Robot Deadlock](https://arxiv.org/abs/2206.01781)，元数据与摘要 | CBF 死锁、KKT/对偶解释与去中心化解锁已有研究。不能将“QP 可行但任务不前进”或标准 KKT 解释作为独有发现。未复核全部解锁算法适用条件。 |
| S8，Sah & Keshavan，2026-09-14，arXiv v1 预印本 | [Exact Feasibility Certification and Optimal Responsibility Allocation for Multi-Robot CBF Safety Filters](https://arxiv.org/html/2609.14935v1)，摘要、§III/IV 与 §VII | 已有逐时刻需求—执行能力的可行性证书及责任分配；§VII 明确不保证任务完成。限制通用方向/可行性诊断的新颖性。本轮未验证论文证明或代码；预印本身份保留。 |
| S9，Bonnabel, Barczyk & Goulette，ACC 2016，arXiv v3 | [On the Covariance of ICP-based Scan-matching Techniques](https://arxiv.org/abs/1410.7632)，元数据与摘要 | 配准协方差依赖误差模型与重匹配语义，点到点和点到面处理不能混用。R3 必须对照本仓库实际估计器，不能借一个通用协方差公式宣称绝对参考可信。未逐项验证其推导。 |
| S10，Wang, Ren, Morgan & Hang，arXiv v4，2025-05-01 | [Caging in Time](https://arxiv.org/html/2410.16481v4)，定义 4.1、算法 1、§5.1–5.2 | 时序笼闭和有限感知操作已有集合传播分析；平面实例使用已选包围圆和线段推手。该信息/硬件模型与 DBACT 不同，可作边界对照，不能声称它已解决本仓库任务，也不能忽略其现有理论。 |

RSS 官方 PDF 本轮无法由网页工具读取，作者 PDF 因 12.9 MB 大小被工具拒绝；随后读取
arXiv HTML §II 获得所需原始假设，不据失败链接猜测。另检索到不良平衡消除、调制式
避障和 ICP 早期文献，本报告没有在未展开阅读的基础上引用其定理。

综合判断属于**本项目的推断**：R1 尚有“局部有限几何如何与真实接触及信息边界相容”
的潜在空间；R2 的标准凸优化部分、R3 的通用一致性处理和 R4 的通用接触配置思想
均有强先例。没有一条路线仅凭上述检索即可被认定已具备论文级创新。
