# 冻结标签审查范围

通过 GitHub matching-refs/tags API 取得 16 个标签，与本地标签对象逐一比较，全部一致；没有移动标签、更新分支、push 或重写历史。tag_identity.json 记录远端对象与本地 peel 后的 commit。tag_code_review.json 记录所选安全源码的路径、哈希和公式位置；其中 10 个标签有同类混合归一化。没有据此声称每个标签的所有文件或所有实验均已重新验证。

直接检查的历史证据：

| 标签 / commit | 原始文件与发现 | 可用范围 |
| --- | --- | --- |
| archive/2026-10-23/DBACT-research-v3 / a3d5e693d40d541b90d69606624c607223d2e491 | docs/results/v2_shape_matrix/REPORT.txt：180 回合、旧定义成功 54/180 | 12 形状、5 seeds、3 距离；不可合并当前二维完成定义 |
| archive/pre-integration-2026-09-01/A-boundary-aware-closed-loop-v3 / c038829487ad7b6a15046dc79b9dfc95b3bfe83c | docs/results/ARBITRARY_SHAPE_MONTE_CARLO_FINAL_SE2.md：60 未筛选回合、14/60 达到当时合同 | 0.10 m 激活后距离、1500 帧；不是当前 1.2 m 任务 |
| archive/2026-10-23/research-simulation-evidence-closure / 15d62f2e30063c3258ae49ea00b5dedbc0228d8a | README.md 区分静态部署、定理和独立仿真证据 | 静态结论不能当作动态接触运输保证 |

上述已读报告没有建立当前“任意形状或总体成功概率超过 90%”的证据。本地尚未推送的研究按源码快照与收据识别，不能因为 GitHub 缺少它们而视为未完成。
