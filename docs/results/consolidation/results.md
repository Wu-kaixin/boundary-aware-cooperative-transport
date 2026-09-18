由本次运行的 JSON/CSV 生成；所有种子与失败均保留。

| 运行 | 帧数 | 终止 | G500 | 失败原因 |
| --- | ---: | --- | --- | --- |
| near seed 2 | 445 | settled | FAIL | G500: 3 scaled-barrier event(s), smallest factor 0.676; the contract allows 0 |
| near seed 4 | 248 | settled | PASS | — |
| near seed 8 | 352 | settled | FAIL | G500: target reached never happened within the 352-frame run; G500: J=1.1789 m stopped 0.1255 m short of L=1.3044 m, outside the declared arrival tolerance 0.1200 m |
| search seed 7 | 888 | settled | FAIL | G500: target reached never happened within the 888-frame run; G500: J=0.8900 m < target L=0.9037 m; G500: 99 scaled-barrier event(s), smallest factor 0.012; the contract allows 0 |

近场扫描种子 `[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]`：**7/12 通过 G500**。

| 指标 | 均值 ± 总体标准差 | 最小–最大 |
| --- | ---: | ---: |
| J | 1.20977 ± 0.187161 | 0.911304–1.55259 |
| efficiency | 0.988794 ± 0.0143272 | 0.954195–0.999944 |
| direction_error_deg | 6.95319 ± 5.05877 | 0.608667–17.4086 |
| max_cross_track | 0.162624 ± 0.120949 | 0.0272753–0.435453 |
| max_strict_coverage | 0.979167 ± 0.0357521 | 0.875–1 |
| min_inter_agent_distance | 0.285139 ± 0.00547689 | 0.280117–0.296786 |
| min_signed_clearance | 0.0852181 ± 0.00972598 | 0.0700837–0.0978939 |
| contact_ready_frame | 85.5833 ± 23.6518 | 67–148 |
| hold_frame | 255.083 ± 82.7118 | 152–403 |

**静态 oracle-map 理论验证**（3 种形状 × 9 个种子 × 600 帧）。
下表是已声明的严格先验量；图中 J/E/H 为数值观测器估计。

| 形状 | B_E | B_J_prior | B_J_geom | 完成 | 全程 K0 |
| --- | ---: | ---: | ---: | ---: | ---: |
| l_shape | 0.1702700924 | 0.1811413578 | 0.2295107776 | 9/9 | 9/9 |
| rectangle | 0.1287048157 | 0.1380072238 | 0.1802506048 | 9/9 | 9/9 |
| c_shape | 0.1814594174 | 0.1906267518 | 0.2295107776 | 9/9 | 9/9 |

串行/并行完整 600 帧对照仍在运行；下图仅使用并行矩阵轨迹。

![静态部署对照](docs/assets/static-deployment-comparison.png)

完整 QP 状态、K0 与最小净空：[静态安全](docs/results/consolidation/static_safety.json)。

**Gate 5：局部边界接口**
该门的成对实验仍在执行，暂不宣称成功率。

**Gate 6：部署对搬运的作用**
该门的成对实验仍在执行，暂不宣称成功率。

[实验清单与源哈希](docs/results/consolidation/manifest.json)。
[精简结果目录](docs/results/consolidation/)。
