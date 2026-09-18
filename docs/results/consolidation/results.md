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

串行/并行完整 600 帧对照（L 形与矩形，seed 2，容差 1e-12）：**PASS**。

![静态部署对照](docs/assets/static-deployment-comparison.png)

完整 QP 状态、K0 与最小净空：[静态安全](docs/results/consolidation/static_safety.json)。

**Gate 5：局部边界接口**
成对形状：L、矩形、C；种子 2、5、8。每一对共享同一初值。
仅经验测量。真值几何只给外部评估器，不进入局部控制，也不做真值净空否决。

| 形状 | 地图 | 完成 | 终态 H（均值） | 终态质心残差²（均值） | 最小安全裕量 | 终态地图覆盖（均值） | QP 干预（均值） |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| l_shape | oracle | 3/3 | 0.15965 | 0.0035678 | 0.0021663 | 1.0000 | 0.1174 |
| l_shape | local | 3/3 | 0.15756 | 0.0058908 | -0.016768 | 1.0000 | 0.1127 |
| rectangle | oracle | 3/3 | 0.12386 | 0.0016987 | 0.0023582 | 1.0000 | 0.0871 |
| rectangle | local | 3/3 | 0.12396 | 0.002114 | -0.013497 | 1.0000 | 0.0539 |
| c_shape | oracle | 3/3 | 0.16252 | 0.0032334 | 0.0020473 | 1.0000 | 0.2264 |
| c_shape | local | 3/3 | 0.16409 | 0.007623 | -0.015039 | 1.0000 | 0.1585 |

局部运行中观测到物体安全违反的次数：**8/9**。对照完成并不构成未知边界安全定理。

[Gate 5 逐种子结果与定义](docs/results/consolidation/gate5_pairs.json)。

**Gate 6：部署对搬运的作用**
成对形状：L、矩形、C；种子 2、5、8。每一对共享同一初值。
仅替换 CONTACT_READY 之前的部署目标；感知、安全层与后续搬运保持相同。

| 形状 | 部署 | G500 通过 | 收定 | 接触就绪秒（均值；观测 n） | 接触数（均值） | 终态距离误差（均值） | QP 干预（均值） |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| l_shape | boundary_cvt | 1/3 | 3/3 | 4.05; n=3 | 2.444 | -0.06754 | 0.08075 |
| l_shape | nearest_observed_target | 0/3 | 3/3 | 11.75; n=3 | 2.074 | -0.02092 | 0.2262 |
| rectangle | boundary_cvt | 0/3 | 3/3 | 4.8; n=3 | 2.219 | -0.02013 | 0.1968 |
| rectangle | nearest_observed_target | 0/3 | 3/3 | 7.233; n=3 | 2.083 | 0.03917 | 0.23 |
| c_shape | boundary_cvt | 1/3 | 2/3 | 4.3; n=3 | 3.113 | 0.5945 | 0.1816 |
| c_shape | nearest_observed_target | 0/3 | 3/3 | 7.967; n=3 | 1.867 | 0.05147 | 0.2838 |

完整成对 JSON 含接触方位分布、观测力/力矩、法向瞬时扳手容量、方向误差、横向偏移与安全最小值。该容量是瞬时接触模型界，不是动力学保证。这些小样本对照不构成普遍的搬运优越性。

[Gate 6 逐种子结果与定义](docs/results/consolidation/gate6_pairs.json)。

[实验清单与源哈希](docs/results/consolidation/manifest.json)。
[精简结果目录](docs/results/consolidation/)。
