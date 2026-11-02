# 全部回合能力表

| 条件 | 臂 | 完成 | 终点误差 m | 结束阶段 | 物理目标区内最长低速 s | G500 | 硬安全 |
|---|---|---|---:|---|---:|---|---|
| circle | reference | False | 0.833580 | TRANSPORT | 0.00 | False | none |
| circle | radial | False | 0.850110 | TRANSPORT | 0.00 | False | none |
| circle | patch | False | 0.664174 | TRANSPORT | 0.00 | False | none |
| circle | combined | False | 0.669576 | TRANSPORT | 0.00 | False | none |
| ellipse_parallel | reference | False | 0.345632 | BRAKE | 0.00 | False | none |
| ellipse_parallel | radial | False | 0.345632 | BRAKE | 0.00 | False | none |
| ellipse_parallel | patch | False | 0.039752 | BRAKE | 47.85 | False | none |
| ellipse_parallel | combined | False | 0.039752 | BRAKE | 47.85 | False | none |
| c_open_away | reference | False | 0.783854 | BRAKE | 0.00 | False | none |
| c_open_away | radial | False | 0.783854 | BRAKE | 0.00 | False | none |
| c_open_away | patch | False | 0.120817 | BRAKE | 0.00 | False | none |
| c_open_away | combined | False | 0.120817 | BRAKE | 0.00 | False | none |

物理低速诊断不替代原 HOLD＋连续40帧低速的结束规则。观察截止不是成功完成时间。未完成回合的 completion_seconds 在 CSV 中为空。
