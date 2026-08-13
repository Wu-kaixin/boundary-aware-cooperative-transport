# Stage 1: Progress/Wrench Closed-Loop Ablation

This result is a controller-development ablation, not a cross-shape statistical
claim and not a finite-time theorem. All variants use the same seed-0 random
simple polygon, pose, 360-degree sampled task direction, contact physics, CBF-QP,
and acceptance thresholds. Step 900 is a failure timeout, not a success
deadline.

| Variant | Termination | Steps | J (m) | Efficiency | Coverage | Rho relaxations |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Fixed feed-forward | TIMEOUT | 900 | -0.2861 | -0.841 | 1.000 | 522 |
| Progress PI only | TIMEOUT | 900 | -0.3391 | -0.815 | 0.975 | 797 |
| PI + consensus wrench allocation + contact release | SUCCESS_HOLD | 765 | +0.0791 | 0.995 | 1.000 | 134 |

The successful variant reached detection/map/enclosure/contact/transport/BRAKE/
HOLD at frames 127/268/234/271/282/739/759. It executed 13,770 hard-QP solves
with zero fallback and zero infeasible solve.

The ablation supports a causal engineering conclusion: PI pressure alone does
not resolve the contact geometry. A consistent nonnegative force/zero-torque
allocation is also insufficient while unallocated leading-side robots remain in
passive contact. Consensus allocation plus explicit local contact release changes
the signed task outcome from reverse motion to efficient positive transport.

It does **not** establish cross-seed robustness, a formal caging theorem, or an
analytic transport-time bound. The 134 optional robustness-margin relaxations
remain an open defect for Stage 2.

Reproduce with:

```bash
python scripts/run_progress_ablation.py \
  --variants fixed_feedforward,pi_only,pi_wrench_release \
  --seeds 0 --max-steps 900 --out runs/progress_ablation_stage1
```
