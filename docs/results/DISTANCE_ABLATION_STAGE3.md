# Stage 3: activation-relative long-distance transport

Date: 2026-08-09
Branch: `Codex-boundary-aware-closed-loop-v1`
Config: `configs/sim/research/adaptive_progress_closed_loop.yaml`
Seed: 0
Safety timeout: 1800 frames (not a success deadline)

## Definition correction

Transport progress is now evaluated using the same activation-relative definition as the controller:

\[
J=(x_{obj}-x_{activation})^T d_{goal}.
\]

`episode_total_J` is retained separately and includes passive displacement during SEARCH/MAP/ENCLOSE. The two quantities must not be mixed. The distance runner uses a declared 1 mm controller reserve for estimator/integration discretisation; it does not lower `j_min`.

## Final distance matrix

All three runs reached `SEARCH -> MAP -> ENCLOSE -> TRANSPORT -> BRAKE -> HOLD`. The controller never writes cargo pose or velocity; cargo motion is produced only by the penalty contact dynamics.

| Requested distance | HOLD frame | Transport J | Estimate error | Efficiency | Max cross-track | Max abs rotation | Max penetration | Min robot distance | fallback / infeasible / rho relax | fps |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.10 m | 339 | 0.07689 m | +0.00004 m | 0.858 | 0.04594 m | 7.19 deg | 0.01226 m | 0.32107 m | 0 / 0 / 0 | 42.10 |
| 0.25 m | 393 | 0.22933 m | +0.00012 m | 0.956 | 0.07067 m | 7.81 deg | 0.01322 m | 0.32022 m | 0 / 0 / 0 | 36.33 |
| 0.50 m | 960 | 0.48140 m | -0.00470 m | 0.899 | 0.23471 m | 7.84 deg | 0.01719 m | 0.32020 m | 0 / 0 / 414 | 22.52 |

The success contract uses `j_min = L - 0.025 m`; thus the measured J values pass without changing the evaluation threshold. Strict boundary coverage remained 1.0 throughout TRANSPORT for all three runs.

## Controller changes

- Distributed two-dimensional motion integration now retains both longitudinal progress and cross-track displacement.
- The allocated wrench direction includes lateral feedback, `normalize(sign*d_goal - k_perp*e_perp)`.
- Wrench allocation excludes a contact whose measured full ISSf clearance reserve is nearly exhausted; zero allocation activates local release using the latest raw safety scan.
- PI pressure authority was reduced to avoid plant-level windup during long contact: position gain 0.5, bias 0.1, effort limit 0.6, integral limit 0.2.
- BRAKE re-engages at 0.026 m while HOLD remains strict at 0.025 m, removing the prior supervisor dead zone without weakening the terminal tolerance.

## Truth-audit result

The controller-independent truth audit reproduced the same trajectory and measured:

| Distance | Raw boundary point max | Persistent map point max | Object velocity max | Boundary velocity max | CBF projection max | Audit verdict |
|---:|---:|---:|---:|---:|---:|---|
| 0.10 m | 0.00983 m | 0.24253 m | 0.15973 m/s | 0.31954 m/s | 0.31751 m/s | bound-valid |
| 0.25 m | 0.00983 m | 0.33320 m | 0.23923 m/s | 0.31954 m/s | 0.31751 m/s | map bound exceeded |
| 0.50 m | 0.00983 m | 0.33288 m | 0.22167 m/s | 0.31954 m/s | 0.31751 m/s | map bound exceeded |

The declared persistent-map bound is 0.27 m. It was not enlarged after seeing the results. Two attempted median SE(2) map-consensus variants made the maximum map error worse (0.373 m and 0.357 m) and were removed rather than committed.

## Claim status

- **Empirically validated:** activation-relative closed-loop completion at 0.10, 0.25 and 0.50 m for seed 0; no hard-QP fallback or infeasibility; >20 fps for every non-audited run.
- **Robust-margin validated:** 0.10 and 0.25 m retain fallback=0, infeasible=0 and rho-relaxation=0.
- **Not robust-margin validated:** 0.50 m completes but uses 414 rho relaxations.
- **Unsupported at long distance:** the 0.27 m persistent-map error premise fails at 0.25 and 0.50 m. The raw local safety observations remain within their bound, so this is a planning-map tracking limitation rather than evidence that the raw safety-point bound was exceeded.
- **Not a finite-time theorem:** the observed HOLD frames are samples, not a proved upper bound or an empirical upper confidence bound.

Raw per-frame outputs are under `runs/distance_ablation_stage3_final_v4` and `runs/distance_ablation_stage3_audited_v1` (git-ignored). The compact checked-in record is `docs/results/distance_ablation_stage3.json`.
