# Checkpoint

Status: **PARTIAL** (not CLOSED). Session work is complete relative to the stop conditions.

## Source

* Worktree: `D:/boundary-aware-cooperative-transport-certificate-repair`
* Branch: `feat/apriori-certificate-repair`
* Base: `7b9fb36d94ffba572881c11b8050d1d4cee2811d`
* Original workspace left intact: `D:/boundary-aware-cooperative-transport-apriori`

## Completed

* Integrator remainder repair + tests.
* Frozen priors beating geometry at constant level.
* Stage C/D/E closed-loop matrix (18 runs × 600 steps). No live workers.

## Blocking condition

Full-horizon K0 (0 ∈ F_ρ every step) fails on C5 (dev; includes 0 ∉ F_hard) and L11/L17 (validation; margin-only). Route B with ‖Π_F(0)‖ ≤ u_max is useless against geometry. See `k0_diagnosis.json` and README.

## Resume (if continuing a different route)

```bat
cd /d D:\boundary-aware-cooperative-transport-certificate-repair
python scripts/run_apriori_centroid_bound.py --out artifacts/apriori_certificate_repair_2026-09-15 --resume
```

Do not rerun completed hashed cases. Do not treat seeds 11/17/23 as unused if you change the method using them.
