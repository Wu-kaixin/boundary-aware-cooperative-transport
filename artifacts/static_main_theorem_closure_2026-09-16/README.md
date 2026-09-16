# Static main-theorem closure — 2026-09-16

Status: **PARTIAL** (not CLOSED).

Worktree: `D:/boundary-aware-cooperative-transport-static-main-theorem-closure`  
Branch: `feat/static-main-theorem-closure`  
Base commit: `7e948a812a781f5ce3c16fad3525c7138bb672ec`  
Original research worktree left intact: `D:/boundary-aware-cooperative-transport-certificate-repair`

## What is decided

* Aggregate K0 failures (C5/L11/L17) are 90° convex-corner infinite-plane switches, not empty F, clamp, or cap. See `barrier_update_diagnosis.md`.
* Theorem-mode object rows are Euclidean segment distances on the one-step cover Φ (`2 u_max Δ`). Invariance write-up: `barrier_invariance_derivation.md`.
* Legal source reach is `min(√2 R+sσ, R+2d_c+n_σ σ)`, not `R+sσ`.
* Recomputed priors beat geometry on rectangle and L-shape, **not** on C-shape (`constant_ledger.json`).

## Experiments

* Stage D, *single-nearest* (pre-cover): 18/18, 9600 QP `optimal`, empty K0-fail, 16 workers, wall 182 s. Directory `stage_d_k0/`. Not the proved algorithm.
* Feature-cover Stage C (L2/C5/L11/L17): 4/4, 9600 `optimal`, empty K0-fail, wall 80 s. Directory `stage_c_k0_cover/`.
* Feature-cover Stage D (3×{2,5,8,11,17,23}): 18/18, 9600 `optimal`, empty K0-fail, 16 workers, wall 193 s. Directory `stage_d_k0_cover/`.
* Feature-cover Stage E (3×{29,31,37}, pre-declared, unused in design): 9/9, 9600 `optimal`, empty K0-fail, 9 workers × 2 CVT, wall 89 s. Directory `stage_e_k0_cover/`.
* Serial/parallel consistency (L-shape seed 2, 600 steps): 0 safety mismatches; `h_true`, `J` proxy, and hold clearance match to 0. Log: `serial_parallel_consistency.json`.
* Priors: `priors_recomputed/`, 18 cases, wall 1.8 s.

This package does not reuse pre-repair trajectories as new-algorithm results.

See `checkpoint.md` for resume commands. Claims: `paper_claims.md`. Dependencies: `proof_dependency.md`.
