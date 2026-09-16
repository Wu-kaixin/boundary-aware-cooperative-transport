# Checkpoint — static main theorem finalization (session end)

Date: 2026-09-16
Branch: `feat/static-main-theorem-finalization` from `c178f76`. **Not pushed, not merged.**
Worktree: `D:/boundary-aware-cooperative-transport-static-main-theorem-closure`
Artifact: `artifacts/static_main_theorem_finalization_2026-09-16/`

Working tree is dirty (proofs, projection budget, range-truncation contract, dump threshold). Control mapping unchanged.

## Closed for the declared static sampled scope

- Feature-cover hold safety with Φ / cover-exclusion / range-truncation split, first crossing, joint recursive feasibility from P0. \(\mathcal K_0\) is a corollary.
- Projection-optimality \(B_E\) (duplicate extra removed). C-shape \(B_E=0.1814594174\), \(B_{J,\mathrm{prior}}=0.1906267518<0.2295107776\). All three shapes beat geometry. \(h^\star=0.004\) frozen.
- Dissipation (★) on sat→unclamped \(F_\rho\) cascade with that \(\mathcal K_0\).
- 27/27 feature-cover trajectories: full-horizon \(\mathcal K_0\), 259200/259200 `optimal`, real mass-weighted \(J,E,H^\star\). Independent seeds 29/31/37 included.
- Serial vs spawn: max abs \(J,E,H,\)slack \(=0\) on 80-frame L2 and rectangle-2, and vs the 600-frame prefix.

## Open / not claimed

- IEEE-754 QP = exact Euclidean projection
- Polar observer as a quadrature certificate (slacks \(\le 1.13\cdot 10^{-4}\), undetermined at the observer envelope)
- PDF compile on this host (`pdflatex` missing; see `latex_compile_log.md`)
- Dynamic transport, global CVT, “small \(J\) ⇒ coverage”

## Restore / resume

```
python scripts/run_apriori_centroid_bound.py --out artifacts/static_main_theorem_finalization_2026-09-16/jeh_matrix --shapes l_shape rectangle c_shape --seeds 2 5 8 11 17 23 29 31 37 --grids 20 --frames 600 --workers 0 --resume
```

JEH matrix is complete; `--resume` should reuse COMPLETE.json cases if source_fp matches.
