# Static main theorem finalization — 2026-09-16

Worktree: `feat/static-main-theorem-finalization` from `c178f76`. Not pushed. Historical `artifacts/static_main_theorem_closure_2026-09-16/` is untouched.

## Three judgements (do not collapse)

| Judgement | Status |
|---|---|
| Exact-arithmetic main theorem (safety from P0, joint recursive feasibility, projection \(B_E\), dissipation ★) | **CLOSED** for the declared static sampled scope |
| Certificate \(B_{J,\mathrm{prior}}<M_{\mathrm{plane}}u_{\max}^2\) on rectangle, L-shape, **and C-shape** | **CLOSED** after dropping the duplicate projection extra; \(h^\star=0.004\) frozen |
| Float QP ≡ exact projection; polar \((H,g,m,c)\) as a quadrature certificate | **open**, labelled, not claimed |
| Feature-cover closed-loop real \(J,E,H^\star\) | **27/27 complete**, full-horizon \(\mathcal K_0\), 259200/259200 `optimal`. Observer estimates, not the theorem. Serial/spawn max \(\lvert J,E,H\rvert=0\) on the 80-frame L2/R2 check |

Control mapping is unchanged from `c178f76`. Seeds 29/31/37 remain independent validation of that mapping. Analysis-only change: projection-optimality \(B_E\) and the range-truncation contract.

## Mathematics

- `main_theorem.md` / `main_theorem.tex`
- `projection_error_lemma.md`
- `barrier_invariance_derivation.md` (A1–A6, first crossing, (RT))
- `proof_dependency.md`
- `paper_claims.md`
- `discrete_dissipation_derivation.md` (imported, re-checked)
- `constant_ledger.json`

\(\mathcal K_0\) is a corollary of P0 + feature-cover, not an extra “assume the rest of the trajectory” hypothesis.

Range truncation (same scalars as the filter): \(0.60-0.35\cdot 0.05=0.5825\ge 0.065+0.02/8=0.0675\).

## Certificate constants (written before the JEH matrix)

| shape | \(B_E\) | \(B_{J,\mathrm{prior}}\) | \(B_{J,\mathrm{geom}}\) |
|---|---:|---:|---:|
| rectangle | 0.1287048157 | 0.1380072238 | 0.1802506048 |
| L-shape | 0.1702700924 | 0.1811413578 | 0.2295107776 |
| C-shape | 0.1814594174 | 0.1906267518 | 0.2295107776 |

C-shape matches the cross-check \(0.2358637193-0.0544043019\).

## Reproduce

CPU: 16 physical = 16 logical. Outer pool uses usable CPUs; BLAS/OpenMP = 1 per worker.

```
python scripts/run_apriori_centroid_bound.py --out artifacts/static_main_theorem_finalization_2026-09-16/jeh_matrix --shapes l_shape rectangle c_shape --seeds 2 5 8 11 17 23 29 31 37 --grids 20 --frames 600 --workers 0 --resume
```

Priors only:

```
python scripts/run_apriori_centroid_bound.py --out artifacts/static_main_theorem_finalization_2026-09-16/priors_recomputed --shapes l_shape rectangle c_shape --seeds 2 --priors-only --workers 0
```

\(J_k=\sum_i m_i^\star\|U_{k,i}\|^2\). Do not use `J_proxy_speed2`.

Closed-loop matrix: `jeh_matrix/` (27 cases, 16 outer workers, 4110 s wall, ~87% CPU, 16 children). Compact table: `jeh_table.json`, `jeh_summary.csv`. Serial/parallel: `serial_parallel_consistency.json`. Resource: `resource_usage.csv`.

Observed \(\bar J\) is about \(0.0086\)–\(0.0126\) (post-hoc), far below \(B_{J,\mathrm{prior}}\). This is not a coverage-success claim.

## LaTeX

`main_theorem.tex` is the compile source. This environment had no `pdflatex` on PATH; a portable tectonic download did not unpack. Compile locally:

```
pdflatex -interaction=nonstopmode main_theorem.tex
```

## Not claimed

Dynamic transport, global CVT stationarity, “small \(J\) ⇒ coverage success”, or that every IEEE-754 QP call is the exact projection.
