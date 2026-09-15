# A priori centroid bound — 2026-09-15

Static sampled `theorem_mode`, N=16, 3 shapes × 3 seeds × 3 controller grids, Δ=0.05, 600 steps (30 s). Evaluation polar grid fixed at (nθ=128, nr=12). Priors were written **before** each trajectory. Old `theorem_audit_theorem_mode_2026-09-11` artifacts were **not** inputs.

Worktree: `feat/apriori-centroid-bound` based on `a531515`. Code SHA recorded in summaries is the base commit; the working tree that produced these runs is dirty (this branch’s new files). Do not merge or push unless asked.

## Short answers

1. **Valid prior?** Yes as a **conditional** majorant, not an unconditional guarantee. `B_E = min(moment form, 4 R² M)` is rigorous under the truncated-cell geometry. Instantiating (★) still requires full-horizon K₀ (every QP `optimal` and `zero_input_feasible_with_rho`) and `a>0`. Polar quadrature of H, g, m, c is assessed at P₀ only and is **not** a strict certificate. The **active** `B_E` on every case is the diameter `4 R² M` (~4.80 L/C, ~3.77 rectangle): the grid-dependent moment form is looser at n∈{20,40,80}.

2. **Prior vs geometry?** **No configuration** has `B_J_prior < M_tot u_max²`. On the 24 applicable cases the prior is about **18× looser** than geometry (L/C: 4.10 vs 0.230; rectangle: 3.22 vs 0.180). The **post-hoc** bound that uses trajectory `Ē` **does** beat geometry on all 24 applicable cases (~0.025–0.034 vs 0.180–0.230). That is explanatory, not a priori.

3. **Grid refinement?** **Measured `Ē` falls with controller n** (eval grid fixed), roughly **50× from n=20 to n=80**. Mean over seeds:

   | shape | n=20 | n=40 | n=80 |
   |---|---|---|---|
   | L | 1.32e-4 | 2.11e-5 | 4.21e-6 |
   | rectangle | 1.40e-4 | 1.48e-5 | 1.73e-6 |
   | C | 2.10e-4 | 3.24e-5 | 5.78e-6 |

   Certificate `B_E` **does not** fall (diameter is active). `J̄` is almost independent of n. `H*(P₀)` is independent of the controller grid (paired seeds). Observer polar |ΔH| at P₀ is ~6e-6 (128/12 vs 256/16).

4. **Bottleneck?** **Proof tightness of the CVT endpoint grid**, not the measured integration error. Oracle L¹ (analytic) ~3e-3 and restrict disk L¹ (site grid) ~1e-4 are small; the N-fold Lip-over-disk tube is huge, so `B_E` collapses to `4 R² M`. The `H₀/(a K Δ)` term is ~0.03, already below geometry. **QP/K₀** is a separate applicability failure: C-shape seed 5 is inapplicable at **all three** resolutions (agent 14); the failing frames **move** with n (241–246 / 224–228 / 232–236). Hard-coding 241–246 would have been wrong.

5. **Next step?** Tighten or replace the **LocalCVT endpoint-grid analysis/integrator** (midpoint or clipped polygons, not global Lip × disk area). Do not promote trajectory `Ē` to a prior. Raising `influence_sigmas` or shrinking oracle spacing will not move the certificate while the diameter is active. This round does **not** extend to dynamic transport or an N=16 Hessian.

## Reproduce

```text
python scripts/run_apriori_centroid_bound.py --out artifacts/apriori_centroid_bound_2026-09-15 --frames 600
python -m pytest tests/test_apriori_centroid_bound.py tests/test_theorem_mode.py tests/test_local_cvt.py tests/test_density.py tests/test_safety_filter.py
```

Smoke (done first): 4 frames, L-shape seed 2, n=20. Then all 27 combinations were re-simulated.

## Run tally

- 27/27 `complete_30s`, no aborts.
- 24/27 full-horizon K₀. Inapplicable: `c_shape` seed 5 at n=20, 40, 80.
- Geometric comparator is `(M_observer + 1e-10) u_max²`, never `N m_+ u_max²`.
- n=20 L/rectangle/C seeds 2,5,8 match the 2026-09-11 nine-case `J̄`, `Ē`, `H₀` at paired initials.

## Layout

- `derivation.md` — formulae, domains, floors vs refinement
- `prior_constants.json` / `priors/` — saved **before** each closed-loop run
- `configs/` — per-shape overlays with `grid_resolution`
- `runs/<shape>/n<grid>/seed_*/` — trajectory.npz, step_records.json, metrics.csv, summary.json
- `summary.csv`, `figures/`, `pytest_affected.log`, `run_list.json`, `branch_manifest.json`
