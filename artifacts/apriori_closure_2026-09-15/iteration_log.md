# Iteration log — a priori centroid bound closure

## Snapshot

- Recovered deleted working tree from transcript `04adce76` after on-disk loss of source/docs/priors.
- Local commit `175fd67` (tree hash in `_recovery/source_snapshot.json`).
- Baseline `a531515`. Runs under `artifacts/apriori_centroid_bound_2026-09-15/` retained on disk (gitignored).

## Round 1 — formula audit (no new closed loop)

### Hypotheses checked against source

1. `density.restrict` deletes by raw `b_j` with reach `R + max(offset) + s σ`.
2. Kernel centres are `ξ_j = b_j + offset_j n_j`, so deleted sources obey `||q-ξ_j|| ≥ sσ` on `B(p,R)`.
3. Truncated Voronoi cells `Ω_i = V_i ∩ B(p_i,R)` are pairwise disjoint (null boundary).
4. Oracle weights are nonnegative arc lengths; with conf=1 and no explore/gap, `W = L`.

### Partition restrict lemma (proved)

\[
\sum_i \int_{\Omega_i}|\phi_{\mathrm{full}}-\phi_{\mathrm{restrict},i}|\,dq
\le 2\pi\sigma^2 e^{-s^2/2}\,W.
\]

Wolfram: plane tail integral `= 2 π σ² Exp[-s²/2]`.

L-shape numbers: partition `dm_rst = 0.0201` vs legacy N-fold `2.573`.

### Endpoint-grid tube (blocked as primary path)

Even with partition restrict and line-edge `φ_max`, N-fold Davenport tubes give `dm_grid ≫ 0.066` remaining budget at `n≤80`. Constant-level screen: **cannot** beat `B_{J,geom}` without changing the integrator.

### Edge-Green integrator (selected)

Inscribed regular `n_gon` ∩ Voronoi ∩ domain; Green edge forms for Gaussian mass/moments; composite trapezoid with explicit `M2` majorant (Wolfram max `|g''|≈12.53` at `σ=0.2` → majorant 15·(0.2/σ)⁴).

Defaults: `integration_method=edge_green`, `edge_n_gon=256`, `edge_panels=48`.

Constant-level (crude `B_H0=R²M_plane`):

| shape | `B_E` | `B_J_prior` | `B_J_geom` | beats |
|---|---|---|---|---|
| L | 0.0979 | 0.120 | 0.230 | yes |
| rectangle | 0.0733 | 0.091 | 0.180 | yes |
| C | 0.106 | 0.127 | 0.230 | yes |

Label fixes: geom comparator uses `M_plane` (not scipy.quad); mesh `φ_max` diagnostic only; `B_H0` certificate is `R²M_plane` only.

## Round 2 — stage-3 closed loop (in progress)

Target: `artifacts/apriori_closure_2026-09-15/` with L-shape seed2 and C-shape seed5, `edge_green`, frames=600.
