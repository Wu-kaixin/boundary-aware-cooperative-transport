# Audit findings — certificate repair 2026-09-15

Base commit: `7b9fb36d94ffba572881c11b8050d1d4cee2811d`.  
Worktree: `D:/boundary-aware-cooperative-transport-certificate-repair` on `feat/apriori-certificate-repair`.  
Old PARTIAL artifacts under `artifacts/apriori_closure_2026-09-15*` were **not** overwritten.

## A. Clipped long edges (reproduced)

`partition_edge_mass_moment_remainder` declared `ell = 2 R sin(π/n_gon) ≈ 0.01963446 m` as a uniform edge-length bound. `clip_cell_polygon` produces wall/Voronoi chords.

L-shape seed 2, actual `clip_cell_polygon`, n_gon=256, R=0.8:

* declared chord ≈ 0.01963 m;
* realised max edge ≈ 1.40 m (≤ 2R = 1.6 m);
* 16/16 cells have an edge longer than the chord.

Regression: `test_clipped_voronoi_edges_exceed_unclipped_chord`, `test_lshape_seed2_all_cells_have_long_clipped_edges`.

Fix: remainder uses diameter 2R, perimeter 2πR, frozen h_max=0.004 m, panels `ceil(L/h_max)` on each clipped edge. Trajectory max-edge is not a prior.

## B. Second-derivative majorant

Old `edge_second_derivative_majorant` used a finite-window NMaximize value ≈12.53 at σ=0.2, times 15/12.53, times (0.2/σ)^4. Scaling is wrong for both F1 (∼1/σ) and σ²k (∼1).

Fix: analytic bounds, all (x,y) and all unit directions, σ>0:

* |k''| ≤ 1/σ²;
* |(σ²k)''| ≤ 1;
* |F1''| ≤ (φ/√e + √(2π))/σ, φ=(1+√5)/2.

NMaximize is diagnostic only.

## C. Moment conversion, mixture φ_max, culling, centroids

* Robot-centred δμ includes ‖ξ−p‖|δm| with ‖ξ−p‖≤R+sσ after bbox culling.
* Weights enter as W=∑w_j≥0, not n_sources unit kernels.
* Continuous line φ_max is labelled not-a-discrete-mixture-bound; disk-gap uses min(φ0+W, φ0+n_e(δ+σ√(2π))).
* Restrict tail and cull tail are both charged (partition form, no N).
* Green centroids are projected onto B(p,R); non-positive mass falls back to the site; fallback is in B_E.
* Float remainder is separate from analytic truncation.

## D. Labels

Numerical H0 cannot issue `rigorous_on_K0`. README 0.087–0.124 from the PARTIAL package used that column and is **not** inherited. Analytic B_H0 candidates 0.091/0.120/0.127 depended on the invalid chord remainder and are also not inherited.

Repaired constant-level screen (K=600, crude H0 = R² M_plane):

| shape | B_J,prior | B_J,geom |
|---|---|---|
| rectangle | 0.1435 | 0.1803 |
| L | 0.1883 | 0.2295 |
| C | 0.1978 | 0.2295 |

## E. Margin clamp

Clamp still exists as an opt-in (`--clamp-margin`). Baseline is unclamped original aᵀu≥b. Original and effective RHS are dumped on K0 / dissipation anomalies. K0 uses the original ρ set: 0∉F vs F=∅ are distinct fields.

## F. Parallelism

16 physical / 16 logical CPUs, 31.5 GB, Python 3.14.6, OpenBLAS, BLAS threads=1 per worker. 8-frame L seeds 2&5: serial 27.8 s vs process pool 17.4 s (2 outer × 8 CVT threads), metrics identical. Stage C: 2×8, wall 1827 s. Stage D: 9×2, wall 1882 s (2 resumed). Stage E: 9×2, wall 2020 s. Did not revert to one worker while independent cases remained. Per-worker CPU ≈ 1.2 after polar observer/dumps dominate.

## G. Stage E K0 (validation)

L11 frames 238–240 agent 15 and L17 frames 318–322 agent 14 are all $0\in F_{\mathrm{hard}}$, $0\notin F_\rho$, $F\neq\emptyset$, single object row with $b>0$. Dumps: `stage_e_seeds_11_17_23/runs/l_shape/.../constraint_dumps/`. C5 remains the only case with $0\notin F_{\mathrm{hard}}$. Route B with $\|\Pi_F(0)\|\le u_{\max}$ is geometric-order and is the documented blocker.
