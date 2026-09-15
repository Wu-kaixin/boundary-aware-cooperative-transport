# A priori centroid-bound closure — 2026-09-15

## Status: **PARTIAL**

Static sampled coverage only. Not claimed: unknown-object transport or global convergence.

### Closed

- **Partition restrict lemma** (disjoint truncated Voronoi cells) — proved; Wolfram-checked plane tail.
- **Edge-Green cell integrator** with explicit disk–polygon and trapezoid remainders.
- **Label hygiene**: plane mass for geom/`B_H0`; mesh `φ_max` and `scipy.quad` not certificates.
- **Constant-level and closed-loop prior** \(B_{J,\mathrm{prior}} < M_{\mathrm{plane}} u_{\max}^2\) on the 9-case matrix below under `edge_green` (`n_gon=256`, `panels=48`).

### Not closed (blocks FULL CLOSED)

- **C-shape seed 5 \(\mathcal{K}_0\)**: after correcting margin clamp (`aᵀu≥b` orientation), **2 frames** (227–228) still have hard `zero_input_feasible=False` (agent 14). Margin-only frames cleared. Route B needs an *a priori* \(\|\Pi_F(0)\|\) bound; trajectory residuals are forbidden as priors. See `k0_diagnosis.json` and `artifacts/apriori_closure_2026-09-15_c5_clamp_v2/`.

### 9-case matrix (`edge_green`, grid label n=20, 600 steps)

| shape | seed | full-horizon K0 | priorJ | geom | prior beats geom |
|---|---|---|---|---|---|
| L | 2 | yes | 0.1129 | 0.2295 | yes |
| L | 5 | yes | 0.1123 | 0.2295 | yes |
| L | 8 | yes | 0.1146 | 0.2295 | yes |
| rectangle | 2 | yes | 0.0871 | 0.1803 | yes |
| rectangle | 5 | yes | 0.0875 | 0.1803 | yes |
| rectangle | 8 | yes | 0.0882 | 0.1803 | yes |
| C | 2 | yes | 0.1232 | 0.2295 | yes |
| C | 5 | **no** (5 zr in this matrix; 2 with clamp_v2) | 0.1235 | 0.2295 | constants yes / theorem inapplicable |
| C | 8 | yes | 0.1242 | 0.2295 | yes |

Measured \(\bar E \sim 10^{-7}\)–\(10^{-8}\). `gradient_residual_bar` / `centroid2_bar` in `summary.csv`. No aborts; no rho reduction; failures retained.

### Code versions

| commit | role |
|---|---|
| `a531515` | baseline sampled theorem_mode |
| `175fd67` | recovered prior-session snapshot |
| `6df60f1` | partition restrict + edge-Green |
| `14d81a2` | PARTIAL docs + clamp (wrong inequality once) |
| `830ea86` | clamp direction fix |

### Reproduce

```bat
artifacts\apriori_closure_2026-09-15\reproduce.cmd
```

or

```bash
python scripts/run_apriori_centroid_bound.py --out artifacts/apriori_closure_2026-09-15 \
  --shapes l_shape rectangle c_shape --seeds 2 5 8 --grids 20 --frames 600 \
  --integration-method edge_green --edge-n-gon 256 --edge-panels 48
python -m pytest tests/test_edge_green_and_restrict.py tests/test_apriori_centroid_bound.py tests/test_local_cvt.py -q
```

### Worth continuing?

Yes: E-budget line already beats geometry on all K0-valid runs. Next bottleneck is the **2 hard standstill-infeasible frames** on C-seed5 (design-level CBF / route-B residual), not further grid refinement.
