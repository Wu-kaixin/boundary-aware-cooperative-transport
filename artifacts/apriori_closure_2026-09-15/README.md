# A priori centroid-bound closure — 2026-09-15

## Status: **PARTIAL**

The mass-weighted centroid-error budget is closed for static sampled coverage under
`integration_method=edge_green` with declared constants
(`edge_n_gon=256`, `edge_panels=48`), and the resulting prior \(B_{J}\) **strictly
beats** \(M_{\mathrm{plane}} u_{\max}^2\) whenever the full horizon lies in
\(\mathcal{K}_0\).

Full **CLOSED** is blocked by C-shape seed 5: five frames leave \(\mathcal{K}_0\)
(agent 14; mix of hard zero-input infeasibility and rho-margin infeasibility).
Those steps are retained; \(\rho\) is not reduced; failures are not deleted.

### What is proved

- Partition restrict tail (disjoint \(\Omega_i\)): see `theorem_and_proof.md`.
- Oracle midpoint \(L^1\) floor; plane mass \(M_{\mathrm{plane}}\); \(B_{H0}=R^2 M_{\mathrm{plane}}\).
- Edge-Green geometric + trapezoid remainders with explicit \(M_2\) majorant.
- Moment-form \(B_E\) and \(B_{J,\mathrm{prior}}\) under full-horizon \(\mathcal{K}_0\).

### What is numerical / open

- Polar observer remainder (diagnostic only).
- Mesh \(\phi_{\max}\) (diagnostic only; not a global upper bound).
- `scipy.quad` observer mass (not used in the geometric comparator).
- C-shape seed 5 \(\mathcal{K}_0\) gap: margin clamp (route C, partial) implemented as
  `theorem_clamp_margin_to_keep_zero`; hard-barrier frames still need route B
  with a design-level bound on \(\|\Pi_F(0)\|\) (not available from trajectory
  residuals).

### Stage-3 evidence (`edge_green`, n_gon=256, panels=48, frames=600)

| case | complete | full-horizon K0 | priorJ | geom | prior beats geom |
|---|---|---|---|---|---|
| L seed2 | yes | yes | 0.113 | 0.230 | yes |
| L seed5 | yes | yes | 0.112 | 0.230 | yes |
| C seed2 | yes | yes | 0.123 | 0.230 | yes |
| C seed5 | yes | **no** (5 frames) | 0.123 | 0.230 | yes (constants); theorem **inapplicable** |

Measured \(\bar E\sim 10^{-7}\)–\(10^{-8}\); coverage residuals / centroid² reported in summaries.

### Reproduce

```bash
python scripts/run_apriori_centroid_bound.py \
  --out artifacts/apriori_closure_2026-09-15 \
  --shapes l_shape rectangle c_shape \
  --seeds 2 5 8 \
  --grids 20 \
  --frames 600 \
  --integration-method edge_green \
  --edge-n-gon 256 \
  --edge-panels 48
```

Code commits: `175fd67` (recovery snapshot), `6df60f1` (partition restrict + edge-Green).

### Worth continuing?

Yes for the E-budget / integrator line — it already beats geometry on K0-valid runs.
K0 for C-seed5 needs either a proveable control change that keeps the hard barrier
satisfied at standstill, or a new dissipation inequality with an *a priori*
\(\|\Pi_F(0)\|\) majorant. Do not treat trajectory residuals as priors.
