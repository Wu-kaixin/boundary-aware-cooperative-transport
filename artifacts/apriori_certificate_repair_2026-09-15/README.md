# A priori certificate repair — 2026-09-15

## Status: **PARTIAL**

Static sampled theorem_mode only. Not claimed: unknown-object transport, global convergence, N=16 Hessian.

### Proved in this package

- Partition restrict lemma (inherited, disjoint truncated cells).
- Analytic M2 for Green edge integrands F1 and σ²k (all positions, all unit directions, σ>0).
- Clipped-edge trapezoid remainder using diameter 2R, perimeter 2πR, frozen h★=0.004 m, and W=∑ w_j ≥ 0. The unclipped chord 2R sin(π/n) is **not** a bound (reproduced: L-shape seed 2, 16/16 cells, max edge ≈ 1.40 m vs chord ≈ 0.0196 m).
- Robot-centred first-moment conversion ‖δμ_p‖ ≤ ‖δμ_ξ‖ + ‖ξ−p‖ |δm| with ‖ξ−p‖ ≤ R+sσ after culling.
- Discrete-mixture φ_max, separate float vs analytic truncation, disk projection of Green centroids.
- Constant-level B_J,prior < M_plane u_max² on rectangle / L / C with the repaired remainder (not the PARTIAL 2026-09-15 numbers).

### Not closed

- **C-shape seed 5 K0**: frames 228–232, agent 14, QP optimal (F nonempty).
  - 228–229: 0 ∉ F_hard (object row b>0 even without ρ).
  - 230–232: 0 ∈ F_hard but 0 ∉ F_ρ (margin-only).
  - Unclamped baseline. Clamp would hide 230–232 only; it is **not** treated as preserving original ρ safety.
  - Route B with ‖Π_F(0)‖ ≤ u_max returns a geometric-order bound and **erases the prior's advantage**. A tighter prior needs a lower bound on the object CBF h that is not available without an unproved invariant. Measured b or ‖u‖ are forbidden as priors.
- Dissipation slacks ~10^{-5} on many in-K0 frames. They are **not** promoted to theorem counterexamples; they also lack a certified observer envelope that would settle the sign. Polar H,g,m,c remain numerical.
- Sample-and-hold safety of the executed QP is recorded (9600/9600 optimal on Stage C); that is not coverage of the dissipation identity.

### 9-case development matrix (seeds 2/5/8, unclamped, 600 steps)

| shape | seed | K0 | J_bar | E_bar | B_J,prior | geom |
|---|---|---|---|---|---|---|
| L | 2 | yes | 0.01132 | 1.09e-7 | 0.1883 | 0.2295 |
| L | 5 | yes | 0.01077 | 4.97e-8 | 0.1883 | 0.2295 |
| L | 8 | yes | 0.01153 | 6.83e-8 | 0.1883 | 0.2295 |
| rectangle | 2 | yes | 0.00892 | 1.16e-7 | 0.1435 | 0.1803 |
| rectangle | 5 | yes | 0.00910 | 2.48e-8 | 0.1435 | 0.1803 |
| rectangle | 8 | yes | 0.00922 | 4.25e-8 | 0.1435 | 0.1803 |
| C | 2 | yes | 0.01197 | 1.68e-7 | 0.1978 | 0.2295 |
| C | 5 | **no** (228–232, agent 14) | 0.01233 | 4.51e-8 | 0.1978 | 0.2295 |
| C | 8 | yes | 0.01244 | 6.91e-8 | 0.1978 | 0.2295 |

### Validation seeds 11/17/23 (not used to choose h_max or M2)

| shape | seed | K0 | J_bar | failure |
|---|---|---|---|---|
| L | 11 | **no** | 0.01133 | 238–240, agent 15, margin-only |
| L | 17 | **no** | 0.01086 | 318–322, agent 14, margin-only |
| L | 23 | yes | 0.01029 | — |
| rectangle | 11 | yes | 0.00935 | — |
| rectangle | 17 | yes | 0.00870 | — |
| rectangle | 23 | yes | 0.00855 | — |
| C | 11 | yes | 0.01255 | — |
| C | 17 | yes | 0.01199 | — |
| C | 23 | yes | 0.01178 | — |

All 9600 QP solves per 600-step case were `optimal`. No aborts. K0 failure is therefore not unique to C-shape seed 5. L11/L17 are $0\notin F_\rho$ with $F$ nonempty and the hard barrier still admitting 0. C5 additionally has two frames with $0\notin F_{\mathrm{hard}}$.

### Blocking condition (why this is not CLOSED)

Identity (★) requires the full window in K0: every QP optimal **and** 0 ∈ F_ρ. That fails on C5 (dev) and L11/L17 (validation). Route B with the only a priori envelope ‖Π_F(0)‖ ≤ u_max recovers a bound of order M u_max² and **cancels the prior's advantage over geometry**. A tighter envelope would need a uniform lower bound on the object CBF h; that is an unproved invariant, and trajectory-measured b or ‖u‖ are forbidden as priors. Clamp hides some margin-only frames by relaxing ρ and is not the original-safety baseline.

Remaining closed-loop safety of the executed QP (all solves optimal, no abort) does not cover (★).

### Labels

Numerical H(P0) is `numerical_a_priori` and **cannot** issue `rigorous_on_K0`. Old README 0.087–0.124 used that column. Old analytic 0.091/0.120/0.127 used the invalid chord remainder. Neither is inherited.

### Parallelism

Intel Ultra 9 285H, 16 physical / 16 logical CPUs, 31.5 GB, Python 3.14.6, OpenBLAS, BLAS threads = 1 per worker.

8-frame L seeds 2 and 5: serial 27.8 s vs 2 processes × 8 CVT threads 17.4 s (speedup 1.60). J, E, and K0 identical.

Stage C: 2 processes × 8 CVT threads, wall 1827 s. Per-worker CPU ≈ 1.2 cores — the polar observer and constraint dumps dominate after CVT. The runner did not fall back to a single worker while independent cases remained.

### Reproduce

```bat
artifacts\apriori_certificate_repair_2026-09-15\reproduce.cmd
```

```bash
python scripts/run_apriori_certificate_repair.py --resume
python -m pytest tests/test_edge_green_and_restrict.py tests/test_apriori_centroid_bound.py tests/test_local_cvt.py tests/test_safety_filter.py -q
```

`--resume` reuses a case only if `COMPLETE.json` matches the source and prior-rule hashes. Partial runs are not marked complete.
