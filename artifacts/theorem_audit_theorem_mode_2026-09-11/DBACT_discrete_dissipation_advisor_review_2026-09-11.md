# DBACT sampled discrete dissipation — review note for the advisor

**Date:** 2026-09-11  
**Branch / baseline:** `feat/sampled-theorem-mode` on frozen `98ba28e` (not merged)  
**Artifacts:** `artifacts/theorem_audit_theorem_mode_2026-09-11/`  
**Derivation attachment:** `discrete_dissipation_derivation.md`  
**Delivery patch:** `delivery/feat_sampled_theorem_mode_vs_98ba28e.patch`  
(`sha256=29f93d8b5c4cb12394417e5e2a0ac5f0d6f4d88fdddb953a5c2efea1fa582691`, 231309 bytes; includes scripts/finalize_advisor_delivery.py + fixed three_command_hyp)

**One-line verdict.**  
We repaired the sample-and-hold execution interface and derived a discrete truncated-coverage inequality with mass-weighted command energy \(J\). Post-hoc numbers look informative, but we **do not** yet have an a priori centroid-error bound that turns the new estimate into a certified quantitative improvement over geometry. Hessian / N=16 local stability remains **unproved**. Static oracle runs are theory baselines, not a certificate for unknown-object transport.

---

## 1. Point-by-point responses

### A5 and hard switching

**Status unchanged in substance.** Assumption 5 (locally Lipschitz continuous-time feedback) is still incompatible with the endpoint-grid CVT path: hard disk / Voronoi membership on a finite \(20\times20\) lattice produces finite centroid jumps under arbitrarily small site moves (reproduced previously). Soft `clip` / ball projection alone do not remove that discontinuity.

**What changed.** The present branch does **not** claim that the production CVT now satisfies A5. It isolates a **different** theorem interface: static sample-and-hold cover with wall rows, no position clipping, abort-on-illegal, and Lipschitz-certified object hold clearance. Any paper rewrite must either (i) replace A5 with a sampled discrete regularity statement, or (ii) redesign the continuous cell integrals. We did not paper over A5.

### Why the original Theorem 1 bound lacks quantitative value

Under default static parameters the PDF asymptotic formula evaluates to \(\approx 1.90\times10^4\) for the gradient-squared residual, while a direct total-mass geometric ceiling is \(\approx 8.99\). Measured 30 s time averages on N=16 static oracle L-runs are \(\approx 0.011\)–\(0.013\). The formula is therefore **worse than a model-free geometry bound by ~\(2\times10^3\)**, and orders of magnitude above observations. Replacing the printed number by \(\min(\text{formula},\text{geometry})\) would hide the lack of information; it would not create theory.

### What the new discrete results do establish

1. **Proved (fixed \(\phi^\star\)):** frozen truncated-partition majorant ⇒  
   \(H^\star(P+\Delta u)-H^\star(P)\le \Delta {g^\star}^\top u+\Delta^2\sum m_i^\star\|u_i\|^2\)  
   (Lemma A).  
2. **Proved under \(a=2/k_c-\Delta>0\) and full-horizon \(\mathcal{K}_0\):** with per-robot saturation gains \(\lambda_i=\min(k_c,u_{\max}/\|\hat c_i-p_i\|)\) and final Euclidean QP projection, the finite-horizon mass-weighted bound  
   \(\bar J\le 2H^\star(P_0)/(a K\Delta)+4\bar E/a^2\)  
   holds as a mathematical identity when \(\bar E\) is left symbolic (Theorem C in the derivation).  
3. **Conditional:** \(\mathcal{K}_0\) requires \(0\in F_\rho\) (object rows **with** \(\rho\)) at **every** step of the summation window. C-channel seed 5 fails at frames 241–246 (agent 14); the full-horizon bound is marked **inapplicable** there — failing steps are not deleted to stitch a proof.  
4. **Post-hoc only:** substituting trajectory-measured \(\bar E\) into (★) is explanatory. Against \(M_{\mathrm{tot}}u_{\max}^2\), no resolution-dependent a priori \(E\) bound beats geometry.  
5. **\(J\) is not the original gradient residual.** Vanishing command speed does not imply vanishing \(g^\star\).

### N=16 Hessian: still no certificate

No Hessian / curvature / invariant-neighborhood certificate was computed or claimed. This remains an open item from the original review. Expanding that proof is **out of scope** for this round by design.

### Scope of unclipped sample-and-hold safety

Applicable to the **static sampled theorem_mode** path: wall half-planes, forbid fallback, hold-segment pairwise distance, Lipschitz lower bound on object clearance, abort if a hold map would leave \(D\).  

**Not claimed:** production multimodal transport with SEARCH/APPROACH/REDEPLOY, map registration under unknown motion, or that removing clipping alone fixes every legacy run. Coordinate clipping remains unsafe as a silent “protection”; the repaired path refuses illegal holds instead.

### A3 — same Gaussian kernel

Agree with the clarification request. The offset density and the continuous observer both use the same unnormalized Gaussian edge kernel \(K_\sigma\) with a single \(\sigma\) and nonnegative weights. The manuscript should write that interface explicitly (same \(K_\sigma\), same \(\sigma\), weight budget) so Lemma-type constants have a clear domain. This round’s static oracle experiments use that kernel throughout.

---

## 2. Quantitative summary (static oracle, N=16, 30 s)

| Block | Theorem-1 / analytic | Geometric | Measured |
|------|----------------------|-----------|----------|
| Gradient residual \(\|g^\star\|^2\) avg | formula \(\approx1.90\times10^4\) (**not a certificate**) | \(\approx8.99\) | \(\approx0.011\)–\(0.013\) (L) |
| Mass-weighted \(J\) | post-hoc (★) \(\approx0.029\)–\(0.032\) | \(M u_{\max}^2\approx0.230\) (preferred); cellwise \(\approx28\) | \(\bar J\approx0.011\) (L) |
| Centroid residual \(E\) | — | diameter \(4R^2M\approx4.80\) (too loose) | \(\bar E\sim10^{-4}\) (**post-hoc**) |

**A priori error bound sufficient to beat geometry?**  
**No.** Against the preferred geometric \(J\) ceiling \(M u_{\max}^2\), one needs \(\bar E\le E^\star\approx0.235\). The only model-only bound we have is the diameter bound \(\approx4.8>E^\star\). It is resolution-independent and does **not** control grid/oracle centroid error. We **refuse** to treat trajectory \(\max E\) as a uniform prior. Post-hoc explanation: measured \(\bar E\sim10^{-4}\) makes (★) look useful; that is explanatory, not certified.

**Nine static oracle cases** (L / rectangle / C-channel × seeds 2/5/8): **9/9 completed 30 s, 0 aborts.**  
Condition failures: C-channel seed 5 recorded 6 zero-input-with-\(\rho\) failures (4 margin-band steps). Inequality slacks remained OK at quadrature tolerance.  
**Label:** `static_oracle_theory_baseline` — **not** a certificate that unknown-object transport is proved.

**Three-command check (fixed):** each stage uses its own hypothetical next pose \(P+\Delta u\); on refinement \(H\), \(g\) **and** \(m\) are all recomputed on the same fine observer (the earlier code mixed a coarse-record \(m\) with fine \(H,g\)). Refined max slack \(O(10^{-6})\)–\(O(10^{-7})\) across all nine cases: numerical consistency only. `strict_numerical_certificate` is now **always `false`** (reason `no_rigorous_quadrature_error_bound`); three layers `coarse_verification` / `refined_verification` / `unresolved_quadrature_error` are recorded per stage.

### Final advisor delivery (`final_advisor_delivery/`)

The finite-time \(J\) bound with a **per-robot effective gain** \(\lambda_i=\min(k_c,u_{\max}/\|\hat c_i-p_i\|)\) is computed offline in `scripts/finalize_advisor_delivery.py`. Because \(\lambda_i\le k_c\), \(a_i=2/\lambda_i-\Delta\ge a=2/k_c-\Delta>0\), so saturation never breaks the sampling condition; per-step \(a_{\mathrm{eff},k}\) and \(\min_k a_{\mathrm{eff},k}\) are logged. The **full-horizon** bound (★) is applied only when *every* step is in \(\mathcal{K}_0\): **8/9 cases qualify**; **C-channel seed 5 is inapplicable** (frames 241–246, agent 14), with \(\bar J,\bar E\) reported only as `not_covered_by_theorem` descriptive stats. The primary geometric \(J\) ceiling is \(M_{\mathrm{tot}}u_{\max}^2\) (from `observer.total_mass`), not \(N m_+ u_{\max}^2\). Deliverables: `nine_case_summary.json`, `nine_case_comparison.csv`, `three_command_hyp/`, `j_bound/`, `figures/`, and a compiled report `latex/main.pdf` (includes the corrected derivation `latex/derivation.tex`).

---

## 3. Suggested paper revisions

1. **Split interfaces.** Continuous A5-style claims vs sampled hold + abort. Do not present the default controller as the continuous Theorem 1 plant.  
2. **Retire the coarse Theorem 1 formula as the main quantitative claim.** Report geometry alongside any analytic bound; if the analytic bound loses to geometry a priori, say so.  
3. **Introduce the discrete candidate as Theorem 1′ (sampled, fixed density)** with Lemma A, \(a>0\), and explicit \(\mathcal{K}_0\) conditioning for projection arguments.  
4. **Keep \(e_i=\hat c_i-c_i^\star\) and mass weights visible.** Separate \(J\) from \(\|g^\star\|^2\). Mark post-hoc substitutions.  
5. **Write A3 with a single \(K_\sigma\).**  
6. **State N=16 Hessian as open.** Do not imply local stability for the default swarm size.  
7. **Safety subsection:** wall rows + no clip + abort; object hold uses a Lipschitz clearance lower bound, not QP feasibility alone.

---

## 4. Open problems (honest list)

1. A **resolution-dependent a priori** bound on \(\|\hat c_{\mathrm{grid}}-c^\star\|\) tight enough that (★) beats \(M u_{\max}^2\) without trajectory \(E\). Current answer: **not available**.  
2. Closing dissipation for final QP commands **outside** \(\mathcal{K}_0\) without deleting \(\rho\).  
3. N=16 Hessian / conditional local stability certificate.  
4. Extending beyond static oracle to **unknown moving** objects with map error bounds.  
5. Replacing hard-grid CVT if a continuous A5 statement is still desired.

---

## 5. Reproduce (short)

```text
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src
python -m pytest tests/test_theorem_mode.py -q
python scripts/audit_JE_and_bounds.py --dissipation-dir <out>/n16_discrete_dissipation --out <out>/n16_JE_bounds
python scripts/run_multi_shape_static.py --seeds 2 5 8 --frames 600 --out <out>/n16_multi_shape
python scripts/export_delivery_bundle.py
```

Worktree: `E:\dbact-sampled-theorem-mode`. Live research tree left untouched; **do not merge** until reviewed.
