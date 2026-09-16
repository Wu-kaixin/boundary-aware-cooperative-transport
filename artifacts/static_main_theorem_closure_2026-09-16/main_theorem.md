# Main theorem (static sampled theorem_mode)

Status: **PARTIAL**. Object-hold invariance is proved for the **feature-cover** rows (`barrier_invariance_derivation.md`). Rigorous `B_J,prior` beats geometry on rectangle and L-shape, **not** on C-shape, after the legal reach/projection repairs. Finite K0 matrices are implementation checks, not the theorem domain.

Scope: N=16, frozen object, oracle map, single integrator + sample-and-hold, original `r_safe, ρ, u_max, d_min`. No dynamic transport, no global CVT stationarity, no N=16 Hessian.

## 0. Model and checkable hypotheses

Dynamics at sample `t_k = kΔ`, `Δ = 0.05`:

1. Inject oracle polyline of the frozen polygon `S`.
2. Nominal `u_nom,i = k_c (ĉ_i − p_i)`, speed-saturated to `u_max`.
3. QP: `U_k = argmin ‖u − u_sat‖` s.t. agent / wall / object rows and `‖u‖ ≤ u_max`.
4. Hold: `p_i(t_k+τ) = p_i(t_k) + τ U_{k,i}` for `τ ∈ [0,Δ]`. No coordinate clipping.

Object rows: `object_row_mode = nearest_feature`, implemented as Euclidean distance to every polygon **segment** in the one-step cover

`Φ(p) = { F : dist(p,F) ≤ sd(p)+2 u_max Δ } ∩ { dist ≤ object_row_range }`.

Infinite planes of non-nearest edges are forbidden. Reflex two-plane infinite AND is not used. `ρ` and `r_safe` are unchanged. When `v_obj=0` and `h≥ρ/γ`, `_cap_to_reachable` does not bind.

**P0 (checkable).** For every robot: `p_i ∈ D`, pairwise `‖p_i−p_j‖ ≥ d_min`, `sd(p_i,S) ≥ r_safe + ρ/γ_obj`. Oracle map. `γ_obj Δ ≤ 1`, `γ_agent Δ ≤ 1`, `a = 2/k_c − Δ > 0`.

## 1. Geometry / integral-error lemmas (inherited, repaired)

Partition restrict, clipped diameter/perimeter, analytic `M2`, frozen `h★ = 0.004 m`, robot-centred first moments: as in the 2026-09-15 certificate package, with two repairs:

1. **Source reach.** `dist(ξ, bbox(P)) ≤ sσ` and `P ⊂ B(p,R)` do **not** give `‖ξ−p‖ ≤ R+sσ`. Valid bound: `min(√2 R + sσ, R + 2 d_c + n_σ σ)` after `density.restrict`.
2. **Centroid projection.** Exact nonnegative integrals on `Ω ⊂ disk` have centroid in the disk. Trapezoid/float quadrature may leave the disk; the implementation projects. Extra charge `2R(‖δμ_quad‖+R|δm_quad|)` on trapezoid/float only.

Labels: exact-arithmetic algorithm vs implementation corollary vs numerical observer. Polar `H,g,m,c` stay `numerical_a_priori`. Certificate `B_H0 = R² M_plane`.

Recomputed priors (`priors_recomputed/`, legal reach):

| shape | `B_J,prior` (crude H, rigorous E) | `B_J,geom` | beats |
|---|---:|---:|---|
| rectangle | 0.1739 | 0.1803 | yes |
| L-shape | 0.2273 | 0.2295 | yes (narrow) |
| C-shape | 0.2367 | 0.2295 | **no** |

`E★` to meet geometry on C is 0.2273; active moment `B_E` is 0.2359. The projection extra on the trapezoid remainder is the dominant extra charge. Old C-shape `B_J≈0.1978` used invalid `R+sσ` and is withdrawn. `B_J` status `rigorous_on_K0` in the JSON is a **template** until K0 is the proved hypothesis of §3, not a property of the prior file itself.

## 2. Barrier-update and safety

Full write-up: `barrier_invariance_derivation.md`. Mechanism of the aggregate failure: `barrier_update_diagnosis.md`.

### Lemma A (fixed affine plane)

`h(p)=nᵀp−d−r_safe` fixed, `v_obj=0`, uncapped `nᵀu ≥ −γ h+ρ`, `p⁺=p+Δu`, `γ>0`, `0 ≤ γΔ ≤ 1`. Then `h(p⁺) ≥ (1−γΔ)h(p)+ρΔ`. If `h(p₀)≥ρ/γ` then `0 ∈ F_ρ` for this row.

### Lemma B (segment distance)

`h_F(p)=dist(p,F)−r_safe` is convex. Same discrete inequality along the whole hold, not only the endpoint.

### Lemma C (one-step cover)

Any edge nearest at some `q∈B(p,u_max Δ)` satisfies `dist(p,F)≤sd(p)+2 u_max Δ`, or the robot is out of `object_row_range` by more than `u_max Δ` (paper numbers: 0.60−0.0175 ≫ r_safe+ρ/γ).

### Lemma D (true sd on the hold)

Under cover rows on Φ(p): `h_true(p+τu) ≥ (1−γτ) h_true(p)+ρτ` for `τ∈[0,Δ]`. Continuity of sd is used only as `h_true=min_F h_F` outside S, not as a substitute for covering Φ. Aggregate-plane switches are a different `(n,d)` jump (C5/L11/L17) and are not this lemma.

### Lemma E (monitor)

`hold_segment_object_clearance` remains a runtime abort. It is **not** required for Lemma D.

### Lemma F (robots)

Shared-responsibility `h_ij = ‖p_i−p_j‖² − d_min²`. `h_ij⁺ ≥ (1−γ_agent Δ) h_ij + Δ²‖u_i−u_j‖²`. Hold-segment minimum of two linear motions is exact. Omitted pairs: `‖p_i−p_j‖ > d_min + 2 u_max Δ`.

### Lemma G (walls)

Four linear rows `p+Δu ∈ D`, `D` a closed rectangle, hence the hold segment stays in `D`.

## 3. Joint recursive feasibility

`0 ∈ F_ρ` iff every stacked row has `b ≤ 0`:

- agent: `h_ij ≥ 0`;
- wall: `p ∈ D`;
- object: `h_F ≥ ρ/γ` on every covered edge, equivalent to `h_true ≥ ρ/γ`;
- cap does not raise `b` (Lemma D hypotheses).

P0 + Lemmas D–G keep these at every sample. The feasible set is a nonempty closed convex subset of the speed ball (it contains 0), so the Euclidean projection QP attains a minimizer. `forbid_fallback` never fires on the proved set. **Single halfspace ∩ ball nonempty does not replace this conjunction.**

Stage C/D empty K0-failure sets are implementation consistency of this proposition, not its proof.

## 4. Discrete dissipation (unchanged cascade)

Identity (★) of `discrete_dissipation_derivation.md` on the exact sat→QP cascade, **full window in K0**:

`J̄ ≤ 2 H*(P0)/(a K Δ) + 4 Ē / a²`,

with `H*` replaced by `B_H0=R²M_plane` and `Ē` by `B_E` for the prior. K0 is the cover `F_ρ`, not the aggregate-plane `F_ρ`. Route B (`‖Π_F(0)‖ ≤ u_max`) is not used.

Polar observer remainders stay numerical. Dissipation slacks `~10^{-5}` are not theorem counterexamples.

## 5. Main statement (declared scope)

**Theorem (static sampled feature-cover CBF-QP).**
Assume P0 and the parameter conditions of §6. Under the executed cascade of §0:

1. **Safety.** At every sample and along every hold, `sd(p_i,S) ≥ r_safe`, `‖p_i−p_j‖ ≥ d_min`, `p_i ∈ D`, or the case aborts (no unsafe continuation). Object interval safety is Lemma D, not the monitor.
2. **Recursive feasibility.** `0 ∈ F_ρ` at every sample, QP attains `optimal`, no fallback.
3. **A priori performance.** If (2) holds on `{0,…,K−1}`, then (★) with `B_E` from §1 and `B_H0 = R² M_plane`. Comparison with `M_plane u_max²` holds for rectangle and L-shape, **fails for C-shape** at the rigorous level (table in §1).
4. **Not claimed.** Global convergence, coverage stationarity, unknown-object transport, `J` small ⇒ coverage success.

## 6. Parameter conditions

`Δ=0.05`, `k_c=0.9`, `a>0`, `γ_obj=8`, `γ_agent=6`, `ρ=0.02`, `r_safe=0.065`, `d_min=0.28`, `u_max=0.35`, `R=0.8`, `σ=0.2`, `d_c=0.105`, oracle `δ=0.04`, `h★=0.004`, `n_gon=256`, cull `s=6`, restrict `n_σ=3`, N=16, K=600, `object_row_range=0.60`. `γΔ≤1` both families. Cover radius `2 u_max Δ = 0.035 m`.

## 7. Experiments vs proof

| Item | Role |
|---|---|
| 15-Sep 18-case aggregate matrix | Documents K0 failure of infinite-plane aggregate |
| This package diagnosis | Mechanism: 90° convex-corner plane switch |
| Stage C/D single-nearest | Implementation: C5 mechanism gone; **not** the proved cover |
| Stage C/D feature-cover | Implementation consistency of Lemmas D–G; 18/18 empty K0-fail |
| Seeds 29/31/37 | Frozen-method independent check; 9/9 empty K0-fail; unused for design |

Finite success does not define the theorem's domain. The domain is P0 + §6.

## Closure tag

`PARTIAL`.

Remaining for CLOSED_FOR_DECLARED_SCOPE (static sampled, excluding C-shape quantitative J-advantage): cover Stage C/D + independent seeds logged; serial/parallel consistency log; `main_theorem.tex` aligned; dissipation anomalies labelled numerical.

Remaining for the original CLOSED target: a legal tightening of C-shape `B_E` below `E★=0.2273` without invalid reach, clamp, or ρ change; or an honest scope that drops C-shape from the J-vs-geometry claim.
