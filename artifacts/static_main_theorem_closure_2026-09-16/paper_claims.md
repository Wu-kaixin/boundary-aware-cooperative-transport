# Paper claims — static sampled theorem_mode

Status of this package: **PARTIAL**. Finite experiments do not define the theorem domain.

## Proved (mathematical, matching the executed cascade)

1. **Fixed affine discrete CBF.** For a frozen plane, uncapped `nᵀu ≥ −γh+ρ`, `p⁺=p+Δu`, `0≤γΔ≤1`: `h(p⁺)≥(1−γΔ)h+ρΔ`. If `h(p₀)≥ρ/γ` then that row keeps `0∈F_ρ`.
2. **Convex feature.** Euclidean distance to a segment is convex. The same discrete inequality holds for `h_F=dist(p,F)−r_safe` with `n_F=∇dist`.
3. **One-step cover.** Any edge that can become nearest during a hold of length `Δ` at speed `≤u_max` satisfies `dist(p,F)≤sd(p)+2 u_max Δ`.
4. **Object interval invariance (cover).** Under `nearest_feature` rows on Φ(p), frozen object, `v_obj=0`, uncapped CBF (cap inactive when `h≥ρ/γ`), `h_true(p+τu)≥(1−γτ)h_true(p)+ρτ` for `τ∈[0,Δ]`. Hence `{h_true≥ρ/γ}` is invariant along the hold, and `0∈F_ρ^{object}` iff `h_true≥ρ/γ`.
5. **Infinite-plane aggregate is not this barrier.** C5/L11/L17: 90° convex-corner switch of a supporting plane of the hull; `h_plane` jumps negative while `sd` stays large. Diagnosis, not a safety certificate of aggregate.
6. **Agents.** Shared-responsibility `h_ij=‖p_i−p_j‖²−d_min²`; omitted pairs farther than `d_min+2 u_max Δ`; hold-segment minimum of two linear motions is exact. Walls: rectangle, `P+ΔU∈D` implies the hold stays in `D`.
7. **Cap inactivity.** `v_obj=0` and `h≥ρ/γ` ⇒ uncapped object RHS `≤0` ⇒ `_cap_to_reachable` does not bind.
8. **Integral certificates (repaired).** `dist(ξ,bbox(P))≤sσ` and `P⊂B(p,R)` do **not** give `‖ξ−p‖≤R+sσ`. Legal reach: `min(√2 R+sσ, R+2d_c+n_σ σ)`. Projection extra `2R(‖δμ_quad‖+R|δm_quad|)` on trapezoid/float only. Analytic `M2`, frozen `h★=0.004`, partition restrict tail, robot-centred moments.
9. **Dissipation identity (★)** on the sat→QP cascade, **conditional on full-horizon K0** and `a=2/k_c−Δ>0`. Route B `‖Π_F(0)‖≤u_max` is not used.

## Supported only by experiment (not a proof)

1. Stage C/D K0 emptiness for **single-nearest** (before cover) on the 18-seed matrix, 9600/9600 `optimal`. Implementation evidence that the C5 mechanism is gone; not interval invariance.
2. Cover closed-loop K0: Stage C 4/4, Stage D 18/18, Stage E 9/9 (seeds 29/31/37, pre-declared, unused for design). All 9600/9600 `optimal`, empty K0-failure sets. Implementation consistency, not the domain of the theorem.
3. Polar observer `H,g,m,c` at P0 vs a finer grid: numerical error assessment, **not** a quadrature certificate.
4. Dissipation slacks `~10^{-5}` on in-K0 frames: not a counterexample and not a verification of (★).

## Must not be claimed

1. Global convergence, coverage stationarity, or “J small ⇒ coverage success”.
2. Unknown-object transport, SEARCH/APPROACH/TRANSPORT phases, or moving cargo.
3. N=16 Hessian / local CVT uniqueness.
4. Original aggregate-plane `0∈F_ρ` on C5/L11/L17.
5. Unconditional discrete dissipation without K0.
6. `B_J,prior` beating `M u_max²` **on C-shape** at the **rigorous** (crude-H, legal reach, projection extra) level: after repairing `R+sσ`, C-shape has `B_J,prior≈0.2367 > B_J,geom≈0.2295`. Rectangle (`0.1739<0.1803`) and L-shape (`0.2273<0.2295`) still beat. Old C-shape `≈0.1978` used an invalid reach and is withdrawn.
7. `rigorous_on_K0` as a property of a prior JSON written before K0 is proved. Priors are geometry/P0 constants; K0 is a separate hypothesis.
8. Clamp, ρ reduction, or RHS truncation as a feasibility proof.
9. Treating polar `H*(P0)` as a rigorous `B_H0`. Certificate `B_H0` is `R² M_plane`.
