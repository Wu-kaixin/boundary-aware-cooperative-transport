# Theorem and proof — a priori centroid-error bound (static sampled coverage)

Status of this note: **mathematical claims below are proved under the stated hypotheses**. Closed-loop validation is recorded separately in `iteration_log.md` / `summary.csv`.

## 0. Scope

Static sampled `theorem_mode`, frozen object, oracle boundary map, uniform cage offset, sample-and-hold
`P_{k+1}=P_k+Δ u_k`. Not claimed: unknown-object transport, Hessian/local stability, research-v3 certificate mode.

## 1. Definitions

Controller density (offset mixture):
\[
\phi_{\mathrm{ctrl}}(q)=\phi_0+\sum_j w_j\,k(q,\xi_j),\quad
k(q,\xi)=\exp\!\big(-\|q-\xi\|^2/(2\sigma^2)\big),\quad
\xi_j=b_j+d_j n_j.
\]
Observer density `φ★` is the continuous uniform-offset edge measure (erf line integrals).

Truncated Voronoi cells `Ω_i = V_i ∩ B(p_i,R) ∩ D` (pairwise disjoint up to null sets).

Mass-weighted indicators (observer mass `m★`, controller centroid `ĉ`, observer centroid `c★`):
\[
J_k=\sum_i m_{i,k}^★\|u_{i,k}\|^2,\qquad
E_k=\sum_i m_{i,k}^★\|\hat c_{i,k}-c_{i,k}^★\|^2,
\]
\[
a=2/k_c-Δ>0.
\]

## 2. Discrete dissipation identity (imported)

Under Theorem C of `discrete_dissipation_derivation.md` (exact cascade, full window in `K₀`):
\[
\bar J_K\le \frac{2H^★(P_0)}{aKΔ}+\frac{4\bar E_K}{a^2}.\tag{★}
\]
`K₀`: every agent QP status is `optimal`, every `zero_input_feasible_with_rho` is true, and `a>0`. Missing flags fail closed.

## 3. Moment form for `E` (no `1/m₋`)

Both centroids lie in `B(p_i,R)`, so `\|e_i\|≤2R` and
\[
m_i^★\|e_i\|^2\le 2R\,\|m_i^★ e_i\|,\qquad
m_i^★ e_i=(\hatμ_i'-μ_i^{★'})-\hat c_i'(\hat m_i-m_i^★).
\]
Hence
\[
E_k\le 2R\Big(\sum_i\|\deltaμ_i'\|+R\sum_i|\delta m_i|\Big).\tag{M}
\]
Also `E_k≤4R² M` with `M≥∑ m_i^★` (diameter). Certificate uses the minimum.

## 4. Lemma (partition restrict tail)

**Hypotheses.** Weights `w_j≥0`, `W=∑w_j`. Restrict drops `b_j` when
`\|b_j-p_i\|>R+\max_j d_j+sσ`. Then on `q∈B(p_i,R)`, every dropped kernel obeys `\|q-ξ_j\|≥sσ`.
Cells `Ω_i` are pairwise disjoint.

**Claim.**
\[
\sum_i\int_{Ω_i}|\phi_{\mathrm{full}}-\phi_{\mathrm{restrict},i}|\,dq
\le 2\pi\sigma^2 e^{-s^2/2}\,W.
\]

**Proof.** The integrand equals `∑_{j∈Drop_i} w_j k(q,ξ_j)`. Switching sums,
\[
\sum_i\int_{Ω_i}\sum_{j∈Drop_i}w_j k
=\sum_j w_j\sum_{i:\,j∈Drop_i}\int_{Ω_i}k(q,ξ_j)\,dq.
\]
For fixed `j`, on every cell that drops `j` one has `Ω_i⊂{\|q-ξ_j\|≥sσ}`, and those cells are disjoint, so
\[
\sum_{i:\,j∈Drop_i}\int_{Ω_i}k\le\int_{\|u\|≥sσ}e^{-\|u\|^2/(2σ^2)}\,du
=2\pi\sigma^2 e^{-s^2/2},
\]
where the last identity is the polar Gaussian integral (Wolfram). Multiply by `w_j` and sum. □

**Code match.** `boundary_density.restrict` uses raw `points` and `reach = radius + max(offsets) + influence_sigmas·σ`.

## 5. Oracle midpoint `L¹` floor

On each edge panel of length `≤δ`, midpoint quadrature matches the first along-edge moment. With
`∫|∂²k/∂s²|dA=4√(2π/e)` (Wolfram),
\[
\|\phi_{\mathrm{ctrl}}-\phi^★\|_{L^1(ℝ²)}\le \sqrt{2π/e}\,L\,δ²/6.
\]
Covered cells partition a subset of `D`, so this bounds `∑|δm_den,i|` and `∑‖δμ_den,i‖≤R` times the same.

## 6. Edge-Green cell integrator

Replace `Ω_i` by an inscribed regular `n`-gon `P_i` clipped by Voronoi half-planes and `D`.

**Geometric error.** `Area(B(p,R)\P_n)=πR²-(n/2)R²\sin(2π/n)`. Missing sets inside the disjoint cells satisfy
\[
\sum_i\mathrm{Area}(Ω_i\P_i)\le N\cdot\mathrm{Area}(B\P_n).
\]
Charge with the proved pointwise majorant `φ≤φ₀+n_{\mathrm{edges}}σ√(2π)` (each infinite line contributes at most `σ√(2π)`).

**Exact structure.** For `k`, Green’s theorem with
\[
F_1(x,y)=σ\sqrt{2π}\,\tfrac12\big(1+\mathrm{erf}(x/(σ√2))\big)\,e^{-y²/(2σ²)}
\]
(`∂F₁/∂x=k`, Wolfram) yields `∫_P k=\oint F₁\,dy`. Moments use `∮ -σ² k\,dy` / `∮ σ² k\,dx`.

**Edge trapezoid remainder.** Composite trapezoid with `m` panels on an edge of length `≤ℓ` has
`|E|≤ℓ³ M₂/(12 m²)`. Majorant `M₂=15(0.2/σ)⁴` (Wolfram max `|g''|≈12.53` at `σ=0.2`, safety factor). Sum over agents × edges × sources for mass and two moment components.

## 7. Assembled prior

\[
B_E=\min\Big(2R\big(∑\|\deltaμ\|+R∑|\delta m|\big),\;4R² M_{\mathrm{plane}}\Big),
\]
\[
M_{\mathrm{plane}}=φ₀|D|+2πσ² L,\qquad
B_{H0}=R² M_{\mathrm{plane}},
\]
\[
B_{J,\mathrm{prior}}=\frac{2B_{H0}}{aKΔ}+\frac{4B_E}{a²},\qquad
B_{J,\mathrm{geom}}=M_{\mathrm{plane}} u_{\max}².
\]

Error sources in `∑|δm|`: oracle midpoint + partition restrict + disk–polygon geometry + edge trapezoid.
Polar observer remainder is **excluded** from the rigorous budget (no enclosure yet).

## 8. Label policy

| quantity | label |
|---|---|
| partition restrict, oracle midpoint, plane mass, edge-Green remainders as above | `rigorous` |
| `H(P₀)` + polar envelope, scipy.quad mass, mesh `φ_max` | `numerical_a_priori` / diagnostic — **not** certificate inputs |
| trajectory `Ē` | `post_hoc` |

## 9. Open items

1. Full-horizon `K₀` for C-shape seed5 (rho zero-input failures) — not closed by the E bound alone.
2. Polar quadrature enclosure for the observer (currently diagnostic only).
3. Independent seed batch (11,17,23) after method freeze.
