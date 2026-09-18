> 精确算术下的静态采样安全和先验界在所声明的 oracle-map 范围内闭合；控制残差与有意义部署驻点的联系、局部未知边界接口以及部署对搬运的作用仍是当前论文需要补齐的内容。

# Discrete dissipation candidate — corrected derivation

Branch: `feat/sampled-theorem-mode` (frozen baseline `98ba28e`)  
Date: 2026-09-11 (revised)  
Scope: fixed reference density \(\phi^\star\), sample-and-hold \(P_{k+1}=P_k+\Delta\,u_k\), truncated coverage cost \(H^\star\).

**Notation (no overloaded symbols).**  
Sites \(P=(p_1,\ldots,p_N)\). Commands \(u=(u_1,\ldots,u_N)\).  
Uncovered region \(\mathcal{U}(P)=D\setminus\bigcup_i\Omega_i(P)\).  
Never use \(U\) for both the command and the uncovered set.

**Status labels used below.**  
- **Proved:** mathematical under stated hypotheses.  
- **Conditional:** proved only on execution intervals where named conditions hold; not inferred globally from finite runs.  
- **Post-hoc numerical:** trajectory evaluation / quadrature.  
- **Strict numerical certificate:** not claimed in this delivery (refined slacks remain \(O(10^{-6})\)–\(O(10^{-7})\), quadrature-limited).

---

## Lemma A (Frozen truncated partition majorant)

Fix \(\phi^\star\ge0\) on domain \(D\), radius \(R>0\). Truncated cells
\[
\Omega_i(P)=V_i(P)\cap B(p_i,R),\qquad
\mathcal{U}(P)=D\setminus\bigcup_i\Omega_i(P),
\]
and
\[
H^\star(P)=\sum_i\int_{\Omega_i(P)}\|q-p_i\|^2\phi^\star(q)\,dq
+R^2\int_{\mathcal{U}(P)}\phi^\star(q)\,dq.
\]
Mass and centroid:
\[
m_i^\star(P)=\int_{\Omega_i(P)}\phi^\star,\qquad
c_i^\star(P)=\frac1{m_i^\star}\int_{\Omega_i(P)}q\,\phi^\star\quad(m_i^\star>0).
\]
Gradient convention (observer-compatible):
\[
g_i^\star(P)=2\,m_i^\star(P)\,(p_i-c_i^\star(P)).
\]

At step \(k\), freeze \(\Omega_{i,k}=\Omega_i(P_k)\) and \(\mathcal{U}_k=\mathcal{U}(P_k)\). For trial sites \(Q=(q_1,\ldots,q_N)\) define
\[
\widehat H_k(Q)
=\sum_i\int_{\Omega_{i,k}}\|q-q_i\|^2\phi^\star
+R^2\int_{\mathcal{U}_k}\phi^\star.
\]

Then:

1. \(\widehat H_k(P_k)=H^\star(P_k)\).  
2. \(H^\star(Q)\le\widehat H_k(Q)\) for every \(Q\) (re-partitioning can only reduce the cost; uncovered mass is a \(Q\)-independent constant in \(\widehat H_k\)).  
3. With \(Q=P_k+\Delta u\),
   \[
   \widehat H_k(P_k+\Delta u)-\widehat H_k(P_k)
   =\Delta\,{g_k^\star}^\top u
   +\Delta^2\sum_i m_{i,k}^\star\|u_i\|^2.
   \]

**Corollary (discrete inequality — proved).**
\[
H^\star(P_k+\Delta u)-H^\star(P_k)
\le
\Delta\,{g_k^\star}^\top u
+\Delta^2\sum_i m_{i,k}^\star\|u_i\|^2.
\]
Dissipation of \(H^\star\) is a consequence when the right-hand side is nonpositive; it is not an assumption used in the proof.

---

## Sampling-period condition

Let \(k_c>0\) be the CVT gain and \(\Delta>0\) the hold period. Define
\[
a:=\frac{2}{k_c}-\Delta.
\]
**Standing requirement:** \(a>0\), i.e. \(\Delta<2/k_c\).  
For paper defaults \(k_c=0.9\), \(\Delta=0.05\), one has \(a=2/0.9-0.05\approx2.172>0\).

This condition appears in the exact-centroid dissipation pairing and in the approximate-centroid finite-horizon \(J\) bound. It is independent of zero-input feasibility.

### Per-robot effective gain \(\lambda_i\)

With \(d_i=\hat c_i-p_i\) (controller grid centroid minus site) and Euclidean-ball saturation, the executed command is parallel to \(d_i\) with an effective gain
\[
\lambda_i=\min\!\Big(k_c,\ \frac{u_{\max}}{\|d_i\|}\Big)\quad(\|d_i\|>\varepsilon),\qquad \lambda_i=k_c\ \text{(idle)} .
\]
Since \(\lambda_i\le k_c\), the per-robot slack \(a_i=2/\lambda_i-\Delta\ge a>0\); hence \(a_{\mathrm{eff}}=\min_i a_i\ge a\) and saturation never breaks the sampling condition. For diagnostics we also record \(\lambda_i^{\mathrm{sat}}=(u_i^{\mathrm{sat}}\cdot d_i)/\|d_i\|^2\). Per-step \(a_{\mathrm{eff},k}=\min_i(2/\lambda_{i,k}-\Delta)\) and \(\min_k a_{\mathrm{eff},k}\) are logged in `final_advisor_delivery/j_bound/*.json`.

---

## Zero-input feasibility (conditional; not global)

Let \(F_{\rho,i,k}\) be the final planar QP feasible set for agent \(i\) at step \(k\) (agent + wall + object rows **including** ISSf margin \(\rho\), and \(\|u\|\le u_{\max}\)).

- Barrier certificate flag in code (`zero_input_feasible`) uses the **margin-free** object RHS.  
- Final-set feasibility of \(u_i=0\) is the distinct predicate \(0\in F_{\rho,i,k}\) (`zero_input_feasible_with_rho`).

**Do not** promote “\(0\in F_\rho\) on seeds 2/5/8 of one L-shape run” to a global theorem hypothesis.  
On the C-channel seed 5 run, six agent-steps violate \(0\in F_\rho\) (margin band).  

**Rule used in statements below:** any claim that invokes projection optimality against \(v=0\) is valid only on the execution subset
\[
\mathcal{K}_{0}:=\{k:\ \text{status}_i=\texttt{optimal}\ \text{and}\ 0\in F_{\rho,i,k}\ \forall i\}.
\]
Outside \(\mathcal{K}_0\), state the claim as **inapplicable** (condition failure), without deleting \(\rho\) rows.

---

## Theorem B (Exact-centroid dissipation on \(\mathcal{K}_0\))

**Hypotheses.** Fixed \(\phi^\star\); Lemma A; \(a>0\); \(\hat c_i=c_i^\star\); CVT \(u_i^{\mathrm{cvt}}=k_c(c_i^\star-p_i)\); ball saturation \(u_i^{\mathrm{sat}}=\lambda_i(c_i^\star-p_i)\); final \(u_i=\Pi_{F_{\rho,i,k}}(u_i^{\mathrm{sat}})\) with \(k\in\mathcal{K}_0\).

Then \({g_k^\star}^\top u^{\mathrm{sat}}=-\sum_i (2/\lambda_i)\,m_i^\star\|u_i^{\mathrm{sat}}\|^2\le0\). Projection optimality with \(0\in F_\rho\) yields \((p_i-c_i^\star)^\top u_i\le-\|u_i\|^2/\lambda_i\), hence
\[
{g_k^\star}^\top u\le-\sum_i\frac{2}{\lambda_i}\,m_i^\star\|u_i\|^2\le-\frac{2}{k_c}J_k,
\]
and with Lemma A, \(H^\star(P_{k+1})-H^\star(P_k)\le -a\Delta J_k\) on \(\mathcal{K}_0\).

**Not claimed:** exact centroids in the implemented controller; global \(\mathcal{K}_0=[0,K)\); continuous-time Theorem 1 replacement.

---

## Theorem C (Approximate centroids + saturation + final QP — finite-horizon \(J\))

Mass-weighted indicators (distinct from \(\|g^\star\|^2\)):
\[
J_k=\sum_i m_{i,k}^\star\|u_{i,k}\|^2,\qquad
E_k=\sum_i m_{i,k}^\star\|\hat c_{i,k}-c_{i,k}^\star\|^2,
\]
\(\bar J=(1/K)\sum J_k\), \(\bar E=(1/K)\sum E_k\).

**Hypotheses for the full window.** \(\{0,\ldots,K-1\}\subseteq\mathcal{K}_0\), \(a>0\), and the cascade
\[
u_i^{\mathrm{cvt}}=k_c(\hat c_i-p_i),\quad
u_i^{\mathrm{sat}}=\lambda_i(\hat c_i-p_i),\quad
u_i=\Pi_{F_{\rho,i,k}}(u_i^{\mathrm{sat}}).
\]
Write \(g_i^\star=2m_i^\star(p_i-\hat c_i)+2m_i^\star e_i\) with \(e_i=\hat c_i-c_i^\star\). Projection against \(0\in F_\rho\) gives \((p_i-\hat c_i)^\top u_i\le-\|u_i\|^2/\lambda_i\), so
\[
{g_k^\star}^\top u\le -\sum_i\frac{2}{\lambda_i}\,m_i^\star\|u_i\|^2+2\sum_i m_i^\star\|e_i\|\,\|u_i\|
\le -\frac{2}{k_c}J_k+2\sqrt{E_k J_k}.
\]
Lemma A + Young \(2\sqrt{EJ}\le(a/2)J+(2/a)E\) yield
\[
H^\star(P_{k+1})-H^\star(P_k)\le -\tfrac{a\Delta}{2}J_k+\tfrac{2\Delta}{a}E_k,
\]
and after summing,
\[
\bar J
\le
\frac{2\,H^\star(P_0)}{a\,K\,\Delta}
+\frac{4\,\bar E}{a^2}.
\tag{★}
\]

| Use of (★) | Status |
|------------|--------|
| Identity under sat.+QP cascade with full-horizon \(\mathcal{K}_0\), Lemma A | **Proved** (\(a>0\)) |
| Substituting trajectory-measured \(\bar E\) | **Post-hoc estimate** |
| Applying (★) when any step leaves \(\mathcal{K}_0\) | **Inapplicable** (do not stitch) |
| A priori \(\bar E\) tight enough to beat \(M_{\mathrm{tot}}u_{\max}^2\) | **Negative** |

**Explicitly:** \(\|u\|\to0\) does **not** imply \(\|g^\star\|\to0\). \(J\) is mass-weighted command energy, not the paper’s original gradient residual.

**Full-horizon status.** Eight of nine static-oracle cases have `theorem_conditions_hold_full_horizon: true`. **C-channel seed 5** fails at frames 241–246 (agent 14): `full_horizon_J_bound_status: "inapplicable"`; \(\bar J,\bar E\) are descriptive only (`not_covered_by_theorem`). Failing steps are **not** deleted. Primary geometric \(J\) ceiling: \(M_{\mathrm{tot}}u_{\max}^2\).

---

## Three-command numerical check (consistency, not certificate)

For each stage \(u\in\{u^{\mathrm{cvt}},u^{\mathrm{sat}},u^{\mathrm{cmd}}\}\) use the **hypothetical** hold map
\[
P_k^+(u)=P_k+\Delta u,\qquad
\Delta H^\star_{\mathrm{hyp}}(u)=H^\star\bigl(P_k^+(u)\bigr)-H^\star(P_k),
\]
and compare to \(\mathrm{rhs}(u)=\Delta{g_k^\star}^\top u+\Delta^2\sum m_{i,k}^\star\|u_i\|^2\).  
Do not reuse the applied next state when testing a non-applied command.

**Refinement is same-observer.** When an anomalous frame is re-evaluated on the fine observer, \(H\), \(g\) **and** \(m\) are all recomputed from that single fine observer: \(\Delta H_{\mathrm{fine}}=H_{\mathrm{fine}}(P+\Delta u)-H_{\mathrm{fine}}(P)\), \(\mathrm{rhs}_{\mathrm{fine}}=\Delta g_{\mathrm{fine}}^\top u+\Delta^2\sum m_{\mathrm{fine},i}\|u_i\|^2\). (The earlier code mixed a coarse-record \(m\) with fine \(H,g\); fixed.)

Across all nine cases, refined max slack stays \(O(10^{-6})\)–\(O(10^{-7})\): **numerical consistency at quadrature tolerance**, **not** a strict numerical certificate. Three layers are recorded per stage — `coarse_verification`, `refined_verification`, and `unresolved_quadrature_error` — and `strict_numerical_certificate` is **always `false`** with reason `"no_rigorous_quadrature_error_bound"` (a fixed residual threshold cannot promote a near-zero refined slack to a certificate without a rigorous quadrature error bound).

---

## What this document does not claim

- No N=16 Hessian / local-stability certificate.  
- No a priori grid-centroid error bound sufficient for a certified improvement over geometry.  
- No certificate for unknown-object transport; static oracle runs are theory baselines only.  
- No silent replacement of continuous Theorem 1 without restating sampled hold, fixed density, and \(\mathcal{K}_0\).
