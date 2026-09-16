# Main theorem (static sampled theorem_mode)

Status of the **exact-arithmetic** theorem: **CLOSED for the declared static sampled scope**.

Three separate judgements, not one slogan:

1. **Mathematics.** Feature-cover hold safety, joint recursive feasibility from P0, projection-optimality \(B_E\), and sat→QP dissipation (★) form a closed chain. \(\mathcal K_0\) is a corollary of P0, not an extra future-trajectory hypothesis.
2. **Quantitative priors.** Certificate \(B_{J,\mathrm{prior}}\) (crude \(B_{H0}=R^2 M_{\mathrm{plane}}\), rigorous \(B_E\)) beats \(B_{J,\mathrm{geom}}=M_{\mathrm{plane}}u_{\max}^2\) on rectangle, L-shape, and C-shape after dropping the duplicate projection extra. \(h^\star=0.004\) and the controller are unchanged.
3. **Implementation / experiments.** Float QP flags, polar observer \((H,g,m,c)\), and closed-loop \(J,E,H\) are labelled separately. Feature-cover control mapping is unchanged from commit `c178f76`; seeds 29/31/37 remain independent validation of that mapping. This document does not treat a completed simulation as a proof.

Scope: \(N=16\), frozen object, oracle map, single integrator + sample-and-hold, original \(r_{\mathrm{safe}},\rho,u_{\max},d_{\min}\), feature-cover object rows. No dynamic transport, no global CVT stationarity, no \(N=16\) Hessian, no claim that small \(J\) is coverage success.

## 0. System and cascade

Sampling instants \(t_k=k\Delta\), \(\Delta=0.05\). Domain \(D\subset\mathbb{R}^2\) is a closed rectangle. Frozen simple polygon \(S\subset D\) with oracle vertices. Reference density \(\phi^\star=\phi_0+\sum_j w_j k_\sigma(\cdot-\xi_j)\) with uniform cage offset \(d_c\) (no lead/gap/explore terms). \(N=16\) single-integrator robots, \(\dot p_i=u_i\), \(\lVert u_i\rVert\le u_{\max}\).

At each \(t_k\), with positions \(P_k\) frozen:

1. Inject the oracle polyline of \(S\) (map source `oracle`).
2. Form truncated cells \(\Omega_i=V_i(P_k)\cap B(p_i,R)\) and numerical masses/centroids \((\hat m_i,\hat c_i)\). Centroids are Euclidean-projected onto \(B(p_i,R)\) when the raw Green centroid leaves the disk; if \(\hat m_i\le\texttt{mass_floor}\), \(\hat c_i:=p_i\). Cells with exact mass \(m_i^\star=0\) contribute \(0\) to mass-weighted sums; \(c_i^\star\) is not formed.
3. Nominal CVT input \(u_i^{\mathrm{cvt}}=k_c(\hat c_i-p_i)\).
4. Euclidean ball saturation \(u_i^{\mathrm{sat}}=\lambda_i(\hat c_i-p_i)\) with \(\lambda_i=\min(k_c,u_{\max}/\lVert\hat c_i-p_i\rVert)\) (idle \(\lambda_i=k_c\)).
5. QP: \(U_{k,i}=\Pi_{F_{\rho,i,k}}(u_i^{\mathrm{sat}})\), the Euclidean projection onto the unclamped original-\(\rho\) feasible set
   \[
   F_{\rho,i,k}
   =\{u:\lVert u\rVert\le u_{\max}\}
   \cap\{\text{shared-responsibility agent rows}\}
   \cap\{\text{wall rows }p_i+\Delta u\in D\}
   \cap\{\text{feature-cover object rows}\}.
   \]
6. Hold \(p_i(t_k+\tau)=p_i(t_k)+\tau U_{k,i}\) for \(\tau\in[0,\Delta]\). No coordinate clipping. Object velocity \(v_{\mathrm{obj}}=0\).

Object rows: `object_row_mode=nearest_feature`,
\[
\Phi(p)=\{F:\mathrm{dist}(p,F)\le d_\partial(p)+2u_{\max}\Delta\}
\cap\{F:\mathrm{dist}(p,F)\le R_{\mathrm{row}}\}.
\]
Infinite planes of non-nearest edges are forbidden. Reflex two-plane infinite AND is not used. YAML clamp/scale tiers are idle on the invariant set and are **not** part of this cascade.

Indicators (theorem, not proxies):
\[
J_k=\sum_i m_{i,k}^\star\lVert U_{k,i}\rVert^2,\qquad
E_k=\sum_i m_{i,k}^\star\lVert\hat c_{i,k}-c_{i,k}^\star\rVert^2,
\]
with the convention that a zero-mass cell contributes \(0\). \(\bar J=(1/K)\sum_{k=0}^{K-1} J_k\), likewise \(\bar E\). \(\Sigma_i\lVert U_{k,i}\rVert^2\) is **not** \(J_k\).

**P0 (checkable).** For every \(i\): \(p_i\in D\); pairwise \(\lVert p_i-p_j\rVert\ge d_{\min}\); \(\mathrm{sd}(p_i,S)\ge r_{\mathrm{safe}}+\rho/\gamma_{\mathrm{obj}}\); oracle map; communication dropout \(=0\); \(\mathrm{comm\_range}\ge 2R\); \(\gamma_{\mathrm{obj}}\Delta\le 1\); \(\gamma_{\mathrm{agent}}\Delta\le 1\); \(a=2/k_c-\Delta>0\); range truncation
\[
R_{\mathrm{row}}-u_{\max}\Delta\ge r_{\mathrm{safe}}+\rho/\gamma_{\mathrm{obj}}. \tag{RT}
\]
Paper numbers: \(0.5825\ge 0.0675\). Code: `range_truncation_holds` uses the same scalars as the filter.

## 1. Integral-error lemmas

Partition restrict, clipped diameter/perimeter, analytic \(M_2\), frozen \(h^\star=0.004\,\mathrm{m}\), \(n_{\mathrm{gon}}=256\), robot-centred first moments: as in the 2026-09-15/16 certificate package, with:

1. **Source reach.** \(\mathrm{dist}(\xi,\mathrm{bbox}(P))\le s\sigma\) and \(P\subset B(p,R)\) do **not** give \(\lVert\xi-p\rVert\le R+s\sigma\). Used: \(\min(\sqrt{2}R+s\sigma,\,R+2d_c+n_\sigma\sigma)\).
2. **Projection optimality** (`projection_error_lemma.md`). Raw Green \(\delta\mu=\hat\mu-\mu^\star\), \(x=\hat\mu/\hat m\), \(y=\Pi_{B(0,R)}(x)\), \(e=y-c^\star\). Exact Euclidean projection gives \(\langle x-y,c^\star-y\rangle\le 0\), hence
   \[
   m^\star\lVert e\rVert^2\le 2R\bigl(\lVert\delta\mu\rVert+R\lvert\delta m\rvert\bigr)
   \]
   for \(\hat m>\texttt{mass_floor}\). The duplicate extra \(2R(\lVert\delta\mu_{\mathrm{quad}}\rVert+R\lvert\delta m_{\mathrm{quad}}\rvert)\) is **not** added. Fallback cells keep \(R^2(\sum\lvert\delta m\rvert+N\varepsilon)\). Float `project_to_disk` is not identified with exact \(\Pi\).

Certificate \(B_{H0}=R^2 M_{\mathrm{plane}}\). Polar \(H^\star(P_0)\) is `numerical_a_priori`.

Recomputed certificate constants (this package, `priors_recomputed/`, \(h^\star=0.004\)):

| shape | \(B_E\) | \(B_{J,\mathrm{prior}}\) | \(B_{J,\mathrm{geom}}\) | beats |
|---|---:|---:|---:|---|
| rectangle | 0.1287048157 | 0.1380072238 | 0.1802506048 | yes |
| L-shape | 0.1702700924 | 0.1811413578 | 0.2295107776 | yes |
| C-shape | 0.1814594174 | 0.1906267518 | 0.2295107776 | yes |

Cross-check against the previous ledger after subtracting only the duplicate extra: C-shape \(B_E=0.2358637193-0.0544043019=0.1814594174\), \(B_{J,\mathrm{prior}}=0.1906267518\), \(B_{J,\mathrm{geom}}=0.2295107776\). Implemented values match those three numbers.

Old C-shape \(B_{J,\mathrm{prior}}\approx 0.1978\) used invalid \(R+s\sigma\) and remains withdrawn. The 2026-09-16 closure package with the duplicate extra (\(B_E=0.23586>E^\star=0.22733\)) is superseded for the certificate column only; trajectories are unchanged.

## 2. Feature-cover hold safety

Full write-up: `barrier_invariance_derivation.md`.

- **A1 (in \(\Phi\)).** Convex \(\mathrm{dist}(\cdot,F)\), \(p\notin F\) under P0 so \(\nabla\mathrm{dist}\) exists, \(0\le\gamma\tau\le 1\), cap inactive when \(h_F\ge\rho/\gamma\): \(h_F(p+\tau u)\ge(1-\gamma\tau)h_F(p)+\rho\tau\).
- **A2 (cover-excluded, in range).** \(1\)-Lipschitz: \(\mathrm{dist}(p+\tau u,F)>d_\partial(p)+u_{\max}\Delta\).
- **A3 (range-excluded).** \(\mathrm{dist}(p+\tau u,F)>R_{\mathrm{row}}-u_{\max}\Delta\). Under (RT), \(h_F>\rho/\gamma\). These edges are **not** treated as cover-distance exclusions.
- **Margin (M).** \(d_\partial(p(\tau))\ge r_{\mathrm{safe}}+\rho/\gamma\) on the hold. This is what recursive feasibility uses. The stronger contraction (C) for \(h_{\mathrm{true}}\) is a corollary when \(\Phi\neq\emptyset\) and the paper comparisons of A2/A3 against A1 hold; it is **not** claimed for empty \(\Phi\).
- **A4 (first crossing).** Start in the open exterior with (M) at \(\tau=0\). Continuous hold plus (M) on unsigned \(d_\partial\) excludes a first hit of \(\partial S\). Exterior along the hold is a conclusion, not a circulating hypothesis. Empty \(\Phi\), far robots, nearest-edge switches, ties, endpoints, and reflex corners are treated in that note.

Aggregate infinite-plane switches (C5/L11/L17) are a different mechanism and are not this barrier.

## 3. Joint recursive feasibility

`barrier_invariance_derivation.md` §A5. \(0\in F_{\rho,i}\) iff every stacked family has nonpositive RHS at \(u=0\): agents \(h_{ij}\ge 0\); walls \(p_i\in D\); objects \(h_{\mathrm{true}}\ge\rho/\gamma\) (or empty \(\Phi\) out of range); speed ball contains \(0\); cap inactive.

Omitted pairs: \(\mathrm{comm\_range}\ge 2R=1.6>d_{\min}+2u_{\max}\Delta=0.315\), dropout \(0\), symmetric threshold. Walls: rectangle. Update order: all QPs from frozen \(P_k\), then simultaneous hold.

**Induction.** P0 \(\Rightarrow\) \(0\in F_{\rho,i}\) for all \(i\) \(\Rightarrow\) nonempty closed convex QP set \(\Rightarrow\) exact Euclidean projection attains \(U_k\) \(\Rightarrow\) hold safety (A1–A4, agents, walls) restores a P0-type invariant at \(t_{k+1}\).

Hence every sample lies in
\[
\mathcal K_0=\{k:\ \text{exact QP attains the projection and }0\in F_{\rho,i,k}\ \forall i\}.
\]
\(\mathcal K_0\) is **proved** from P0, not assumed by looking at future frames. A single object row feasible does not replace the conjunction.

## 4. Discrete dissipation

Identity (★) of `discrete_dissipation_derivation.md` (Theorem C), re-checked against this cascade:

- Frozen \(\phi^\star\); partition majorant Lemma A; \(a>0\); saturation then projection onto **unclamped** \(F_\rho\); \(J_k,E_k\) as in §0 with \(\hat c\) the implemented (projected / fallback) centroid and \(U_k\) the QP command.
- Full window in \(\mathcal K_0\): now a corollary of §3.

Then
\[
\bar J\le\frac{2H^\star(P_0)}{a K\Delta}+\frac{4\bar E}{a^2}
\le\frac{2B_{H0}}{a K\Delta}+\frac{4B_E}{a^2}=B_{J,\mathrm{prior}},
\]
with \(B_{H0}=R^2 M_{\mathrm{plane}}\). Route B (\(\lVert\Pi_F(0)\rVert\le u_{\max}\)) is not used.

Polar observer remainders and dissipation slacks of size \(\sim 10^{-5}\)–\(10^{-6}\) are numerical observations, not theorem counterexamples and not a strict numerical covering certificate unless an error envelope is supplied. Controller-error \(B_E\) and observer-error envelopes are distinct.

## 5. Main statement

**Theorem (static sampled feature-cover CBF-QP).**
Assume P0 and the parameter list of §6. Under the exact-arithmetic cascade of §0:

1. **Safety.** For every sample and along every hold: \(\mathrm{sd}(p_i,S)\ge r_{\mathrm{safe}}+\rho/\gamma_{\mathrm{obj}}\), \(\lVert p_i-p_j\rVert\ge d_{\min}\), \(p_i\in D\). Object interval safety is A1–A4, not the clearance monitor.
2. **Recursive feasibility.** \(0\in F_{\rho,i}\) at every sample; the QP attains the Euclidean projection; fallback/abort do not occur.
3. **A priori performance.** (★) with \(B_E\) from §1 and \(B_{H0}=R^2 M_{\mathrm{plane}}\). The certificate \(B_{J,\mathrm{prior}}\) is strictly below \(M_{\mathrm{plane}}u_{\max}^2\) on rectangle, L-shape, and C-shape (table in §1).
4. **Not claimed.** Global coverage convergence or stationarity; unknown-object or moving-object transport; “\(J\) small \(\Rightarrow\) coverage success”; that every IEEE-754 QP call returns the exact projection; that polar \((H,g,m,c)\) are exact.

**Implementation corollary (not the theorem).** The enumerative 2-D solver with feasibility tolerance \(10^{-7}\) reports `optimal` when a float candidate passes that test. Closed-loop matrices record that flag, real mass-weighted \(J,E,H^\star\) (observer estimates), and dissipation slacks. Monitor abort, if it ever fired, would stop the case; it is not an alternative safety proof. Float projection of centroids is not exact \(\Pi_{B(0,R)}\).

## 6. Parameter list

\(\Delta=0.05\), \(k_c=0.9\), \(a=2/0.9-0.05>0\), \(\gamma_{\mathrm{obj}}=8\), \(\gamma_{\mathrm{agent}}=6\), \(\rho=0.02\), \(r_{\mathrm{safe}}=0.065\), \(d_{\min}=0.28\), \(u_{\max}=0.35\), \(R=0.8\), \(\sigma=0.2\), \(d_c=0.105\), oracle spacing \(0.04\), \(h^\star=0.004\), \(n_{\mathrm{gon}}=256\), cull \(s=6\), restrict \(n_\sigma=3\), \(N=16\), \(K=600\), \(R_{\mathrm{row}}=0.60\). Cover radius \(2u_{\max}\Delta=0.035\,\mathrm{m}\). \(\gamma\Delta\le 1\) both families. (RT) as above.

## 7. Experiments versus proof

| Item | Role |
|---|---|
| 15-Sep aggregate matrix | Documents K0 failure of infinite-plane aggregate |
| Closure diagnosis | Mechanism: 90° convex-corner plane switch (C5/L11/L17) |
| Feature-cover Stage C/D/E | Implementation consistency of the mapping frozen at `c178f76`; 18+9 empty K0-fail sets. Not the domain of the theorem |
| This package priors | Certificate constants after projection optimality; three shapes beat geometry |
| This package \(J,E,H\) | Observer-estimated theorem indicators on feature-cover trajectories, 27/27 full-horizon \(\mathcal K_0\), 259200/259200 `optimal`; not \(J_{\mathrm{proxy}}=\sum\lVert u_i\rVert^2\) |
| Seeds 29/31/37 | Independent check of the **unchanged** control mapping; 9/9 full-horizon \(\mathcal K_0\) |
| Serial/parallel | 80-frame L2 and rectangle-2: \(\max\|J,E,H\|_{\mathrm{serial}-\mathrm{spawn}}=0\) |

Finite success does not define the theorem domain. The domain is P0 + §6. Observed \(\bar J\in[0.0086,0.0126]\) and \(\bar E\sim 10^{-7}\) are post-hoc estimates, far below the certificate, and are not a coverage-success claim.

## Closure tags (not a single word)

| Target | Status |
|---|---|
| Exact-arithmetic main theorem, declared static sampled scope | **CLOSED** |
| Three-shape certificate \(B_{J,\mathrm{prior}}<B_{J,\mathrm{geom}}\) | **CLOSED** (C-shape included after dropping the duplicate extra) |
| Float execution identical to exact projection | **open** (solver/projection tolerances; not claimed) |
| Polar observer as a quadrature certificate | **open** (`numerical_a_priori`) |
| Strict numerical covering of dissipation slacks | **open** unless an envelope is supplied; slacks are not counterexamples |
| Dynamic transport / global CVT / “\(J\) small \(\Rightarrow\) coverage” | **out of scope** |
