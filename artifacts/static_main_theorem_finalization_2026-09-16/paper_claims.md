# Paper claims — static sampled theorem_mode

Status: exact-arithmetic main theorem **CLOSED** for the declared static sampled scope. Quantitative certificate advantage **CLOSED** on all three shapes. Float execution and polar observers are **not** the theorem.

## Proved (mathematical, matching the executed exact cascade)

1. **Fixed affine discrete CBF.** Frozen plane, uncapped \(n^\top u\ge-\gamma h+\rho\), \(p^+=p+\Delta u\), \(0\le\gamma\Delta\le 1\): \(h(p^+)\ge(1-\gamma\Delta)h+\rho\Delta\).
2. **Convex feature.** Euclidean distance to a segment is convex. Same discrete inequality for \(h_F=\mathrm{dist}(p,F)-r_{\mathrm{safe}}\) off the segment.
3. **One-step cover (in range).** Any edge that can become nearest during a hold of length \(\Delta\) at speed \(\le u_{\max}\) satisfies \(\mathrm{dist}(p,F)\le d_\partial(p)+2u_{\max}\Delta\).
4. **Range truncation.** Edges with \(\mathrm{dist}(p,F)>R_{\mathrm{row}}\) satisfy \(\mathrm{dist}(p+\tau u,F)>R_{\mathrm{row}}-u_{\max}\Delta\). Under \(R_{\mathrm{row}}-u_{\max}\Delta\ge r_{\mathrm{safe}}+\rho/\gamma\), they stay above the ISSf margin without being placed in \(\Phi\).
5. **Object margin invariance.** Combining cover rows, cover-excluded Lipschitz bounds, and range truncation: \(d_\partial\ge r_{\mathrm{safe}}+\rho/\gamma\) on the hold. First-crossing from the open exterior excludes \(\mathrm{int}(S)\). Strong \(h_{\mathrm{true}}\) contraction is a corollary when \(\Phi\neq\emptyset\) and the paper comparisons hold; it is not claimed for empty \(\Phi\).
6. **\(0\in F_\rho^{\mathrm{object}}\)** iff \(h_{\mathrm{true}}\ge\rho/\gamma\) (or \(\Phi\) empty out of range). Cap inactive when \(h\ge\rho/\gamma\).
7. **Infinite-plane aggregate is not this barrier.** C5/L11/L17: 90° convex-corner switch of a supporting plane.
8. **Agents and walls.** Shared-responsibility \(h_{ij}\); omitted pairs farther than \(d_{\min}+2u_{\max}\Delta\); rectangle walls \(P+\Delta U\in D\).
9. **Joint recursive feasibility from P0.** Conjunction of the four families, not a single object halfspace. \(\mathcal K_0\) is a corollary.
10. **Integral certificates.** Legal source reach; projection optimality on raw Green moments (no duplicate extra); analytic \(M_2\); frozen \(h^\star=0.004\); mass fallback; \(m^\star=0\) contributes \(0\).
11. **Dissipation identity (★)** on the sat→QP cascade, with \(\mathcal K_0\) from (9) and \(a>0\). Certificate \(B_{J,\mathrm{prior}}<M_{\mathrm{plane}}u_{\max}^2\) on rectangle, L-shape, **and C-shape**.

## Supported only by experiment (not a proof)

1. Feature-cover closed-loop K0 emptiness: Stage C/D 18/18, Stage E 9/9 (seeds 29/31/37, unused for design). This package: 27/27 full-horizon \(\mathcal K_0\), 259200/259200 `optimal`, including those independent seeds. Implementation consistency of the mapping frozen at `c178f76`.
2. Polar observer \(H,g,m,c\) versus a finer grid: numerical assessment, not a quadrature certificate.
3. Dissipation slacks \(\sim 10^{-5}\) on in-K0 frames: not a counterexample and not a verification of (★).
4. Closed-loop \(\bar J,\bar E,\bar H\) on the 18+9 matrix: observer estimates of the theorem indicators. \(\sum\lVert u_i\rVert^2\) is not \(J\).

## Must not be claimed

1. Global convergence, coverage stationarity, or “\(J\) small \(\Rightarrow\) coverage success”.
2. Unknown-object transport, SEARCH/APPROACH/TRANSPORT, or moving cargo.
3. \(N=16\) Hessian / local CVT uniqueness.
4. Original aggregate-plane \(0\in F_\rho\) on C5/L11/L17.
5. That every float QP call equals the exact Euclidean projection.
6. Treating polar \(H^\star(P_0)\) as rigorous \(B_{H0}\). Certificate \(B_{H0}\) is \(R^2 M_{\mathrm{plane}}\).
7. Clamp, \(\rho\) reduction, RHS truncation, or “safe or abort” as a safety proof.
8. Restoring the withdrawn C-shape \(B_J\approx 0.1978\) (invalid \(R+s\sigma\)).
9. Using trajectory \(\bar E\) as \(B_E\).
