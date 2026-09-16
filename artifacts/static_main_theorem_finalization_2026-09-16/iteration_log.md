# Iteration log — 2026-09-16 finalization

## 1. Inventory

Confirmed `c178f76` feature-cover controller. Saved diagnosis dumps have `J_proxy_speed2`, not theorem \(J_k=\sum m_i^\star\|u_i\|^2\). Full \(\hat c,m^\star,U\) over 600 frames are not in those dumps, so real J/E/H requires `run_apriori_centroid_bound` closed-loop (allowed: missing records). Control mapping unchanged ⇒ seeds 29/31/37 stay independent validation.

## 2. Projection lemma (implemented after proof)

Identity \(m^\star e=\delta\mu-y\delta m+\hat m(y-x)\) with raw Green \(\delta\mu\). Euclidean projection \(\langle x-y,c^\star-y\rangle\le 0\) drops the duplicate extra. Fallback \(R^2(\sum|\delta m|+N\varepsilon)\) kept. Tests rewritten: out-of-disk raw centroid, robot-centred shift, \(m^\star=0\) raises, leftover `quadrature_dm` kwargs ignored.

## 3. Range truncation contract

`range_truncation_holds`: \(R_{\mathrm{row}}-u_{\max}\Delta\ge r_{\mathrm{safe}}+\rho/\gamma\). Paper 0.5825 ≥ 0.0675. Enforced in `SafetyFilter` for `nearest_feature` and in `assert_theorem_params`. Dropout forbidden in theorem_mode.

## 4. Safety write-up

A1 Φ-rows (convexity, gradient off the segment, cap inactive). A2 cover-excluded Lipschitz. A3 range-excluded Lipschitz with (RT). Margin invariance for recursive feasibility; strong contraction as a corollary when \(\Phi\neq\emptyset\). A4 first crossing from exterior. A5 joint induction. A6 exact vs float; monitor is not a safety proof.

## 5. Priors

3 shapes × seed 2, outer=3. C-shape \(B_E=0.1814594174\), \(B_{J,\mathrm{prior}}=0.1906267518<0.2295107776\). All three beat geometry. Certificate status label changed from template `rigorous_on_K0` to `rigorous` because K0 is now a corollary of P0.

## 6. JEH

27-case matrix completed: outer=16, wall 4110 s, ~87% CPU, 16 children. All 27 full-horizon K0, 259200/259200 optimal, no abort. Real \(\bar J\in[0.00855,0.01258]\), \(\bar E\sim 10^{-7}\). Independent seeds 29/31/37 included. Dump threshold raised to slack \(>10^{-3}\) or K0 fail after the first I/O-bound attempt.

## 7. Serial/parallel

80-frame L-shape seed 2 and rectangle seed 2: in-process serial vs 2-worker spawn, and vs the 600-frame matrix prefix: max abs \(J,E,H,\)slack \(=0\).

## 8. Docs

`main_theorem.md/.tex`, lemmas, claims, ledger. LaTeX PDF not produced on this host (no pdflatex).

