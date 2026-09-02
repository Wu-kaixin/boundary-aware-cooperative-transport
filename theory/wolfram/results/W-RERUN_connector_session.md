# Independent re-run — Wolfram connector session

Second, independent execution of the CAS obligations, run against the Wolfram
connector rather than replaying the committed script outputs.  Purpose: confirm
that the archived verdicts in `verification_manifest.json` were not merely
transcribed.

- Connector: `Wolfram` (MCP). Version: not exposed.
- Session: `s3jj5PPq`.
- Provenance UTC: 2026-09-02T12:15:36Z.
- Local cross-check: WolframScript 1.14.0, all nine `scripts/*.wl` re-executed
  with exit code 0 and outputs matching the archived results verbatim.

---

## W-RR-01 — kernel L1 mass (Lemma 1, K1)

- **target**: `||K_sigma||_1 = 2 pi sigma^2`
- **exact_input**:
  `Assuming[σ > 0, FullSimplify[Integrate[Exp[-(x^2+y^2)/(2 σ^2)], {x,-Infinity,Infinity},{y,-Infinity,Infinity}] == 2 Pi σ^2]]`
- **assumptions**: `σ > 0`
- **exact_output**: `True`
- **evidence_class**: EXACT
- **verdict**: PASS

## W-RR-02 — kernel gradient L1 (Lemma 1, K2)

- **target**: `||grad K_sigma||_1 = sqrt(2) pi^(3/2) sigma`
- **exact_input**:
  `Assuming[σ > 0, FullSimplify[2 Pi Integrate[(r/σ^2) Exp[-r^2/(2 σ^2)] r, {r,0,Infinity}] == Sqrt[2] Pi^(3/2) σ]]`
- **assumptions**: `σ > 0`
- **exact_output**: `True`
- **evidence_class**: EXACT
- **verdict**: PASS

## W-RR-03 — kernel gradient sup-norm (Lemma 1, K3)

- **target**: `||grad K_sigma||_inf = e^(-1/2)/sigma`
- **exact_input**:
  `Assuming[σ > 0, Reduce[D[(r/σ^2) Exp[-r^2/(2 σ^2)], r] == 0 && r > 0, r]]`
  and the endpoint values `0` at `r = 0` and `r -> Infinity`.
- **assumptions**: `σ > 0`, `r >= 0`
- **exact_output**: stationarity `(Re[σ] > 0 && Im[σ] == 0 && r == σ) || (Re[σ] < 0 && Im[σ] == 0 && r == -σ)`;
  value at `r = σ` is `1/(Sqrt[E] σ)`; endpoints `{0, 0}`.
- **evidence_class**: SYMBOLIC
- **verdict**: PASS
- **human_note**: `MaxValue` does not return a closed form here, so the
  supremum is established by the stationary point plus both endpoints, exactly
  as the committed script does. Four numeric spot values
  (`σ ∈ {1/2, 1, 3, 7}`) agreed to the printed precision; that spot check is
  corroboration, **not** the evidence for this entry.

## W-RR-04 — normal-angle identity (Lemma 1, target error)

- **target**: `sqrt(2 - 2 cos θ) = 2 sin(θ/2)` for `0 <= θ <= π`
- **exact_input**: `FullSimplify[Sqrt[2-2 Cos[θ]] == 2 Sin[θ/2], Assumptions -> 0 <= θ <= Pi]`
- **assumptions**: `0 <= θ <= Pi`
- **exact_output**: `True`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS

## W-RR-05 — bounded-step sum (Lemma 1, curve quadrature)

- **target**: `Δs^2 <= h_s Δs` whenever `0 <= Δs <= h_s`
- **exact_input**: `Resolve[ForAll[{a,h}, 0 <= a <= h, a^2 <= h a], Reals]`
- **assumptions**: `0 <= a <= h`
- **exact_output**: `True`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS

## W-RR-06 — centroid ratio denominator (Lemma 2, ε_q)

- **target**: `m_min - eta_m > 0` under the midpoint gate
- **exact_input**: `Resolve[ForAll[{mmin, etam}, mmin > 0 && 0 <= etam < mmin, mmin - etam > 0], Reals]`
- **assumptions**: `m_min > 0`, `0 <= eta_m < m_min`
- **exact_output**: `True`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS
- **human_note**: The stricter query `(eta_a + R_l eta_m)/(m_min - eta_m) > 0`
  returns the residual condition `eta_a + eta_m R_l > 0`, i.e. the ratio is only
  *non-negative* when both numerator terms vanish. That is the correct and
  expected behaviour; the proof needs positivity of the denominator, which is
  what this entry certifies.

## W-RR-07 — Young's inequality (Theorem 1, step 3)

- **target**: `δ x <= (a0/2) x^2 + δ^2/(2 a0)`
- **exact_input**: `Resolve[ForAll[{a0,d,x}, a0 > 0 && d >= 0 && x >= 0, d x <= (a0/2) x^2 + d^2/(2 a0)], Reals]`
- **assumptions**: `a0 > 0`, `δ >= 0`, `x >= 0`
- **exact_output**: `True`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS

## W-RR-08 — comparison lemma solution (Theorem 1, T1-a)

- **target**: the solution of `V' = -λ V + B`, `V(t0) = V0`, equals
  `e^(-λ(t-t0)) V0 + (B/λ)(1 - e^(-λ(t-t0)))`
- **exact_input**:
  `FullSimplify[DSolveValue[{v'[t] == -lam v[t] + B, v[t0] == v0}, v[t], t] == Exp[-lam (t-t0)] v0 + (B/lam)(1 - Exp[-lam (t-t0)]), Assumptions -> lam > 0]`
- **assumptions**: `λ > 0`
- **exact_output**: `True`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS
- **human_note**: This certifies the closed form of the comparison ODE. The
  differential-inequality-to-solution step itself is the standard comparison
  lemma and is argued analytically in `09_theorem1_practical_stability.md`.

## W-RR-09 — frozen perfect-information corollary

- **target**: `B = 0` collapses T1-a to pure exponential decay
- **exact_input**: `Simplify[(Exp[-lam (t-t0)] v0 + (B/lam)(1 - Exp[-lam (t-t0)])) /. B -> 0]`
- **assumptions**: `λ > 0`
- **exact_output**: `E^(lam(-t+t0)) v0`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS

## W-RR-10 — sampled-data descent expansion (Corollary M7)

- **target**: `-a dt g^2 + (L_H/2) dt^2 (a g + δ)^2 + dt δ g`
  equals `-(a - L_H a^2 dt/2) dt g^2 + dt g δ (1 + L_H a dt) + (L_H/2) dt^2 δ^2`
- **exact_input**: `FullSimplify[lhs == rhs]` with the two expressions above.
- **assumptions**: none (polynomial identity)
- **exact_output**: `True`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS

## W-RR-11 — strong convexity implies the PL bound used (Prop. 3, M5)

- **target**: `||g||^2 >= 2 m_H (H - H_min)  =>  (1/2)||g||^2 >= m_H (H - H_min)`
- **exact_input**: `Resolve[ForAll[{g, mH, gap}, mH > 0 && gap >= 0 && g^2 >= 2 mH gap, (1/2) g^2 >= mH gap], Reals]`
- **assumptions**: `m_H > 0`, `gap >= 0`
- **exact_output**: `True`
- **evidence_class**: SYMBOLIC
- **verdict**: PASS
- **human_note**: This is the *generic implication only*. It says nothing about
  whether any actual DBACT branch satisfies the reduced-Hessian premise. A8
  therefore remains CONDITIONAL, matching `W-M5-02`.

---

## What this re-run did not verify

Unchanged from the first pass, and worth restating because a reader may mistake
a longer manifest for a stronger result:

- No pushforward, convolution-translation, curve-quadrature, atom-matching, or
  tail-decomposition step was machine-checked; those are human analytic proofs.
- No Voronoi shape derivative or moving-boundary flux cancellation was
  machine-checked.
- No validated interval enclosure of a reduced Hessian on a named branch and
  parameter tube exists, so no global or uniform PL constant is certified.
