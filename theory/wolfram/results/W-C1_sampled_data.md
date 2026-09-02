# W-C1 - sampled-data corollary

- UTC: 2026-09-02T11:09:21.219Z
- Connector: Wolfram (version not exposed)
- Assumptions: `a0,a1,L,dt>0`, `0<dt<2a0/(L a1^2)`, and nonnegative
  disturbance/gradient norms.
- Evidence: SYMBOLIC
- Verdict: PASS

```text
<|"W-C1-01-descent-expansion" -> True,
  "W-C1-02-young-remainder" -> True,
  "W-C1-03-step-gate-positive" -> True,
  "W-C1-04-geometric-series-limit" ->
    (dt*((delta^2*dt*L)/2 +
      (delta+a1*delta*dt*L)^2/(2*a0-a1^2*dt*L)+nuH))/r|>
```

Substituting `r=abar*mu*dt` gives `B_d/(abar mu)`. A Python randomized positive-
parameter test independently checks that the symbolic bound dominates the
direct one-step descent-lemma expression.
