# W-P1/P3/T1 - ideal field, reference, and Lyapunov algebra

- UTC: 2026-09-02T11:09:21.219Z
- Connector: Wolfram (version not exposed)
- Evidence: SYMBOLIC / EXACT
- Verdict: PASS

```text
<|"W-P1-01-integrand-derivative" -> {2*(px-qx),2*(py-qy)},
  "W-P1-02-fixed-square-gradient" -> True,
  "W-P3-01-offset-rate-product-rule" -> True,
  "W-P3-02-cost-rate-factor" -> True,
  "W-T1-01-young" -> True,
  "W-T1-02-comparison" ->
    (B + E^(lambda*(-t+t0))*(-B+lambda*v0))/lambda,
  "W-T1-03-frozen" -> True|>
```

The fixed-square result is a small exact sanity example, not a proof of general
Voronoi shape differentiation. Shared-boundary and saturation-circle
cancellation are proved in Proposition 1.
