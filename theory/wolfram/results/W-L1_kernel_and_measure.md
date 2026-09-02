# W-L1 - kernel and target checks

- UTC: 2026-09-02T11:09:21.219Z
- Connector: Wolfram (version not exposed)
- Evidence: SYMBOLIC / EXACT
- Verdict: PASS, except the quantified maximum was retained as a derivative
  certificate rather than misreported as a direct `MaxValue` result.

## Exact output

```text
<|"W-L1-01-kernel-l1" -> 2*Pi*sig^2,
  "W-L1-02-gradient-l1" -> Sqrt[2]*Pi^(3/2)*sig,
  "W-L1-03-gradient-linf-derivative" ->
    ((-r + sig)*(r + sig))/(E^(r^2/(2*sig^2))*sig^4),
  "W-L1-03-gradient-linf-at-positive-critical-point" -> 1/(Sqrt[E]*sig),
  "W-L1-03-endpoints" -> {0,0},
  "W-L1-04-normal-angle" -> True,
  "W-L1-05-step-square" -> True,
  "W-L1-05-step-square-sum-two" -> True,
  "W-L1-06-target-error-decomposition" -> True|>
```

## Human note

For `sig>0`, the displayed derivative is positive on `0<r<sig`, zero at
`r=sig`, and negative for `r>sig`; together with both endpoint limits zero this
proves the radial maximum `e^(-1/2)/sig`. The two-term step check plus the
termwise inequality supports finite sums. None of these outputs proves the
pushforward or convolution decomposition; that proof is in Lemma 1.
