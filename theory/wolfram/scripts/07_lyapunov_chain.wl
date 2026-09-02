ClearAll[delta, x, a0, lambda, t, t0, B, v, v0];
Print[InputForm[Association[
  "W-T1-01-Young" -> FullSimplify[
    delta x <= (a0/2) x^2 + delta^2/(2 a0),
    Assumptions -> {a0 > 0, delta >= 0, x >= 0}],
  "W-T1-02-comparison-solution" -> FullSimplify[
    DSolveValue[{v'[t] == -lambda v[t] + B, v[t0] == v0}, v[t], t],
    Assumptions -> {lambda > 0, t >= t0}],
  "W-T1-03-frozen" -> FullSimplify[
    Exp[-lambda (t - t0)] v0 + (0/lambda) (1 - Exp[-lambda (t - t0)]) ==
      Exp[-lambda (t - t0)] v0,
    Assumptions -> {lambda > 0, t >= t0}]
]]];
