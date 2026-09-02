ClearAll[q1, q2, z1sq, z2sq, m];
Print[InputForm[Association[
  "W-M5-01-quadratic-PL" -> FullSimplify[
    (q1^2 z1sq + q2^2 z2sq)/2 >= m (q1 z1sq + q2 z2sq)/2,
    Assumptions -> {q1 >= m, q2 >= m, m > 0, z1sq >= 0, z2sq >= 0}],
  "W-M5-02-quadratic-growth" -> FullSimplify[
    (q1 - m) z1sq + (q2 - m) z2sq >= 0,
    Assumptions -> {q1 >= m, q2 >= m, m > 0, z1sq >= 0, z2sq >= 0}],
  "W-M5-03-actual-branch-certificate" ->
    "CONDITIONAL: no branch, gauge, cell combinatorics, or parameter tube supplied"
]]];
