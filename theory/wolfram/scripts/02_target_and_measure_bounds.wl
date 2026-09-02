ClearAll[theta, ds, hs, en, dmax];
Print[InputForm[Association[
  "W-L1-04" -> FullSimplify[
    Sqrt[2 - 2 Cos[theta]] == 2 Sin[theta/2],
    Assumptions -> 0 <= theta <= Pi],
  "W-L1-05-term" -> FullSimplify[ds^2 <= hs ds,
    Assumptions -> 0 <= ds <= hs],
  "W-L1-05-two-term-sum" -> FullSimplify[
    d1^2 + d2^2 <= hs (d1 + d2),
    Assumptions -> {hs >= 0, 0 <= d1 <= hs, 0 <= d2 <= hs}],
  "W-L1-06" -> FullSimplify[
    Sqrt[2 - 2 Cos[en]] dmax == 2 dmax Sin[en/2],
    Assumptions -> {dmax >= 0, 0 <= en <= Pi}],
  "dimension-exponents-L1" -> Association[
    "translation" -> (1 + 1 + 1),
    "weight" -> (2 + 1),
    "tail" -> (2 + 1)
  ]
]]];
