ClearAll[b, d, n, t, Rl, LK1, Pmax, vxi];
Print[InputForm[Association[
  "W-P3-01-product-rule" -> FullSimplify[
    D[b[t] + d[t] n[t], t] == b'[t] + d'[t] n[t] + d[t] n'[t]],
  "W-P3-02-nuH-substitution" -> FullSimplify[
    2 Rl^2 (LK1 Pmax vxi) == 2 Rl^2 LK1 Pmax vxi],
  "dimension-exponents" -> Association[
    "nu_phi" -> {"length" -> 3, "time" -> -1},
    "nu_H" -> {"length" -> 5, "time" -> -1}]
]]];
