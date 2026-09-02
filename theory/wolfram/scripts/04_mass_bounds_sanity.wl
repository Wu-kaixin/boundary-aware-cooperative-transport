ClearAll[phi0, r0, Rl, Pmax];
Print[InputForm[Association[
  "W-L2-07-mass-floor-positive" -> FullSimplify[phi0 Pi r0^2/4 > 0,
    Assumptions -> {phi0 > 0, r0 > 0}],
  "W-L2-08-mass-ceiling-positive" -> FullSimplify[
    Pi Rl^2 (phi0 + Pmax) > 0,
    Assumptions -> {Rl > 0, phi0 > 0, Pmax >= 0}],
  "dimension-exponents" -> Association[
    "mass" -> (1 + 2), "first-moment" -> (1 + 1 + 2),
    "centroid" -> ((1 + 1 + 2) - (1 + 2))]
]]];
