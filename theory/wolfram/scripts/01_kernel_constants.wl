ClearAll[x, y, r, sig];
Print[InputForm[Association[
  "W-L1-01" -> Assuming[sig > 0,
    FullSimplify[Integrate[Exp[-(x^2 + y^2)/(2 sig^2)],
      {x, -Infinity, Infinity}, {y, -Infinity, Infinity}]]],
  "W-L1-02" -> Assuming[sig > 0,
    FullSimplify[2 Pi Integrate[(r/sig^2) Exp[-r^2/(2 sig^2)] r,
      {r, 0, Infinity}]]],
  "W-L1-03-derivative" -> Assuming[sig > 0,
    FullSimplify[D[(r/sig^2) Exp[-r^2/(2 sig^2)], r]]],
  "W-L1-03-critical-value" -> Assuming[sig > 0,
    FullSimplify[(r/sig^2) Exp[-r^2/(2 sig^2)] /. r -> sig]],
  "W-L1-03-endpoints" -> {0, 0}
]]];
