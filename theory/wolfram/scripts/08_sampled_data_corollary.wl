ClearAll[a0, a1, x, delta, Lh, dt, nuH, mu];
Module[{raw, abar, cdelta, bd},
  raw = -a0 x^2 + delta x + (Lh dt/2) (a1 x + delta)^2 + nuH;
  abar = a0 - (Lh a1^2 dt)/2;
  cdelta = delta (1 + Lh a1 dt);
  bd = cdelta^2/(2 abar) + (Lh/2) delta^2 dt + nuH;
  Print[InputForm[Association[
    "W-C1-01-expansion" -> FullSimplify[
      raw == -abar x^2 + cdelta x + (Lh/2) delta^2 dt + nuH],
    "W-C1-02-Young-remainder" -> FullSimplify[
      -abar x^2 + cdelta x + (Lh/2) delta^2 dt + nuH <=
        -(abar/2) x^2 + bd,
      Assumptions -> {abar > 0, cdelta >= 0, x >= 0, Lh >= 0,
        delta >= 0, dt >= 0}],
    "W-C1-03-step-gate" -> FullSimplify[abar > 0,
      Assumptions -> {a0 > 0, Lh > 0, a1 > 0,
        0 < dt < 2 a0/(Lh a1^2)}]
  ]]]
];
