ClearAll[qx, qy, px, py];
Print[InputForm[Association[
  "W-P1-01-integrand-derivative" -> FullSimplify[{
    D[(qx - px)^2 + (qy - py)^2, px],
    D[(qx - px)^2 + (qy - py)^2, py]}],
  "W-P1-02-fixed-square" -> FullSimplify[{
    D[Integrate[(qx - px)^2 + (qy - py)^2,
      {qx, -1, 1}, {qy, -1, 1}], px],
    D[Integrate[(qx - px)^2 + (qy - py)^2,
      {qx, -1, 1}, {qy, -1, 1}], py]} == 2 4 {px, py}]
]]];
