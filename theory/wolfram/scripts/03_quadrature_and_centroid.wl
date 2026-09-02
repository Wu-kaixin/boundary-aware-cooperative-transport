ClearAll[ahat, a, mhat, m, etaa, etam, R, mmin, rh, Lphi, A, phimax, P];
Print[InputForm[Association[
  "W-L2-01-ratio-identity" -> FullSimplify[
    ahat/mhat - a/m == ((ahat - a) m + a (m - mhat))/(m mhat),
    Assumptions -> {m != 0, mhat != 0}],
  "W-L2-02-positive-denominator" -> FullSimplify[mmin - etam > 0,
    Assumptions -> {mmin > 0, 0 <= etam < mmin}],
  "W-L2-03-monotone-etaa" -> FullSimplify[
    D[(etaa + R etam)/(mmin - etam), etaa] > 0,
    Assumptions -> {R >= 0, mmin > 0, etaa >= 0, 0 <= etam < mmin}],
  "W-L2-04-monotone-etam" -> FullSimplify[
    D[(etaa + R etam)/(mmin - etam), etam] >= 0,
    Assumptions -> {R >= 0, mmin > 0, etaa >= 0, 0 <= etam < mmin}],
  "W-L2-05-eta-m-monotone-rh" -> FullSimplify[
    D[Lphi rh A + phimax (2 P rh + Pi rh^2), rh] >= 0,
    Assumptions -> {Lphi >= 0, rh >= 0, A >= 0, phimax >= 0, P >= 0}],
  "W-L2-06-eta-a-monotone-rh" -> FullSimplify[
    D[(phimax + R Lphi) rh A + R phimax (2 P rh + Pi rh^2), rh] >= 0,
    Assumptions -> {Lphi >= 0, rh >= 0, A >= 0, phimax >= 0, P >= 0, R >= 0}]
]]];
