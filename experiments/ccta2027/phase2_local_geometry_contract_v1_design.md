# Phase 2: explicit local geometry contract review

2026-11-04 Asia/Tokyo. This resumed stage reviews the preceding report's proposed
local-assumption question. It does not adopt a new research definition. Limit:
one hour offline, zero physical episodes, zero controller replay, no parameter
search, no runtime/configuration/frozen-protocol edits. Start of this resumed
window: 2026-11-03 20:05:35 UTC. Earlier stages and their idle gap remain separate.

Question: which explicit local assumptions suffice to bound an unobserved arc
between two returns, are those assumptions compatible with the actual polygonal
shape library, and can the resulting certificate leave a nonempty robust contact
band under the existing constants? Mathematical compatibility is not evidence
that the observation interface can certify these assumptions.

This is AI-assisted exploratory model review after prior failures were seen,
not preregistration, a new control method, or independent physical validation.
The scientific-brainstorming workflow supplies an independent adversarial review.

Before the static numerical audit, fix the following calculations:

- Derive and compare arc-length curvature, angle-parametrized interpolation,
  and a unified bounded-variation tangent contract, including endpoint noise.
- Audit circle64, ellipse24, rectangle, L, C and skew-box geometry definitions;
  separately identify the four frozen physical conditions. Do not import oracle
  sensing settings from geometry YAML files into claims about the task controller.
- Compute the conditional contact-band budget from unchanged epsilon=3*sigma,
  delta_max, rho and gamma. Report strict bounds, not new controller thresholds.
- Use one fixed analytic major-circle-arc example (radius 1 m, endpoint angle
  separation 0.1 rad) to refute replacing unknown arc length by chord length.
- Use one fixed two-edge right-angle example (both legs 0.04 m) to check the
  corner term. No search over examples and no claim that these are observed runs.
- Quantify the current area-scaled circle/ellipse polygon discrepancy from their
  ideal parent curves. This matters only if a smooth ideal model is substituted;
  the actual polygon is not an erroneous simulator geometry.

Sufficient contract under review: a true connected arc q:[0,L]->R^2, parametrized
by arc length, with independently justified L<=Lbar and tangent measure
Dq'=g ds+nu, ||g||<=kappa and total variation |nu|<=J. Endpoints have errors at
most epsilon. Then the proposed bound is

    d_H(true arc, measured chord) <= epsilon + kappa*Lbar^2/8 + Lbar*J/4.

All missing association, visibility, arc-length, variation, coverage, noise-event,
initial safety and execution-time premises must be listed explicitly. No sample
fit residual is accepted as an absolute certificate. One observation-consistent
admitted geometry violating the claimed bound falsifies a proposed admission
rule; lack of a premise certificate is 'uncertified', not a passing test.

Deliver one report, a reproducible static arithmetic/source audit, independent
critique and integrity receipt. Preserve prior receipts and code byte-for-byte,
except an additive current-report entry in README. Stop for the user's decision;
do not implement an admission rule, rerun the 128/64-state comparison or start
Phase 3.
