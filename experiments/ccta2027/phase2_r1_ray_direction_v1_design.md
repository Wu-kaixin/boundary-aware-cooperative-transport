# Phase 2 R1: one fixed ray-direction candidate

2026-11-04 Asia/Tokyo. The user resumed the active goal after the R1 model
assessment. Continue its proposed bounded next step: one empirical candidate
specification and, only if implementable without extra information, one fixed
offline comparison. Zero physical episodes, zero controller replay, no runtime
or configuration edit, no changed noise/safety/task definitions, no parameter
search. At most 7200 s offline wall budget, 5 GiB output, one internal thread.
The preceding turn made progress: it ruled out generic deterministic finite-ray
coverage and direct continuous-to-split execution claims. That assessment is
not evidence of a successful new transport method.

Origin: AI-assisted, source-derived proposal; existing outcome data and earlier
diagnostics have been observed. This is exploratory development evidence, not
preregistration or independent validation. Scientific-brainstorming supplies an
independent adversarial review; numerical evaluation is a separate activity.

## Candidate, fixed before computing its cohort outcomes

Keep the original admitted packets, source/object associations, feature identities,
retained row mask, normals, structural half-gap/point allowance, gamma, rho,
received motion demand, cap function, nonobject rows, speed limit and goal.
Do not merge scans or fit a plane. Retain all original nonadmission/fallbacks.
Truth is used only after construction, as an offline label in the control cohort.

For each kept point/chord feature with endpoints z_k, sensor origin o, original
unit distance-gradient n and allowance eps=3*sigma, let

    d_k = (z_k-o)/||z_k-o||,
    K = conv{z_k + t*d_k : |t| <= eps}.

Use the exact directional support of this endpoint uncertainty set:

    lower_new = n.p - max_k(n.z_k + eps*abs(n.d_k)) - structural,
    h_new = lower_new - r_safe.

The old lower bound is dist(p,F)-structural-eps. For its valid distance normal,
n.p - max_k(n.z_k) = dist(p,F). Hence h_new >= h_old; A does not change and
both uncapped RHS and the existing capped RHS can only decrease. This concerns
the specified endpoint set, NOT a deterministic bound on Gaussian errors or on
the true continuous boundary. Half-gap semantics and the previously documented
hidden-feature counterexample are not repaired. Substep execution is unchanged.

Known directional range errors are the only new use of existing information.
This differs from the existing three-ray line fit: no feature is removed and no
half-gap allowance is replaced by a fit residual. It is a standard support-
function construction, not claimed as original robust-control theory.

Analytic prediction to check independently: for a query at its own scan origin,
an isolated point or endpoint projection yields zero improvement. For an interior
projection on adjacent rays with 5-degree separation and positive ranges, the
improvement is at most eps*(1-cos(2.5 degrees)), about 0.0286 mm. A neighbor
feature can improve more, at most eps, without the own-origin angle bound.
Therefore this minimal mechanism is expected to be insufficient, but that
expectation must not suppress all-row/all-state results.

## Fixed comparison and decision

Reuse all 128 original control-event states (112 admitted, including the fixed
17 subsequent positive-force states) and all 64 candidate first-pusher-loss
states (57 admitted, including four preselected pushers). Keep the cohorts and
subsets separate. Do not change seeds, frames, labels or endpoints.

Reconstruct every original admitted feature and match saved A, h and capped b.
For each admitted state compare original and candidate goal-direction capability
under BOTH the original capped system and its uncapped object rows. These are
two accounting views of one candidate, not two tuned variants. Keep the same
nonobject rows, original received-motion demand and velocity ball. Reuse the
existing analytic 2D/CLARABEL certificate machinery with its existing 1e-7
classification tolerance. No QP objective is changed or replayed.

Save every row's noise support, h change, own/neighbor source, and every state's
four matrices/certificates and saved-command residual. Nonadmitted control
states remain explicitly unsupported; nonadmitted candidate states retain their
saved command/system, separately labeled unchanged fallback.

Report capped and uncapped transitions; the 17 force states and four selected
pushers; all positive-capability losses/gains and inconclusives; true-distance
lower-bound overestimates on the labeled controls; preservation of guards and
all original structural/noise magnitudes. No count denotes restored physical
contact, force, transport, stopping or independent success.

Primary sufficiency question: does noise direction alone remove every original
reverse/infeasible capability obstruction in the fixed 17 force states and four
selected pushers? An inconclusive or remaining obstruction rejects sufficiency.
Improving only a subset is recorded as partial information, not success. New
truth-distance overestimation or any extra-information dependence prevents
recommending deployment even if capability improves. A passing local diagnostic
would still require separate user approval and full-task validation.

Keep the original hidden-protrusion example as a limitation; do not search for
a more favorable synthetic world. Verify the support formula independently by
explicit uncertainty vertices, every numerical certificate using scalar paths,
and all inputs/source hashes. Preserve implementation/test failures and receipts.

Stop after the report. Do not try a second geometry rule, tune an allowance,
switch to R2 implementation, run physics, or advance to Phase 3 in this stage.
