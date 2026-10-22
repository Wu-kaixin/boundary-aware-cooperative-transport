# CCTA 2027 experimental revision

Follow-up, 22 October 2026: the [reliability/theory/manuscript revision](RELIABILITY_REVISION_2026-10-22.md)
classifies these frozen outcomes, retains rejected controller candidates, and
adds an authored manuscript. Its development replays and prospective validation
are separate from the v2 scientific results below.

Scope agreed on 21 October 2026: use the existing CCTA scheme, including static
oracle/local deployment, independent safety checks, three-method transport,
ablations, generalization and robustness. This report distinguishes code defects
from measured task failures and unsupported scientific claims.

## Implemented repairs

| Defect | Repair | Regression evidence |
| --- | --- | --- |
| HOLD dwell accumulated across an interrupted settle | Reset the streak whenever the hold condition fails | Consecutive-frame regression |
| Stale or unobservable motion estimates participated in progress and speed medians | Admit only currently usable directional estimates | Three-agent stale-vote regression |
| Default robot separation contradicted default disc radius | Default `d_min` now satisfies the existing overlap contract | Default-controller construction regression |
| Different factor conditions with the same seed overwrote each other | Explicit `condition_id` participates in pairing; duplicates raise an error | Two-factor and duplicate-episode regressions |
| Direct paired runs reported unresolved completion while the study runner computed it | Use the same settling-plus-Euclidean-tolerance definition at the episode boundary | Shared existing completion contract |
| A source snapshot could be attributed to a run while the live source was edited | Check source hashes again at completion; execute the frozen campaign from a separate source tree | Recorded source integrity and execution snapshot |
| Independent Pymunk engine could not use the paper recorder | Provide the same contact-report interface with copied solver impulses | Complete-episode recorder regression |
| Pymunk advanced robots beyond their committed endpoints and misreported wrench | Integrate from the previous endpoint, accumulate every physical substep, and measure torque | Endpoint and linear/angular momentum regressions |
| A penalty spring force bound was presented for a rigid-contact engine | Mark that capacity inapplicable for Pymunk; retain actual impulse-derived forces | Independent-engine reporting regression |
| Reused seeds across shapes/perturbations could be counted as independent replication | Add seed-cluster analysis; suppress inferential intervals for very few seed clusters | Repeated-seed and censoring regressions |

Earlier repairs remain active: braking no longer depends on first forming a
four-pusher transport quorum; stopping lead uses the velocity-reference ramp;
scaled scatter initialization respects the unchanged clearance contract; local
gap truncation is reported without claiming an online geometry error bound.

## Frozen matrix

The compact design is [revision.yaml](../experiments/ccta2027/revision.yaml).
`campaign freeze` expands complete configurations, preflights every initial state,
records its hash, and saves an immutable execution-source tree.

| Population | Episodes | Purpose |
| --- | ---: | --- |
| Static reference / original local / segment candidate | 27 | Three shapes, seeds 2/5/8, 600-frame whole-hold checks and deployment quality |
| Development | 27 | Three shapes, seeds 2/5/8, CVT / nearest target / local spacing |
| Target ablations | 15 | Full CVT plus four isolated removals; inactive exploration stays explicitly null |
| Active exploration diagnostic | 6 | Nonzero gain versus removal, separated from the main comparison |
| Generalization | 36 | Twelve new seeds with the existing near/search and joint scale/distance factors |
| Robustness | 54 | Two seeds, three shapes, noise / communication loss / slower sensing, three methods |
| Numerical reference | 2 | Exact planar QP and independent CVXPY static reference |
| Contact engine reference | 2 | Penalty and independent Pymunk engine, reported separately |
| Total | 169 | Every planned episode remains in the denominator |

Generalization seeds are 202610210–202610221; robustness seeds are 202610230 and
202610231. Historical seeds 12–31 remain quarantined. The old frozen test is not
relabelled as new validation. No threshold, horizon or seed is replaced after
looking at an outcome.

## Acceptance and interpretation

The implementation gate requires a receipt for every planned episode, zero
execution errors, source/product integrity, matching paired initial states and
tasks, and complete certified static paths for the oracle and segment candidate.
Original local violations are retained as a negative control. Deployment cost
and robot displacement must accompany safety so that immobility cannot masquerade
as successful deployment.

Transport completion requires the existing settling rule and final Euclidean
error within the unchanged task tolerance. CONTACT_READY, physical-contact dwell,
G500, settling and completion are separate measurements. Scientific superiority
has no preselected sign: readiness versus both controls, task rates and failure
conditions must all be reported. There is no unknown-boundary dynamic-safety
theorem in this scope.

The current-source baseline is a separate development regression: 27/27 transport
episodes completed without execution errors or watchdogs, but completion was 4/9
for CVT, 6/9 for nearest target and 3/9 for local spacing. The static baseline found
8/9 original-local path violations, while oracle and segment arms each passed 9/9.
These baseline runs precede the new estimator-vote fix and are not pooled with
the frozen revision.

## Recorded artifacts

The first complete revision recorded all 169 episodes but failed its acceptance
audit because the Pymunk recorder raised an exception. That error and its failed
audit are retained. The infrastructure repair changes the independent engine;
the main controller and penalty-model implementations are unchanged. The full
matrix is repeated rather than selectively substituting the failed episode.

The second frozen protocol, source tree, per-case products and process logs live
under `runs/ccta2027/draft_v2/`. Compact audited results and useful figures are
curated under `docs/results/ccta2027_draft_v2/`. The acceptance audit and scientific
verdicts are reported separately. Two extra complete serial episodes verify
reproducibility and do not enter scientific denominators.

Analysis is versioned separately from simulation. Reused seeds are resampled as
whole clusters, preserving their shape/perturbation blocks. With fewer than five
independent seeds, this project reports descriptive paired differences without a
confidence interval; this is a conservative reporting policy rather than a
coverage theorem. The old condition-only bootstrap fields remain for backward
compatibility and are not the preferred inference for repeated-seed populations.
The archived execution source and the final analysis source are both retained.

## Final measured results

The v2 campaign records **169/169 episodes, zero execution errors**, source
integrity PASS and acceptance PASS. The release regression suite reports
**616 passed, 2 skipped**. Two serial repetitions reproduce their parallel
results, including all 25 dynamic and three static replay arrays. All 168
non-Pymunk outcomes agree with v1 after excluding timing and the added sampling
label; the repair did not select new main-model outcomes.

| Static arm | Certified safe paths | Smallest path clearance | Mean final/initial cost |
| --- | ---: | ---: | ---: |
| Original local samples | 1/9 | 48.33 mm | 0.1533 |
| Segment candidate with gap guard | 9/9 | 66.04 mm | 0.1544 |
| Oracle reference | 9/9 | 67.50 mm | 0.1534 |

The required clearance is 65 mm. Mean robot displacement per episode ranges from
0.810 to 1.013 m for the segment candidate, so its safety result does not come
from standing still. The gap guard truncates 5,315 of its 5,400 control frames;
this intervention and its fixed-polygon assumptions must accompany the claim.
The independent CVXPY and planar QP references both pass the full 600-frame
static checks; their maximum recorded position difference is 1.82 mm. Their
trajectories are not represented as identical.

| Transport population | Boundary CVT | Nearest observed target | Local spacing |
| --- | ---: | ---: | ---: |
| Development | 5/9 | 5/9 | 4/9 |
| New-seed generalization | 5/12 | 3/12 | 1/12 |
| Perturbation robustness | 1/18 | 2/18 | 1/18 |

For generalization, CVT minus nearest-target controller readiness averages
-1.364 s over 11 complete pairs; one pair is censored. The exploratory
seed-cluster 95% interval is [-2.736, -0.145] s. Against spacing the difference
is -1.632 s with interval [-6.668, 1.582] s. Complete-pair analysis can be
selection biased; neither interval establishes universal preparation superiority
or transport reliability. Development and robustness reuse only three and two
independent seeds respectively, so their aggregate rates remain descriptive.

The exploration removal is exactly null in all three baseline shapes because
the baseline gain is zero; all replay arrays agree. The separate gain-six
diagnostic is retained, with mixed task outcomes. One seed per shape cannot
establish an ablation benefit. In the independent-engine check, penalty transport
completes with 0.111 m endpoint error, while Pymunk reaches 0.378 m and its watchdog
fires. Both execute successfully, but engine invariance is unsupported.

The defensible initial-paper direction is static boundary-aware deployment with
measured cost reduction and explicitly limited downstream preparation evidence.
**Robust transport and continuous moving-body safety remain unresolved.** A
paper claiming those properties is not ready on this evidence. Keep all failure
denominators and narrow the conclusions instead of changing acceptance thresholds.
The [result bundle](results/ccta2027_draft_v2/README.md) contains the complete case
table, preserved failed revision, analysis provenance and four exportable figures.
