# Reliability, theory scope and manuscript revision

This development iteration starts from the locally stored paper baseline
`e4b0a19`, on `codex/reliability-theory-manuscript`. The original
[draft-v2 evidence](CCTA2027_EXPERIMENTS.md) is unchanged. No new simulation is
pooled with that campaign or presented as independent validation.

## Failure mechanisms: observations before attribution

The [machine-readable classification](results/reliability_2026-10-22/v2_failure_classification.json)
covers all 140 transport receipts. It uses each recorded task's Euclidean
tolerance, reconstructs the final transverse component from longitudinal and
Euclidean error, and keeps execution, settling, watchdogs, phase censoring and
physical-contact dwell separate. It labels geometric symptoms, not inferred
causes. All 29 static/numerical cases stay in their original safety populations.

| CVT population | Completed | Longitudinal undershoot | Longitudinal overshoot | Transverse error | Mixed error |
| --- | ---: | ---: | ---: | ---: | ---: |
| Development | 5 | 3 | 1 | 0 | 0 |
| New-seed generalization | 5 | 3 | 0 | 3 | 1 |
| Perturbation robustness | 1 | 1 | 0 | 8 | 8 |

Here a longitudinal or transverse component is classified as excessive only
when it individually exceeds the original tolerance. Two components individually
within tolerance can still fail the Euclidean criterion; the classifier retains
that as a distinct combined-error category. Maximum transverse excursion is not
substituted for the final transverse error.

Compact v2 receipts do not contain enough time series to identify estimator or
contact-loss causes. We therefore regenerated the nine CVT development cases and
the two engine references from the archived source in the same new Python
environment as the repair. All 11 baseline task outcomes and endpoint errors
reproduce v2 exactly. These reused seeds are development evidence only.

The regenerated traces reveal positive terminal progress-estimation bias of
0.150 m for C seed 2, 0.155 m for L seed 8 and 0.189 m for rectangle seed 2;
their bias at brake entry is 0.094, 0.095 and 0.130 m respectively. C seed 5,
an overshoot case, instead has negative terminal bias (-0.049 m). Thus a single
global gain correction cannot be justified by assuming all errors have the same
sign. Bias is measured against simulator truth by the evaluator, never injected
into the controller. Observed association does not prove causation.

## Repairs and rejected proposals

The supervisor now invalidates a slow planner's held nominal command when phase,
push suspension or feedback availability changes. HOLD explicitly forbids a
new inward pushing command, even if one robot's progress estimate lags the team.
An experimental guard makes feedback-driven convoy and pressing require a usable estimate. Velocity feedback
can come from the robot's own current directional registration or current usable
one-hop voters; the latter matches the existing progress-sharing permissions.
The last usable source disappearing removes its feedback on the next update.
This is not a claim of arbitrary-delay or Byzantine consensus.

The experimental brake candidate releases stored integral demand once speed reaches the
falling reference. Below the reference a full push arc can retain integral
action to overcome static friction; incomplete arcs use the existing P-only
brake. It does not turn the reference gain into a fictitious physical stopping
acceleration or change the task tolerance.

A first combined candidate cleared the integral on every BRAKE frame and
required every robot's own observation. Its CVT development completion fell
from 5/9 to 3/9, so it was rejected. The
[failed comparison](results/reliability_2026-10-22/rejected_p_only_comparison.json)
retains all 11 cases, source identities, packages and paired initial states.
A second candidate retained low-speed integral action but still rejected useful
neighbour feedback; it also yielded 3/9. These combined comparisons do not
identify an individual modification's causal effect. In particular, the first
candidate's regression cannot be attributed solely to its P-only brake law.

The one-hop feedback candidate also regressed completion (4/9). Accordingly,
`transport_feedback_guard` and `brake_release_integral` both default to `False`.
The archived default transport feedback/brake law is retained; only the explicit
HOLD prohibition and supervisor/feedback-availability command invalidation are
promoted. The candidate implementations and their targeted tests remain available
for mechanism research, with no performance guarantee. Their transient results
are retained separately rather than discarded or pooled.

The final default revision retains **5/9 CVT development completions**, matching
the baseline, with every initial-state/task pair checked. It is a control-response
correctness repair, not a demonstrated increase in completion rate. The complete
release suite passes **629 tests, with two skips**. The expanded validation
protocol is frozen and initialization-preflighted, but its 832 episodes have not
been executed. No additional robustness result is claimed.

| Development version | CVT completed | Penalty endpoint error | Pymunk endpoint error |
| --- | ---: | ---: | ---: |
| Archived baseline | 5/9 | 0.111 m | 0.378 m |
| Rejected P-only / own-feedback candidate | 3/9 | 0.129 m | 0.232 m |
| Rejected low-speed-PI / own-feedback candidate | 3/9 | 0.129 m | 0.232 m |
| Rejected one-hop-feedback candidate | 4/9 | 0.150 m | 0.597 m |
| Final default (experimental options disabled) | 5/9 | 0.111 m | 0.378 m |

No candidate completes the Pymunk task. The [curated result bundle](results/reliability_2026-10-22/README.md)
contains every comparison, trace-derived classification, executed source archive,
test report and the prospective freeze. The final executed default source and
the prospective frozen source are byte-identical.

Run a separately identified paired development regression with:

```powershell
.\.venv\Scripts\python.exe scripts/run_reliability_revision.py --out runs/reliability/new_diagnostic --workers 4
```

The script extracts the archived baseline, snapshots the current source, uses
the same interpreter/packages and exact frozen task/initial-state definitions
in both arms, and rejects pair-identity or execution failures. Each engine case
remains distinct; the repeated penalty case checks deterministic duplication.
`paired_metrics.json` now additionally records phase-response flags and copied
feedback-quality labels; compact receipts retain final estimate and phase flags.

Reanalyse either compact results or retained frame traces using:

```powershell
.\.venv\Scripts\python.exe -m dbact_sim.experiments.reliability --receipts docs/results/ccta2027_draft_v2/case_results.json --out runs/reliability/reanalysis.json
```

Add `--traces <study-directory>` when the corresponding per-case
`paired_metrics.json` files exist. Missing traces remain explicitly unavailable.

## Theory and manuscript

The [applicability supplement](../theory/static_deployment/applicability_and_residual.md)
proves the exact-centroid projected-residual bound
`G <= 2 J + 2 k_c^2 E`. It explains why filter stationarity need not be geometric
stationarity or useful coverage. It also states the missing local-boundary
coverage certificate and derives how a physical row-residual error reduces the
available robust margin. No float guarantee or moving-contact theorem is added.

The [authored manuscript](../manuscript/ccta2027/main.tex) contains the problem,
information permissions, static proposition and proof, residual argument,
frozen-v2 results, failure analysis and limits. Four primary references and the
official format requirement are checked in the [source map](../manuscript/ccta2027/sources.md).
The IEEEtran conference draft compiles to four pages; every page was rendered
and visually reviewed. Author identities and affiliations are pending, and the
title is a working title. It is a review draft, not a submission-ready claim.

## Prospective validation, not additional results

[reliability_validation.yaml](../experiments/ccta2027/reliability_validation.yaml)
expands to 832 preflightable cases. Development/static, robustness and ablation
populations each use 12 seed clusters, with separate seeds for ablations and
robustness. Generalization uses another 12 new seeds. Active-gain ablations use
gain six and explicitly label that diagnostic; the zero-gain main controller is
not relabelled as an active-exploration experiment. Historical plans still expand
to the original 169 cases without changing their configuration semantics.

This larger design has no outcomes yet. Its new-seed episodes must not be used
to tune the controller after outcomes are inspected. It increases replication
of the existing three shapes; it does not itself expand shape, occlusion or
boundary-sampling coverage. That geometry expansion, independent transport
validation, dynamic-contact safety, and physical robot validation remain open.
