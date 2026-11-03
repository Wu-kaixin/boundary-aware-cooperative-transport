# Final publication checkpoint review

Review range: `0c7bf9fd04ad2f141cb8d184e0cf440c0a051729` to `87b69f2cc3e85428e2bf37ea95f3680aca5f043b`.
Scope: the user-requested publication checkpoint; no claim that the research objective or full candidate constraint audits are complete. This is the single fresh final review. No runtime, frozen helper, protocol, tracked file or original receipt was edited, and no physics or full replay was run.

## Strengths

- README and both new reports distinguish terminal task receipts, the earlier seven-receipt snapshot, synthetic engineering preflight, and still-pending full pair/bridge/support audits. All four 600-second candidate task failures remain explicit.
- The analysis adapter checks the declared old/new arm pair and condition/case identities, deep-copies before normalizing only arm, then uses the existing strict scientific comparison. Packet array conversion retains finite values exactly and restores the original conversion in finally. A one-ULP mutation is rejected by the targeted test.
- Contact-event selection uses the predeclared first/peak positive-force rules before new-row feasibility is computed. An independent counterfactual filter retains hard rows; physical force is used for evaluation after commands are fixed. Public prose does not equate hypothetical velocity with physical force or closed-loop causality.
- Original label-mismatch, serialization failure, RED/GREEN and partial receipt identities are retained. Runtime/configuration changes are absent from the review diff, and the freeze checker verifies all 22 frozen helper/test files.

## Issues

### Critical

None.

### Important — P2: export execution-error rows without assuming physical metrics

Locations: `scripts/export_post_support_receipts.py:49-50`; `scripts/publish_post_acquisition_support.py:49-53`, and its figure handling around line 87.

Both tools promise complete scheduled outcome publication, and the receipt exporter explicitly accepts `COMPLETED_WITH_ERRORS`. However, the shared `episode()` returns a minimal row for an execution error, containing status/error and safety defaults but no `budget_terminated`, `endpoint_m`, `g500_failure_reasons`, `seconds`, or rotation metrics. The receipt exporter unconditionally indexes the missing cutoff/endpoint in its progress print, before writing the public report. The full publisher similarly indexes missing scientific metrics, and its figure later rejects missing values. A batch with even one execution error therefore cannot publish all scheduled outcomes. I confirmed the first `KeyError('budget_terminated')` using the actual shared `episode()` error path and the exporter's expression, without modifying a receipt.

Fix in the separately identified publication helpers: preserve the execution error and scheduled row, represent unavailable metrics explicitly as null/blank, and make the figure handle missing observations without drawing them as zero or calling them complete. Add a regression through the exporter entry points with a real minimal error row. Do not change frozen runtime/helpers or regenerate/relabel existing eight-case evidence. The observed eight normally terminated receipts are unaffected by this dormant path.

### Minor

None requiring a change in this checkpoint.

## Checks performed

- Read AGENTS.md, Serena initial instructions, reviewer template, physical plan/design, analysis addendum and contact diagnostic design.
- Reviewed all changed Python helpers/tests, gate changes and their reused strict comparator/episode/serialization contracts; inspected README changes and both publication reports.
- Ran `.\.venv\Scripts\python.exe -m pytest -q tests/test_post_support_analysis.py tests/test_post_support_study.py --basetemp runs/maintenance/2026-11-03-post-acquisition-support-v1/reviewer-pytest-20261103-v1 -p no:cacheprovider`: 12 passed in 3.96 seconds.
- Ran `study.check()` read-only: runtime, protocol, protected configuration, environment/package inventory, source snapshots and all 22 frozen helper/test hashes passed.
- Independently verified all 13 products across the publication manifests.
- Independently matched all eight terminal receipt hashes to raw case receipts; checked seven-receipt partial identity separately and the terminal report's explicit `PENDING` independent replay status.
- Independently checked all 128 public contact rows against raw physical-force, original/counterfactual velocity, admission and capped/uncapped residual fields. Confirmed 17 positive-force states and 13 negative counterfactual goal velocities; none of those 13 original goal velocities was negative.
- Independently recomputed first/maximum eligible force-event indices from every completed control's saved metrics: all four conditions matched the selected events.
- Inspected the terminal execution receipt: eight complete executions, zero execution errors, source integrity PASS. Inspected all eight task CSV rows and retained failure/precheck receipt paths.
- Confirmed no `src/` or `configs/` changes in the review range and a clean tracked worktree before executor fixes.
- Did not independently repeat the reported 1079-test core suite or repository-wide health scan; those retained receipts and the executor's fresh health result were inspected/reported separately.

## Declined to judge

- Full candidate pair/bridge/support replay correctness and executed-row gate verdicts: these audits have not run and are explicitly pending; launching them is outside this publication checkpoint review.
- Causal sufficiency of the contact diagnostic for all four candidate failures: same-state hypothetical velocities do not establish the physical force or closed-loop causal chain; the reports state that limit.
- Generalization, universal dynamic safety, deployment contribution, complete decentralization, hardware and manuscript completion: outside this increment and listed as remaining research work.
- Future corrected exporter behavior: the executor will perform the authorized single fix pass and verification; this review does not pre-approve an unseen patch or request a second reviewer.

## Assessment

Ready to publish: with the error-row export fix and regression verification. No material inconsistency was found in the existing eight-terminal-receipt checkpoint, contact diagnostic counts or scientific/provenance boundaries. Full research acceptance and the full candidate constraint audits remain pending regardless of publication readiness.

## Executor resolution after the independent review

The P2 exporter issue was reproduced through both real entry points using the
shared episode() execution-error schema: two RED failures. The publication-only
fix retains all eight scheduled rows, the original error and null/blank unknown
metrics. Figures mark missing observations as NA, with no zero-height replacement
bars; all eight positions remain visible. Fourteen focused tests passed, and the
final figure/error-path check passed two tests after the explicit plot bounds.
The full current regression result is recorded separately in review_checks.json.

No original eight-case receipt, early7/8snapshot, raw diagnosis, frozen helper,
runtime or configuration was regenerated or changed. The published terminal
summary retains the hash and snapshot of the exporter that actually produced it;
the later error-path repair has its own source/test identity. The figure used to
verify NA rendering is synthetic error-path test output, not research data.

The review's set-aside behaviors remain explicit: full pair/bridge/support replay
and formal gates are pending; the contact diagnostic alone does not establish the
full physical causal chain; generalization, deployment contribution, hardware,
decentralization and manuscript completion remain research work. Future exporter
behavior is now checked by the regression fix pass, rather than pre-approved by
the independent reviewer. No deferred minor findings. No second review requested.

Publication readiness is limited to this evidence checkpoint. It does not accept
the finite-support candidate or complete the broader research objective.
