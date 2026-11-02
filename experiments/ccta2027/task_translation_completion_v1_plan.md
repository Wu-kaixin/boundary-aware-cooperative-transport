# Task-translation completion: live execution ledger

Spec: task_translation_completion_v1_design.md, frozen unchanged. Full goal:
active_goal_20261103.md. Current work starts from published commit 4e1bb4c.
The root project .venv is the only permitted Python environment.

## Completed

- Actual session 3893 returned exit 1. Its log ends with the frozen 3600-second
  wall-budget TimeoutError, after the far-skew 8000/12000 progress line.
- Added a new terminal receipt to the old raw directory without editing any
  previous file: runs/research/2026-11-03-task-translation-v2/execution_outcome.json.
- Verified unchanged source, environment, script/snapshot hashes, all original
  case identities and all three completed products. Completed summaries equal
  the already-published checkpoint. The fourth has only its frozen templates.
- Created and froze a new wrapper calling the unchanged v2 offline_case; no
  runtime, parameter, case, arm, admission threshold or selected candidate change.
- New protocol task_translation_completion_v1.json SHA-256:
  b7cf3464a936e461202e55399e30531132840414788ee6389fdcd2ced0908cad.
- Freeze command completed successfully (session 78945, exit 0). It declares
  three reused cases, one recomputed case and a separate 2700-second budget.
- GitHub CI 41512096250 for published 4e1bb4c is fully successful. Ubuntu and
  Windows each have 876 passes / 2 skips; MAS, health and smoke runner passed.
  Receipt: runs/maintenance/2026-11-03-task-translation-v2/ci_receipt.json.

## Original handle, now terminal

**Exec session 25427 returned exit 1**, after completing and saving all skew
frames. The final hash aggregation had ROOT[o['path']] instead of ROOT/o['path'];
the original source and POSTPROCESSING_FAILED receipt are preserved. It is not
live and must not be restarted. Its started.json records
2026-11-02 19:49:35.428603 UTC; maximum execution deadline is approximately
20:34:35 UTC. Log: runs/maintenance/2026-11-03-task-translation-completion-v1/study.log.
Raw products: runs/research/2026-11-03-task-translation-completion-v1/.
The original frozen design/protocol and source snapshots remain unchanged.
Only the current wrapper's path join is corrected. A separate finalizer audits
all four raw products and original identities without estimator updates/physics.
It completed successfully (session 30915, exit 0); its own manifest/output lives
under runs/research/2026-11-03-task-translation-finalization-v1/.

## Remaining work

1. DONE: terminal execution and postprocessing failure recorded.
2. DONE: all four rows and raw/snapshot identities checked. Original reference
   exact checks total 141429 statuses, 119888 readings, 35099 navigation records;
   scan-count checks total 581968. Candidate paired MAE in mm is
   1.381 / 3.062 / 14.270 / 3.802, all paired availability 1.0, false goal-stop
   frames 0, all evaluation frames have >=4 voters. Sustained streaks are
   824 / 17 / 0 / 2; the original overall screen remains FAIL.
3. DONE locally: complete report/CSV/figures generated without overwriting the
   historical 3/4 checkpoint. New compact figure layout preserves the first
   render and receipt. Source and every old raw product remain unchanged.
4. One fresh final review of this completion-report plus opt-in wiring increment
   before pushing it. The previous checkpoint review is complete and not repeated.
5. Keep all original L/C/skew sustained-feedback FAILs. The all-four results and
   separate position audit justify an exploratory physical question, with no
   automatic adoption. Minimal default-off wiring is implemented; execution and
   engineering verification are recorded in task_translation_wiring_plan.md.
6. Continue toward reliable full transport, matched deployment contribution,
   new-condition validation and the supported CCTA2027 paper.

Ruling: the wrapper's pre-start refusal must not write an execution-failure
receipt into a run that another invocation might already be executing. Failure
receipts will be added only after authoritative terminal observation, as for v2.
If this guard is wrong, a second invocation could contaminate a live run's identity.

Ruling: this analysis-only wrapper is checked through its full freeze/run source
and product assertions; the unchanged runtime's already-passing suite is not
rerun solely because a new orchestrator script was added. Wrong continuation
bookkeeping could still misattribute evidence, hence the explicit origin maps,
published-checkpoint equality checks and final independent review.

Ruling: recover only aggregation after the actual TypeError, without rerunning
complete estimator outputs or rewriting the failed execution receipt. The actual
failed invocation supplies RED evidence; independent finalization of all four
origins/products passes after the path correction. If the recovery attributed
data to the corrected wrapper as its producing source, provenance would be false;
the manifest instead retains both failed and corrected hashes and original origins.

Post-hoc evaluator audit: all four old trajectories were checked. Perfect
position alone cannot enter the 20 mm goal band on old L/C (minimum true error
20.116 / 52.746 mm), even before other stop requirements. Skew has 86 optimistic
position-only frames, so this explanation cannot dismiss every skew failure.
This supports a separate prospective exploratory physical question, not changing
the old FAIL. The 4 baseline controller parameter maps exactly match every
protected resolved value. An earlier audit selected the nonexistent 'original'
arm and produced zero rows; it remains local/invalid, and the corrected 4-row
audit is protected_case_parameters_v2.json. Neither audit is a physical pilot.

Ruling: finish the full replay and verify its terminal state before changing
runtime source, then publish the full results with the minimal default-off wiring
as one reviewed increment — this prepares a concrete next trial while protecting
all historical identities. Cost if wrong: source attribution could be ambiguous;
reports explicitly separate old replay fingerprints from new engineering checks.

Ruling: preserve project run receipts and ledgers instead of deleting review
workspace artifacts — the repository's evidence-preservation rule takes precedence
over generic skill cleanup. Cost: small local maintenance storage remains.
