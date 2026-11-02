# Frozen prospective task-translation physical pilot

2026-11-03; starting published commit 77e5ed9. This is the next development
experiment toward the full goal in active_goal_20261103.md, not its replacement.
The preceding completion/wiring publication was verified progress: local core
884 passed / 2 skipped, MAS 106 passed, independent review without findings,
and both CI platforms passed on 77e5ed9.

## Question and interpretation

Does the single preselected translation/all-captured reference permit additional
motion and reliable physical stopping with the protected common parameters?
The completed four-arm replay retains its original FAIL. Its 40-consecutive-entry
screen is not the physical controller's HOLD-then-settle rule; old L/C paths do
not even enter the 20 mm nominal arrival band with an ideal position readout.
The old skew has position-eligible frames but fails the full offline stop screen.
These observations motivate this separate prospective experiment, not a relaxed
old threshold. No new physical candidate result has been observed at design time.

## Fixed cases and interventions

Eight fresh physics episodes: four conditions, two arms. Conditions are exactly
near_circle, near_l_reference, far_c_open_away_pos0, far_skew_box_s0_r0_pos0 from
the `reference` arm of task_reference_v1.json. Preserve each original seed,
initial-state signature, shape, task, configuration and boundary_cvt method.
These are reused development conditions, not independent validation samples.

- `reference`: the protected common configuration, unchanged.
- `translation_all_captured`: copy that configuration and change only four
  mechanism selectors: initial_reference_acquisition=True,
  initial_task_reference=True, initial_reference_motion_model='translation',
  initial_reference_use_all_captured=True.

No numerical gain, estimator admission threshold, acquisition deadline, sensor,
physics, task or safety parameter changes. Effective parameters for all eight
cases must equal the protected receipt after removing only the three newer
default selectors and the two deliberately changed acquisition/reference flags.
The baseline must equal every old effective value. Do not alter the canonical
best YAML or receipt. Acquisition adds time within the same episode budget.

Keep the original stationary freeze/supporter policy and local/one-hop captures.
No true shape, centroid, yaw or physical-contact truth enters online control.
Zero fitted yaw is explicitly a translation hypothesis, not measured zero or a
rotation-robust error certificate. Shared phase supervision remains a limitation.

The deterministic simulator resets each episode. A separately seeded schedule
(20261103) shuffles the eight jobs; actual physics seeds remain unchanged.
Pairs share all original inputs. Do not count frames or the four reused conditions
as independent population-reliability trials; no significance claim is planned.

## Budgets, provenance and stopping policy

Exactly eight episodes, at most 12000 steps each, dt=0.05 s (600 simulated seconds),
four worker processes, one internal numerical thread each. Aggregate simulation
budget is at most 96000 steps; the existing run_until_settled early finish remains.
No outcome-dependent extra episodes, retuning, retry of partial physics or
per-shape arm choice. Wall-clock duration is measured, not an additional task
success threshold; this pilot has no artificial wall-clock cut that could prefer
shorter conditions. Process errors and budget terminations remain in denominator.

Freeze source, design, runner/helper hashes, resolved cases, root-.venv environment
and installed locked-package versions before any new physics. Preserve the known
joblib/PyYAML lock discrepancies explicitly; do not install dependencies. Freeze
old baseline identities, source snapshots and product receipts for reproduction.
Use only runs/research/2026-11-03-task-translation-physical-v2/ with separate
freeze/ and run/ directories. Never overwrite any original run or frozen design.
Do not edit runtime/configuration/frozen runner/environment while the pool is live.

## Prospective metrics and advance gate

Primary: safe completed task, every HOLD-entry true error, false HOLD (>120 mm),
hard safety, termination/timeout, and valid stationary zero epoch. Completion uses
the existing physical settle rule: 40 frames below 0.005 m/s, with final Euclidean
error <=120 mm. Nominal control arrival stays 20 mm; it is not the task tolerance.
Hard safety retains the existing separation tolerance, nonnegative signed
clearance, penetration budget, and zero solver fallback/infeasible requirements.
Evaluate G500 separately with the original saved ClosedLoopContract, including
all reasons and barrier scalings; do not interpret success as a safety theorem.

Secondary: final/first-HOLD error, completion/termination seconds, pre-epoch
motion/contact, estimated displacement MAE/P95 and availability after original
CONTACT_READY, fresh-voter/quorum coverage, true rotation for evaluation only,
minimum spacing/clearance, penetration, and contact-redeployment attempt counts.
Align a control reading at frame f to replay cargo pose f-1 (pre-physics), and
retain both pre/post-physics first-HOLD errors. Count each new entry into HOLD,
not every dwell frame as an independent event. Distinguish refusal to stop from
physically completed transport. Truth is used only in this offline evaluation.

Before running, verify the evaluation on the four old baseline receipts against
the published task_reference_v1 analysis. This is an evaluator check, not new data.
After running, all four fresh baselines must exactly reproduce every old replay
array and old scientific JSON field. Only git_sha provenance may differ; report
case/cohort/wall-time execution metadata separately. No unlisted scientific field
or diagnostic is silently ignored. Preserve and report any mismatch as a failed
reproduction, even if the candidate appears good.

The advance gate requires: eight complete execution receipts; exact reproduction
of all four baselines; all four candidate tasks safely completed; zero candidate
false HOLDs; all four valid zero epochs; and no loss of any G500 pass of its matched
baseline. Report primary completion and G500 nonregression separately. Passing
only authorizes wider protected-condition regression and unused-condition tests;
candidate_adopted stays false. Failing prompts mechanism diagnosis, with all raw
data retained. Final deployment CVT/spacing/nearest comparison must use the same
validated common transport controller; this pilot cannot prove CVT superiority.

## Review focus

Default/numerical protection, correct reference-arm selection, old/new identities,
no late-zero or truth leakage, control-frame alignment, complete denominators,
strict baseline reproduction, original G500/hard-safety criteria, and no physical
success/adoption claim from incomplete jobs. Preserve all failures and snapshots.

## Pre-execution correction from unexecuted v1

v1 completed freeze/evaluator checks but executed zero new physics. Before launch,
inspection of the old C timeout receipt showed that the runner labels the 12000-step
termination as watchdog, not max_frames. Preserve v1 unchanged and supersede it
with this v2 before new data collection. Count budget terminations by exhausted
step budget without physical settling; preserve the original termination label.
Only analyzer bookkeeping and the completed-with-errors execution label change;
all eight cases, parameters, gates, schedule and physical budgets stay the same.
