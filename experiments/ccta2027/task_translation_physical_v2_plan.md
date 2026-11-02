# Task-translation physical pilot: execution ledger

Spec: task_translation_physical_v2_design.md. Full goal remains
active_goal_20261103.md. Start from published/reviewed 77e5ed9.

Previous goal turn: **progress** — complete four-condition replay and default-off
wiring published, README updated, root core 884/2 skipped, MAS 106, final review
without findings, Windows/Ubuntu CI 41523057948 successful. No physical candidate
trial had yet been run and the original offline screen stayed FAIL.

## Freeze and execution

- Root .venv only; existing joblib/PyYAML versions retained. Host reports 48 logical
  CPUs and about 100 GiB available memory; fixed pool is four workers, one internal
  numerical thread each. No package installation or numeric parameter edits.
- v1 freeze completed (exec 62473 exit 0), zero physics. Before launch, checking an
  original 12000-step C case exposed that budget termination is labeled watchdog,
  not max_frames. v1 remains unchanged and unexecuted, with a separate
  preexecution_outcome.json. No partial or old physical episode was rerun.
- v2 corrects that analysis-only counter by checking exhausted steps and lack of
  settling, plus the execution-with-errors label. Same eight cases, schedule,
  controls, candidate, thresholds and numeric parameters. Read-only old C check
  (exec 25798 exit 0) confirmed budget_terminated=True at 12000 unsettled steps.
- v2 freeze completed (exec 92193 exit 0). All four old baseline evaluation rows
  equal the earlier published analysis on the checked outcomes; all eight resolved
  configurations match protected parameters except the four declared selectors.
  Physics/phase gates and initial-state signatures are unchanged.
- Runtime fingerprint: 9c7927a57d614f6f2d18757c2677c7af8829203882d3b40effd43043f3afa948.
- Protocol SHA-256: eb74bd23df060e067f436ade37e921f5ff57e4af404a6264bbf356c706c064e1.
- New raw root: runs/research/2026-11-03-task-translation-physical-v2/.
  Frozen source/runner/design/protocol: freeze/. Actual physical study: run/.
- The fixed eight-episode study **exec session 11579 is terminal, exit 0**.
  Eight receipts, zero execution errors, source integrity PASS, wall time 210.4 s.
  Log: runs/maintenance/2026-11-03-task-translation-physical-v2/study.log.
  Do not relaunch; this is authoritative terminal observation, not a wait timeout.

## Work after authoritative terminal observation

1. DONE: all eight raw receipts retained, zero execution errors or budget ends.
2. DONE: frozen analyzer exec 91844 exit 0. Four baselines exactly reproduce all
   25 arrays and scientific fields. Safe completion 3/4 to 4/4, false HOLD 1 to 0,
   hard failures 0, G500 2/4 each, all candidate zero epochs valid; advance PASS.
3. DONE locally: complete report, 8-row CSV and visually checked PNG/PDF. Publisher
   exec 46924 exit 0. Post-hoc audit exec 63584 exit 0 reconciles every cumulative
   scale increment to per-agent status: skew 240 all before freeze; identical
   old SE(2) prefix over 789 frames plus packed scans/full step fields. C has
   1 acquisition and 7 post-freeze events. No outcome or gate changed by the audit.
4. DONE: one final fresh-context review of 77e5ed9..dcfdd53, no
   Critical/Important/Minor findings. It independently verified eight receipts,
   recomputed episode rows, four exact baselines, counter/prefix audits and hashes.
   docs/results/task_translation_physical_v2/final_review.md records its scope.
   Synchronize the already-authorized research branch after the review record.
5. Infer the next physical/control step from all outcomes. The full goal still
   requires reliability over wider/unused conditions, final matched deployment
   contribution and an evidence-aligned CCTA2027 paper, not just this pilot.

Ruling: preserve the unexecuted v1 freeze and create v2 before data collection —
the timeout counter was wrong, even though it did not affect online control or
the primary gate. Cost: one additional small frozen provenance record; no extra
physical episode or outcome-dependent change.

Ruling: use the unchanged tested process runner with a fixed simulation-step
budget, four workers and one internal thread, recording elapsed wall time instead
of adding a new wall-clock kill path — every old/new task receives the same
physical opportunity and all outcomes remain in denominator. Cost: difficult
candidate cases may take longer in real time; no conclusion before terminal data.

Ruling: validate the new evaluator against independently published old physical
outcomes and an actual old watchdog receipt before execution — this checks frame
alignment and the observed accounting failure without another scientific run.
Runtime itself has not changed since the reviewed 884-pass version. Cost: these
checks do not establish candidate performance or cover unobserved process errors.

Ruling: retain the original pilot PASS while prioritizing the newly measured
acquisition regression before broadening adoption claims — the gate preserves
existing G500 passes but does not cap counts in already-failing conditions; skew
rose from 2 to 240 scalings despite hard-safety counts staying zero. Cost: wider
validation waits for mechanism analysis; no hiding the secondary adverse outcome.

Ruling: retire the current unexecuted v1 CLI after preserving its frozen source —
otherwise a future invocation could accidentally launch the superseded eight
episodes. The CLI refusal was observed exit 1 with run/started still absent; its
original frozen script remains unchanged. Cost: current v1 helper differs by the
explicit refusal from its frozen historical copy; only v2 is executable here.

Final: Ruling: general reliability, dynamic safety certification, larger rotations,
deployment superiority, locked-environment physical reproduction and adoption
remain unjudged by this bounded review — keep each unfulfilled and the candidate
off by default. Cost if treated otherwise: unsupported claims outside the four
reused development cases. No fixes or minors were deferred.

This increment is complete: eight new episodes and all prespecified outcomes,
exact baseline reproduction, adverse-scaling diagnosis, curated report/figures,
repository checks and independent final review. Full goal remains ACTIVE.
