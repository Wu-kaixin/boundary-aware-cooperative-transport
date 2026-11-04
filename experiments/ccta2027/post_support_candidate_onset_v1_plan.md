# Actual candidate onset implementation ledger

Base: 35d683cc2eb332c8dbc274f51612093f5e4e271d.
Design: post_support_candidate_onset_v1_design.md.

1. Add a bounded receipt reader. Expected: 15 genuine stub failures, then all
   boundary/corruption/bounded-read tests pass. Completed: 15 RED NotImplemented
   failures then 15 GREEN; receipts in runs/maintenance/2026-11-03-candidate-onset-v1.
2. Stream all eight trajectories, retain every frame and predefined events,
   verify original hashes/NPZ/summary and locate exact first command divergence.
   Expected: complete denominators with no silently discarded frames.
3. Exact no-physics replay to first divergence for both arms of every pair;
   preserve actual matrices/nominals/inputs and verified directional certificates.
   Expected: exact saved controller outputs, explicit capability categories.
4. Independently check analysis identities/certificates, run appropriate tests,
   one final fresh-context review, document evidence/limits and publish.

The original full audit session 56448 remains live and untouched. This incremental
design adds only new analysis files outside its frozen helper closure. Existing
worktree is the authorized research branch. Publishing is already authorized.

Ruling: use normal local file tools after Serena's language server fails to start;
the failure does not affect Python execution or require a project change. Risk:
manual source navigation requires explicit file/line verification.

Ruling: keep ledger and research receipts in repository conventions instead of
creating/deleting a separate skill workspace. Historical receipts must remain.

Review focus: premature JSON success/truncation, all denominators, pre/post frame
alignment, actual versus nominal commands, matched-pair first-divergence claims,
complete-system feasibility certificates, source immutability, and avoiding
physical/causal claims from instantaneous saved-state analysis.

Attempt v1 preserved: first L candidate 12000 frames passed all frame/summary
checks, then a control audit's explicit null post_support_audit caused an
AttributeError. Fixed null handling; the complete analysis will run in a fresh
v2 output. Optional already-completed audit files are now copied at start so
their provenance is fixed while the separate original audit continues. No
scientific selectors or thresholds changed. The integration failure is the RED
reproduction; full eight-case completion is the required GREEN verification.

Stage scope changed by user on 2026-11-04: remain in Phase1 bug investigation
and fixes only; no new physical experiment or manuscript writing. Existing
no-physics replays and tests are bug-reproduction work. Publication plans that
only served the earlier open-ended experiment objective are no longer executed.

Task2 complete: v2 streamed and verified all8 episodes /50677 recorded frames;
first paired command differences77/71/381/443, all prior physical states and
current nominal/reference/phase fields identical. L705 capped residual events
individually matched to263 relaxed_margin +442 scaled_barrier statuses.
Task3 first-divergence replay complete:1944 control frames,128 systems.

First-pusher follow-up uses its own pre-execution design and the originally
selected actual-event frames110/96/515/620. First attempt preserved an empty
matrix shape failure after two completed cases. Added a direct regression:
test_empty_serialized_matrix_keeps_two_velocity_columns RED (0,) vs(0,2), then
GREEN with explicit reshape(-1,2). Combined16 tests passed. Core run started
before this new regression completed1102 passed/2 skipped; final source needs
the new test plus completion of the fixed four-event replay.
