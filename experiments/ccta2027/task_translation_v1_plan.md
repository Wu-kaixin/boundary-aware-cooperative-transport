# Task translation implementation and frozen comparison

Use superpowers:executing-plans inline, with one final independent review.
Spec: task_translation_v1_design.md. Root .venv only. Preserve all historical
results and the protected numerical baseline. User delegated design decisions.

## Review focus

- Translation-only means a model assumption, never measured zero rotation.
- No simulator geometry, task start, mass center or contact truth in a tracker.
- Every all-captured template existed before the original freeze; no late reset.
- Default SE(2) numerical operations and existing parameters remain unchanged.
- Exact old template/status/reading reproduction, independent original identities.
- All four crossed arms, all frames and unavailable data remain in denominators.
- Offline stop candidates are not new physical successes or safety guarantees.

## Tasks

- [x] Write failing mechanism tests for pure translation, circle yaw degeneracy,
  flat unsupported geometry, immutable/invalid epochs and rotation-model labeling.
- [x] Add a motion-increment hook preserving the default SE(2) operation, and a
  separate translation-only task tracker using the existing admission/filtering.
- [x] Implement and freeze a four-arm same-input replay with acquisition-buffer
  reconstruction and exact original reference reproduction.
- [ ] Execute once, retain all outcomes and decide the predeclared physical-trial gate.
- [x] Run appropriate regression, perform one whole-change independent review,
  and document the completed checkpoint for the requested branch publication.
- [ ] Continue toward a reliable full system, independent deployment contribution
  and final CCTA2027 paper; do not mark the full goal complete at this screening step.

## Ledger

Ruling: test both motion model and captured-template admission in a fixed crossed
comparison before adding active movement — the previous single-snapshot study
cannot say whether already-collected history was discarded unnecessarily — if
neither produces reliable feedback, safe active observation remains required.

2026-11-03: new tests first failed at collection (new module absent), then passed.
The four new mechanism tests plus initial/reference/surface regressions total
36 passes. Default operation is a method extraction only; exact historical
same-input replay is the stronger numerical preservation check in the study.

Ruling: preserve v1 as an invalidated diagnostic attempt and freeze v2 after
correcting only task-vector reconstruction — strict original navigation equality
detected goal_point-start versus distance*direction roundoff at the first
post-freeze frame (2.22e-16) — loosening that equality would weaken the promised
baseline check. No case summary or candidate outcome completed; arms, source
component, numerical parameters, cases, budget and screening thresholds unchanged.

2026-11-03: v2 froze source dc425ae9f3e4ee96fbe9290505364c32b0c0a20dbec76b7991ae05f87f5fcaa0.
Run launched at 2026-11-02 18:43:52 UTC via exec session 3893. At 18:52 UTC,
the live handle was re-polled and the log reached near_circle frame 4000/12000;
strict original template/status/reading/navigation assertions have passed that
prefix. Session 7611 (v1) is terminal, exit 1, with invalidated.json preserved.
Session 23213 (core tests) is terminal, exit 0: 876 passed, 2 skipped, 147.14 s.
These tests use the same runtime source as v2; the correction changed only the
offline script and output/protocol paths. No candidate scientific outcome yet.

Next continuation MUST poll actual session 3893 before acting; this ledger or
an empty log is not proof of liveness/termination. Do not restart, change frozen
source/scripts, or overwrite v1/v2 outputs while it is live. Keep the original
3600-second budget; a timeout is a retained resource/execution outcome, not a
scientific FAIL or grounds to omit cases. All four cases must be evaluated under
unchanged gates before the preselected candidate can enter physical trials.

2026-11-03 publication preparation: the user explicitly requests GitHub upload,
README refresh and an account of remaining problems. Publish to the existing
paper/boundary-aware-safe-deployment branch after the whole-change review; no
new integration choice or main-branch merge is required. Original local raw
data, source snapshots, failed v1 attempt and this ledger remain preserved.

2026-11-03 partial readout (not a final study decision): circle, L and far C
completed; far skew is still executing under the original budget. Preselected
paired MAE is 1.381 / 3.062 / 14.270 mm versus 2.687 / 4.117 / 58.850 mm for
the exactly reproduced old reference. The L and C sustained-feedback screens
fail (17 and 0 consecutive frames versus the frozen minimum 40). Retain these
outcomes, regardless of the remaining case.

Ruling: record a separately labeled post-hoc screen calibration, without changing
the frozen gate — the original successful L task has 40 physically quiet HOLD
frames but only 9 consecutive estimator-qualified stop frames; the preselected
candidate has 38 qualifying frames in that old 40-frame window and a longest
streak of 17. Confusing the two definitions would misstate what the gate tests.
If this interpretation is wrong, a later exploratory physical-trial rationale
would be too permissive; therefore no candidate is adopted or trial authorized
by this calibration alone.

Ruling: fulfill the user's explicit publication request with a dated partial
checkpoint while the frozen replay continues — this delivery includes all four
arms of the three completed cases and explicitly lists the fourth as pending.
Overall screening remains INCOMPLETE; known failed per-case gates remain FAIL.
Do not change the live source, extend its 3600-second budget, or imply a completed
physical repair. Cost if readers miss the scope: overgeneralizing three old
conditions; the root README, report and machine-readable status all state it.

The final fresh-context review for this publication covers the full increment
since 1eded7c, including the unwired prototype, frozen protocols, checkpoint,
calibration, rotation audit and documentation. The remaining scientific execution
task stays open; this publication is not completion of the original full goal.

2026-11-03 final publication review: fresh reviewer examined 1eded7c..04e052e,
including all 12 completed-arm denominators, hashes, original identities, source
snapshots and test XML. Critical / Important / Minor findings: 0 / 0 / 0.
The exact checkpoint is fit to push. No source fixes or second review required.

Final: Ruling: the reviewer set aside fourth-case completion and final screening —
keep the dated checkpoint INCOMPLETE and collect the eventual run outcome
separately; otherwise missing data could be mistaken for a passing study.
Final: Ruling: the reviewer set aside closed-loop improvement and adoption — no
runtime wiring or new physical trial was performed, so neither is claimed;
otherwise offline accuracy could be mistaken for reliable stopping.
Final: Ruling: the reviewer set aside generalization, substantial rotation and
dynamic-safety guarantees — these remain explicit validation requirements;
otherwise the translation model could be used outside its evidence range.
Final: Ruling: the reviewer set aside final manuscript completeness — the format
check and evidence update do not complete the paper; otherwise obsolete results
could be presented as validation of the final system.
Final: Ruling: the reviewer set aside the pre-existing PyYAML/lock discrepancy —
preserve the live frozen environment and reconcile it in a separate declared
execution; otherwise old results could be assigned the wrong environment.

User authorization already selects a normal push to the existing research
branch. Keep this checkout and all raw/snapshot/ledger files; do not merge main,
force-push, delete historical records, or ask for another integration choice.
