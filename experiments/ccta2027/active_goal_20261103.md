# Active research goal and completion ledger

User goal, 2026-11-03: use local sensing and neighborhood communication to
safely deploy around unknown shapes, transport without attachment, reliably
stop, separately demonstrate the deployment contribution, and finish a
CCTA2027-compliant paper draft. Address the existing issues first, then continue
toward the entire goal. This ledger does not narrow that objective.

Status: ACTIVE. Latest published checkpoint is 77e5ed9; the new eight-episode
physical pilot and acquisition-barrier audit are the current publication increment.
No blocking condition has been established. The full research goal is incomplete.

## Requirements and current evidence

| Requirement | Authoritative evidence | Current assessment / required work |
| --- | --- | --- |
| Preserve best common numerical parameters | configs/research/best_known_transport_v1.yaml; protected_baseline.json and receipt test | Protected; preserve during mechanism work. |
| Safe deployment and unattached transport using allowed information | Existing controller, simulation contact model, 33-condition / 12-start receipts | Partial simulation evidence; shared phase supervisor must be acknowledged. |
| Reliable goal-aligned displacement and stopping | task_translation_physical_v2; earlier transport_validation_v1 / task_reference_v1 | Four reused conditions now safely complete with no false HOLD; default-off candidate still needs acquisition safety, wider protected conditions and unused-condition validation. |
| Dynamic safety and generalization appropriate to claims | G500 / hard-safety receipts and theory/static_deployment | Incomplete; static conditions do not prove dynamic safety. Freeze new conditions only after a usable common controller. |
| Independent boundary-deployment contribution | deployment_contribution_v1, 36 existing matched trajectories | Coverage contribution supported; completion superiority unproven. New matched evaluation required for the final system. |
| Reproducible environment | root .venv and requirements.lock.txt | Installed joblib 1.5.1 / PyYAML 6.0.2 differ from locked 1.6.0 / 6.0.3. Handle in a separate declared check, never by relabeling old runs. |
| CCTA2027 paper draft | manuscript/ccta2027/main.tex / main.pdf; official conference requirements | Existing draft covers old 27-episode census. Reconcile final evidence and claims, recheck official requirements, compile and visually inspect final draft. |

## Current architecture decision

Repeated changes to gains, vote continuity, rejection and stop confirmation have
not produced a reliable common system. Stop adding empirical gates to the same
biased state. First test whether existing sensor information can constrain the
unknown center and whether the transform relevant to that set is observable.

Completed comparison: `task_translation_v1_design.md` and the independently
frozen `task_translation_completion_v1_design.md` test motion-model ambiguity
and whether already-captured initial templates were discarded unnecessarily,
in a fixed four-arm same-input comparison.
Translation-only yaw is an explicit hypothesis, not a measured zero or a
rotation-robust stopping certificate. Preserve the free-ray diagnostic results.

Current evidence: task_translation_physical_v2 completed 8 fresh physical episodes
(exec 11579 exit 0; 210.4 s wall time, no execution error). Safe completion 3/4 to
4/4, false HOLD 1 to 0, hard safety 0 in both arms, G500 2/4 in both. All four
baselines exactly reproduced 25 arrays and scientific JSON fields; candidate
zero epochs are valid. Original prospective advance gate PASS, not adoption.
The new issue is severe acquisition scaling: skew 240 events, all before freeze;
789 control frames, packed scans and complete step fields exactly match the old
SE(2) initial-reference acquisition prefix. C has 1 acquisition and 7 post-freeze
scalings. Next prioritize acquisition geometry/velocity/safety consistency with
numeric parameters unchanged, then wider/unused conditions and final matched
deployment comparisons. Report: docs/results/task_translation_physical_v2/.
The unexecuted v1 freeze is preserved; v2 fixed timeout-label bookkeeping before
new physics. See task_translation_physical_v2_plan.md for execution and review.

Historical replay evidence: all four conditions in task_translation_completion_v1 are
complete, and the original screen is FAIL (L/C/skew sustained-feedback criterion).
Session 25427 is terminal, exit 1 after writing the complete skew data: a path
join typo failed only final aggregation. Original failure/source snapshots are
preserved; separate zero-update finalization completed (session 30915, exit 0).
Report: docs/results/task_translation_completion_v1/. Original v2 session 3893
remains a retained budget stop; v1 remains a retained arithmetic invalidation.
The candidate remains an explicit default-off option after the completed pilot.
The RED-to-GREEN record and 884-pass / 2-skip core verification are recorded in
task_translation_wiring_plan.md. Published 77e5ed9 passed both CI platforms on
the same runtime used in the new physical pilot; research helpers have separate
freeze/run/evaluation checks. Wider conditions and adoption remain unestablished.
The latest official paper-format check is recorded separately in
docs/CCTA2027_REQUIREMENTS_CHECK_2026-11-03.md; manuscript scientific completion
is still unproven and the original full goal remains active.

Historical publication checkpoint captured at 2026-11-02 19:32:52 UTC:
docs/results/task_translation_checkpoint_v1/. Three of four conditions have
complete same-input results; the far-skew condition remains pending. Overall
screening was INCOMPLETE, with the original sustained-feedback gate already
failed for L and C. That snapshot was published while the fourth case was still
pending and remains unchanged. The later complete report supersedes it for
current status, without establishing a physical repair or full-goal completion.

2026-11-03 progress: the frozen four-snapshot audit completed in 147.38 s,
with zero new physical episodes. All 132 conditional sets contained the true
object and center; none of 128 local sets contracted. All-team input contracted
only the near circle and L cases, and added no bound-plus-quorum pass among
16,687 old basic-stop-eligible frames. The component remains unwired. The
scientific conclusion is published in docs/results/free_ray_enclosure_v1/.
Next: acquisition-history coverage and safe additional observation; explicitly
bounded neighbor relaying; separately evaluate task displacement / target-aligned
readouts and real-rotation mismatch. These are dependencies toward the full goal,
not replacements for reliable closed-loop stopping or the final paper.

Completion requires direct current evidence for every requirement above. An
offline result, a single reproduction, passing unit tests, or a paper that merely
describes an unresolved system is insufficient to mark the full goal complete.
