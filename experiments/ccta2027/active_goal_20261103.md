# Active research goal and completion ledger

User goal, 2026-11-03: use local sensing and neighborhood communication to
safely deploy around unknown shapes, transport without attachment, reliably
stop, separately demonstrate the deployment contribution, and finish a
CCTA2027-compliant paper draft. Address the existing issues first, then continue
toward the entire goal. This ledger does not narrow that objective.

Status: ACTIVE. Previous completed turn was progress: the free-ray diagnostic,
report and README were published as 3a9dcda / 1eded7c, both platform checks passed
(872 core tests, two skips on each). No blocking condition has been established.

## Requirements and current evidence

| Requirement | Authoritative evidence | Current assessment / required work |
| --- | --- | --- |
| Preserve best common numerical parameters | configs/research/best_known_transport_v1.yaml; protected_baseline.json and receipt test | Protected; preserve during mechanism work. |
| Safe deployment and unattached transport using allowed information | Existing controller, simulation contact model, 33-condition / 12-start receipts | Partial simulation evidence; shared phase supervisor must be acknowledged. |
| Reliable goal-aligned displacement and stopping | transport_validation_v1 stop audit; task_reference_v1; reference_surface_v1 | Incomplete: far-skew false HOLD; initial-reference stalls; no accepted repair. |
| Dynamic safety and generalization appropriate to claims | G500 / hard-safety receipts and theory/static_deployment | Incomplete; static conditions do not prove dynamic safety. Freeze new conditions only after a usable common controller. |
| Independent boundary-deployment contribution | deployment_contribution_v1, 36 existing matched trajectories | Coverage contribution supported; completion superiority unproven. New matched evaluation required for the final system. |
| Reproducible environment | root .venv and requirements.lock.txt | Existing PyYAML 6.0.2 / 6.0.3 identity difference must be handled in a separate declared check, never by relabeling old runs. |
| CCTA2027 paper draft | manuscript/ccta2027/main.tex / main.pdf; official conference requirements | Existing draft covers old 27-episode census. Reconcile final evidence and claims, recheck official requirements, compile and visually inspect final draft. |

## Current architecture decision

Repeated changes to gains, vote continuity, rejection and stop confirmation have
not produced a reliable common system. Stop adding empirical gates to the same
biased state. First test whether existing sensor information can constrain the
unknown center and whether the transform relevant to that set is observable.

Current work: `task_translation_v1_design.md` and `task_translation_v1_plan.md`.
They test motion-model ambiguity and whether already-captured initial templates
were discarded unnecessarily, in a fixed four-arm same-input comparison.
Translation-only yaw is an explicit hypothesis, not a measured zero or a
rotation-robust stopping certificate. Preserve the free-ray diagnostic results.

Current execution: corrected protocol task_translation_v2.json, exec session
3893, log runs/maintenance/2026-11-03-task-translation-v2/study.log. Re-poll the
actual handle; never infer liveness from this note. Original v1 stopped on a
strict task-vector arithmetic mismatch and is preserved as invalidated, with
no completed candidate result. Runtime tests now pass 876 with two skips.
The latest official paper-format check is recorded separately in
docs/CCTA2027_REQUIREMENTS_CHECK_2026-11-03.md; manuscript scientific completion
is still unproven and the original full goal remains active.

Publication checkpoint captured at 2026-11-02 19:32:52 UTC:
docs/results/task_translation_checkpoint_v1/. Three of four conditions have
complete same-input results; the far-skew condition remains pending. Overall
screening is INCOMPLETE, with the original sustained-feedback gate already
failed for L and C. The user requested GitHub upload and README updates, so this
dated checkpoint is being published without waiting for or altering the live
run. The fourth case must still be collected; none of these offline results
establishes a physical repair or completion of the original full goal.

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
