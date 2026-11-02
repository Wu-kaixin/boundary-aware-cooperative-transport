# Position, velocity and admission: a bounded transport-feedback study

Mission: `2026-11-01-feedback-components-v1`. Simulation only, no attachment,
unchanged physical and task/safety criteria, unchanged manuscript/title. The
default controller remains `navigation_estimator=accumulated`.

## Existing logs, examined before the intervention

Eight frozen episodes were inspected: previous vector-CVT and observed-reference
CVT on C-away, circle, parallel ellipse and perpendicular ellipse. Replaying the
phase monitor on saved inputs reproduces every phase response exactly. Its
`transport_active` signal counts the **previous** control frame's designated
pushers, not actual physical contacts. Recorded observer frame 1 corresponds to
controller frame 0; analysis checks this alignment explicitly.

The C-away observed-reference episode reaches CONTACT_READY at 3.8 s and
TRANSPORT at 9.0 s. At 13.9 s, twenty consecutive zero-pusher signals suspend
pushing. Seven robots still report contact-ready, but the two actual contacts
then exert force away from the physical goal. There are sixteen accepted
reference readouts. After eighty paused frames (4 s), BRAKE is entered at 17.9 s
with reason `contact_recovery_failed`; physical goal error remains 1.070 m.
This is not an unavailable estimate or a near-goal HOLD transition.

The previous vector-CVT episode reaches TRANSPORT later, at 32.8 s, and enters
BRAKE at 285.95 s for estimated stopping. Thus a final-frame explanation alone
misses an early change in contact evolution. This temporal comparison is not
enough to assign the change to velocity: position readout also rotates the
desired push direction and changes the active arc, while admission changes the
set of feedback voters.

Reference velocity was reconstructed from accepted displacement readouts and
the frozen filter, with a maximum shared-speed residual below 1.4e-15 m/s over
the four reference episodes. This is a roundoff-checked reconstruction, not a
previously recorded raw velocity signal. New episodes additionally log actual
local navigation velocity, reference velocity and the exact shared navigation
message so that this reconstruction is unnecessary in future analyses.

Full traces remain available; the first-40-second timing figure is an explicitly
cropped diagnostic view, without smoothing. The old vector episode's later
BRAKE lies outside that view. Actual forces use evaluation truth only and are
reported before ground friction. A designated pusher, CONTACT_READY and useful
physical contact are different quantities.

## Why a gate-only control is necessary

The last estimator replaced both readout channels **and** the admission rule.
Two mixed-channel arms alone would not separate these effects. The new 2x2
design therefore uses the same fresh accepted reference readings for admission
in all four arms, after CONTACT_READY:

| Arm | Position readout | Velocity readout | Admission |
|---|---|---|---|
| `gate_cvt` | Existing gained map displacement | Existing map velocity | Fresh accepted reference |
| `position_cvt` | Fixed observed-reference displacement | Existing map velocity | Same rule |
| `velocity_cvt` | Existing gained map displacement | Fixed observed-reference velocity | Same rule |
| `reference_cvt` | Fixed observed-reference displacement | Fixed observed-reference velocity | Same rule |

The gate-only arm is an explicit diagnostic intervention; its old values are
not newly certified by the reference gate. It is **not the original vector-CVT**.
Within a given recorded input all four arms have the same admitted IDs; across
closed-loop trajectories those IDs can diverge as an outcome of the intervention.
No missing readout becomes a fresh zero or a stale fallback.

Before CONTACT_READY, all arms keep the original feedback. The reference fit,
bootstrap, gains, quality thresholds and filtering remain byte-identical to the
previous estimator module. Local messages remain own/one-hop; the existing
all-team median phase supervisor is explicitly retained. Position influences
steering/arrival; velocity influences convoy motion, effort and the common slow
guard. This study does not further separate the consumers of velocity.

## Budget and interpretation fixed before execution

Nine existing shape–orientation conditions, reused development seed 2, four
arms: **36 new episodes**, each capped at the existing 600 physical seconds.
Eight workers, one internal thread, one version, at most 60 minutes batch wall
time. The nine both-reference episodes repeat the previous candidate and must
reproduce its saved arrays and historical per-frame fields. Original CVT,
spacing and vector-CVT are retained as **27 historical paired task references**
with verified receipts, source identities and the same physical/configuration
budget; they are not relabeled as new executions.

Primary mechanistic endpoint: C-away completion under the original HOLD,
physical settling and 0.12 m endpoint definition, with original hard limits
reported separately from G500. All nine conditions, off-target settling, errors,
cutoffs, contact events and adverse results remain in the table. The 150 s view
comes from the same runs, not a separate sample. No threshold, gain, horizon or
seed is adjusted after the outcome.

For each condition and each separate outcome Y, the conditional position effects
are `Y10-Y00` and `Y11-Y01`; velocity effects are `Y01-Y00` and `Y11-Y10`.
`Y11-Y10-Y01+Y00` describes their interaction on that outcome's scale. This is a
paired development intervention, not an additive physical law or an estimate of
population reliability. Frames and contact samples are not independent trials.

- If velocity-only reproduces early contact loss while position-only avoids it,
  investigate velocity consumers before proposing a new contact controller.
- If position-only produces that loss, inspect steering and push membership.
- If gate-only changes outcomes, admission must remain part of the explanation.
- If only the combination changes outcomes, retain the interaction instead of
  assigning blame to one channel.
- If none rescues C-away, stop this component search at its fixed budget; use its
  evidence to decide whether contact reconfiguration is the next problem.

These branches guide interpretation; none authorizes extra episodes inside this
mission. No candidate becomes the default or gains a reliability claim from this
reused seed. All frozen predecessor conclusions remain unchanged.

## Recovery-loop finding during the frozen comparison

A further read-only audit found an implementation limitation in the existing
pause/resume path. Let `a_i(k)` denote whether a robot passes all current pusher
predicates except global suspension, and let `S(k)` denote that suspension.
The enacted flag is `p_i(k) = (1-S(k)) a_i(k)`. The phase supervisor's next
active count is `A(k+1) = sum_i p_i(k)`. Once paused, every enacted flag is zero,
so the same count cannot build the positive quorum required to clear the pause.
The timeout eventually enters BRAKE and clears suspension; that is not a
successful TRANSPORT resume. This concerns the inspected vector-transport path,
not a claim that any physical contact is impossible.

The phase implementation and pusher expression match the previous frozen source.
All robots' C-away pusher decisions over 12,000 frames were reproduced from
recorded predicates with zero mismatches. Its 80 paused frames had zero enacted pushers and zero active
signals throughout. However, the shadow count that removes only suspension
also never reached the four-pusher quorum during that pause. Thus the code-level
unreachable recovery guard is real, but merely changing its counter is not
shown to rescue this specific configuration. The shadow count is geometric
eligibility, not measured physical contact or a safety certificate.

This issue was found after the comparison was frozen. The current batch keeps
the existing phase logic unchanged for all arms; a correction must have a
separate execution identity and validation. Its effects must not be silently
folded into a position/velocity ablation. The local evidence is
`artifacts/research/2026-11-01-feedback-components-v1/recovery-loop-audit.json`.

## Execution and provenance

- Log diagnosis: `scripts/diagnose_feedback_contact_loss.py`.
- Freeze: `scripts/freeze_feedback_components.py`; refuses an existing protocol.
- Frozen protocol: `experiments/ccta2027/feedback_components_v1.json`.
- Raw new runs: `runs/research/2026-11-01-feedback-components-v1/run/`.
- Analysis: `artifacts/research/2026-11-01-feedback-components-v1/`.
- Protection and state: `artifacts/migrations/2026-11-01-feedback-components-v1/`.
- Regression logs: `runs/maintenance/2026-11-01-feedback-components-v1/`.

Completed outcomes are appended only after the corresponding receipts exist.

## Completed fixed comparison

All 36 executions finished without an execution error in 2,350.67 wall seconds,
within the predeclared 3,600-second resource bound. No extra run, gain change or
seed replacement was made. The four common-admission arms completed 3/9, 6/9,
4/9 and 6/9 tasks; G500 accepted 2/9, 2/9, 1/9 and 3/9 respectively. Original
CVT, spacing and vector-CVT remain 27 separately identified historical references,
each with 5/9 task completions, and G500 counts 5/9, 4/9 and 1/9.

Position-only and both-reference preserve the same five completed conditions
and add the perpendicular ellipse. They terminate at 342.20 and 483.85 seconds,
with endpoint errors 0.04576 and 0.04478 m. Their 24 and 25 barrier scalings
still fail G500. Position-only is therefore a useful simpler development option,
not a generally superior replacement: its total G500 count is lower than both,
and the remaining conditions contain time/error tradeoffs.

Circle, parallel ellipse and C-away fail under all four combinations. C-away
endpoint errors are 0.405178, 0.721097, 0.345503 and 0.783854 m. Changing
velocity reduces error by 59.7 mm with old position, but increases it by 62.8 mm
with new position. This interaction rejects a single-channel account of the
closed-loop failure. It is not evidence that more accurate displacement alone
must produce a better contact trajectory.

The gate-only arm settles just outside tolerance in both L conditions; the
velocity-only arm settles outside tolerance in the perpendicular ellipse and
C-toward. These outcomes remain failures under the original 0.12 m rule.
Across trajectories, estimator MAE is not a same-input accuracy experiment.

Gate-only also introduces two hard-limit violations: parallel ellipse exceeds
its recorded penetration budget and has negative signed clearance; perpendicular
ellipse exceeds its recorded penetration budget. These are adverse outcomes of
the experimental admission change, retained without changing any threshold.
The other three new arms have no recorded violation of these checked hard
limits, which is narrower than G500 or a general dynamic-safety guarantee.

A post-hoc fixed final-10-second C-away force audit uses evaluation truth only.
The unchanged model gives a translational static-friction threshold of
4.1175 kg × 0.6 × 9.81 m/s² = 24.235605 N. At all 200 sampled control starts,
position-only, velocity-only and both-reference have net contact force below
that threshold and zero subsequent recorded speed. Gate-only exceeds it at
14 samples but still does not arrive. This is consistent with insufficient
late net force, not an all-substep reachability proof or authorization to relax
friction, penetration or contact limits.

The decision is to stop this feedback-component search, retain the useful
reference-position options, and isolate the recoverability bug from any new
contact-reconfiguration method. No experimental option becomes the default.
The [complete results](results/feedback_components_v1/README.md),
[condition table](results/feedback_components_v1/capability_table.md) and
[research decision](results/feedback_components_v1/DECISION.md) preserve the
adverse outcomes, original criteria and next-stage stopping conditions.
