# Initial task translation and captured-template admission: fixed 2 x 2 study

2026-11-03, starting HEAD 1eded7c. Full objective remains in
active_goal_20261103.md. Previous goal turn was progress: the conditional
free-ray component and four-snapshot evidence were published, with both CI
platforms passing. There is no blocker.

## Mechanism and scope

The initial task reference still fits SE(2) on partial noisy geometry. Its
rotation check did not prevent invented circle yaw, and its center bound often
rejects stopping. Acquisition freezes at the first four strongly supported
templates; other robots' already-captured initial observations are discarded as
potential references even though some may support a translation fit.

Test two mechanisms separately, without another gain or stopping-gate change:
(1) a translation-only fit to the immutable zero-epoch template; (2) initialize
every robot from its own already-captured initial template using the existing
tracker bootstrap admission. A newly admitted robot receives no later zero
reset and no other robot's full map. These are two crossed factors, not a
per-shape policy. No current runtime/controller wiring changes in this stage.

The translation hypothesis is EXACT for pure translation, including circular
geometry whose yaw is unobservable. It is only an approximation for a rotating
noncircular object; a zero fitted angle is NOT a measured zero physical yaw and
must not be called a center-error certificate. Record true rotations only in the
evaluator. Successful development screening does not remove this limitation or
constitute closed-loop reliability evidence.

Keep every best-known numerical parameter, the original acquisition policy and
epochs, old bootstrap/match count and normal-rank tests, matched-support gate,
five registration iterations, residual rejection, speed bound, velocity filter,
four-voter quorum, 20 mm arrival band, 20 mm/s estimated hold speed, 120 mm task
tolerance, 40-frame physical settling rule and safety evaluator. In the
translation model only the two translation columns are fitted; the yaw-specific
geometry rejection has no corresponding estimated degree of freedom. SE(2)
arms retain the existing yaw check. No clipping of physical truth or errors.

## Four arms and four reused development trajectories

Use every original `arm=task_reference` case in task_reference_v1: near circle,
near L, far C, far skew box. Same control-before-physics scans for every arm:

1. se2_supported: exact reproduction of the original task reference.
2. translation_supported: same supporters, two-dimensional motion model.
3. se2_all_captured: old motion model, all already-captured initial templates.
4. translation_all_captured: both changes, the preselected candidate.

Reconstruct ALL acquisition frames to the recorded freeze epoch using the
original deterministic scanner, merge only own/current one-hop packets, and
apply the original bounded voxel capture. Compare supported original template
records exactly, then verify every original tracker status, reading and team
navigation output after freeze against the old diagnostics. Keep unavailable
robots and frames in the output. Validate original manifests, independent case
identities, raw products and source snapshots; freeze new source/scripts and
environment before execution. Existing best parameter file hash must match.

## Evaluation and screening gate declared before outcomes

For each arm report all local statuses, fresh team availability and quorum,
all-available and paired-frame displacement MAE/RMSE/P95/max, nominal goal error,
true-rotation range, and every potential stop frame after old CONTACT_READY.
Potential stop uses the same point-goal feedback, speed and quorum thresholds.
The historical SE(2) workspace gate remains in the SE(2) counterfactual; for
translation arms its algebra is the maximum local goal error under the declared
translation hypothesis, not a rotation-robust certificate.

For translation_all_captured to justify a bounded physical development trial,
all four conditions must have: paired availability relative to original >= .9;
paired MAE <= .03 m and <= original + .01 m; zero potential goal-stops outside
the existing .12 m task tolerance; and at least 40 consecutive potential-stop
frames with true error <= .12 m. Also report unavailable/error-frame denominators
and insufficient-quorum frames, not just the selected interval. This 40-frame
screen is evidence of sustained feedback on old data, NOT a new controller gate
or an assertion of physical settling. Retain failures without tuning or choosing
different arms by shape. All other arms are mechanism controls, not fallback
candidates if the preselected arm fails.

Budget: four reused trajectories, four estimator arms, one worker/internal
thread, 3600 seconds wall time, zero physical episodes. A timeout or integrity
failure is preserved; do not restart an unobserved live process or overwrite a
started output. New runs/research/2026-11-03-task-translation-v1 only.

## Relation to the full objective

Free-ray results remain a valid acquisition-coverage limitation. Reusing the
actual historical acquisition buffers is a direct test of what information was
discarded; it does not solve missing boundary coverage. Any candidate entering
physical trials must retain prior task successes, rescue false stopping, include
rotation/model-mismatch checks, preserve safety and all numerical parameters,
and later pass unused conditions and a matched deployment-method comparison.
Active observation and explicit neighbor relaying remain needed if existing
captured data cannot supply useful task feedback. CCTA2027 manuscript completion
remains downstream of supported current evidence, not of passing this screen.
