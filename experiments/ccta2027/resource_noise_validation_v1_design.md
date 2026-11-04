# Current-source resource and noise sensitivity validation

Frozen-design intent, 2026-11-04; to be bound to an expanded JSON protocol before
execution. This is a bounded validation of the protected best-known common
transport configuration, not a search for a replacement method.

## Question and comparison

Does the observed deployment/complete-task distinction persist when robot count
or sensor noise changes, under common local-information permissions? Evaluate
all three existing deployment modules, with no method-specific retuning:
`boundary_cvt`, `local_spacing`, `nearest_observed_target`.

Five previously studied geometry conditions are fixed now: circle,
ellipse-perpendicular, rectangle-parallel, L-reference, C-opening-away. Use the
normalized outlines in `cargo_near_v1.json`, one common annulus and +x goal of
1.2 m. Freeze the protected `best_known_transport_v1.yaml` controller/physics
and the actual current source. Default-off rejected options remain off.

Four regimes: reference (16 robots, range sigma 0.01 m); fewer robots (8,
0.01 m); more robots (24, 0.01 m); noisier sensing (16, 0.02 m). This is a
one-factor-at-a-time sensitivity check, not a factorial interaction study.
Keep the original four-pusher quorum and all gains/thresholds: any resource
failure under that fixed controller is a measured limitation. Check initial
clearance before freezing. Compare identical initial configurations and task
hashes across the three methods within every shape/regime block. No later
replacement for a difficult initialization is permitted.

## Budget and statistical unit

- Exactly 5 × 4 × 3 = 60 scientific episodes; seed 2026110401, checked against
  existing frozen protocols before execution. Shapes/regimes share that seed.
- At most 12000 control steps (600 simulated seconds) per case. Original HOLD
  and speed <= 0.005 m/s for 40 consecutive frames settling; independent spatial
  completion additionally needs final Euclidean error <= 0.12 m. No time extension.
- Single unseen seed is one random scene block; shapes and perturbations are
  designed conditions. Report counts, paired differences, and every case. No
  population confidence intervals or claim of general reliability.
- Primary outcomes: spatial completion and hard safety failure, all planned
  cases in denominators; execution failures/timeouts remain failures with reason.
- Secondary: CONTACT_READY/actual-contact/TRANSPORT/BRAKE/HOLD events, G500,
  coverage and maximum gap, contact force, endpoint/cross-track/direction/yaw,
  termination time, robot path, and barrier scaling. Distinguish task duration
  for failures from successful completion time.
- No superiority acceptance threshold is invented after results. For each
  perturbation report the within-method difference from its paired reference;
  no retuning, selective reruns, or promotion based solely on a favorable metric.

## Engineering, stopping, identity

Existing benchmark favors 8 workers over 1/4 for an older load. Before scientific
execution, compare a fixed 8-case, 100-step engineering prefix at 1/4/8 workers,
with one internal numerical thread and exact replay comparison; this subset is
excluded from science. Include the more costly 24-robot regime and profile one
of those same prefixes. Select the fastest tier without execution or identity
failure. This is a workload check, not a complete optimization over 48 threads.

Use the existing study runner, per-case hashed products, immutable source
snapshot, and verified completed-case resume. Freeze helpers/environment too.
Resource guard: 16 GiB available memory, 20 GiB free disk; 4-hour execution wall
budget. If a resource guard or execution failure prevents completion, preserve
partial output and report the planned denominator; do not select a new seed.
No controller/source changes during the batch. Simulation and later media
rendering remain separate. All new outputs use a fresh `runs/` directory.

Preselected real-trajectory media: reference CVT in all five shapes, all methods
for rectangle reference and C-away reference, plus complete-case outcome plots
for every regime. Failure trajectories retain their measured termination and
are not replaced by successful examples.
