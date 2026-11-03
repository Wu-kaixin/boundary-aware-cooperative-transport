# Finite acquisition support candidate

Base: 35c853373ac0190c0459f0f0514d9690c118fc14. Existing protected numerical
parameters, physics, original success/safety gates and old run identities remain.
The user authorized subsequent research decisions; implement inline with one final
review. The preceding three-alternative probe is frozen separately and remains
negative: existing one-hop flat triples cover only16/26 nearby-return sets and
overestimate true clearance in3 states. Own aggregate has2 empty-support states.

Implement an empirical continuous-ray chord cover, disabled by default. Each scan
is processed separately using only point, ray_index, object_id, source origin and
timestamp. No simulator normals, edges, pose, centroid or shape enters the packet.
Join consecutive rays on the same object only when chord length is positive and
at most twice the existing GAP_GUARD_CAP_M (0.04 m). Give the chord a structural
allowance of half its length. This is an EMPIRICAL_ONLY guard for unsampled local
shape, not a certificate for arbitrary unseen continuation. Do not add a new
tuning parameter or bridge a missing ray/object/source boundary.

Use fresh own and one-hop packets only. All returns within the uncertainty-aware
nearest-return hold cover must be endpoints of accepted chords; otherwise retain
the original aggregate rows. Empty support, stale packets and sparse returns must
never count as successful constraint removal. Keep every competing chord within
the uncertainty-aware one-hold cover; do not truncate to max_object_rows.

Acquisition already uses r_safe = robot_radius + 3*range_noise_std + 2*max_speed*dt.
Count the existing range-noise allowance exactly once: geometric lower distance
is distance_to_chord - half_chord - 3*noise_std, while the clearance target for
that lower distance is r_safe - 3*noise_std. Thus h = distance_to_chord - half_chord
- r_safe. Preserve r_safe itself. No noise or hold margin is lowered.

Motion must be an explicit direction-independent nonnegative finite scalar bound;
its contribution is that scalar for each unit radial row, never the projection
of a velocity aligned with an unrelated old face normal. The feature-switch cover
includes both robot and boundary motion. Retain original rho, gamma, reach cap,
agent/wall constraints, speed bound and relaxation accounting.

First implement and test the observational geometry module, then run a separate
all26-state probe with zero and original bounded motion and independent solver
checks. The initial map's stationary-acquisition model is unchanged and is not a
safety certificate. Runtime wiring is allowed only after checking the measured
coverage/failures. If wired, use a default-false acquisition_finite_support flag,
require the existing reference-map/aggregate research stack, and apply only in
stationary_acquisition; after that keep the original control path. Record guard
usage/fallback reasons. Reject incompatible modes rather than silently ignoring
missing motion bounds. Preserve disabled behavior and information locality.

Tests must exercise hand-derived line/corner/end-point rows, strict gaps/objects/
sources, noisy/invalid input, age, empty/sparse support, moving-bound coverage,
zero-distance normals and unchanged original rows on fallback. A later matched
physical study gets its own protocol freeze: four reused conditions × two arms,
no shape-specific adjustment, all original gates and original-control exact
reproduction. Report acquisition scaling, post-freeze scaling, hard safety,
false HOLD, full task completion, G500 and C activation separately.
