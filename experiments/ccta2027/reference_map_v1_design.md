# Reference-driven boundary map v1

Goal: correct the demonstrated divergence between navigation's immutable initial
task reference and historical map-point transport, without changing best numeric
parameters. The map-provenance trace exactly reproduces933 old frames; one ghost
survives412 registration shifts without fusion/carving. Its accumulated map shift
is (0.3965,0.3130)m versus true center motion (1.1298,0.1928)m. An independently
accepted same-agent initial-reference displacement places it1.187mm from the
current boundary instead of748.755mm. This motivates a controller experiment;
the one-point counterfactual is not a physical success or rotation certificate.

## Chosen mechanism

Add boolean `initial_reference_map_motion`, default False. Require initial task
reference, its translation model, transport, enabled motion compensation, and
every-frame sensing/planning.
Keep the rejected acquisition-geometry flag False in the prospective comparison.

For this candidate only, form the same fresh own/one-hop scan batches and update
the immutable reference before updating maps. During the existing stationary
acquisition and its freeze frame, use zero displacement/velocity. Afterwards,
use the existing rotation-invariant point-goal aggregation over fresh accepted
readings from exactly the robot and its current one-hop neighbours. No all-team
feedback, true object pose, shape or contact data may enter this operation.

Move retained map points by the change of that absolute translation reference;
rekey normally. Own-scan registration remains available for its original quality
diagnostics but cannot move the map. Fuse the original fresh packets with all
original weights, timestamps and thresholds; do not integrate fusion adjustment
as additional rigid motion. Use the reference velocity as map velocity, without
a second filter. Normals remain unchanged under translation.

When no fresh permitted reference is available after acquisition: mark the local
map reference unavailable, retain/prune its last anchored history without fusing
unregistered moving observations, and expose only current own/one-hop returns.
Set nominal motion to zero; retain all hard safety rows. For these fresh boundary
rows use the existing max_object_speed as a per-normal disturbance bound, never
fabricate a measured zero velocity or a fresh displacement. On reacquisition,
move the retained history by the full displacement since its last accepted
reference, then resume normal fusion. Existing object-row caps remain unchanged;
this empirical fallback is not a new dynamic safety certificate.

## Verification and prospective study

Test exact absolute shifts, no registration/fusion double counting, normals and
timestamps, invalid data before mutation, fresh one-hop admission, unavailable
gaps and recovery, default compatibility, and numeric parameter protection.
Run the affected tests and the full core suite using the root .venv.

Freeze four original translation-all-captured development conditions × baseline
and this single candidate = eight new episodes, same seeds, initial states,
12000-frame/600s budgets, physical settings and success/safety/G500 criteria.
Baseline trajectories must exactly reproduce their old25 arrays and scientific
fields. Do not mix methods by shape or change outcomes/gates after execution.

Advance only with8 completed executions,4 exact baselines,4 safe candidate
completions, no false HOLD/hard failure,4 valid zero epochs, preservation of each
baseline G500 pass, zero candidate acquisition scaling and no per-condition
post-acquisition scaling increase. Also publish unavailable-reference robot-steps,
all endpoint/time/G500 outcomes and all negative cases. Even a pass only licenses
wider original-success and unused-condition checks; no deployment-superiority
claim is licensed. All numeric parameters, old receipts and negative conclusions
remain unchanged. The full goal stays active through contribution and paper work.
