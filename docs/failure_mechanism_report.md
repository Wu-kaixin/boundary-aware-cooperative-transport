# Failure mechanism report

Status: EMPIRICAL_ONLY. Development and diagnostic runs on 7 October 2026. Original downstream comparisons are `study_v3/development` and `baseline_transport`. Single-factor diagnostics are separate and are not counted as boundary-CVT gains. Correlation is not treated as a cause. Items marked OPEN are not identified.

## What was held fixed
Each development pair shares the initial robot state, task direction, task distance, sensing, safety layer, contact physics, and downstream phase law. The only intended difference before CONTACT_READY is the centroid target. `boundary_cvt` is the proposed target. `nearest_observed_target` picks the closest observed offset target. `local_spacing` is an engineering dispersion rule over the same observed targets.

## C-shape seed 8, original downstream
This is the archived watchdog case, reproduced in the development batch.

Boundary CVT enters CONTACT_READY at 4.15 s and never leaves it. Over 3000 control frames the final phase is still CONTACT_READY. True progress is 3.095 m against a target of 1.304 m, so the absolute progress error is 1.791 m. The run is a watchdog, not settled, and not task-complete. Estimated progress at the last frame is 3.256 m. After CONTACT_READY, net force along the goal is positive on 63.8% of frames and its mean is 5.75 N. Mean contact count is 4.66. The object is being pushed.

The phase law leaves CONTACT_READY only when `transport_active >= min_push_agents` (4) or, in the diagnostic recovery switch, when estimated progress reaches the existing brake fraction. In this CVT trace `transport_active` has maximum 3 and is never at least 4 (0 frames). The paired nearest-target run on the same initial state reaches `transport_active >= 4` for 8 frames, then HOLD, with absolute progress error 0.039 m. Local spacing also reaches 4 active pushers and HOLD, with absolute progress error 0.018 m.

Supported for this case: CONTACT_READY is not a stable effective-contact state. The CVT arrangement keeps producing forward contact force without ever meeting the pusher quorum that arms braking. The overshoot is then a missing phase transition, not an absence of contact. The paired controls do meet the quorum, so the missing fourth pusher is tied to the CVT target on this initial state. It is one condition, not a general law.

Not supported: a claim that safety filtering removed the forward command. Forward force remains large. Not supported: a claim that the estimate was too small. The estimate is larger than truth and already past the brake threshold; the original phase law does not consult it until TRANSPORT has started.

## Diagnostic interventions, same five CVT-loss conditions
These five conditions are the development pairs in which CVT absolute progress error exceeds nearest-target: C seeds 2 and 8, L seed 2, rectangle seeds 2 and 8.

`study_v1/diagnostic` set an invalid yaw flag. All 10 episodes failed before simulation. Those errors stay in that batch's denominator. They are an implementation failure.

`study_v2/diagnostic` enables `estimate_object_yaw` only. C seed 8 CVT then settles. Absolute progress error is 0.043 m and G500 passes. Nearest-target on that condition also completes. Across the five pairs, CVT task completion is 3/5 and nearest-target is 3/5. This does not show that yaw estimation is the cause of the original C seed 8 deadlock: the original trace already had an estimate past the goal while `transport_active` stayed below 4. OPEN how much of the v2 change comes from yaw itself versus a changed registration path.

`study_v3/recovery` enables estimated-progress phase recovery and leaves the brake threshold unchanged. C seed 8 CVT again never records `transport_active >= 4` (maximum 3) but now reaches BRAKE and HOLD because estimated progress crosses the existing brake fraction. Absolute progress error is 0.005 m. The other four CVT-loss conditions do not all improve: recovery CVT task completion is 2/5, nearest-target is 4/5. The recovery law can release the C seed 8 latch. It is a downstream change. The main three-method comparison keeps the original phase law, so this result is not a CVT preparation effect.

## The other four larger CVT progress errors
C seed 2, L seed 2, and rectangle seeds 2 and 8 all settle under the original downstream for both CVT and nearest-target. Absolute progress differences are about 0.04 m to 0.09 m, except rectangle seed 8 where CVT error is 0.110 m and nearest-target is 0.016 m. None is a watchdog. No single contact-loss or force-cancellation trace was established for all four. OPEN. They limit a transport-improvement claim without identifying one shared mechanism.

## Physical-readiness latch
The evaluator marks physical readiness only after `min_push_agents` contacts with positive projected force persist for `contact_dwell` frames. On the 27 development episodes that latch fires once: C seed 8 CVT at 51.65 s, during the overrun. Settled successes, including nearest-target and local spacing, remain right-censored. Mean contact counts on those successes are about 1.4 to 3.6, below the quorum of 4. The latch therefore does not measure the contacts that actually finish the task. Effective-contact superiority is INCONCLUSIVE under this definition. The definition was not relaxed.

## Ablations
Arc-weight removal, normalized arc-weight removal, and direction-offset removal change seed-2 trajectories and the readiness/progress numbers. Effects differ by shape. For example, removing arc weights on C seed 2 makes the episode task-complete, while the full CVT episode is settled but outside the Euclidean tolerance. Removing direction offsets on rectangle seed 2 also changes the endpoint error. These are three shapes at one seed. They do not justify a general ranking of the terms.

`no_exploration` matches the full CVT trajectory hash on all three shapes. Density `explore_gain` is already 0 in this protocol, so the ablation does not remove an active term.

## Safety repair, static local mode only
Independent path evaluation of the original local controller finds observed clearance below 65 mm in 8/9 episodes. L seed 2 path minimum is 48.33 mm. The oracle reference is certified safe on 9/9 under the fixed-polygon linear-hold evaluator. The fresh scan-segment candidate is certified safe on 9/9, with path minima from 65.85 mm to 75.92 mm. Mean robot displacement on the candidate is 0.133–0.284 m, against about 0.83–1.02 m on the original local runs. The candidate is safer on this evaluator and moves less. Unknown map regions remain uncertified. The transport safety layer was not switched to this candidate. Stagnation is not hidden: displacement is reported beside the safety label.
