# Reference-loss reacquisition v1

The contact_activation_v1 diagnosis reconstructs4317 original controller frames
exactly and279908 saved final cage/push calls. Both circulation prefixes contain
zero contact-ready robot-steps whose true normal can produce the required forward
component. Four distant robots per case are still stopped at frame1200: their
nonzero search/approach nominal commands are replaced by zero for missing local
motion reference. All8 original QPs permit nonzero commands when given the
original pre-override nominal; independent CLARABEL solutions agree. This is a
one-step conditional witness, not evidence of successful future reacquisition.

Approaches considered: redesign productive-contact allocation for the whole
boundary; or repair the earlier information-permission cycle that prevents late
robots from obtaining observations. Choose the latter first because it isolates
a directly witnessed cause without changing established contact control. Capacity
increases and acceptance relaxation remain excluded by evidence/user constraints.
The existing contact-band redeployment lock may still need a separate mechanism.
This choice is the executor's under the user's delegation, not a user-picked design.

Add boolean `reference_map_reacquisition=False`, requiring reference-map motion
and its existing initial-translation/every-frame/aggregate-row guards. When the
initial reference is frozen, the local map reference is unavailable and phase is
before TRANSPORT, permit only original nominal modes `search`, `recall`, `approach`
with push_side=False and effort exactly0. Label the allowed command
`map_reference_reacquire_<original mode>`. All other missing-reference commands
retain the original zero override, including redeploy/cage/push, unavailable
initial epoch, TRANSPORT, BRAKE and HOLD. Fresh-reference behavior is identical.

These modes use exploration/token targets or current fresh geometry, not absent
task-displacement feedback. Existing anchored history stays untouched and only
fresh own/one-hop packets form the unavailable-reference view. Preserve the
maximum-object-speed disturbances and pass every command through the original QP.
No numerical parameter, speed limit, safety row, map admission, deadline, physical
model, success criterion or phase quorum changes. This does not certify contact
avoidance or future dynamic safety; passive contact and constraint scaling remain
independently evaluated.

Verification: default off and numerical preservation; invalid selector contracts;
real complete-controller empty-view search and token recall; permitted approach;
rejection of push/effort, stale committed targets and later/invalid stages; original
speed cap and missing-motion row bounds. RED first, focused GREEN, full core suite.

Prospective screen:8 new episodes, same4 reused development conditions and original
circulation baseline versus circulation+reacquisition. Preserve seeds, initial
states, all numeric parameters and600-second budget. Freeze before execution,
run once, publish every failure. All4 baseline episodes must exactly reproduce
their25 arrays and scientific fields. Candidate acquisition through the original
freeze frame must be identical: this mechanism intentionally cannot repair the
known13/13 acquisition scaling events that occurred earlier.

Mechanism gate: all8 executions complete,4 exact baselines,4 candidate safe
completions/valid zero epochs, no falseHOLD/hard failure, preserve baseline near
G500 passes and no per-condition post-acquisition scaling increase. Separately
retain the full-chain gate requiring zero acquisition scaling and all original
completion/safety requirements. Even a mechanism-gate PASS is not adoption or
completion of the goal; the full-chain gate cannot pass while those earlier
scalings remain. No posthoc tuning or shape-specific method combination.

Then address remaining joint feasibility/activation using the actual outcomes,
followed by wider preserved-success and unused-condition validation, matched
deployment contribution, and the CCTA2027 draft. Whole-system decentralization,
rotation robustness and hardware still require their own evidence.
