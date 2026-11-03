# Residual constraint and pre-activation contact diagnosis

2026-11-03, base dabaa940bd3c40053d74aefc3bf46b4591fdf3fc.
Previous goal turn made progress: corrected-runtime eight-episode validation,
publication and exact-head Windows/Ubuntu CI completed. The research goal is
not complete. Preserve every numeric parameter, original gate and raw receipt.

This is retrospective diagnosis, not a new physical experiment or evidence of
controller improvement. Initial inspection found agent rows and negative finite
object margins in both already-reported residual witnesses, and confirmed the
saved physical samples contain total force/torque but no per-agent substep torque.
No runtime change, physical integration, search over new seeds or gate relaxation.

Part A uses exactly the two original_scaled witnesses in
docs/results/finite_acquisition_support_v2/audit_witnesses.json: C frame365 and
skew frame399, agent_05. Preserve original and corrected row identities, full-rho
and rho-free demands and their original speed cap. For each corrected system:
evaluate agent-only, object-only and joint feasibility at the declared cap and
without a speed cap; wall rows, if present, are a separate reported group.
Cross-check bounded feasibility with original2D solver and CLARABEL; report the
unbounded minimum-norm feasible speed, not a proposed new runtime speed setting.
Find an inclusion-minimal infeasible bounded subset by deterministic ascending
row deletion, then verify the retained subset and each one-row removal with the
independent solver. This is not a minimum-cardinality claim. Map retained agent
rows to neighbors using saved PRE-step positions; show physical distances and
true object clearance only as observer diagnostics. Also evaluate the same
subset without the speed cap to distinguish geometric contradiction from speed
restriction. Do not infer a distributed coordination fix from feasibility alone.

Part B covers ALL8 new physical-v2 episodes, not only the adverse C pair. Rebuild
the original phase monitor from every saved phase signal and require exact phase
and response on every frame. Verify active count from previous diagnostics;
preserve one-based saved frames vs zero-based monitor/control times. Report
contact-ready/push-side/positive-physical-contact counts and first quorum times,
all phase durations, real translation and rotation at transitions and their
pre-TRANSPORT/pre-BRAKE shares. Phase assignment is the phase that issued that
frame's commands, also used for that frame's substeps. Integrate saved TOTAL
physical force/torque over all substeps using dt/substeps; these are sampled
impulses, not causal attribution of final rotation. Per-agent force distributions
use the separately labeled first-substep contact samples only. Source inspection
confirms PenaltyContactModel.step returns the report computed before its loop.
Verify saved contact totals against substep zero for every frame. Reconstruct
first-substep lever-arm torque from recorded same-substep object pose/robot
positions using frozen signed-distance geometry; verify normals within1e-10 and
summed torque against the saved total within1e-8. These are numerical audit
tolerances, not runtime safety thresholds. Partition every positive contact
sample by the same frame's controller push flag; this is a descriptive mismatch,
not proof that a false flag caused that physical force. Never label these sums
as full per-agent physical impulses. No truth enters a controller.

Freeze design/script/helper/source and input product hashes in a fresh runs/
directory before executing analysis. Abort and preserve failures if identity,
receipt, row, phase or independent-solver checks disagree. Publish complete
denominators and negative findings. A targeted engineering fixture suite must
cover speed-only vs cap-independent conflict, redundant-row deletion and use of
all physical substeps. Any future controller candidate needs its own design,
meaningful RED/GREEN tests and independent physical freeze; this diagnosis alone
cannot adopt a candidate or complete the final research goal.
