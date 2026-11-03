# Post-hoc contact-row decomposition, zero new physics

The eight physical episodes have ended: four controls complete their tasks and
all four finite-support candidates reach the600sbudget without completion. Full
candidate pair/bridge/support replay is currently running in its original single
analysis attempt. Do not modify runtime, frozen helpers or that analysis.

Use only the existing contact-compatibility-v2 analysis, whose predeclared first
and peak positive-force events contain128robot states on four control trajectories.
Preserve all128rows, all112finite admissions and the already-defined primary
subset of all17positive-force states. Do not select new frames or states by the
new decomposition. This is post-hoc mechanism attribution, not independent
validation and not evidence about newly generated physical forces.

For each admitted state reconstruct every uncapped finite object-row residual:

    r = A u_original + gamma_obj h - rho - received_speed_norm.

Check its minimum against the stored diagnostic residual, and separately verify
the capped minimum from the stored capped matrix. For the deterministically
chosen minimum-uncapped-residual row, record all four additive terms and its
normal, h, original velocity and configured parameters. Also compute the minimum
over ALL rows with just the motion term removed: r_no_motion=r+received_speed.
This algebraic diagnostic does not change an executed command, rho, safety radius,
controller, solver or parameter. It does not claim that zero estimated motion is
valid, or propose using it as a replacement controller.

Report whether the original command still violates the object rows when the
motion term is removed, on all admitted states and on the original positive-force
subset (including its11declared pushers). Distinguish geometric clearance/margin
and motion contributions; a sufficient remaining inequality conflict is not a
proof of the complete closed-loop failure mechanism. Do not label the geometry
term as scan noise or half-gap alone without a further packet-level decomposition.

Freeze this design, the analysis script, input hashes, runtime/protocol identity
and each condition's actual effective safety parameters in a fresh raw directory.
All original negative results, first-failure snapshots and full-audit attempts
remain untouched. No candidate adoption or additional physical episodes.
