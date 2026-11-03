# Directional capability of the unchanged contact-state constraints

The packet-geometry diagnostic exactly replayed all 1,749 predecessor frames and
retained the same 128 states / 112 admissions / 17 positive-force states / 11
positive-force declared pushers. Limiting rows come from both own and one-hop
scans; removing both structural and noise terms algebraically still leaves six
original positive-force commands conflicting under the existing motion demand.
No term was changed in an executed controller.

Before redesigning the controller, distinguish exclusion of one old command
from exclusion of every positive goal-directed velocity. On EVERY admitted
state solve this evaluation-only two-dimensional convex problem:

    maximize g.u
    subject to A u >= b, ||u|| <= original_max_speed.

Here g is the actual saved task unit direction. Compare three declared systems:
the original aggregate rows, the new finite-support rows with the existing
reachable RHS cap, and the same finite rows without that cap. Keep all original
agent/wall/non-object rows, rho, speed and all geometry/motion inputs. Nonadmitted
states remain explicit rows with their original guard, not fabricated solves.
There is no altered command, parameter, row deletion or new physical episode.

## Verification

Use a planar geometric enumeration of possible optima (directional speed-circle
extremum, boundary feet, line/circle intersections and pairs of boundaries).
Independently cross-check with the installed CLARABEL convex solver. Do not
equate a solver status with a certificate.

For any nonnegative row multiplier lambda, the following is a dual upper bound:

    g.u <= V*||g + A.T lambda|| - b.lambda.

Record a verified primal point, minimum row residual, speed, nonnegative dual
multipliers and independently recomputed upper bound. Require primal/dual gap
and geometric/solver objective agreement within 1e-6, and primal constraints
within the unchanged 1e-7 audit tolerance. Clip a tiny negative numerical dual
multiplier to zero before recomputing its bound; do not alter A, b or the primal.

If the original system is infeasible, obtain an independent normalized separator
lambda>=0, sum(lambda)=1, with

    V*||A.T lambda|| - b.lambda < -1e-7.

Such a witness contradicts A u>=b throughout the original speed ball. If no
verified primal/dual or infeasibility witness is obtained, record INCONCLUSIVE
and stop short of a capability claim; preserve the full attempted computation.
Synthetic known optima, a strictly reversing feasible example, and an infeasible
ball/half-plane example must be checked before cohort execution.

## Interpretation and reporting

Report all 128 states and every attempted system, with original/new selected
goal velocities alongside the maximum. Classify verified positive capability
only from a feasible point above 1e-7; a negative upper bound below -1e-7 certifies
that all feasible velocities point backward along the goal. Near-zero intervals
remain boundary/uncertain. Keep infeasible and inconclusive cases separate.
Positive goal velocity is NOT a contact-force or task-success certificate. No
oracle geometry enters these optimizations. Individual instantaneous velocity
optima do not establish simultaneous team/contact feasibility. Retain all original subgroup and
condition denominators; no independent-sample or generalization inference.

Snapshot this design, new analyzer/test, existing helper identities, predecessor
analysis hashes and saved goal directions in a fresh runs/research directory.
Preserve the original live full audit and every previous artifact. The result
decides whether the next mechanism work concerns the feasible constraints, the
nominal/objective choice, or both; it does not authorize candidate adoption.
