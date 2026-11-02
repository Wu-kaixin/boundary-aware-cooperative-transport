# Free-ray enclosure: information feasibility before another controller change

2026-11-03. Starting source commit 7da54cc. Preserve all numerical parameters of
the protected best-known configuration. No dependency changes or physical runs
are authorized by this stage's protocol; user authorization for the larger goal
remains in force for subsequently designed stages.

## Problem and alternatives

The previous initial task reference uses the whole workspace as its unknown
center set. This is valid conditional on a correct rigid fit but often prevents
HOLD. Its noisy circular fit also invents yaw. A smaller set cannot by itself
repair that transform. Partial-point concave hulls previously excluded large
unseen geometry and are not a sufficient solution.

Chosen first step: test enclosure by measured free line segments. Alternatives
are stronger object-size/shape priors, or new active observation. A size prior
would change task information; active observation is needed only after measuring
what current packets can establish. Do not treat gaps between rays as observed
free sectors and do not add beam widths or curvature assumptions.

## Conditional geometry

Assume one connected positive-area rigid object K lies inside declared convex
workspace D. A line network L consists exclusively of free ray segments. If
K does not intersect L, connectedness places K in one component of D minus L.
Each noisy target return must be within the declared observation margin of that
component. Retain *all* arrangement faces consistent with *every* target return.
The convex hull of their union contains any positive-mass centroid of K under
these assumptions. Taking the convex hull is essential for nonconvex objects.

Implementation constructs the line arrangement using Shapely, clips segments to
D, polygonizes the network together with the boundary of D, filters faces by
distance to each target return, then returns the convex hull of all candidates.
No target returns, no consistent face, or invalid partition => abstention with
the original workspace bound. Never silently select the smallest face or return
an empty center set as a certificate.

Input API accepts workspace vertices, explicitly measured free segment starts /
ends, observed target points, and the existing carving margin. It never accepts
object vertices, center, pose, mass, contact, shape label or task start.
The root environment already includes Shapely. No controller or sensor behavior
changes in this stage.

The audit will reconstruct raw range scans from recorded scenes, remove all
truth-only fields, and retain measured range, known ray direction, robot origin
and existing object label. Hits shorten their measured ray by the existing
carve_margin=.06 m; no-hit rays use sensor_range minus that same margin. No
interpolation between rays. Gaussian range noise is unbounded: this is a
conditional/empirical enclosure, not an unconditional safety certificate.

## Frozen audit after component tests

Use all four original task-reference arms in task_reference_v1, at each actual
initial acquisition freeze time. Reconstruct the control-before-physics pose,
verify original own-scan counts, initial stationarity and source/receipt identity.
Use only that single sensor snapshot; no unrecorded exploration or accumulated
multi-hop relaying. Each robot gets its own packet, then its own plus current
one-hop packets as a separate information comparison. An all-team union is an
explicit evaluator-only information upper bound, never a deployable local arm.

Truth is used only to reconstruct sensors and evaluate complete object-vertex
and area-center containment. Report all agents, including empty/inconsistent/
unbounded results. Report center-set area/diameter versus workspace and original
stop-gate bounds evaluated on the same historical transforms. The latter is a
counterfactual diagnostic, not a new task success. Circle yaw ambiguity remains
a separate failure channel.

Freeze exact cases, snapshot times, input receipts, numerical values, script and
source hashes before execution. One worker/internal thread; 1800 s wall budget.
No physical episodes. Do not tune margins after seeing outcomes. Any observed
true-object exclusion rejects that packet's enclosure as usable evidence. A
usable local mechanism requires consistent object containment and meaningful
center-set contraction on the available local packets; lack of closure guides
the next sensing/communication design rather than changing the task tolerance.

## Full-goal relevance

This component addresses only the center-set information gap. Before adoption,
a complete candidate must also handle transform ambiguity, preserve allowed
communication and best parameters, avoid both false HOLD and indefinite BRAKE,
retain all existing successes, pass new conditions and safety checks, and be
compared across deployment methods. Do not claim the larger goal is solved.
