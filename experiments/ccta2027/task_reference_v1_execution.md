# Initial task reference execution decisions

2026-11-03, supplement to the preserved initial design.

The acquisition pilot found support before any contact/motion in all four
observed-standoff arms. Its 60-second skew template is contaminated and is not
reused. Runtime acquisition freezes at the first qualifying sensor frame;
the pilot sampled readiness only every ten frames, so timestamps can differ.

The first closed-loop stage is deliberately four reused development blocks
(circle, L, far C, far skew), three arms and twelve episodes. Each retains the
600-second watchdog and original physical success/safety/G500 rules. The other
four old development blocks are a subsequent regression gate only if this pilot
passes; independent deployment validation comes after a usable common system.

Selected initial readout: convert every local rigid fit to q, the known workspace
center. Read position and velocity at that same point, using Euclidean geometric
medians over permitted fresh messages. The shared supervisor additionally requires
each fresh eligible fit's maximum goal residual over the workspace vertices to
be within the unchanged task tolerance, with at least the existing push quorum
of eligible fits. The original nominal arrival band and speed/settling gates
remain. Initial templates cannot be extended after release or initialized by late
observers. Safety maps and their motion feedback stay separate.

This is a conditional unknown-center rule, not a registration certificate.
The initial physical area centroid must be inside the declared workspace and
the estimated transform must be correct. The acquisition zero epoch is an
empirical assumption evaluated separately from task completion. Pose ambiguity
is rejected when independent rotational leverage is at most three noise scales;
an ideal circular boundary cannot supply measured yaw and may therefore fail
this first candidate. The workspace set may also be too conservative to allow
HOLD. Those failures must be retained rather than relaxing arrival thresholds.

No bounded-size assumption, object-shape identifier, global center estimate,
reference relay beyond permitted one-hop scans, or new contact truth is added.
All switches remain off by default. The current environment retains PyYAML 6.0.2;
the lock records 6.0.3. Current local runs and locked GitHub CI are distinct
environment identities.

Engineering: baseline research synchronized at 27d17ad. Portable numerical-audit
imports fixed at 6e7958c; GitHub run 41468642725 passed Windows and Ubuntu core,
MAS and smoke checks. New local implementation: 841 passed, 2 historical skips,
receipt in runs/maintenance/2026-11-02-task-reference-v1/core-tests.xml.
