# Explicit local-prior admission design: authorized bounded stage

2026-11-04 Asia/Tokyo. User explicitly selected: “批准局部先验设计及新增 1 小时预算”.
This supersedes the prior scope/budget blocker, not the frozen experimental
definitions. New window starts 2026-11-03 20:37:14 UTC, at most 3600 s.
The goal UI still reported blocked on entry; no tool available here can change
it to active. The substantive authorization is clear, so work proceeds without
asking again or replacing the long-term goal.

Scope: design local-prior families, evidence requirements, admission/failure
semantics, a mathematical certificate, and falsifiers. Zero physical episodes,
zero controller replay, no candidate QP/state comparison, no runtime integration,
no parameter sweep, no sensor/noise/physics/task-threshold changes. Existing
data and negative results are known; this is exploratory design, not validation.

Scientific question: can an explicit local shape class plus the current legal
observations support a noncircular geometry admission certificate, and what is
additionally required for that certificate to apply to the final executed step?

Compare three design families qualitatively, without a weighted winner score:

1. Chord-arc length and bounded tangent variation, invoking the previous bound.
2. All observation-consistent local polygon/occupancy models, retaining ambiguity.
3. A continuous visible radial branch with a declared regularity bound.

Criteria, in order: soundness under explicit premises (veto if circular);
observable evidence versus assumed class membership; compatibility with actual
circle64/ellipse24/rectangle/L/c_shape/skew_box; physical-task usefulness and
contact conservatism; finite, auditable construction; implementation and message
cost; clear failure/timeout behavior and one falsifier. Standard set-membership,
geometric distance and safety-filter ideas are not original theory claims.

Reinspect actual raw-scan and downstream safety interfaces. Preserve the
distinction between assumption, online consistency check and truth-only audit.
Do not feed simulator geometry, true normals, hidden edge IDs or future motion
into a hypothetical online certificate. Raw scans from actual one-hop sources
are the maximum proposed observation scope; any message expansion is disclosed.

Use the scientific-brainstorming workflow already applied in this conversation;
the existing independent reviewer generates alternatives and challenges the
chosen design. Record prior-stage formulas as anchors. Check a bounded set of
primary sources and explicitly limit novelty claims.

Deliver a readable design/report, machine-readable design contract, reviewer
notes, source/environment preservation receipt and final decision. Mathematical
counterexamples/inequalities may be evaluated algebraically; do not run a
controller, physics, synthetic sensing experiment or new state cohort.

Stop after design assessment and report. This approval does not authorize
implementing a candidate, adopting numerical local-prior bounds, increasing
the scientific matrix, or moving to Phase 3.
