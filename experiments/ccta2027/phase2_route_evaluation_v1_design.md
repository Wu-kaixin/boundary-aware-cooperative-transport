# Phase 2a: bounded research-route evaluation

Authorization: user approved route comparison only, zero new physical episodes,
at most two hours of offline computation, then stop for review. This document
is recorded before the new matrix calculations and literature comparison. The
Phase 1 results and all four selected pusher outcomes have already been seen;
this is exploratory analysis of reused development evidence, not preregistered
confirmation or held-out validation.

Question: which minimal mechanism could make local finite-boundary control
compatible with cooperative transport, and which observations would refute it?

Initial independently formulated route set (before this stage's external search):

- R1: Contact-compatible finite boundary geometry with defensible signed relative
  motion and uncertainty assumptions. Preserve the physical safety definition;
  do not delete margins to create apparent success.
- R2: Direction-capability-aware command selection within the unchanged feasible
  set. Separate constrained inability to advance from minimum-deviation projection
  choosing a reverse command even when forward velocities exist.
- R3: Consistent local reference/uncertainty propagation through mapping,
  navigation and stopping. This is not a revival of the rejected anchored readout.
- R4: Task-oriented deployment based on locally inferable contact/wrench capability,
  rather than geometric coverage alone. Unknown center of mass, friction and actual
  contact are explicit obstacles, not silently available information.

Evaluation criteria, fixed before ranking: explanation of observed failures;
novelty relative to identified primary literature; mathematical assumptions and
what is actually provable; information/hardware permissions; engineering burden;
complete-task value; sharp falsifiers; and value of a negative result. Use
qualitative evidence levels and uncertainty, no opaque weighted winner score.
Invalid safety assumptions or hidden truth feedback veto adoption regardless of
estimated benefit. Preserve objections to the favored route.

New offline calculation cohort: all 16 saved agent states at each of the four
previously selected candidate events (circle110, L96, C515, skew620), exactly
64 states. The four preselected suppressed pushers are a labeled subgroup.
Inputs come from `runs/research/2026-11-04-phase1-first-pusher-loss-v3/`; read and
hash all four full-state files, the summary/outcome and controlling sources.
No new event selection, physical integration, controller replay, seed search,
parameter sweep, or runtime change.

Let F={u: A u >= b, ||u|| <= V} be each saved full original row set, g its saved
unit task direction. Compute once per state:

1. Original feasibility/directional support and original minimum-deviation
   objective, checked against the stored certificates/command when applicable.
2. Feasibility and least-deviation command in F intersect {g.u>=0}. This zero
   threshold diagnoses reversal avoidance, not useful positive progress, contact
   force, safety of simultaneous execution, or success. Do not tune a positive
   speed threshold. Report all infeasible states and applied/not-applied strata.
3. Diagnostic minimum epsilon>=0 reducing ONLY object-row RHS uniformly to make
   F_epsilon intersect {g.u>=0} nonempty; keep every nonobject row and speed limit.
   Epsilon is a lower bound on a model change required for nonreverse capability,
   NOT permission to relax any deployed safety constraint. Never execute it.
4. For support-applied rows, reconstruct original capped b from saved h, demand,
   rho, gamma and cap rule. Compute the extreme zero-demand algebraic system with
   everything else unchanged, and its directional capability. This is an optimistic
   diagnostic of whether the nonnegative isotropic motion-demand term ALONE can
   explain reversal. It is not a proposed bound, signed-motion implementation,
   safe controller, or valid assumption for a moving body.

Require independent primal residual/speed checks for returned solutions, preserve
inconclusive solver statuses, and cross-check directional optima against the
existing geometric/dual certificate analyzer. Do not merge these related systems
or agent states into independent sample counts. Do not infer any trajectory-level
improvement. Keep all saved geometry/noise allowances and protected parameters.

Computation limits: sequential default, BLAS/OpenMP threads 1, at most four
processes if needed; no GPU migration; overall offline calculation wall budget
7200 s and output budget 5 GiB. Save timings, input/source hashes, package identity,
solver status and numerical residuals in the new run. Small synthetic optimizer
checks are engineering checks, not new physics or research samples.

Evidence search: targeted primary sources for contact-aware barriers, CBF-QP
deadlock/liveness, unknown-object multi-robot pushing, observability/reference
uncertainty and task-dependent contact configurations. Use public generic queries;
do not send unpublished trajectories or private repository content to external
services. Record query/date, source version, reading depth, precedents and limits.
No novelty claim follows from failure to locate a paper in this bounded search.

Deliverables: four-route comparison, current-source and mathematical evidence,
bounded offline diagnostic, adversarial critique, main/backup recommendation,
rejection criteria and proposed next budget. The recommendation is for user review;
do not activate a candidate or launch new physical experiments.
