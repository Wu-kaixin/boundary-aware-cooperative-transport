# Joint safety QP reconstruction, v1

## Question

The acquisition geometry v1 screen failed: skew scaled-barrier robot-steps
240 to 689, C post-freeze 7 to 16. Determine which actual QP constraints conflict
before modifying any controller. All best numeric parameters, thresholds and
old receipts remain unchanged. This is a retrospective mechanism diagnostic,
not independent performance validation and not a new physical experiment.

## Fixed inputs and extent

Use both arms of the completed acquisition_geometry_v1 run, for far-C and
far-skew only: four traces. Reconstruct each prefix through its last recorded
scaled-barrier frame (C baseline579, C geometry933, skew baseline784,
skew geometry1115), inclusive. Four prefixes, 3411 control frames, no physics.
Use each run's frozen source and original seed/configuration in the root .venv.
Saved pre-physics poses enter only the original simulated sensor and agent state.
Do not substitute cargo truth for any controller measurement or map estimate.

Run the complete original controller on these forced recorded inputs. Every
frame must exactly match recorded commands, diagnostics, solver statistics,
phase response/guards and the packed map snapshot. Stop and label INVALID if
any reconstruction comparison fails; keep its failure products. No approximate
match, resumed partial pass or relabeling of old data is acceptable.

## Recorded interventions (diagnostic only)

For every exactly reconstructed scaled robot-step, classify feasibility with
the original bounded-speed planar QP solver for full rows, margin-free rows,
hard agent/wall rows alone, object rows alone, and the combined margin-free set
with only estimated object/point velocity set to zero. The last is a
counterfactual diagnostic, not authorization to change control or assume a
moving object is stationary. Keep each subset's denominator and actual h/value
of the velocity estimate. Any state-mutating counterfactual is forbidden.

Save complete original QP matrices, inputs and object-row selection traces for
the first and worst (lowest barrier scale, earliest tie) event per phase in each
case. Independently cross-check these selected witnesses with CVXPY/CLARABEL,
including minimum required speed without the speed cap. Compare true clearance
and true motion only in the evaluation, never as inputs to control.

The decision must separate false motion demand, observed geometry, hard-neighbor
constraints and speed-limit conflicts. A subset becoming feasible is a causal
statement about that frozen QP only; it is not evidence that removing constraints
online is safe or that an unexecuted fix succeeds. Preserve all contradictions.

## Execution and next step

Preflight the replay implementation on a short retained prefix before freezing
the diagnostic runner and input/source/environment identity. Then execute the
four bounded prefixes once in a new runs/research directory. No runtime edit or
extra physics is included. Report original/frozen identities, full counts,
selected witnesses and a justified next implementation hypothesis. Review and
publish this evidence before promoting any control change. The full goal remains
open: safe local deployment/transport/stop, matched deployment contribution,
wider validation and the evidence-aligned CCTA2027 paper are not achieved here.
