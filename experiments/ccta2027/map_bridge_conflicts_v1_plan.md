# Exact map-bridge conflict plan and ledger

Spec: map_bridge_conflicts_v1_design.md. Basebceb1c4. Execute inline with
systematic-debugging and executing-plans; one final independent review.

Pre-flight: safety finalization can be deferred during acquisition; correlate
prepare/finish by object identity. Fresh support packets must be captured even
outside acquisition, where they are NOT runtime filter inputs. Original map
normals and ground-truth nearest normals must remain separately labeled.

- [x] Task1: freeze/execute separate20 and full517frame exact replay, all18states.
- [x] Task2: original/counterfactual joint decomposition and finite-edge geometry;
  tests for any new diagnostic arithmetic, no truth-assisted controller change.
- [ ] Task3: decide source correction from evidence, meaningful regression and
  separate prospective physical validation where justified; otherwise preserve
  specific unresolved conditions and next discriminating action.
- [ ] Task4: complete evidence/report, one fresh review, update README, normal
  branch sync and exact-head CI verification.

Review focus: complete18-state denominator; deferred filter capture identity;
unchanged hard rows/speed/rho; clipped/capped versus uncapped distinctions;
point-only/normal-only substitutions; true finite-edge versus infinite-plane
distances; exact replay; no physical claim from counterfactual feasibility.

Ruling: keep durable ledger/raw evidence in experiments/runs rather than skill
scratch — original evidence must survive — cost: local storage use.

## Publication checkpoint (2026-11-03)

The user requested uploading the completed results and updating README. The
completed map_reference_bridge_v1 result is already published through bceb1c4;
this unfinished design, ledger and replay helper remain local research drafts.
The separate preflight completed20frames with exact reconstruction and captured
zero selected conflict states. Full517frame replay and Tasks2--3 have NOT run.
No new physical episodes or completed conflict diagnosis are claimed.

Publication inspection found a draft interface error before the full run:
Cargo exposes world-space polygon vertices as `cargo.vertices`, whereas the
selected-state writer currently asks for `cargo.world_vertices`. The20frame
preflight never reaches this branch (first selected frame402), so it does not
validate that writer. Fix and verify the capture path before full replay; keep
the existing preflight script/design/output receipt unchanged and use a fresh
attempt directory for any changed helper.

## Task1 complete (source unchanged; implementation base430327f)

Previous goal turn: progress — README corrected and430327f pushed, exact-head
Windows/Ubuntu CI successful. This resumes the existing conflict diagnosis.
The selected-state writer now snapshots `cargo.vertices` and is covered by a
translated/rotated actual-Cargo regression (RED1failure, GREEN11tests including
the reused decomposition arithmetic). A fresh `--output` option refuses existing
directories and paths outside runs/. All helper/test snapshots are bound.

`preflight-capture-v2`: exact20frames, zero selected states. `replay-capture-v2`:
terminal exit0, exact517frames, all18selected frames/288robot snapshots captured,
38.859seconds. Every command, diagnostic, solver statistic, phase/guard/response
and map point matches the original receipt. Runtime remainsde7e895a..; new
physical episodes0. The original20frame preflight remains unchanged.

Task2 preflight: point/normal substitutions rerun original selection/aggregation
while retaining the original received velocity vectors and associations. For
the true-nearest row, use the maximum projected bounded received point velocity
(or the bounded common translation when no point velocities exist); do not
invent a velocity at an unobserved location. All17real aggregate states use the
common translation input. Counterfactuals are evaluation-only, retain every
agent/wall row and original rho/limit, and are reported capped and uncapped.

## Task2 complete and runtime candidate checkpoint

Completed226 decompositions across all18 selected states. All17 post-acquisition
robot projections extend beyond the finite associated true edge. Correcting
only map points or only normals yields0/17 uncapped joint feasibility; correcting
both yields1/17, while the true nearest finite feature yields17/17. True geometry
is used for diagnosis only. Independent HiGHS cross-check agrees with all864
unbounded CLARABEL group/conflict outcomes.

The truth-free finite observer preserves hard rows, rho and speed. Using the
full configured0.2m/s bound yields16/17; using the bounded received-speed norm
yields17/17. The frozen17-state design erroneously says0.6: its original bytes
remain, and map_bridge_finite_observer_v1_motion_cap_erratum.json corrects only
that prose. The executed arithmetic always reads the actual0.2 parameter.

All272 robot states at the17 selected post-acquisition frames were then screened.
181 admit current finite support;91 fall back (83 motion unavailable,6 uncovered
isolated returns,2 outside range). Received-speed demand gives181/181 uncapped
feasible systems with17 rescued and no new infeasibility. Full-bound demand
introduces6 new uncapped peer conflicts. These are reused states, not independent
trials or a reliability estimate. New physical episodes0.

Task3 implementation and181-state exact row agreement are recorded in
post_acquisition_finite_support_v1_plan.md. The new flag stays defaultFalse.
The user's latest instruction requests publication now; the independent
eight-episode freeze/run remains pending. Publish completed evidence and mark
this remaining validation explicitly in README rather than inventing outcomes.
