# Map ancestry investigation and execution ledger

Base: 3b61c33d002f63a783723c120d394ef06d408ec7. Execute inline under systematic
debugging and executing-plans; retain one fresh final review. User authorization
continues for normal branch publication. Best numerical settings stay fixed.

Question: where did agent_05's frame989 selected map point, measured 438.753 mm
outside the true boundary, come from? Trace the actual local-pair candidate
trajectory, including every map alignment, fusion, carving and fresh one-hop
reference input. Commands must match the original exactly. Geometry is evaluation
only and cannot influence commands. No new physical episode in the tracing stage.

Pre-flight: existing trace_boundary_map targets an older experiment and its method
signatures omit apply_motion/integrate_motion. A separate helper will use the
current frozen physical protocol and retain all arguments. Existing helpers,
source snapshots, protocols and receipts remain unchanged.

- [x] Task1: capture all map stages on separate 20-frame preflight then complete
  989-frame replay. Assert all commands, diagnostics, statistics, phase signals,
  phase response and published map arrays match. Freeze helper/design identities.
- [x] Task2: reconstruct selected cell ancestry, check fusion weights and all
  translations, identify the earliest responsible operation and its reference
  inputs. Preserve branching ancestry and distinguish evidence from hypotheses.
- [ ] Task3: implement a source-level correction only if the cause is established;
  regressions must fail first. Otherwise publish the concrete diagnosis and the
  next discriminating experiment, without calling it a repair. Physical changes
  require a separate frozen matched protocol, original controls and full gates.
- [ ] Task4: report metrics/limitations, update README, fresh independent review,
  normal branch sync and verify the exact-head checks.

Review focus: provenance across rekey collisions and scan fusion; exact replay;
map/reference epochs; source and raw artifact identity; truth only in evaluation;
no post-hoc retuning, physical or generalization claim from replay.

Ruling: retain the plan and ledger here and all raw evidence under runs rather
than disposable skill scratch — repository convention requires preserving run
receipts — cost: a larger local evidence directory.

Task1 complete: separate20-frame preflight and989-frame complete reconstruction,
both terminal0; commands/diagnostics/statistics/phases/map arrays exact. Runtime
6a24d760.., helper4298197e.. and original plan snapshot frozen in each run folder.

Task2 complete: lineage tests RED2missing implementation then GREEN2. Verified
606286 cell comparisons and191273 graph nodes; chosen point has17 contributing
nodes and3 original returns from source03, saved976/981/983, true gaps14.765/
10.809/12.045mm. Their coefficients.309307/.349560/.341133 sum to1. Translations
contribute[-.34289245,.00199833]m. At989 newvoter06 .183816 vs09 1.111963 makes
the absolute aggregate jump-.451596m instead of09's+.012478m. This establishes
the map-motion input defect, not the upstream reason for biased registrations.

Task3 mechanism spec: map_reference_bridge_v1_design.md. RED6 tests then
GREEN33 including existing map tests; no numeric settings changed. Add two
prospective gate tests RED2thenGREEN2. First core run:983passed/2skipped,
68temp-directory errors and2new-default-field comparison failures. Preserve
the receipt; explicitly assert the new selector isFalse in both protected
comparison tests, and rerun with a fresh workspace basetemp.

Ruling: validate same-voter increments as a new default-off mechanism — the
source of the map jump is proved, but increment integration and map resets can
alter trajectories — cost: existing registration bias and physical regressions
may remain; matched trials and the original gates decide, not the unit tests.
