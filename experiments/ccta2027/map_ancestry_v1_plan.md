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
- [x] Task3: implement a source-level correction only if the cause is established;
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

Task3 complete: final core1055passed/2skipped,151.13s,terminal0. Frozen physical
runtime de7e895a.., protocol29930eb5..; commit1f50d8d retains implementation and
protocol. Eight episodes completed,5455physical steps, execution_errors0 and
source_integrityPASS. Four original controls reproduce25arrays and scientific
fields exactly. Both arms safe4/4, falseHOLD/hardfail/cutoff0, G5002/4. Candidate
skew post-acquisition scaling1→17; Crotation4.942→5.839degrees and noTRANSPORT.
42832candidate robot steps passed message membership/increment audits. Mechanism
andfullgatesFAIL. All processes terminal; no repeat physical attempt.

Post-study screen: all18candidate scaled states, no newphysics/replay. New17are
agent_14 frames469–517 (ENCLOSE15, CONTACT_READY2), truegap159.829–185.857mm,
mapdisplacementerror0.925–5.125mm, continuousbridge/noreset. Selected safetyrows
and joint infeasibility still untraced. This changes the next action to exact
row/geometry reconstruction rather than assuming another large map jump.

Ruling: do not adopt this candidate after its failed physical gate — eliminating
the roster-jump mechanism did not improve the full task and increased scaling —
cost: the original map jump remains in the prior experimental control until a
combined estimator/geometry solution passes the matched tests.

Task4 publication:9generated products, all8outcomes, separate18-state screen,
and the complete point ancestry. PNG visually inspected. Independent review and
GitHub exact-head CI remain before delivery completion.

Task4 independent final review3b61c33..2a09f3f: noCritical/noImportant/noMinor;
reviewer independently reran10newtests, allpass, verified9products and separate
screen/freeze/source/trace identities. HealthPASS;309local links verified. No
runtime changes or second review; only final review/operational records follow.

Final: Ruling: generalization/hardware/asynchronous delivery and fully local
supervision remain unproved — retain the explicit empirical development scope —
cost: broader deployment and paper claims require further evidence.
Final: Ruling: sparse/empty-return reanchor safety is not certified — retain
the default-off experiment without a geometric guarantee — cost: unobserved
constraints may be lost, so this mechanism is not ready for hardware adoption.
Final: Ruling: upstream bias and17new conflicts remain unresolved — reconstruct
actual safety rows before another correction — cost: the known failures remain
while their causes are distinguished rather than presumed.
Final: Ruling: accept receipt/hash-based review of the expensive completed runs
alongside this session's actual1055pass core and exact controls — avoid redundant
physical runs — cost: independent review does not provide a second execution.
Final: Ruling: remote publication and exact-head CI are executor follow-up —
complete both before reporting delivery — cost: wait for CI, no premature sync claim.
Final: minor (deferred): none.

This committed ledger snapshot precedes final push verification. The immutable
local delivery receipt records final HEAD/remote/CI completion; the full research
goal remains active because failed mechanism/full gates and paper evidence remain.
