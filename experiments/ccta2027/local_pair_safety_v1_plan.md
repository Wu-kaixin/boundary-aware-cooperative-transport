# Acquisition local pair implementation plan and ledger

> Execute inline using superpowers:executing-plans; one fresh final review.

Goal: negotiate local mutual safety responsibility before executed fallback,
then evaluate the complete unchanged physical gates.
Spec: local_pair_safety_v1_design.md and the preserved local_pair_safety_v1_spec.md.
Base:77de03d150eb83d3ff7131ee65f9b39e87552925. Root .venv for all Python.
Source stays in src/, helpers scripts/, tests tests/, designs experiments/;
all new outputs in fresh runs/ directories. Numeric settings and old evidence immutable.

Architecture: prepare/finalize safety problems; a separate local-pair protocol
routes current one-hop offers/proposals/acks; controller integrates selected
commands before final accounting, and audits application before position mutation.
SciPy is already a core dependency. Do not label SLSQP success a safety proof.

Review focus:
1. Retained peer/third-neighbor rows, mutual-row signs and distinct speed balls.
2. No global matching, stale reply acceptance or two simultaneous peer commitments.
3. Post-clipping commands and entire hold separation satisfy the accepted rows.
4. Proposal checks do not alter executed scaling/G500 accounting; failed pairs
   retain original fallback; default path remains exactly reproducible.
5. Observational support scope, source identities, all8 physical denominators,
   baseline exactness and adverse C results remain honest.

- [x] Task1: prepare/finalize in src/dbact/safety/filter.py; tests/test_safety_preparation.py.
  RED missing preparation; GREEN side-effect-free preparation, parameter/duplicate
  rejection, original infeasible fallback accounting and interleaved row counts.
- [x] Task2: src/dbact/safety/local_pair.py; tests/test_local_pair_safety.py.
  RED real repaired/impossible fixtures and handshake failures; GREEN local-only
  offers, solver, independent validation, deterministic disjoint two-ACK commit.
- [x] Task3: src/dbact/control/controller.py and tests/test_local_pair_wiring.py.
  Only one default-off selector; preserve old default and raw diagnostics;
  integrate before finalization, check applied commands; full core suite.
- [x] Task4: separate physical design/protocol and scripts/local_pair_study.py.
  Freeze source/environment/inputs first; execute4+4 episodes once; independently
  audit pair execution and matched baseline evidence; publish complete metrics.
- [ ] Task5: one fresh independent final review, fix material findings once,
  record all rulings/minors, synchronize normally and verify exact-head CI.

Preflight: semantic language server failed; targeted source reads are available.
No experiments are running or being restarted. Prior upload/CI completed.

Task1: four meaningful RED failures for missing preparation; GREEN47passed
(preparation, original safety filter, finite-support wiring),5.39s. Preparation
copies nominal/rows and has no solver accounting; finalization restores local
row counts, verifies the parameter snapshot and refuses duplicate accounting.
Original fallback and zero-input counters retained. Logs/XML under
runs/maintenance/2026-11-03-local-pair-safety-v1/preparation-{red,green}.*.

Task2:18RED missing-protocol failures, then22GREEN including preparation,2.49s.
Local offers/one outgoing proposal/one endpoint choice/two digest-matching ACKs
produce disjoint commitments. Full peer/non-mutual/wall rows and individual caps
are checked; post-clipping velocities and continuous hold separation are checked.
SLSQP failure is logged as unverified, never proof of mathematical infeasibility.
Log/XML pair-red and pair-green1 retained. No new physical episodes yet.

Task3 wiring RED8failures/2passes. First GREEN attempt57passes/1failure:
the projection-backend negative fixture hit the pre-existing paper-config ban
before pair validation; use cvxpy to exercise the intended unsupported-backend
guard. Added a meaningful raw-failed-proposal record RED; retained both peers'
original matrices in failed trials. Final targeted GREEN59passed,4.78s.
Core first run1032passed/2skipped/1failure,154.14s: the protected-receipt test
needed to explicitly pop/assert the newly default-off boolean; all140 existing
parameters matched. Added that assertion without changing the protected receipt.
All failed logs/XML retained; a fresh full core-final run follows.

Task3 complete: core-final1033passed/2skipped,152.16s,exit0. All root-venv
runtime checks complete; no live core process remains. Pair selector defaults
off, execution is validated before mutation, and current diagnostics distinguish
independent proposals from final solver status. Physical freeze not yet run.
Task4 audit RED6missing-helper failures, GREEN6passed2.38s. Uses recorded rows,
actual commands, current neighbor positions, SHA identities and both ACKs rather
than importing protocol acceptance. Existing original control arm label retained
to compare every scientific field without an arm-label normalization waiver.

Physical evaluation preflight added an execution-error/absent-audit fixture:
RED caught None audit handling; use a failed gate rather than an analysis
exception. The test's insertion location was corrected before its GREEN run.
Study-green2:7passed, including all6earlier fixtures. This is separate from the
1033-pass runtime core run. Helper --help import succeeds; no new physics yet.

Task4 freeze succeeded before physics:8cases, runtime sourceSHA
6a24d76064d89dbc9485c5f5b3623c4dc7ad2ef187436c32319fe6bf2b782727;
protocolSHA d991f0ffa0eeceedcdff2a68475a9662fd1b8be258294db5703032b073bd8230.
Every prior effective-controller hash matches after removing only the new boolean;
canonical YAML and original physical parameters/gates/initial states are exact.
Frozen source, helpers, designs, tests and environment receipts are preserved.

Task4 physical execution completed8/8,0execution errors,source integrityPASS,
terminal0 (session51882 closed). Independent analysis also terminal0 (2173closed):
all4original controls exactly reproduce all25arrays and scientific fields;
all7accepted pair commitments/14robot steps verified from recorded rows and
measured commands. Both arms safe4/4,falseHOLD/hardfail/timeouts0,G5002/4.
C: acquisition scaling1->0,rotation9.071854->4.942476deg,time59.45->53.70s,
endpoint17.690->6.939mm,still noformalTRANSPORT. Skew: acquisition scaling1->1,
post-freeze0->1,rotation.792236->2.289195deg,time49.30->55.35s,
endpoint18.149->12.933mm. Near circle/L physical outcomes unchanged.
MechanismFAIL(acquisition scaling and post-freeze nonincrease); full-chainFAIL.
Candidate remains default-off and unadopted. No retuning or extra physical runs.

Read-only residual inspection: skew saved402 agent05 has one active reach cap,
so correctly rejected by this candidate's predeclared scope; saved989 occurs
inBRAKE after reference freeze, also outside scope. Accepted pair frames in the
audit are zero-based; publication must convert to one-based saved frames or label
them explicitly. C4/skew3accepted pairs, allagent05/09;22failed peer solver trials
are numerical nonverification, not22proofs of infeasibility. Do not remove scope
guards to claim success; diagnose the uncapped402demand and989motion rows first.

Task4 publication complete: nine generated products under
docs/results/local_pair_safety_v1/, all8rows and all7accepted pairs retained.
Figure visually inspected: complete labels/units/denominators and adverse skew
results visible. Narrative separates received-row audit from unknown-boundary
proof, current comparison from older better C rotation/time, and all test runs.
Task5 starts with current publication/navigation verification, then one fresh
whole-increment review, normal authorized branch push and exact-head CI.

Task5 final review: one fresh read-only reviewer checked77de03d..2a96cdd,
noCritical/noImportant/noMinor; ready to publish the default-off experiment.
Independent checks include9products,137runtime files,13helpers,all8denominator,
4exact controls and7pairs. Fresh full core-publication run:1040passed/2skipped,
138.92s,exit0. Committed-tree repository healthPASS. No runtime fix or new physics.
The first ad-hoc publication verifier imported the wrong class name and stopped
before checking anything; correcting its command to DBACTParams yieldedPASS.
The failed command and successful verification are separately recorded locally.

Final: Ruling: asynchronous/hardware timing and untrusted network input remain
outside this synchronous in-process increment — retain the documented scope —
if applied to a real network prematurely, its message/timing assumptions may fail.
Final: Ruling: unknown object geometry is not certified by the received-row audit —
publish only the observational evidence — otherwise safety claims would be overstated.
Final: Ruling: generalization, wider old-success preservation and independent
deployment benefit need separate experiments — retain all4reused conditions as
development evidence — otherwise regressions or incorrect attribution may be hidden.
Final: Ruling: C activation/rotation and skew regression remain disclosed research
failures, with candidate unadopted — do not soften predeclared gates to ship it —
premature adoption would retain observed scaling and physical regression.
Final: minor (deferred): none.

This committed ledger is the pre-push publication snapshot. Task5 delivery is
completed only when the same head is verified on GitHub and its Windows/Ubuntu
CI passes; external delivery receipts are appended under the maintenance folder,
without rewriting this snapshot or the frozen experiment evidence.
