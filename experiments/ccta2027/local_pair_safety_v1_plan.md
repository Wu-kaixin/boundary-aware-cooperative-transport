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
- [ ] Task3: src/dbact/control/controller.py and tests/test_local_pair_wiring.py.
  Only one default-off selector; preserve old default and raw diagnostics;
  integrate before finalization, check applied commands; full core suite.
- [ ] Task4: separate physical design/protocol and scripts/local_pair_study.py.
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
