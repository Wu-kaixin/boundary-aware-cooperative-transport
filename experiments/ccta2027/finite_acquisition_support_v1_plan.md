# Finite acquisition support implementation plan

> Use superpowers:executing-plans inline. One fresh final reviewer covers the increment.

**Goal:** replace unsupported acquisition plane extrapolation with a measured
finite-support candidate without altering protected numerical parameters.
**Spec:** finite_acquisition_support_v1_design.md.
**Architecture:** a sensing-only packet/chord module, explicit guarded safety rows,
and default-off acquisition-only controller integration after the saved-state gate.
**Tech stack:** root .venv, NumPy, existing 2D QP and CLARABEL.

## Global constraints

Preserve original physics/config/gates and frozen data. New outputs under fresh
runs/ directories. Truth stays observer-only. No unmeasured safety certificate.
All Python uses .\.venv\Scripts\python.exe. Existing research checkout is reused.

## Review focus

1. Missing/stale/isolated observations must fall back, never remove all object rows.
2. Source/object/ray discontinuities must not become fictitious boundary segments.
3. Noise allowance must not be omitted or counted twice at acquisition radius.
4. Motion bounds must remain valid after normals rotate to radial feature rows.
5. Disabled behavior, one-hop scope and post-acquisition behavior must remain exact.

### Task 1: observational geometry and saved-state gate

Create src/dbact/sensing/finite_support.py and tests/test_finite_support.py.
Interfaces: scan_support(returns, origin, timestamp, noise_std) -> ScanSupport;
support_rows(position, packets, params, timestamp, motion_bound) -> SupportRows
with ready/reason/A/h/lower_distance/coverage diagnostics. No QP mutation here.

- [x] Write analytical/failure-mode tests; observe missing-feature RED.
- [x] Implement finite chords and guarded cover with unchanged existing constants.
- [x] Geometry21 GREEN; first core959 passed/2 skipped. v2 adds7 RED/GREEN cases.
- [x] Freeze separate26-state evaluations; retain all three alternatives and both
  chord versions.312 first-probe comparisons,0 v1 admitted QPs,104 v2 comparisons.
- [x] Decide wiring from coverage and constraint evidence; ruling below.

### Task 2: optional acquisition-only integration and physical verification

If Task1 admits wiring, add default-false flag and guarded filter/controller path.
Test incompatibility, locality, missing motion, fallback and disabled equivalence.
Run core suite. Freeze a separate matched physical protocol before new episodes.
Do not promote candidate or infer general reliability from reused conditions.

### Final

- [ ] Publish bounded results, update next actions, perform one fresh final review.
- [ ] Fix material findings once; preserve raw failures and all review rulings.
- [ ] Integrity checks and authorized normal research-branch sync.

## Execution ledger

Previous turn: progress (publication and exact-head CI). Three-alternative probe
completed exit0 with312 solver comparisons; no runtime change or physical episode.
Serena symbolic Python backend failed initialization again; use targeted file
operations under the already authorized task, without changing its environment.
Pre-flight shared interfaces: sensing packets feed safety rows, then controller;
the scalar motion bound and existing noise allowance are explicit in the spec.

Task1 progress: geometry21 RED->GREEN; frozen chord-only gate0/26 (uncovered
returns), preserved raw v1. Separate point-guard-v2 design and7 new RED->GREEN
tests retain isolated returns as explicit radial guards. v2 admission26/26,
zero-motion full-rho feasibility18/26; bounded-motion15/26; lower distance never
exceeds observer truth in these26 states. This is not a safety certificate.

Ruling: proceed with a matched acquisition-only candidate despite8 infeasible
saved states — earlier finite geometry changes the path before those states;
physical feasibility is unproven — cost if wrong: a negative matched study,
which must retain all failures rather than retune the protected parameters.

Task2 progress: wiring12 missing-feature failures then107 targeted tests pass.
First full wiring suite978 pass/2 fail/2 skip: both failures were protected-value
tests not yet recognizing the new default-false boolean. Add explicit False
checks, retain exact equality of all140 prior fields and original YAML hash.

Ruling: also require initial_reference_status=='acquiring', excluding the freeze
frame even though its map state is still stationary_acquisition — runtime restores
the transport radius on that frame — cost if wrong: incorrect margin accounting
could reach contact control. New freeze-boundary RED/GREEN regression covers it.
Reach-cap use is separately traced; a capped demand is not an uncapped certificate.

The freeze-boundary/cap diagnostics tests both failed before the repair and then
passed in a65-test targeted run. Five new physical-evaluator tests went RED; the
first GREEN attempt exposed an inaccurate synthetic fixture (it labelled the old
C baseline G500-pass). Correct the fixture to the actual old C/skew failure state;
all5 then pass, without changing the preservation gate. Preserve both logs.

Physical design frozen in finite_acquisition_support_physical_v1_design.md before
new episodes. Final core suite987 passed/2 skipped in150.02s, exit0; original
receipt core-final.xml retained. Study freeze completed exit0:8 cases,96,000
maximum total steps, all old numerical/physics/phase/initial-state checks pass.
Execute the new frozen study exactly once and preserve its process handle. The
current runtime has not been scientifically validated yet; no candidate adoption.

Task2 complete: the original single run process terminated exit0. All8 cases
COMPLETED, execution_errors0, source_integrity PASS. Analysis terminated exit0;
four original controls exact in all25 arrays and scientific records. Both arms
safe complete4/4, false HOLD/hard violations/timeouts0, G5002/4. Candidate C/skew
acquisition scaling13->1 each; skew post-acquisition6->0. C still lacks formal
TRANSPORT; rotation1.471->9.072deg, duration50.45->59.45s, error3.748->17.690mm.
Mechanism and full-chain gates FAIL. Preserve all outcomes; no adoption or tuning.
Both residual events used candidate rows with no reach cap; saved diagnostics
alone do not identify the remaining joint-QP conflict. All4 zero epochs valid.

Publication in progress: a separate presentation helper verifies frozen source,
protocol,8 receipts, old publication products and all three diagnostic snapshots.
No changes to frozen runtime, evaluators, physical design or tests during/after
execution. Root README and navigation now distinguish current results from prior
observation-reacquisition results and expose rotation/time regressions.

Publication complete:8 CSV rows, full analysis,2 saved residual diagnostics,
three separately hashed preliminary probes, PNG/PDF and engineering receipt.
Initial visual inspection found adjacent numeric labels touching; staggered arm
labels in presentation v2. Preserve initial publisher/outputs under maintenance
publication_layout_v1; v2 analysis/CSV/probes/residuals/README byte-identical.
No research rerun or changed conclusion. Final review and branch sync remain.
