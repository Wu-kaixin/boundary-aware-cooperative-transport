# Acquisition normals and formal activation: retrospective audit

Base: 9bbe274504167c4f7a9aab34bedefa9c40e54a0a. The previous goal turn made
progress: published the8-case screen, updated navigation and verified both-platform
CI on that exact head. The full goal remains incomplete.

Known evidence: the reacquisition candidate completes4/4 reused development cases
but has13/13 acquisition scaled robot-steps in far C/skew, plus6 post-freeze skew
events. All26 acquisition events concern agent05. Two selected states show wrong
local support geometry despite positive true clearance; aggregate coordinate
normalization is already enabled. C never formally entersTRANSPORT. A preliminary
saved-record read finds its maximum active5 only inBRAKE; CONTACT_READY maximum3.
This is a posthoc diagnosis, not a prospective success-rate experiment.

Questions and fixed scope:

1. Exactly replay the current frozen far C/skew candidate controllers through
   their first frozen frame375/409. Record agent05's map immediately before and
   after every update, incoming own/one-hop points/normals/confidences/source ids,
   and every original scaled QP in that prefix. Verify commands, all diagnostics,
   solver totals, phases/guards/responses and recorded map points every frame.
   Original simulator poses are forced from saved data; no physics step is run.
2. Trace selected object-row map cells through the actual fusion history and back
   to originating scan returns. Rebuild PCA neighborhoods with the original sensor
   algorithm and existing noise/parameters. Separate source-estimation error,
   cell mixing and support-plane extrapolation. True geometry may be evaluated
   by the observer only and may never replace an online input. Preserve all26
   acquisition conflict states, including any contrary evidence.
3. Reconstruct the original phase monitor for all four completed candidate
   episodes from saved signals, verifying each recorded response. Split active
   counts by phase and record the first braking/quorum events. Distinguish a late
   quorum from a missing transition when all applicable guards were satisfied.
4. Do not modify runtime, thresholds, configuration or old artifacts in this
   audit. A later repair needs its own design, tests and frozen physical study.
   No claims of controller improvement, general reliability, independent
   deployment advantage or default adoption follow from this audit alone.

Use the root.venv only. Freeze the diagnostic script/design identity before each
execution. First run a20-frame replay preflight in a separate directory; retain
failures without overwriting. Then run each full prefix once in a fresh directory
under runs/research/2026-11-03-acquisition-normal-audit-v1. The full trace budget is
784 controller frames plus the separate20-frame engineering preflight, zero new
physical episodes. Phase-only signal reconstruction is counted separately.

Publication will include compact tables, lineage examples, complete event counts,
source identities and uncertainty. Large raw traces stay local. Under the existing
executing-plans workflow, one fresh final reviewer covers this increment once it
is ready; the previously published study is not re-reviewed. User authorization
to continue research and normal research-branch publication persists.

Execution ledger:
- [ ] Trace helper and20-frame exact preflight.
- [ ] Exact375+409-frame acquisition replay; preserve all26 scaled events.
- [ ] Original phase-monitor reconstruction for all4 candidate episodes.
- [ ] Source scan/fusion lineage analysis; determine the next concrete repair.
- [ ] Evidence publication, one final review, integrity checks and normal sync.
