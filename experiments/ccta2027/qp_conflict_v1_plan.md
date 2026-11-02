# QP conflict v1 execution ledger

Spec: qp_conflict_v1_design.md. Base: 8e7920d4308fab4654a396de7ea6387aef2aa930.

1. Inspect owning safety/controller/logging paths; implement non-mutating replay
   and exact output validation. Preflight a short original prefix, retain failure.
2. Freeze four existing far-field prefixes, runner/source/input/environment and
   diagnostic operations before execution. No runtime or numeric changes.
3. Execute the bounded replays once; inspect precise matrix/feasibility witnesses,
   validate selected witnesses with an independent solver; publish all results.
4. One fresh-context final review, normal authorized research-branch push and CI.
   Select the next real repair from evidence; keep the whole research goal active.

Pre-flight: replay must preserve original perception RNG and complete controller
state; comparing only the final command would not establish internal identity.
Frozen-source imports must occur before any dbact module is loaded. Counterfactual
row assembly must not alter online statistics or subsequent filter state.

Ruling: run diagnostics on the four existing far-field prefixes — they contain
all relevant scaling outcomes for the rejected geometry candidate and its matched
reference; no new physical data or numeric retuning is needed. Cost: conclusions
are conditional on those development trajectories, not a general safety proof.

Status: source investigation and design complete; replay preflight pending.

Step 1: complete. First 20-frame preflight stopped at frame1: live undefined
diagnostic values versus original dump's null serialization. Its failure/log and
original runner copy are retained in maintenance/preflight1. Match the original
tuple/non-finite serialization without changing values or tolerances; second
20-frame preflight is EXACT_RECONSTRUCTION for every command/diagnostic/solver/
phase/map check, zero new physics. These are validation runs, not scientific
scaling evidence. py_compile passed for both diagnostic helpers.

Step 2: frozen before diagnostic execution. Four prefixes,3411 control frames,
953 recorded scaled robot-steps. Protocol SHA-256:
7422c188312484b0ca663851e1910e01d0f04cc052aa65a2148e98f8d614c318.
Root: runs/research/2026-11-03-qp-conflict-v1/; originals/source/environment
verified. Runtime remains b602d6d64413d897f95e0a2b58914159f9eb97c1c2f6870431aaed3479b5511d.
Step 3: the one frozen four-process diagnostic has started; wait for its terminal
handle, do not duplicate it or treat an observation timeout as a stopped run.

Step 3: complete. Exec session92136 terminal exit0; all four child exits0 and
all3411 frames exact. Analyzer exit0. All953 events classified: full/no-rho
combined0 feasible, hard-only953, object-only953, zero-estimated-motion205.
All12 selected labels (11 unique frame-agent witnesses) agree with independent
CLARABEL. No runtime source/configuration/physical change.

Publisher session52821 terminal exit0; all953 CSV rows and 4-case table retained.
Post-hoc linear certificates independently explain 10 labels (9 unique events)
as inconsistent even without speed cap; remaining two require0.631/1.529m/s.
Figure visually checked. C geometry frame917's nearest map point has boundary
distance lower bound0.590052m by triangle inequality; this diagnoses invalid
geometry, not yet the precise map-update source of the defect.

Ruling: keep original freezes/data and add post-hoc algebraic witnesses in a
separate product — matrix feasibility is already frozen; these explain its
contradictions without changing selection, outcomes or gates. Cost: neither the
geometry discrepancy nor a contradictory local QP proves a future repair works.

Step 4: final review and authorized publication pending. Full goal remains active.

Step 4 review: complete for 8e7920d..9db6972. Fresh-context read-only reviewer
reported no Critical, Important or Minor findings. Independently reconstructed
all 11 unique selected matrices and five feasibility sets with CLARABEL; verified
all 953 event indices/aggregates, nine contradiction certificates, two minimum
speeds and the C geometry lower bound. Source/environment/original/published
identities and frame indexing match. See docs/results/qp_conflict_v1/final_review.md.

The review did not rerun 3411 frames, new physics or the full core suite. It did
not judge future repair success, general reliability, team-wide impossibility,
dynamic certification, hardware or paper readiness. These remain unmet goal work.
The executor's final README clarifications explicitly name the rho-free zero-motion
counterfactual and distinguish the earlier eight physical episodes from this
zero-new-physics diagnostic. No frozen product or runtime/configuration change.

Repository health before the diagnostic commit: PASS, 1359 tracked files, 445
Python files, 15 navigation documents, pip check and git fsck; health.json records
the precommit HEAD and staged working files. Final publication checks and remote
head/CI outcomes are retained separately under the same maintenance directory.
Normal push to the existing research branch is authorized; no force or main merge.
This diagnostic and its review are complete; the full research goal stays ACTIVE.
