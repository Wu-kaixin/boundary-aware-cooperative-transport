# Reference-driven boundary map implementation plan

> Required workflow: superpowers:executing-plans, inline implementation and one
> fresh final review. User delegated research choices and normal branch publication.

Goal: enforce a shared local translation reference for map geometry and velocity,
then test it under the existing physical protocol. Python/NumPy, root .venv only.
Spec: reference_map_v1_design.md. Base: fa8b6194d65d8027303c97cdb207ecb6cf02742a.

Global constraints: preserve numeric parameters/physics/criteria and all old
receipts. New runs only; default selector False. No hidden truth/global inputs.

Review focus: reference dropout and recovery; out-of-hop data exclusion; double
integration through fusion; inactive-path compatibility; post-freeze rotations.

- [x] Add tests in tests/test_reference_map.py for real map shifts and fusion,
      invalid input, zero epoch and one-hop gap/recovery; watch failures.
- [x] Implement align_translation and opt-out motion integration in boundary_map.py;
      integrate the candidate through control/reference_map.py and controller.py.
      Add flag validation/trace/episode metadata and default protection assertions.
- [x] Run targeted and full core tests; preserve exact baseline behavior.
- [x] Freeze eight physical episodes in a new reference_map_v1 protocol/run directory,
      execute once and analyze all outcomes using unchanged original evaluators.
- [ ] Publish provenance and physical results, updated README and all negative
      decisions; fresh final review, authorized normal branch push and CI.

Ruling: choose reference-driven geometry over deleting old points by age or by a
no-return angular fan — the traced error is wrong transport of an unrefreshed
point; sparse free rays do not certify empty sectors for arbitrary shapes. Cost:
translation model and changing local voter sets may still fail physical gates.
Missing reference is a measured condition, not permission to use global state.

Tasks1–3 complete. Initial RED:10 failures for missing map API/selector/helper.
First GREEN:9 passes,1 existing earlier every-frame validator message; accept
the existing validator rather than change behavior. Added complete-controller
missing-reference guard check. Targeted71 pass; core903 pass/2 skip,139.26s,
terminal exec2201 exit0. Receipts in runs/maintenance/2026-11-03-reference-map-v1/.
No local MAS repetition; final CI will independently run its normal matrix.

Task4 frozen before new physics: protocol SHA-256
7f61ac1b581c4d1b0b9b908791f839bd02f622297a68fbbdd4cd4798fcc7648d;
runtime source dc9e9ac1d5360f54c4bc7b85d1066c8ec01b1baa8955d90c181385857b918f37.
All4 old episode evaluators match published rows; effective parameters, physics,
phase gates and initial states verified for all8 prospective cases. Protected
config SHA remains a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112.
Single execution session35111 is running; inspect its terminal result, do not
duplicate a live run or treat a polling timeout as termination.

Task4 complete. Session35111 terminal exit0,8 executions,0 execution errors,
source integrity PASS. Analyzer99792 terminal exit0; publisher57952 terminal
exit0. Four baselines reproduce all25 arrays and all scientific JSON fields.
Candidate safe completion3/4 vs4/4, false HOLD/hard failures0 each, G5002/4 each.
C scaling8 to0 but TRANSPORT quorum never attained. Skew no reference freeze,
600s budget termination, endpoint1.2m, acquisition1422/post-acquisition21608
scaled robot-steps; total23030. The latter are all status=unavailable, not a
successful freeze. All candidate absent-reference nominal commands are zero;
all declared voters independently checked fresh and within one-hop distance.

Ruling: reject this candidate under the original gates and keep it default off.
One traced geometry defect is explained and C scaling is removed in this screen,
but acquisition/deployment and coordinated participation are not repaired. Cost
of adopting by C outcome alone would be losing the previously successful skew.
No extra episodes are appended to this frozen study; subsequent experiments
need a separate prospective protocol under the user's continuing authorization.

One fresh-context reviewer is inspecting fa8b619..implementation41a3183 plus the
final evidence. Important interim finding: nearest_feature row mode ignores
per-point velocities, so this new selector must reject that unsupported mode.
All frozen cases use aggregate mode; preserve their runtime/source/protocol
identity and handle the compatibility guard in a separately tested followup.

Final review completed on fa8b619..a565a18: With fixes, exactly one Important
finding and no unresolved Minor/Critical findings. Executor's single fix pass
rejects the unsupported nearest_feature combination. New focused regression
failed before the guard (DID NOT RAISE); targeted33 then passed; current core
904 passed/2 skipped in149.61s, terminal session33498 exit0. All8 aggregate-row
configurations still construct. Current source2a49a2bc273f40b8a775c5879db666a0619
df377effe04866c45b3e68f79211d differs from physicaldc9e9ac1 only in this guard.
All frozen files, products and protected parameters remain unchanged. See
docs/results/reference_map_v1/final_review.md and guard_followup_checks.json.
Publication documents are ready; normal push and exact-head CI will be recorded
in this run's ignored maintenance receipts after the publication commit.
