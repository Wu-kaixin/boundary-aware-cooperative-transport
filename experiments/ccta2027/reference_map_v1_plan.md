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

- [ ] Add tests in tests/test_reference_map.py for real map shifts and fusion,
      invalid input, zero epoch and one-hop gap/recovery; watch failures.
- [ ] Implement align_translation and opt-out motion integration in boundary_map.py;
      integrate the candidate through control/reference_map.py and controller.py.
      Add flag validation/trace/episode metadata and default protection assertions.
- [ ] Run targeted and full core tests; preserve exact baseline behavior.
- [ ] Freeze eight physical episodes in a new reference_map_v1 protocol/run directory,
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
