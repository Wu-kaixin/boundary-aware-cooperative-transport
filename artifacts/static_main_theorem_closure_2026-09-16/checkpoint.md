# Checkpoint — static main theorem closure

Status: **PARTIAL** / **IN_PROGRESS**

## Source

* Worktree: `D:/boundary-aware-cooperative-transport-static-main-theorem-closure`
* Branch: `feat/static-main-theorem-closure`
* Base: `7e948a812a781f5ce3c16fad3525c7138bb672ec`
* Original workspace intact: `D:/boundary-aware-cooperative-transport-certificate-repair`

## Done

* Barrier diagnosis (aggregate) in `barrier_diagnosis/`
* Stage C/D single-nearest K0 in `stage_c_k0/`, `stage_d_k0/`
* Feature-cover implementation in `SafetyFilter._nearest_feature_rows`
* Priors recomputed in `priors_recomputed/`
* Tests: `pytest tests/test_safety_filter.py tests/test_theorem_mode.py tests/test_edge_green_and_restrict.py tests/test_apriori_centroid_bound.py` (49+34 passed, 2 skipped on the combined safety/edge/apriori run)

## In flight / next

```bat
cd /d D:\boundary-aware-cooperative-transport-static-main-theorem-closure
python scripts/diagnose_barrier_update.py --out artifacts/static_main_theorem_closure_2026-09-16 --stage-c --subdir stage_c_k0_cover
python scripts/diagnose_barrier_update.py --out artifacts/static_main_theorem_closure_2026-09-16 --stage-d --subdir stage_d_k0_cover
```

Independent seeds 29/31/37 completed: `stage_e_k0_cover/` (9/9 empty K0-fail).

Serial/parallel L2: exact match on 600-step K0, QP, h_true, J proxy, hold clearance. See `serial_parallel_consistency.json`.

Do not merge or push. Do not delete historical artifacts. Do not reuse `stage_d_k0/` as cover results.
