# Provisional simulation protocol and interfaces

Current executable entry points are documented in
[experiments/ccta2027](../experiments/ccta2027/README.md). The complete historical
protocol files and their source freezes remain unchanged. The refactored study v2
runner uses new identities and cannot resume a historical v1 run as current evidence.

The development protocol is stored as complete case/config manifests in paper_simulation_protocol.development.yaml. It is provisional and has not been approved by the professors. All original safety, G500, stopping and watchdog thresholds are retained. The new physical completion metric requires the original settling rule and Euclidean endpoint error within the existing task tolerance; it is separate from G500 and will not rewrite historical verdicts.

## Methods
All method interventions act only on the centroid before CONTACT_READY. Original CVT mass and redeployment guards remain common. boundary_cvt is unchanged. nearest_observed_target selects the closest observed offset target. local_spacing is an engineering baseline, not a reproduced literature algorithm: among observed offset targets within the local radius, maximize min(neighbor clearance, 2*d_min) minus 0.25 times travel distance, with deterministic first-index tie breaking. Only communicating neighbor positions are used. All methods receive the same information permissions, not identical trajectory-dependent maps.

The four separate centroid ablations remove arc weights, remove arc weights with confidence-weighted total mass normalization, remove direction offsets, and remove the exploration density term. Each rebuilds the density for the centroid computation only; all other returned cell fields stay from the unmodified computation. These are specifically target-generation ablations, not claims about removing the mechanism from every control subsystem.

## Safety candidate
Fresh triples of consecutive rays from one object may form a short segment when gaps <= 0.08 m, RMS residual <= 0.004 m and tangent cosine >= 0.98. Support points, endpoints, fit residual, empirical error (residual + 3 noise standard deviations), confidence, ray IDs and timestamp are retained. No true normal or vertex is consumed. All segments within the observed one-hold feature-switch cover contribute rows; no arbitrary row cap applies. Features older than one hold are ignored. Missing accepted features request zero nominal velocity, but other safety rows can still demand motion. This is an empirical finite repair of static theorem-local mode only. Gaussian 3-sigma and three collinear returns are not a guaranteed error bound or evidence of complete geometry. The full transport safety layer remains unchanged.

## Diagnosis and fairness
The diagnostic split enables the existing estimate_yaw mechanism alone on the five archived CVT absolute-progress-loss cases, for both target methods. Results remain diagnostic and cannot be credited as CVT improvements. No mechanism is assumed causal in advance. The main three-method comparison keeps the original downstream controller.

Development: 9 paired conditions x 3 methods. Ablations: 3 shapes x seed 2 x 4 isolated changes. Safety: 9 repaired local episodes, compared with separately rerun 9 local/oracle pairs. Diagnostic: 5 loss conditions x 2 methods. No development seed is called held out. Seeds 12..31 are quarantined because a missing historical study may have used them.

## Execution and attribution
Use D:/miniconda/envs/dbact/python.exe with PYTHONUTF8=1. The runner hashes all Python/config sources, complete case config, protocol and environment; each case owns its directory. Completed resume requires matching hashes and all output-file hashes. Failed episodes and process errors stay in the denominator. Incomplete directories are preserved and rejected rather than silently overwritten. Main guard supports Windows spawn. Outer worker count is explicit; BLAS and inner CVT threads are limited to one. resources.csv samples actual system CPU and parent memory; it must not be interpreted as per-worker utilization.

Statistics use episodes or matched cases, never frames as independent samples. A 95% Wilson interval describes unconditional task rates; paired bootstrap uses resampling seed 20261007, and missing events stay censored. Preparation, effective contact, safety and transport claims receive separate verdicts.

The development controller's density `explore_gain` is 0. The `no_exploration` ablation therefore rebuilds an identical density. Its three seed-2 trajectories match full boundary CVT byte for byte. That is a null intervention, not evidence that exploration is unimportant. `kp_explore` is a separate nominal search command and was left common. Turning on a nonzero `explore_gain` would be a new study.

## Frozen test
`docs/paper_simulation_protocol.test_frozen_v1.yaml` is provisional and not professor-approved. It was written before test execution. It contains 12 conditions (3 shapes × near/search × two joint scale/distance settings) × 3 methods, plus two verification episodes that are not part of the test denominator. Seeds 910071–910082 were absent from the accessible historical seed audit. Seeds 12–31 stay quarantined. Direction is drawn by the existing task sampler inside each seed; it is not a crossed factor. Partial boundary visibility is not a stratum. Scale and distance ratio change together.

Invalid initial states are retained. They are not replaced with new seeds. A contract failure counts in the denominator for every method that shares that initial state.

```powershell
$env:PYTHONUTF8='1'
& D:\miniconda\envs\dbact\python.exe scripts/run_paper_simulation_study.py --protocol docs/paper_simulation_protocol.development.yaml --split development --workers 3 --resume
& D:\miniconda\envs\dbact\python.exe scripts/run_paper_simulation_study.py --protocol docs/paper_simulation_protocol.test_frozen_v1.yaml --split test --workers 4 --resume
& D:\miniconda\envs\dbact\python.exe scripts/build_paper_evidence.py --batch development=runs/evidence_closure/study_v3/development --batch test=runs/evidence_closure/frozen_test_v1/test --out docs/results/evidence_closure/paper_evidence
```

## Literature baseline candidates
The RAS 2023, 2024 and 2025 PDFs were not in this workspace, so no algorithm from those papers is reproduced here. The prompt's stated lessons are already reflected: separate allocation time from task completion, keep contact-level evidence, and repeat conditions. `local_spacing` is an engineering baseline. Candidates left for the professors, and not implemented as named paper methods, are a uniform arc-length spacing rule without Lloyd iteration, and a local-normal push/brace role split. Neither should be labeled as one of those papers.
