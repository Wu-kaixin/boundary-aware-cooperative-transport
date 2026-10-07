# Provisional simulation protocol and interfaces

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

Independent test cases, source freeze and budget will be fixed after pilot cost/variance and historical-seed audit, before reading any test results. No adaptive per-method tuning on test data is allowed. Statistics use episodes or matched cases, never frames as independent samples. A 95% Wilson interval describes unconditional task rates; paired bootstrap describes continuous effects, with missing events separately censored. Preparation, effective contact, safety and transport claims receive separate verdicts.

```powershell
$env:PYTHONUTF8='1'
& D:\miniconda\envs\dbact\python.exe scripts/run_paper_simulation_study.py --protocol docs/paper_simulation_protocol.development.yaml --split development --workers 3 --resume
```
