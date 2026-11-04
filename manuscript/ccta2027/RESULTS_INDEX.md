# Evidence and reproduction index

Updated 2026-11-04. Paths below link curated reports; their receipts point to the local raw records and exact source/environment snapshots. The manuscript is an empirical research draft, not a new-algorithm acceptance report.

| Evidence population | Complete denominator | Owning report and scope |
|---|---|---|
| Historical normalized geometry census | 9 conditions × 3 methods = 27, shared seed 2 | [Census](../../docs/results/multishape_census_v1/README.md). CVT/spacing/nearest spatial completion 5/9, 5/9, 2/9; 150 s budget. |
| Later new-start transport validation | 12 conditions × 3 methods × 2 estimator priors = 72 | [Transport validation](../../docs/results/transport_validation_v1/README.md). Common 0.45 prior gives 11/12 for each method. This prior is not a safety velocity bound. |
| Deployment contribution within that validation | All 36 common-prior trajectories | [Deployment contribution](../../docs/results/deployment_contribution_v1/README.md). Twelve designed conditions with common-prefix coverage; post hoc estimator, not 36 new episodes. |
| Finite-support physical comparison | 4 control/candidate pairs = 8 | [Checkpoint](../../docs/results/post_support_validation_checkpoint_v1/README.md). Control 4/4, candidate 0/4, four 600 s timeouts. Full candidate constraint replay is incomplete and quarantined. |
| Fixed-state contact-capability diagnosis | 8 events, 128 robot states, 112 admitted; 336 systems | [Capability audit](../../docs/results/post_support_contact_geometry_capability_v1/README.md). Original positive-force subset has 17 states. 1749 control frames replayed, zero new physical trajectories. |
| Earlier feedback oracle and anchored rejection | 6 oracle trials; separate 9-arm candidate comparison | [Readiness v1](../../docs/results/readiness_diagnostic_v1/README.md), [progress v2](../../docs/results/progress_local_v2/README.md). Selected development cases; cannot pool with the census. |
| Current-source resource/noise validation | 5 shapes × 3 methods × 4 regimes = 60 | [Full report](../../docs/results/resource_noise_validation_v1/README.md), [all cases](../../docs/results/resource_noise_validation_v1/all_60_episodes.csv), [paired differences](../../docs/results/resource_noise_validation_v1/paired_differences.csv). New shared seed 2026110401; 58 spatial completions, 56 without original hard safety failures. |
| Older broad study | 169 episodes, including 54 perturbation trials | [Historical study](../../docs/results/ccta2027_draft_v2/README.md). Historical failures retained; no current-source validation claim. |

The populations above overlap where explicitly stated. Their denominators must not be added into a purported independent reliability sample. All failures and execution budgets are owned by the original frozen protocols.

## Current executable identity

- Branch and starting HEAD: `paper/boundary-aware-safe-deployment`, `35d683cc2eb332c8dbc274f51612093f5e4e271d`. Existing uncommitted research work remains local; no remote state is asserted.
- Runtime source SHA-256: `e6b7456f5f0a4d71ddd4b33159c058e215115aae35c01b6c5c9eae7e68f32767`.
- Protected YAML SHA-256: `a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112`.
- Current validation protocol SHA-256: `caa29e613ceb6bfd56a97096e6e9d8651be4de0782f64a4643943c40d0a8fa4b`.
- Actual root environment: Python 3.12.14; versions are in `runs/research/2026-11-04-resource-noise-validation-v1/freeze/receipt.json` and `packages.json`. Two existing lock discrepancies (joblib and PyYAML) were preserved, not silently repaired during frozen execution.
- Original current run: `runs/research/2026-11-04-resource-noise-validation-v1/run/`. `analysis_v1/` verifies all case product hashes, identities, initial states, complete step logs, spatial endpoints and independent evaluator outcomes. `freeze/` stores the prospective design and execution helpers.

`configs/research/best_known_transport_v1.yaml` is a protected selected configuration, not the global default. The current sensitivity protocol resolves its actual switches, gains and physical settings. Rejected anchored and finite-support options remain off. No implementation improvement is inferred simply by comparing historical and current success counts: multiple source/configuration factors differ.

## Equations and information permissions

| Manuscript item | Owning implementation/evidence | Validity limit |
|---|---|---|
| Local CVT deployment and internal target controls | `src/dbact/deployment/cvt.py`, `src/dbact/control/controller.py`; case resolved configs | Generic centroid formula summarizes the executed boundary/directional/exploration density; not a new coverage theorem. |
| Unilateral forces and sampled execution | `src/dbact/contact_dynamics.py` | Penalty contact, kinematic discs, four physics substeps; no attachment or hardware equivalence. |
| Shared PI, navigation and phases | `src/dbact/control/transport.py`, `boundary_navigation.py`, `phase.py`, `controller.py` | Shared median/coverage/readiness supervision prevents a fully decentralized end-to-end claim. Phase labels do not replace actual activation events. |
| Spatial completion versus G500 | `src/dbact_sim/experiments/deployment.py`, `src/dbact/contracts.py` | HOLD plus linear-speed dwell and Euclidean error; no angular-speed settling criterion. G500 uses a separate projected-arrival contract. |
| Directional capability and minimum-deviation projection | `src/dbact/safety/qp2d.py`; [336-system witnesses](../../docs/results/post_support_contact_geometry_capability_v1/README.md) | Standard convex bounds on frozen states; forward robot velocity does not imply forward cargo force or task completion. |
| Identifiability and cumulative accounting | `supplementary_diagnostics.tex`; progress-v2 records | Fixed-correspondence translation model and bookkeeping identities, not physical estimator accuracy or new theory. |
| Static safety theory | `theory/static_deployment/main_theorem.tex` | Static oracle-map assumptions do not transfer to uncertain moving contact. |

The [Phase 1 audit](../../docs/PHASE1_BASELINE_REVIEW_2026-11-04.md) records this information separation. Simulator object labels and localization are assumptions; evaluation truth is not fed to ordinary local control. The labeled historical oracle is an explicit exception. The later local-polygon admission proposal is unimplemented and carries no experimental acceptance claim.

## Reproduce or inspect without overwriting originals

Current validation commands and fresh output rules are in its [report](../../docs/results/resource_noise_validation_v1/README.md). Use the root `.venv` for every Python command. Runtime source/config identities must match before replaying a frozen batch. A new experiment requires a new `runs/` directory and an honest new identity if any source/environment differs.

| Helper | Function |
|---|---|
| `scripts/resource_noise_validation.py` | Freeze, verify, fixed worker benchmark, profile, execute exactly 60 cases; refuses silent overwrite. |
| `scripts/analyze_resource_noise_validation.py --out <fresh-study-subdirectory>` | Recheck current raw products and derive the complete case table and sensitivity plot. |
| `scripts/render_resource_noise_media.py --out <fresh-study-subdirectory>` | Preselected measured stages, overview/GIF and two synchronized comparisons; full decode checks. |
| `scripts/diagnose_resource_noise_results.py --out <fresh-study-subdirectory>` | Current vector-navigation and post hoc C8 failure diagnostics; no new physics. |
| `scripts/build_ccta2027_latest_evidence.py --out <fresh-directory>` | Reverify 288 historical deployment products and derive matched/capability figures. |
| `scripts/publish_resource_noise_evidence.py` | Curate compact public-size records; refuses to replace the existing result directory. |

The manuscript copy of `resource_noise_results.tex` uses the analyzer's numbers with an editorially clarified caption. Figure data were not manually adjusted. Latest historical evidence is in `runs/research/2026-11-04-ccta2027-draft-refresh-v1/latest_evidence_v3/`; earlier failed plot attempts and their reasons remain local.

Media files and selection rules are listed in [MEDIA_AND_CLAIMS.md](MEDIA_AND_CLAIMS.md). LaTeX compilation is in [README.md](README.md). Final audit hashes and test receipts are in [FINAL_AUDIT.md](FINAL_AUDIT.md).
