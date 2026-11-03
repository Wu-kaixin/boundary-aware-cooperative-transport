# Final review: contact-row publication increment

Review date: 2026-11-03. One fresh-context review by the dispatched reviewer; no further reviewers or subagents were used.

Base: `f1a8497bb6831fb085aebeb8850f7bbd7d59c10f`

Head: `2c5e50377d0f7f2d89ed3499f81fd042fafcf303`

## Strengths

- Both analyses retain the existing eight-event, 128-state cohort and all 16 nonadmissions. The 112 admissions, 17 actual positive-force states, and 11 positive-force declared pushers agree with the raw contact analysis. The public positive-force table is exactly the corresponding subset of the complete table.
- The decomposition uses the actual filter sign convention. For an admitted finite object row, `A u >= demand_speed - gamma_obj h + rho`; consequently the reported residual is `A u + gamma_obj h - rho - demand_speed`. The geometry and motion terms have not been conflated. Removing the same scalar motion demand from every row preserves the minimizing row, which makes the subsequent witness selection valid.
- The motion-envelope witness is mathematically sound: with fixed original command and row geometry, the maximum of `n.u + gamma h - rho - n.v` over `||v|| <= B` is `n.u + gamma h - rho + B||n||`, attained at `v = -B n/||n||`. The implementation checks attainment against the actual original matrix, rather than only reusing an exported minimum.
- Runtime finite-support normals are radial unit vectors, including explicit degenerate-distance handling. All 3,813 saved object-row normals checked here have norms from 0.9999999999999998 to 1.0000000000000002. The formulas do not silently require an unverified normalization: the envelope retains the norm factor. Normal and command products, `gamma_obj*h`, `rho`, and motion terms all have velocity units; `h` and `r_safe` have length units.
- Public prose consistently limits the conclusion to the same original commands. It explicitly leaves nonnegative one-row bounds inconclusive, other pushing commands unresolved, the configured ball uncertified as true object motion, and the full closed-loop causal explanation unresolved. The README retains the task regression and default-disabled candidate.
- Provenance binds the two raw analyses, their predecessor, scripts, designs, runtime and protocol. The public snapshot binds an exact log prefix, so later progress does not invalidate the recorded 3,600-frame checkpoint. It is plainly labeled partial, not a full audit or gate result.

## Issues

### Critical (Must Fix)

None found.

### Important (Should Fix)

None found.

### Minor (Nice to Have)

None requiring a change in this publication increment.

## Evidence checked

### Complete changed-file review

Read the complete added analysis/publisher scripts, both designs, all changed prose and ledger additions, and every added JSON product. Parsed and compared every row and exported field in both CSVs. The 18 reviewed changed files are:

1. `README.md`
2. `docs/DATA_INDEX.md`
3. `docs/results/README.md`
4. `docs/results/post_support_contact_row_decomposition_v1/README.md`
5. `docs/results/post_support_contact_row_decomposition_v1/all_128_states.csv`
6. `docs/results/post_support_contact_row_decomposition_v1/all_17_positive_force_states.csv`
7. `docs/results/post_support_contact_row_decomposition_v1/analysis_checkpoint.json`
8. `docs/results/post_support_contact_row_decomposition_v1/provenance.json`
9. `docs/results/post_support_contact_row_decomposition_v1/publication.json`
10. `docs/results/post_support_contact_row_decomposition_v1/summary.json`
11. `docs/results/post_support_contact_row_decomposition_v1/verification.json`
12. `experiments/ccta2027/README.md`
13. `experiments/ccta2027/post_acquisition_finite_support_v1_physical_plan.md`
14. `experiments/ccta2027/post_support_contact_motion_envelope_v1_design.md`
15. `experiments/ccta2027/post_support_contact_row_decomposition_v1_design.md`
16. `scripts/bound_post_support_contact_motion.py`
17. `scripts/decompose_post_support_contact_rows.py`
18. `scripts/publish_post_support_contact_rows.py`

### Owning-source and preserved-evidence checks

- Read the requesting-code-review template, repository `AGENTS.md`, and Serena initial instructions before the coding review.
- Read the predecessor selection and replay/export logic in `scripts/probe_post_support_contact_compatibility.py`, including first/peak eligible event selection, actual force sign, pusher membership, and object-only `h` indexing.
- Read relevant implementation in `src/dbact/safety/filter.py`, `src/dbact/safety/post_support.py`, and `src/dbact/sensing/finite_support.py`: actual row sign, motion clipping, finite-support override, reachable RHS cap, radial normals and geometric margin construction.
- Read `scripts/research_io.py`, relevant frozen study identity logic, and `post_acquisition_support_study.check()`.
- Read the local one-time `verify_publication.py` without executing its writer. Read the stored local verification and health receipts. The health receipt reports PASS at the precommit check while HEAD was the base commit; this review does not relabel it as a newly executed health check at the final commit.
- Read the preserved eight physical task receipts and execution outcome referenced by the updated README. They distinguish successful execution of all eight episodes from task completion: controls 4/4, candidates 0/4, each candidate truncated at 600 seconds.

### Independent bounded verification performed

Executed a separate read-only calculation through `.\.venv\Scripts\python.exe -B -`; did not invoke any analysis/publisher main function or the exclusive publication-verifier writer.

- Verified outcome status and analysis SHA-256 for contact compatibility, decomposition and envelope, plus the predecessor hash chain and all runtime/protocol identities.
- Verified each decomposition/envelope script and design against its preserved snapshot and started/analysis identity, plus the publisher hash and all four public product hashes.
- Called the read-only frozen study `check()`: runtime, environment, protocol, all 22 frozen helpers, source snapshots and protected identities passed. Independently verified protected YAML SHA-256 `a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112`. Runtime identity remains `e6b7456f5f0a4d71ddd4b33159c058e215115aae35c01b6c5c9eae7e68f32767`.
- Reconstructed effective safety parameters from each of the four saved case configurations without stepping physics; all match the decomposition parameter records. Each has `r_safe=0.065`, `gamma_obj=8`, `rho=0.02`, `max_speed=0.35`, and `object_velocity_bound=0.2`.
- Verified the four raw per-condition results equal their entries in the predecessor aggregate analysis and that saved selections correspond to the scheduled first/peak events. Frames are circle 79/162, L 71/124, C 478/572, skew box 559/891; each event contains 16 distinct states.
- Verified identity sets, all admission/force/pusher flags and all guard reasons across raw predecessor, decomposition, envelope and public rows. All 128 keys are unique, and the 17-row table equals the exact positive-force subset.
- Independently recomputed all 3,813 admitted object rows, the capped and uncapped residual minima, deterministic limiting indices, all four additive terms, the minima after removing motion, row normals, original commands and selected geometric margins. Compared every exported scalar/flag to its source, including blank diagnostics on nonadmissions.
- Independently recomputed all 112 motion maxima and attaining vectors, including their norms, against the original matrices. The one-row exclusion flags and every published subgroup count agree. An additional check found no selected exclusion witness made acceptable by the existing reachable RHS cap on this cohort.
- Recomputed nonadmission reasons: outside range 4; isolated gap exceeds cap 4; no current map motion 7; no current own returns 1. None of the 16 nonadmissions is a positive-force state.
- Recomputed zero-motion conflict counts: 27/112 admitted; 17/17 positive-force; 11/11 positive-force pushers. Positive-force residual range is -0.4873293826661949 to -0.06038075628787597, matching the rounded prose.
- Recomputed motion-ball exclusions: 12/112 admitted; 9/17 positive-force; 8/11 positive-force pushers. Positive-force exclusions by condition are circle 3/5, L 2/4, C 1/3, skew box 3/5. Selected negative-h counts are 20/112, 11/17 and 8/11, respectively.
- Verified checkpoint started-receipt hash and identity, read exactly its 1,508-byte log prefix and verified its hash, and verified its last recorded progress is the first L candidate at 3,600 frames. The local and public verification receipts agree. The growing log was neither rewritten nor required to remain unchanged beyond that prefix.
- `git diff --check` over the requested range passed. HEAD was the requested final commit and the working tree was clean before this report was written. The commit diff contains no runtime/configuration/test or frozen-helper change.

## Recommendations

Publish this bounded diagnostic checkpoint with the existing inference limits and default-off status. Preserve the active audit attempt; later complete-audit conclusions should have their own evidence and dated update rather than replacing this snapshot's historical meaning.

## Declined to judge

- Completion, safety, detailed pair/bridge/support results and formal gates of the running full candidate audit: its live analysis remains outside this completed publication increment; partial progress cannot establish completion.
- Future state of the live process after the saved checkpoint: the publication intentionally makes a timestamped claim, and this review checked that exact preserved prefix rather than promoting subsequent progress.
- Independent replay of the predecessor's 1,749 control frames or any new physical episode: explicitly excluded from this bounded review; raw stored matrix identities and all relevant scalars were checked instead.
- Global nonexistence of an alternative safe pushing command: fixed-command witnesses cannot establish this, and the publication explicitly avoids the inference.
- Joint feasibility or rescue of states with nonnegative single-row bounds: the converse does not follow from this witness; no new optimization is part of the increment.
- Certification that actual object translation, rotation-induced point velocity, or measurement error lies inside the configured motion ball: this is an empirical/configured assumption, not established by these diagnostics.
- Separation of half-gap, scan noise, normal error and true contact distance inside the geometric term: requires packet/contact-level work outside the declared two diagnostics.
- Complete closed-loop causality of the four failed candidate trajectories or population-level/generalization claims: post-hoc states on repeatedly used development trajectories do not establish either.
- Adoption, redesign, tuning or additional experiments for the default-disabled candidate: this increment publishes completed evidence and preserves the unresolved work, with no such change authorized within this review.
- The wider acquisition/reference-bias, C-shape activation, deployment-contribution and paper-completion goals listed in the README: these are explicitly unresolved research tasks outside this diff.
- Re-audit of unchanged historical publications, their full numerical conclusions, or the prior f1 publication review: outside this requested increment; only predecessor evidence needed to validate the new claims was inspected.
- Re-running the unchanged full test suite, a new full repository-health run, or independently querying remote CI/GitHub push state: excluded from this bounded scientific/publication review; the stored health receipt was read and final-range whitespace checks were performed. Historical CI wording and future push execution are not certified by this review.
- General-purpose support for arbitrary future cohorts or reusable controller safety guarantees from these fixed-path research scripts: they deliberately assert the frozen cohort and create fresh output directories; this review assesses the declared dataset and its publication.

## Assessment

**Ready to publish/merge? Yes.**

The complete increment matches the declared zero-physics diagnostic designs, and independent raw-matrix recomputation and provenance checks support every new numerical claim. No publication-blocking issue was found; readiness is limited to this evidence checkpoint and does not imply candidate adoption, complete audit/gate validation, or completion of the research goal.
