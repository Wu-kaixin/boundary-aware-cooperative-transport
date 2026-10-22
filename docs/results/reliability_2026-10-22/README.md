# Development reliability revision, 22 October 2026

Read [the interpretation](../../RELIABILITY_REVISION_2026-10-22.md) before using
these development diagnostics. Frozen v2 results are unchanged.

## Files

- `v2_failure_classification.json`: all 140 transport receipts, including failures
  and censored events; no frame traces available in the compact v2 input.
- `rejected_p_only_*`: the first combined controller candidate, 3/9 development
  completion versus 5/9 in the baseline.
- `rejected_own_only_*`: low-speed brake integral restored, but fresh one-hop
  feedback still rejected; also 3/9.
- `rejected_relay_*`: one-hop velocity feedback restored; 4/9. This candidate
  remains disabled by default too.
- `rejected_sources.zip`: raw-byte source snapshots, expanded protocols and
  environment/source manifests for all three rejected candidates. The common
  archived baseline remains in the v2 `execution_snapshot.zip`.
- `default_*`: final default controller, all 11 pairs valid, 5/9 development
  completions. Penalty and Pymunk endpoint errors remain 0.111 and 0.378 m;
  the Pymunk task still fails. `default_source.zip` retains the executed source,
  protocol and verified-baseline reuse receipt.
- `test_results.json`: final default-controller regression results, 629 passed
  and two skipped. Unit tests establish control contracts, not task superiority.
  `test_report.xml` is the complete corresponding JUnit report.
- `validation_freeze.zip`: prospective 832-case protocol, source snapshot and
  design. `validation_freeze_manifest.json` records source hashes and successful
  initialization preflight. **These episodes have not been run.**
- `artifact_integrity.json`: exact raw-byte hashes of this curated bundle,
  excluding the integrity manifest itself. Git line-ending conversion is disabled
  for this evidence directory.

Each comparison retains all 11 pairs: nine CVT development cases and two engine
references. The repeated penalty condition checks deterministic duplication;
it is not a tenth independent development task. Each arm runs the exact same
recorded initial state and task under the same Python environment. All regenerated
baseline endpoint errors equal the original v2 values exactly.

`baseline_classification` and `revision_classification` include frame-derived
progress bias, contact absence and phase-response observations. These describe
signals and do not establish a causal mechanism from correlation. Full raw
per-frame products remain in `runs/reliability/2026-10-22*`; they are not embedded
in this compact bundle. Rejected source archives permit reconstruction.

## Reproduce

From a repository environment with the declared dependencies:

```powershell
.\.venv\Scripts\python.exe scripts/run_reliability_revision.py --out runs/reliability/new --workers 4
```

For repeated development comparisons, `--baseline-run` can reuse a baseline
directory only after its source/protocol/environment identities and every case
product hash pass verification. It never rewrites that baseline.

To execute the prospective design, extract `validation_freeze.zip` into a fresh
directory, point `PYTHONPATH` to its `execution_source/src` and `DBACT_ROOT` to
`execution_source`, then run the archived protocol with the chosen reproduction
environment. Use an independent output directory and preserve all failures:

```powershell
python -m dbact_sim study --protocol <absolute-path>/frozen_protocol.yaml --split all --workers 4 --out <absolute-new-run>
python -m dbact_sim campaign audit --protocol <absolute-path>/frozen_protocol.yaml --run <absolute-new-run> --out <absolute-audit.json>
```

The prospective design has 12 independent seed clusters in each principal
population and a separate nonzero-gain ablation population. Its repeated shapes
and perturbations must still be clustered by seed. It does not establish new
geometry/occlusion coverage or moving-object safety.
