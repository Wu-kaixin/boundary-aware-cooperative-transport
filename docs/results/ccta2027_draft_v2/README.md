# CCTA 2027 draft v2: results and reproduction

Recorded 21 October 2026. The agreed 169-case matrix is complete with zero runtime
errors and a passed implementation audit. Scientific failures are retained:
boundary CVT completes only 1/18 robustness tasks and the Pymunk task fails.
See [the interpretation](../../CCTA2027_EXPERIMENTS.md) before using the figures.

## Start here

This bundle belongs to `paper/boundary-aware-safe-deployment`. Model execution,
final analysis and historical v1 failure are separately versioned. For the
repository workflow use [the experiment entry point](../../../experiments/ccta2027/README.md);
for all batches use [the result index](../README.md).

## Files and provenance

- `case_results.json`: all 169 compact case receipts and original run identity.
- `protocol_index.json`: all planned inputs and expected initial-state hashes.
- `episodes.csv`, `pairs.csv`, `summary.json`: final analysis, separated by population.
  `pairs.csv` lists CVT/nearest pairs; `summary.json` also includes spacing comparisons.
- `failure_cases.csv`: 113 task failures or observed static violations; these are
  measured outcomes, not exceptions. No failure is removed from its denominator.
- `acceptance.json`: coverage, source/product identity, paired task/initial-state
  checks and the declared static safety checks. PASS does not mean every task succeeds.
- `serial_parallel.json`: two complete serial/parallel reproductions, excluded from
  scientific denominators. `engine_repair_regression.json` retains the comparison
  of all 168 non-Pymunk cases across revisions.
- `test_results.json`: 616 passed, two skipped in the release regression suite.
- `execution_snapshot.zip`: exact raw-byte model sources, frozen expanded protocol,
  design and freeze manifest. Separate `analysis_source/src` contains final analysis
  and figure modules; it is not substituted into the frozen simulation.
- `failed_v1_snapshot.zip`, `v1_*`: the original failed campaign and its recorder
  exception, preserved independently of the repaired v2 campaign.
- `snapshot_integrity.json`: source/archive identities; `artifact_integrity.json`
  hashes all curated files other than itself. These establish byte identity, not
  independent certification of the reported science.

Full per-frame replay arrays, diagnostics, raw output and logs remain locally in
`runs/ccta2027/draft_v2/`; this compact Git bundle does not contain all of them.
Case receipts retain hashes of those larger products. The archive permits rerunning
the matrix; the compact summaries permit reanalysis without rerunning physics.
Raw archives preserve source identity across Git line-ending conversion.

## Reproduce

Use the Python environment described by `environment.yml` in `execution_source`
and exact recorded package versions from the manifest. Extract the archive to a
new directory. Set `PYTHONPATH` to its `execution_source/src` and `DBACT_ROOT` to
its `execution_source`; use absolute paths and a new output directory:

```sh
python -m dbact_sim study --protocol /absolute/archive/frozen_protocol.yaml --split all --workers 16 --out /absolute/new_run
python -m dbact_sim campaign audit --protocol /absolute/archive/frozen_protocol.yaml --run /absolute/new_run --out /absolute/acceptance.json
```

Switch `PYTHONPATH` to `analysis_source/src` for final analysis. For compact-only
reanalysis, copy `case_results.json` to `summary.json` in a fresh input directory.
The figure module reads that summary. The evidence module reads individual case
receipts; materialize them from the same compact bundle first:

```python
import json
from pathlib import Path
folder = Path("/absolute/compact_input")
for result in json.loads((folder / "summary.json").read_text())["results"]:
    case = folder / result["case_id"]
    case.mkdir(parents=True, exist_ok=True)
    (case / "case_result.json").write_text(json.dumps(result), encoding="utf-8")
```

```sh
python -m dbact_sim.experiments.evidence --batch revision=/absolute/compact_input --by-split --out /absolute/reanalysis
python -m dbact_sim.experiments.report --run /absolute/compact_input --out /absolute/figures
```

Contact time-series plots require the original diagnostic files; summary-only
reanalysis cannot recreate those traces. The four curated paper figures below
require only compact results. PNG files support inspection; PDFs are vector exports.

## Figure captions

1. **Static deployment:** all nine shape/seed cases per arm, including the eight
   original-local violations; 65 mm clearance threshold and cost ratios. This is
   fixed-polygon, full-hold path evidence, not a moving-boundary safety theorem.
2. **Task completion:** all three methods with separate development, generalization
   and robustness denominators. Wilson intervals appear only for the distinct-seed
   generalization population. Reused-seed populations are descriptive.
3. **Paired preparation:** CVT minus each control, with complete-pair counts and
   censoring retained. Only sufficient seed-cluster populations receive exploratory
   bootstrap intervals. Negative means earlier controller readiness, not task success.
4. **Target ablations:** readiness and endpoint-progress error differences use
   separate units and axes; one seed per shape, with no inferential claim. The
   zero-gain exploration removal is intentionally identical to the baseline.
