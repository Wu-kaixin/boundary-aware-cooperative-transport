# CCTA 2027 experiments

Run after `python -m pip install -e ".[dev,qp,sim,analysis,publication]"`.
The [conference snapshot](conference.yaml) records official requirements and unknowns.

```sh
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --dry-run
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/ccta2027/smoke_parallel
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 1 --out runs/ccta2027/smoke_serial
python -m dbact_sim study --protocol experiments/ccta2027/smoke.yaml --split smoke --workers 2 --out runs/ccta2027/smoke_parallel --resume
python -m dbact_sim evidence --batch smoke=runs/ccta2027/smoke_parallel --out runs/ccta2027/smoke_evidence
```

Five cases cover oracle/local static execution and CVT/nearest/spacing transport. Four or twelve
frames do not evaluate scientific outcomes. Use a fresh directory for changed code, protocol or
environment. `--resume` verifies every identity and listed product before writing. Failures stay
in the denominator. Execution/process errors return nonzero; resume retains recorded errors.

```text
manifest.json                   hashes, environment, Git state and split
resolved_protocol.json          full selected case configurations
source_snapshot/                source and environment files
resources.csv                   system CPU and parent memory samples
<case_id>/resolved_case.json     exact inputs
<case_id>/case_result.json       identity, product hashes and measured result
<case_id>/...                    replay and diagnostics
summary.json                    all recorded cases, denominator and errors
```

Historical complete-case protocols under `docs/paper_simulation_protocol.*.yaml` stay unchanged.
Inspect the 36 archived test cases without running them:

```sh
python -m dbact_sim study --protocol docs/paper_simulation_protocol.test_frozen_v1.yaml --split test --dry-run
```

This checks case resolution. Execution enforces the original source freeze and rejects the
refactored hash. Use archived sources/environment for historical reproduction, or define a new
protocol and freeze for new research. v1 receipts cannot resume as v2 receipts. Source identity
changes after relocation even when control behavior agrees.

Cases require `case_id`, `split`, `gate`, `shape`, `seed`, `variant` and positive `frames`.
Gate 5 accepts `oracle`/`local`; Gate 6 uses the explicit deployment registry. Supply complete
`config`, repository-relative `config_ref`, or omit both for the gate/shape factory. The smoke
file illustrates the last form. Freeze expanded configurations for full studies. `DBACT_ROOT`
locates repository scenarios when an installed command runs outside the checkout.

Historical definitions and limits remain in [paper_simulation_protocol.md](../../docs/paper_simulation_protocol.md).
Smoke success does not change their verdicts.

## Complete draft experiment matrix

The agreed 169-episode scheme is in [revision.yaml](revision.yaml); definitions and
repairs are described in [CCTA2027_EXPERIMENTS.md](../../docs/CCTA2027_EXPERIMENTS.md).

```sh
python -m dbact_sim campaign freeze --plan experiments/ccta2027/revision.yaml --out runs/ccta2027/next_revision/frozen_protocol.yaml
```

Run `study --split all` with `PYTHONPATH` pointing to the freeze directory's
`execution_source/src` and `DBACT_ROOT` to `execution_source`. Pass absolute protocol
and output paths. This isolates workers from ongoing edits. The recorded case split
remains intact, so populations can be reported separately:

```sh
python -m dbact_sim campaign audit --protocol runs/ccta2027/draft_v2/frozen_protocol.yaml --run runs/ccta2027/draft_v2/run --out runs/ccta2027/draft_v2/acceptance.json
python -m dbact_sim evidence --batch revision=runs/ccta2027/draft_v2/run --by-split --out runs/ccta2027/draft_v2/evidence
python -m dbact_sim.experiments.report --run runs/ccta2027/draft_v2/run --out docs/results/ccta2027_draft_v2/figures
```

A passed audit establishes the declared implementation/coverage checks. It does
not turn task failures into successes or establish unsupported scientific claims.
The completed v2 results, raw source archives and reproduction instructions are in
[the audited result bundle](../../docs/results/ccta2027_draft_v2/README.md).
Use a new directory for future freezes; never overwrite a recorded campaign.
