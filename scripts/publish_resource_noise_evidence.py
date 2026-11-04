"""Curate immutable validation evidence into a small, reviewable result index."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import shutil
import pstats
import resource_noise_validation as study
from dbact_sim.experiments.deployment import dump


def main():
    study.guard(); study.verify()
    root = study.ROOT
    out = root/'docs/results/resource_noise_validation_v1'
    if out.exists():
        raise FileExistsError(out)
    data_file = study.BASE/'analysis_v1/analysis.json'
    data = study.read(data_file)
    assert len(data['episodes']) == 60
    assert data['analysis_sha256'] == study.sha(root/'scripts/analyze_resource_noise_validation.py')
    rows = data['episodes']
    out.mkdir(parents=True)
    shutil.copyfile(study.BASE/'analysis_v1/all_60_episodes.csv', out/'all_60_episodes.csv')
    dump(out/'analysis.json', {k:v for k,v in data.items() if k!='series'})
    paired = []
    for row in rows:
        if row['regime']=='reference':
            continue
        ref = next(r for r in rows if r['regime']=='reference' and r['shape']==row['shape'] and r['method']==row['method'])
        paired.append(dict(case_id=row['case_id'], reference_id=ref['case_id'],
            safe_completion_delta=int(row['safe_completed'])-int(ref['safe_completed']),
            spatial_completion_delta=int(row['completed'])-int(ref['completed']),
            endpoint_error_delta_m=row['endpoint_error_m']-ref['endpoint_error_m'],
            actual_duration_delta_s=row['seconds']-ref['seconds'],
            ready_coverage_delta=row['ready_coverage']-ref['ready_coverage'],
            mean_robot_path_delta_m=row['mean_robot_path_m']-ref['mean_robot_path_m']))
    with (out/'paired_differences.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(paired[0])); writer.writeheader(); writer.writerows(paired)
    for ext in ('pdf','png','svg'):
        shutil.copyfile(study.BASE/('analysis_v1/resource_noise_outcomes.'+ext),out/('resource_noise_outcomes.'+ext))
    for file in ['freeze/receipt.json','execution_outcome.json','monitor/science-measurements.json','benchmark.json']:
        shutil.copyfile(study.BASE/file,out/Path(file).name)
    stats = pstats.Stats(str(study.BASE/'engineering/profile.pstats'))
    selected = []
    wanted = ['sense_view','compute','solve_min_norm_2d','step','compute_contacts','register','update','dump']
    for key,val in sorted(stats.stats.items(),key=lambda kv:kv[1][3],reverse=True):
        if 'boundary-aware-cooperative-transport' in key[0] and key[2] in wanted:
            selected.append(dict(file=key[0].replace(str(root),'').lstrip('\\/'),line=key[1],function=key[2],
                calls=val[1],self_s=val[2],cumulative_s=val[3]))
    dump(out/'profile_summary.json',dict(total_s=stats.total_tt,entries=selected,
        note='One frozen 100-step 24-robot C-away prefix. Nested cumulative times overlap; no full-horizon speed prediction.'))
    table = []
    for group in study.REGIMES:
        cells=[]
        for method in study.METHODS:
            x=data['by_regime'][group][method]
            cells.append(f"{x['completed']}/5 | {x['safe_completed']}/5 | {x['g500']}/5")
        table.append('| '+group+' | '+' | '.join(cells)+' |')
    readme = '''# Current-source resource and range-noise validation

Completed 2026-11-04. This is a frozen, one-seed sensitivity evaluation, with all 60 planned cases retained. It is not a reliability estimate for arbitrary objects.

[All 60 cases](all_60_episodes.csv) · [45 paired perturbation differences](paired_differences.csv) · [Compact analysis](analysis.json) · [Outcome figure](resource_noise_outcomes.png) · [Protocol](../../../experiments/ccta2027/resource_noise_validation_v1.json) · [Prospective design](../../../experiments/ccta2027/resource_noise_validation_v1_design.md)

## Main result

Each method/regime has five tasks: circle, ellipse-perpendicular, rectangle-parallel, L-reference and C-away. `reference` is 16 robots and range noise sigma 0.01 m; `noise002` changes only sigma to 0.02 m. The 8- and 16-robot layouts are prefixes of the common 24-robot layout. Three methods share initial states, physics, sensing, downstream control, four-pusher quorum, and stopping rules within each block. None was retuned after this validation.

| Regime | CVT spatial | CVT spatial + hard safety | CVT G500 | Spacing spatial | Spacing spatial + hard safety | Spacing G500 | Nearest spatial | Nearest spatial + hard safety | Nearest G500 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
'''+ '\n'.join(table)+'''

All 60 cases reach CONTACT_READY and actual physical contact. Spatial completion is 58/60; 56/60 also avoid the existing hard safety failures. The three methods do not have a common ranking across every resource and G500 outcome. Counts across regimes summarize the designed cases only; the statistical randomization unit is one new shared seed, 2026110401. No frame-level tests or confidence intervals are used.

## Every incomplete or unsafe case

| Case | Observed outcome | Interpretation boundary |
|---|---|---|
| C-away, 8 robots, spacing | 600 s watchdog in BRAKE; endpoint 0.110190 m; no HOLD | Inside positional tolerance but does not satisfy the frozen settling contract. Final speed alone does not establish the required HOLD dwell. |
| C-away, 8 robots, nearest | Off-target HOLD at 48.65 s; endpoint 0.823656 m | Stop and completion are separate events. |
| L-reference, 24 robots, CVT | Spatial completion at 14.95 s, endpoint 0.037505 m; penetration 0.120428 m exceeds 0.0975 m budget | Excluded from spatial-plus-hard-safety completion. |
| Ellipse-perpendicular, 24 robots, spacing | Spatial completion at 12.65 s, endpoint 0.026751 m; penetration 0.115719 m exceeds 0.1002 m budget | Excluded from spatial-plus-hard-safety completion. |

C-away/8-robot CVT completes only at 571.55 s, endpoint 0.088994 m. Its completion advantage over those two matched cases is accompanied by a long duration, and does not establish superiority over a population. More robots do not monotonically improve the fixed controller's safety. All 15 doubled-noise tasks spatially complete without a hard safety failure, but this single noise increment does not validate general robustness.

G500 additionally rejects specified missing events, coverage, and scaled barriers. It is not interchangeable with the primary spatial or hard-safety outcomes. Hard safety uses the existing evaluator, including its configured penetration budget and discrete overshoot allowance; passing it is not a continuous-time guarantee. The CSV retains every G500 reason and each event, endpoint, path, direction, yaw, contact, force, navigation error and safety measurement. `outcome=COMPLETED` denotes spatial completion; consult the separate `safe_completed` and `hard_safety` columns.

Paired duration differences include actual termination for failures; they are not differences in successful completion time. Readiness coverage is measured at each method's own readiness event and is not the common-prefix estimator from the separate deployment-contribution study. Navigation errors compare recorded vector navigation against pre-physics truth; the replay's auxiliary scalar estimate is not substituted for navigation input.

## Execution and identity

Execution: 60 terminal receipts, zero execution errors, source integrity PASS; 779.104 s including startup, peak sampled process-tree RSS 6.551 GiB. Runtime source and protected configuration were unchanged during and after the batch. All case product hashes, initial-state/config identities, endpoint errors, full step-log counts and independent evaluator outcomes were checked by `scripts/analyze_resource_noise_validation.py`.

- Source: `e6b7456f5f0a4d71ddd4b33159c058e215115aae35c01b6c5c9eae7e68f32767`.
- Protocol: `caa29e613ceb6bfd56a97096e6e9d8651be4de0782f64a4643943c40d0a8fa4b`.
- Protected YAML: `a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112`.
- Local raw records: `runs/research/2026-11-04-resource-noise-validation-v1/run/`; source/environment/helper snapshots in the sibling `freeze/` directory. No old result is represented as a current-source rerun.

The fixed eight-case, 100-step engineering benchmark took 92.374/34.126/22.108 s with 1/4/8 workers. All 25 replay arrays and full scientific products matched. A receipt-checked resume skipped all eight finished cases without new physics. The fastest measured tier, eight workers with one numerical thread each, was used. Engineering prefixes and the single profiling prefix are excluded from the 60 scientific episodes. Profiling shows controller/map work dominates the short prefix; nested cumulative timings are in [profile_summary.json](profile_summary.json). RTX A400 4 GiB was verified; no unmeasured GPU speedup is claimed. Matplotlib/FFmpeg rendering runs after simulation.

## Reproduction and media

Use only the repository root environment. Existing frozen helpers refuse to overwrite or silently restart original cases. Identity verification:

```powershell
.\\.venv\\Scripts\\python.exe -B scripts/resource_noise_validation.py verify
```

To repeat science, use a fresh run output with the frozen protocol and matching source/environment:

```powershell
$env:OMP_NUM_THREADS='1'
$env:MKL_NUM_THREADS='1'
$env:OPENBLAS_NUM_THREADS='1'
$env:NUMEXPR_NUM_THREADS='1'
$env:DBACT_CVT_WORKERS='1'
.\\.venv\\Scripts\\python.exe -m dbact_sim study --protocol experiments/ccta2027/resource_noise_validation_v1.json --split validation --workers 8 --out runs/research/resource-noise-reproduction-NEW
```

Choose an unused output name. Keep the original receipts unchanged. The analyzer and media helpers deliberately target the original frozen study; adaptation to a repeated output must preserve that repeat's identity rather than overwrite these records.

Preselected media: `runs/research/2026-11-04-resource-noise-validation-v1/media_v1/` contains five-shape overview MP4/GIF, rectangle and C-away three-method comparisons, and stage figures. Their measured trajectories are synchronized by physical time, at 0.25 s sampling and 10 fps (2.5x playback). Ended episodes hold their actual final state. Agent 00 fused maps are labeled; no raw laser rays are fabricated.

Post hoc diagnostics: sibling `diagnostics_v1/` contains reference vector-navigation curves and the C-away/8-robot failure comparison. This additional MP4 retains the whole 600 s measured horizon at 1 s sampling, 10 fps (10x playback); it includes the CVT completion, spacing watchdog and nearest off-target stop. No physical episode was rerun for rendering. Complete decoder/frame-count checks and replay hashes accompany both media packages.
'''
    (out/'README.md').write_text(readme,encoding='utf-8')
    dump(out/'curation.json',dict(analysis_sha256=study.sha(data_file),
        generator_sha256=study.sha(Path(__file__)),products={p.name:study.sha(p) for p in out.iterdir() if p.is_file()}))
    print(json.dumps(dict(output=str(out),spatial=sum(r['completed'] for r in rows),
        spatial_without_hard_failure=sum(r['safe_completed'] for r in rows))))


if __name__=='__main__':
    main()
