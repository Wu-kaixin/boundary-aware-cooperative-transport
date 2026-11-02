"""Freeze/run/analyse one finite, non-adaptive development parameter screen."""
import argparse
import copy
import gc
import json
import os
from pathlib import Path
import sys
import benchmark_workstation as monitor  # Set worker/BLAS limits before NumPy.
from prepare_transport_tuning import ROOT, MISSION, AUDIT, OUT, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import source_hashes, environment_versions, verify_source_snapshot

PRODUCTS=ROOT/'artifacts/research'/MISSION
SETTINGS={
    'reference': {},
    'slower_reference': {'transport_reference_speed': .035},
    'lower_effort': {'transport_effort_max': .18},
    'slower_deployment': {'kp_cage': .6},
    'tighter_registration': {'registration_gate': .08},
    'wider_motion_prior': {'max_object_speed': .45},
}
SCREEN=[('near','circle_boundary_cvt'),('near','ellipse_parallel_boundary_cvt'),
        ('near','rectangle_parallel_boundary_cvt'),('near','c_open_away_boundary_cvt'),
        ('near','u_tray_s0_r90_boundary_cvt'),('near','t_load_s0_r90_boundary_cvt'),
        ('far','skew_box_s0_r0_pos0_unknown_location'),('far','c_open_away_pos1_initial_location_hint')]
TRANSFER=[('near','ellipse_perpendicular_boundary_cvt'),('near','skew_box_s1_r90_boundary_cvt'),
          ('far','c_open_away_pos0_unknown_location'),('far','skew_box_s0_r0_pos1_unknown_location')]
METHODS=['boundary_cvt','local_spacing','nearest_observed_target']

def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))

def assert_environment():
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'

def case_from(source,setting,method):
    c=copy.deepcopy(source)
    condition=c['stratum']+'_'+c['condition_id']
    c.update(condition_id=condition,case_id=condition+'_'+method+'_'+setting,
             arm=setting,variant=method,frames=12000,split='development')
    c['config']['controller'].update(SETTINGS[setting])
    c['factors'].update(cohort='parameter_development',origin_case=source['case_id'],
                        parameter_setting=setting,parameter_changes=SETTINGS[setting])
    return c

def write_protocol(stage,cases,design):
    path=ROOT/f'experiments/ccta2027/transport_tuning_v1_{stage}.json'
    if path.exists():raise RuntimeError('Frozen protocol already exists')
    plan=dict(version='transport-tuning-v1-'+stage,status='FROZEN_BEFORE_EXECUTION',cases=cases,
        run_dir=f'runs/research/{MISSION}/{stage}',frozen_source_sha256=encode(source_hashes()),
        budget=dict(episodes=len(cases),workers=8,internal_threads=1,physical_seconds=600,
                    frames=12000,wall_seconds=14400,additional_episodes=0,retuning=False),
        environment=environment_versions(),design=design,
        primary='Original task completion AND original hard safety; G500 retained separately',
        completion='HOLD + speed <=.005 for40frames and Euclidean endpoint<=.12m, unchanged',
        information='Own/one-hop observations plus existing shared team phase supervisor; no new truth access')
    dump(path,plan)
    load_protocol(path,'development',ROOT)
    return dict(protocol=path.relative_to(ROOT).as_posix(),sha256=sha(path),cases=len(cases))

def freeze(dry_run=False):
    assert_environment()
    sources={s:read(ROOT/f'experiments/ccta2027/cargo_{s}_v1.json') for s in ('near','far')}
    def obtain(pairs):
        return [next(c for c in sources[s]['cases'] if c['case_id']==name) for s,name in pairs]
    origin=obtain(SCREEN); transfer=obtain(TRANSFER)
    cases=[case_from(c,arm,'boundary_cvt') for c in origin for arm in SETTINGS]
    assert len(cases)==48
    # Freeze all possible transfer configurations before the first screen outcome.
    template={arm:[case_from(c,a,m) for c in transfer for m in METHODS for a in ('reference',arm)]
              for arm in SETTINGS if arm!='reference'}
    for c in cases:
        original=next(o for o in origin if o['case_id']==c['factors']['origin_case'])
        difference={k for k in set(c['config']['controller'])|set(original['config']['controller'])
                    if c['config']['controller'].get(k)!=original['config']['controller'].get(k)}
        assert difference==set(SETTINGS[c['arm']])
        assert c['config']['g500']==original['config']['g500']
        assert c['config']['evaluation']==original['config']['evaluation']
    design=dict(settings=SETTINGS,screen=SCREEN,transfer=TRANSFER,transfer_methods=METHODS,
        selection='Require no hard-safety cases or execution errors. Qualify if safe completions increase >=1 with G500 not decreasing, OR safe completions unchanged and G500 increases >=2. Rank qualified settings by safe completions, G500, lower mean endpoint error; final tie uses declared setting order.',
        stop='No qualifying setting: stop after48. Otherwise exactly24 transfer episodes, no parameter recombination or retuning. Reject global adoption if transfer loses safe completion, adds hard-safety failure, or fails to improve either completion or G500 in total.',
        scientific_scope='Previously examined development conditions, one seed. Transfer is not independent validation or a population probability.',
        constraints='No changes to physics, observations, robot resources, successful arrival, settling, safety constraints or G500 limits. Motion prior is estimator acceptance only; safety velocity bound unchanged.',
        history={f'experiments/ccta2027/cargo_{s}_v1.json':sha(ROOT/f'experiments/ccta2027/cargo_{s}_v1.json') for s in sources})
    if dry_run:
        print(json.dumps(dict(screen_cases=len(cases),possible_transfer_cases=24,design=design)));return
    for stage in ('baseline','repair','offline-audit'):
        assert (OUT/f'{stage}.exit.txt').read_text(encoding='utf-8-sig').strip()=='0',stage
    dest=AUDIT/'design.json'
    if dest.exists():raise RuntimeError('Do not refreeze a design')
    dump(dest,dict(design=design,transfer_templates=template,source_sha256=source_hashes(),
                   script_sha256=sha(Path(__file__))))
    receipt=write_protocol('screen',cases,design)
    dump(AUDIT/'freeze_receipt.json',dict(screen=receipt))
    state=read(AUDIT/'mission_state.json');state.update(stage='SCREEN_FROZEN',protocols=dict(screen=receipt))
    dump(AUDIT/'mission_state.json',state)
    print(json.dumps(receipt))

def run(stage,dry_run=False,resume=False):
    assert_environment()
    receipt=read(AUDIT/'freeze_receipt.json')[stage]
    path=ROOT/receipt['protocol'];plan=read(path)
    assert sha(path)==receipt['sha256'] and encode(source_hashes())==plan['frozen_source_sha256']
    cmd=[sys.executable,'-m','dbact_sim','study','--protocol',str(path),'--split','development',
         '--workers','8','--out',str(ROOT/plan['run_dir'])]
    if resume:cmd+=['--resume']
    if dry_run:print(json.dumps(dict(command=cmd,budget=plan['budget'])));return
    if not resume and (ROOT/plan['run_dir']).exists():raise RuntimeError('Output exists; inspect before resume')
    monitor.OUT=OUT;monitor.AUDIT=AUDIT
    state=read(AUDIT/'mission_state.json');state.update(stage='RUNNING_'+stage.upper())
    dump(AUDIT/'mission_state.json',state)
    result=monitor.run_command(stage+('-resume' if resume else ''),cmd,14400,os.environ.copy())
    (OUT/f'{stage}.exit.txt').write_text(str(result['exit_code'])+'\n')
    state.update(stage=stage.upper()+'_FINISHED',last_execution=result['status'])
    dump(AUDIT/'mission_state.json',state)
    if result['exit_code']!=0:raise SystemExit(result['exit_code'])

def analyze(stage):
    from analyze_cargo_discovery import one_case
    from analyze_feedback_components import csv_rows
    from analyze_dynamic_contact_study import replay_equal,old_fields_equal
    path=ROOT/read(AUDIT/'freeze_receipt.json')[stage]['protocol'];plan=read(path)
    batch=ROOT/plan['run_dir'];manifest=read(batch/'manifest.json');batch_summary=read(batch/'summary.json')
    assert batch_summary['source_integrity']=='PASS'
    assert batch_summary['completed_denominator']==len(plan['cases'])
    verify_source_snapshot(batch,manifest['source_files'])
    assert encode(manifest['source_files'])==plan['frozen_source_sha256']
    dest=PRODUCTS/stage
    if dest.exists():raise RuntimeError('Analysis exists; do not overwrite')
    dest.mkdir()
    rows=[];series={};exact={};inputs={}
    for case in plan['cases']:
        folder=batch/case['case_id'];print('Audit '+case['case_id'],flush=True)
        row,data,_=one_case(case,folder,manifest)
        rows.append(row);series[case['case_id']]=data
        inputs[case['case_id']]=sha(folder/'case_result.json')
        if stage=='screen' and case['arm']=='reference':
            old=ROOT/'runs/research/2026-11-02-cargo-discovery-v1'/case['stratum']/case['factors']['origin_case']
            equal,arrays=replay_equal(old/'replay.npz',folder/'replay.npz')
            fields=old_fields_equal(read(old/'paired_metrics.json')['steps'],read(folder/'paired_metrics.json')['steps'])
            exact[case['condition_id']]=dict(replay_exact=equal,fields_exact=fields,arrays=arrays)
            assert equal and fields,case['case_id']
        gc.collect()
    def total(group):
        return dict(cases=len(group),safe_completed=sum(r['safe_completed'] for r in group),
            completed=sum(r['completed'] for r in group),g500=sum(r['g500'] for r in group),
            errors=sum(r['execution_status']!='COMPLETED' for r in group),
            hard_safety_cases=sum(bool(r.get('hard_safety','EXECUTION_ERROR')) for r in group),
            barrier_scalings=sum(r.get('barrier_scalings',0) for r in group),
            mean_endpoint_error_m=sum(r.get('endpoint_error_m',1e6) for r in group)/len(group))
    totals={arm:total([r for r in rows if r['arm']==arm]) for arm in dict.fromkeys(r['arm'] for r in rows)}
    selected=None
    if stage=='screen':
        baseline=totals['reference'];eligible=[]
        for arm in SETTINGS:
            if arm=='reference':continue
            a=totals[arm]
            qualifies=(not a['errors'] and not a['hard_safety_cases'] and
                ((a['safe_completed']>=baseline['safe_completed']+1 and a['g500']>=baseline['g500']) or
                 (a['safe_completed']==baseline['safe_completed'] and a['g500']>=baseline['g500']+2)))
            if qualifies:eligible.append(arm)
        if eligible:
            selected=max(eligible,key=lambda a:(totals[a]['safe_completed'],totals[a]['g500'],-totals[a]['mean_endpoint_error_m']))
    by_method={m:{a:total([r for r in rows if r['method']==m and r['arm']==a]) for a in totals}
               for m in dict.fromkeys(r['method'] for r in rows)}
    result=dict(stage=stage,totals=totals,by_method=by_method,selected=selected,episodes=rows,
        exact_reference_reproduction=exact,case_receipts_sha256=inputs,source_sha256=manifest['source_files'],
        protocol_sha256=sha(path),analysis_script_sha256=sha(Path(__file__)),
        scope='One reused development seed, fixed conditions; all starts count; no independent reliability estimate.')
    dump(dest/'analysis.json',result);dump(dest/'plot_data.json',series)
    csv_rows(dest/'all_episodes.csv',rows)
    if stage=='screen':
        state=read(AUDIT/'mission_state.json');state.update(stage='SCREEN_ANALYZED',selected=selected)
        if selected is not None:
            frozen=read(AUDIT/'design.json')
            assert frozen['source_sha256']==source_hashes()
            receipt=write_protocol('transfer',frozen['transfer_templates'][selected],frozen['design'])
            receipts=read(AUDIT/'freeze_receipt.json');receipts['transfer']=receipt
            dump(AUDIT/'freeze_receipt.json',receipts);state['protocols']=receipts
        dump(AUDIT/'mission_state.json',state)
    print(json.dumps(dict(totals=totals,selected=selected)))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['freeze','run','analyze']);p.add_argument('--stage',choices=['screen','transfer'],default='screen')
    p.add_argument('--dry-run',action='store_true');p.add_argument('--resume',action='store_true')
    a=p.parse_args()
    if a.action=='freeze':freeze(a.dry_run)
    elif a.action=='run':run(a.stage,a.dry_run,a.resume)
    else:analyze(a.stage)
