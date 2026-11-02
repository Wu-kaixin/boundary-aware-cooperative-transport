"""Close the finite reference-origin study after verifying all retained evidence."""
import difflib
import json
import re
import shutil
import subprocess
from urllib.parse import unquote
from pypdf import PdfReader
import reference_origin_study as study
from dbact.control.controller import DBACTParams
from dbact_sim.experiments.study import cached_result, verify_source_snapshot


def main():
    study.guard()
    state=study.read(study.AUDIT/'mission_state.json')
    if state['status']=='COMPLETED_FIXED_BUDGET':raise RuntimeError('Already finalized')
    freeze=study.read(study.AUDIT/'freeze_receipt.json');plan=study.read(study.PROTOCOL)
    analysis=study.read(study.PUBLIC/'analysis.json');batch=study.ROOT/plan['run_dir']
    assert study.source_hashes()==freeze['source_files']
    assert study.environment_versions()==freeze['environment']
    assert study.sha(study.PROTOCOL)==freeze['protocol_sha256']
    assert study.sha(study.AUDIT/'design.json')==plan['design_sha256']
    assert study.sha(study.ROOT/'.git/index')==state['index_sha256']
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==state['head']
    assert subprocess.check_output(['git','show-ref'])==(study.AUDIT/'backup/refs.txt').read_bytes()
    assert {str(p.relative_to(study.ROOT)):study.sha(p) for p in (study.ROOT/'manuscript/ccta2027').rglob('*') if p.is_file()}==state['manuscript']
    assert all(study.sha(study.AUDIT/'backup'/p)==h for p,h in state['protected_originals'].items())
    prior=study.read(study.AUDIT/'prior_origin_audit.json')
    assert study.sha(study.AUDIT/'backup/audit_reference_origin_prior_version.py')==prior['script_sha256']
    assert study.sha(study.AUDIT/'prior_origin_audit.json')==study.read(study.AUDIT/'design.json')['selection_evidence_sha256']
    params=DBACTParams()
    assert params.reference_early_anchor is False and params.reference_bootstrap_gain is None
    assert params.navigation_voter_continuity is False and params.navigation_hold_dwell==1
    manifest=study.read(batch/'manifest.json');summary=study.read(batch/'summary.json')
    assert summary['completed_denominator']==24 and summary['source_integrity']=='PASS'
    verify_source_snapshot(batch,manifest['source_files'])
    for case in plan['cases']:
        folder=batch/case['case_id'];receipt=study.read(folder/'case_result.json')
        cached_result(folder,receipt['identity'])
        assert study.sha(folder/'case_result.json')==analysis['input_receipts'][str((folder/'case_result.json').relative_to(study.ROOT))]
    for origin in freeze['origins'].values():
        folder=study.ROOT/origin['path'];receipt=study.read(folder/'case_result.json')
        assert study.sha(folder/'case_result.json')==origin['receipt_sha256']
        cached_result(folder,receipt['identity'])
    assert len(analysis['original_reproduction'])==8
    assert all(c['replay_exact'] and all(c['scientific_fields_exact'].values()) for c in analysis['original_reproduction'].values())
    assert len(analysis['pre_reference_control_equivalence'])==16
    assert all(c['physical_arrays_exact'] and c['control_records_exact'] for c in analysis['pre_reference_control_equivalence'].values())
    for label in ('baseline','checks','dry-run','comparison','analysis','publish-v2','pip-check','diff-check'):
        result=study.read(study.OUT/(label+'-result.json'))
        assert result['exit_code']==0 and result['status']=='COMPLETED',label

    message='参考初始化固定消融完成：24回合（8个已知开发条件×3臂），三臂均7/8完成；取消初始化放大救回斜箱但损失远场C形，提前参考未救回斜箱且G500下降。两项均未采用，原判据和默认方法不变。'
    entries={'README.md':('docs/results/reference_origin_v1/README.md','最新参考初始化消融'),
             'docs/DATA_INDEX.md':('results/reference_origin_v1/README.md','参考初始化数据'),
             'docs/results/README.md':('reference_origin_v1/README.md','参考初始化消融'),
             'experiments/ccta2027/README.md':('../../docs/results/reference_origin_v1/README.md','固定24回合结果')}
    for name,(link,label) in entries.items():
        path=study.ROOT/name;before=path.read_bytes()
        if link.encode() in before:continue
        assert study.sha(path)==state['protected_originals'][name],f'Concurrent edit: {name}'
        newline=b'\r\n' if b'\r\n' in before else b'\n'
        head,sep,tail=before.partition(newline)
        entry=(f'[{label}]({link})：{message}').encode('utf-8')
        path.write_bytes(head+sep+newline+entry+newline+newline+tail)
    for label,names in [('runtime',list(state['protected_originals'])[:3]),('documentation',list(entries))]:
        patch=[]
        for name in names:
            before=(study.AUDIT/'backup'/name).read_text(encoding='utf-8').splitlines(True)
            after=(study.ROOT/name).read_text(encoding='utf-8').splitlines(True)
            patch.extend(difflib.unified_diff(before,after,fromfile='before/'+name,tofile='after/'+name))
        (study.AUDIT/(label+'_changes.patch')).write_text(''.join(patch),encoding='utf-8')
    links=[]
    for path in [study.PUBLIC/'README.md',*[study.ROOT/n for n in entries]]:
        for target in re.findall(r'\]\(([^)]+)\)',path.read_text(encoding='utf-8')):
            if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'):continue
            relative=unquote(target.split('#')[0].strip('<>'))
            if not relative:continue
            assert (path.parent/relative).resolve().exists(),(path,target)
            links.append(dict(document=str(path.relative_to(study.ROOT)),target=target))
    figures=study.read(study.PUBLIC/'figure_manifest.json');pdfs={}
    assert figures['script_sha256']==study.sha(study.ROOT/'scripts/publish_reference_origin.py')
    for name,figure in figures['figures'].items():
        for fmt,h in figure['files'].items():assert study.sha(study.PUBLIC/(name+'.'+fmt))==h
        for name_input,h in figure['inputs'].items():
            p=(study.PUBLIC/name_input) if name_input=='analysis.json' else (study.ROOT/name_input)
            assert study.sha(p)==h
        path=study.PUBLIC/(name+'.pdf');pdf=PdfReader(path);fonts=[]
        for page in pdf.pages:
            for font in page['/Resources'].get('/Font',{}).get_object().values():
                obj=font.get_object();descendants=obj.get('/DescendantFonts')
                descriptor=descendants[0].get_object().get('/FontDescriptor') if descendants else obj.get('/FontDescriptor')
                embedded=bool(descriptor and any(k in descriptor.get_object() for k in ('/FontFile','/FontFile2','/FontFile3')))
                assert embedded
                fonts.append(dict(name=str(obj.get('/BaseFont')),embedded=True))
        pdfs[path.name]=dict(pages=len(pdf.pages),fonts=fonts,sha256=study.sha(path))
    subprocess.run(['git','diff','--check'],check=True)
    roots=dict(new_raw=batch,engineering=study.OUT,audit_and_backup=study.AUDIT,curated=study.PUBLIC)
    sizes={n:sum(p.stat().st_size for p in d.rglob('*') if p.is_file()) for n,d in roots.items()}
    scripts=['reference_origin_study.py','audit_reference_origin.py','analyze_reference_origin.py',
             'publish_reference_origin.py','finalize_reference_origin.py']
    integrity=dict(source_sha256=analysis['source_sha256'],historical_cases_verified=8,new_cases_verified=24,
        default_flags='Unchanged: early anchor=False, bootstrap gain=None, continuity=False, HOLD dwell=1',
        git='HEAD, refs and index unchanged; no commit or push',manuscript='All bytes unchanged',
        source_snapshot='PASS',protected_originals='PASS',links_verified=len(links),pdfs=pdfs,
        sizes_bytes=sizes,sizes_scope='Measured before final report/state write; audit includes preserved first plot revision.',
        disk_free_bytes=shutil.disk_usage(study.ROOT).free,
        scripts={n:study.sha(study.ROOT/'scripts'/n) for n in scripts},
        environment='Same root venv, pre-existing PyYAML6.0.2 vs lock6.0.3 retained',
        visual_review='Three PNG figures inspected; time-series columns share physical x-axis; PDF fonts embedded')
    study.dump(study.AUDIT/'final_integrity.json',integrity)
    validation='''# Validation report

- PASS: 40 baseline tests; 818 final core tests, 2 historical skips.
- PASS: all 24 fixed-budget executions; no scientific rerun or extra episode.
- PASS: original eight reruns exactly reproduce historical replay arrays and scientific fields; config identity hashes separately recomputed.
- PASS: 16 candidate pre-reference prefixes have exactly equal physical states, controls and stage/navigation records.
- PASS: original success, safety, G500, 600s cap; every failure retained. Both adoption gates FAIL is a scientific outcome, not a hidden engineering failure.
- PASS: all new and eight historical products rehashed, frozen source and snapshots verified.
- PASS: pip check, diff check, dry-run, analysis, figure-data hashes, local document links, embedded PDF fonts.
- FAIL_PREEXISTING: installed PyYAML6.0.2 versus locked6.0.3; no installation or environment edits.
- NOT_APPLICABLE: MAS/hardware; simulation-only scope.
- NOT_RUN: paper build; manuscript and title unchanged.
- NOT_AVAILABLE: Windows CPU frequency/disk-wait performance counters; raw monitor errors retained.
- First offline audit failed on a missing explicit gain field. Corrected to resolve the actual default; original failure log retained, no simulation executed by this audit.
- First figure revision used separate x-axis extents across time-series rows. Preserved and regenerated with a shared physical time axis; no data or scientific analysis changed.

Commands, exit codes and logs: runs/maintenance/2026-11-02-reference-origin-v1/.
'''
    (study.AUDIT/'validation_report.md').write_text(validation,encoding='utf-8')
    (study.AUDIT/'FINAL_REPORT.md').write_text(
        '# Reference-origin fixed-budget study completed\n\n'
        '24/24 executed and verified. Original, early reference and unscaled bootstrap each safely completed 7/8. '
        'G500: 6/8, 5/8, 6/8. Both adoption gates FAIL; defaults unchanged.\n\n'
        '[Full results and limitations](../../../docs/results/reference_origin_v1/README.md)\n\n'
        +json.dumps(analysis['aggregates'],ensure_ascii=False,indent=2)+
        '\n\nRecovery: backup/ contains original dirty source, documents, worktree/index diffs and refs. '
        'Review runtime_changes.patch and documentation_changes.patch before selective restoration; never reset the whole worktree. '
        'Raw data remain in runs/research/2026-11-02-reference-origin-v1/comparison/. '
        'Same-disk copies protect against local mistakes, not disk failure. No unique research data deletion, commit, push or ref movement.\n',encoding='utf-8')
    state.update(status='COMPLETED_FIXED_BUDGET',stage='FINALIZED',episodes=24,
        adoption_gates=analysis['adoption_gates'],source_after=freeze['source_files'],
        result_path=str(study.PUBLIC.relative_to(study.ROOT)),integrity=integrity)
    study.dump(study.AUDIT/'mission_state.json',state)
    (study.AUDIT/'CONTINUE.md').write_text(
        'Fixed 24-episode comparison finished. No active/pending study. Do not add cases or retune under this identity. '
        'Both interventions failed the frozen adoption gate; preserve options and all adverse results, keep defaults unchanged. '
        'Next proposed step is same-input offline separation of reference-point rotation and registration errors, not automatically authorized extra episodes. '
        'Read FINAL_REPORT.md and docs/results/reference_origin_v1/README.md. Git refs/index/HEAD and manuscript unchanged.\n',encoding='utf-8')
    study.event('finalize',status=state['status'],adoption_gates=analysis['adoption_gates'])
    print(json.dumps(dict(status=state['status'],sizes=sizes,links=len(links),gates=analysis['adoption_gates'])))


if __name__=='__main__':main()
