"""Verify frozen products, preserve user changes, and close this finite mission."""
import difflib
import json
import re
import shutil
import subprocess
from urllib.parse import unquote

from pypdf import PdfReader
import voter_continuity_study as study
from dbact_sim.experiments.study import cached_result, verify_source_snapshot


def main():
    state=study.read(study.AUDIT/'mission_state.json')
    if state['status']=='COMPLETED_FIXED_BUDGET': raise RuntimeError('Mission already closed')
    freeze=study.read(study.AUDIT/'freeze_receipt.json')
    analysis=study.read(study.PUBLIC/'analysis.json')
    initial_analysis=study.read(study.AUDIT/'analysis_v1_analysis.json')
    assert analysis['episodes']==initial_analysis['episodes']
    assert analysis['diagnostics']==initial_analysis['diagnostics']
    assert study.source_hashes()==freeze['source_files']
    assert study.environment_versions()==freeze['environment']
    assert study.sha(study.PROTOCOL)==freeze['protocol_sha256']
    assert study.sha(study.AUDIT/'design.json')==study.read(study.PROTOCOL)['design_sha256']
    assert study.sha(study.ROOT/'.git/index')==state['index_sha256']
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==state['head']
    assert subprocess.check_output(['git','show-ref'])==(study.AUDIT/'backup/refs.txt').read_bytes()
    assert {str(p.relative_to(study.ROOT)):study.sha(p) for p in (study.ROOT/'manuscript/ccta2027').rglob('*') if p.is_file()}==state['manuscript']
    assert all(study.sha(study.AUDIT/'backup'/p)==h for p,h in state['protected_originals'].items())
    plan=study.read(study.PROTOCOL);batch=study.ROOT/plan['run_dir']
    manifest=study.read(batch/'manifest.json')
    verify_source_snapshot(batch,manifest['source_files'])
    for c in plan['cases']:
        folder=batch/c['case_id'];r=study.read(folder/'case_result.json')
        cached_result(folder,r['identity'])
        assert study.sha(folder/'case_result.json')==analysis['input_receipts'][str((folder/'case_result.json').relative_to(study.ROOT))]
    for origin in freeze['origins'].values():
        folder=study.ROOT/origin['path'];r=study.read(folder/'case_result.json')
        assert study.sha(folder/'case_result.json')==origin['receipt_sha256']
        cached_result(folder,r['identity'])
    assert all(c['replay_exact'] and c['old_scientific_fields_exact'] for c in analysis['original_reproduction'].values())
    assert study.read(study.OUT/'checks-result.json')['exit_code']==0
    assert study.read(study.OUT/'ablation-result.json')['exit_code']==0
    assert study.read(study.OUT/'analysis-v2-result.json')['exit_code']==0
    # Only curated navigation is edited; historical evidence remains untouched.
    message='投票集合与停止确认固定消融：32回合（8个已知开发条件×4臂），分别检验换票读数连续化和1秒到达确认，保留全部失败与截尾；默认关闭、原成功安全判据不变。'
    entries={'README.md':('docs/results/voter_continuity_v1/README.md','最新投票与停止消融'),
             'docs/DATA_INDEX.md':('results/voter_continuity_v1/README.md','投票与停止数据'),
             'docs/results/README.md':('voter_continuity_v1/README.md','投票与停止消融'),
             'experiments/ccta2027/README.md':('../../docs/results/voter_continuity_v1/README.md','固定32回合结果')}
    edits=[]
    for name,(link,label) in entries.items():
        path=study.ROOT/name; before=path.read_text(encoding='utf-8')
        entry=f"\n[{label}]({link})：{message}\n"
        if link not in before:
            head,sep,tail=before.partition('\n')
            path.write_text(head+sep+entry+'\n'+tail,encoding='utf-8')
        after=path.read_text(encoding='utf-8')
        edits.extend(difflib.unified_diff((study.AUDIT/'backup'/name).read_text(encoding='utf-8').splitlines(True),
                                         after.splitlines(True),fromfile='before/'+name,tofile='after/'+name))
    (study.AUDIT/'documentation_changes.patch').write_text(''.join(edits),encoding='utf-8')
    links=[]
    for path in [study.PUBLIC/'README.md',*[study.ROOT/n for n in entries]]:
        for target in re.findall(r'\]\(([^)]+)\)',path.read_text(encoding='utf-8')):
            if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'): continue
            relative=unquote(target.split('#')[0].strip('<>'))
            if not relative: continue
            resolved=(path.parent/relative).resolve()
            assert resolved.exists(),(path,target)
            links.append(dict(document=str(path.relative_to(study.ROOT)),target=target))
    pdfs={}
    for path in study.PUBLIC.glob('*.pdf'):
        pdf=PdfReader(path);fonts=[]
        for page in pdf.pages:
            for font in page['/Resources'].get('/Font',{}).get_object().values():
                obj=font.get_object()
                descendants=obj.get('/DescendantFonts')
                descriptor=(descendants[0].get_object().get('/FontDescriptor') if descendants else obj.get('/FontDescriptor'))
                embedded=bool(descriptor and any(k in descriptor.get_object() for k in ('/FontFile','/FontFile2','/FontFile3')))
                fonts.append(dict(name=str(obj.get('/BaseFont')),embedded=embedded))
                assert embedded
        pdfs[path.name]=dict(pages=len(pdf.pages),fonts=fonts,sha256=study.sha(path))
    paths={'new_raw':batch,'engineering':study.OUT,'audit_and_backup':study.AUDIT,'curated':study.PUBLIC}
    sizes={name:sum(p.stat().st_size for p in root.rglob('*') if p.is_file()) for name,root in paths.items()}
    integrity=dict(source_sha256=analysis['source_sha256'],checks='PASS',historical_cases_verified=8,
        new_cases_verified=32,links_verified=len(links),pdfs=pdfs,sizes_bytes=sizes,
        disk_free_bytes=shutil.disk_usage(study.ROOT).free,git='HEAD, refs, index unchanged',
        manuscript='All bytes unchanged',source_snapshot='PASS',protected_originals='PASS',
        environment='Same root venv throughout; pre-existing PyYAML lock mismatch 6.0.2 vs 6.0.3 retained',
        scripts={n:study.sha(study.ROOT/'scripts'/n) for n in ['voter_continuity_study.py','audit_voter_continuity.py',
            'analyze_voter_continuity.py','publish_voter_continuity.py','finalize_voter_continuity.py']})
    study.dump(study.AUDIT/'final_integrity.json',integrity)
    validation='''# Validation report

- PASS: 36 baseline targeted tests; 807 final core tests, 2 pre-existing skips.
- PASS: fixed 32 scheduled episodes; original independent evaluator and all failures retained.
- PASS: eight reference reruns exactly match every replay array and historical scientific field; two independently recomputed config identity hashes are explicitly separate.
- PASS: permissions, missing/disjoint voters, unchanged-roster equivalence, bias preservation and stop confirmation regressions.
- PASS: pip check, diff check, dry-run; frozen source and all new/historical product hashes.
- FAIL_PREEXISTING: PyYAML installed 6.0.2, lock 6.0.3; no environment edits.
- NOT_APPLICABLE: MAS/hardware; simulation-only request.
- NOT_RUN: manuscript build; manuscript bytes and title unchanged.
- Initial Git textconv preflight and null-reference offline audit errors were corrected before execution; premature analysis refused incomplete study. No scientific retries.
- Initial offline comparison incorrectly included config identity hashes. Its output is preserved as analysis_v1_*. Final offline verification checks exact config equivalence except explicit False/1 flags, recomputes both identity hashes, and requires exact equality on all remaining fields. No numerical tolerance changed.

Commands, exit codes and logs: runs/maintenance/2026-11-02-voter-continuity-v1/.
'''
    (study.AUDIT/'validation_report.md').write_text(validation,encoding='utf-8')
    (study.AUDIT/'FINAL_REPORT.md').write_text(
        '# Fixed-budget voter continuity study completed\n\n'
        '32 scheduled episodes finished; adoption gate: '+analysis['adoption_gate']+
        '. All failures retained; options remain off by default.\n\n'
        '[Complete results and scientific limitations](../../../docs/results/voter_continuity_v1/README.md)\n\n'
        +json.dumps(analysis['aggregates'],ensure_ascii=False,indent=2)+
        '\n\nLocal integrity: final_integrity.json; validation: validation_report.md.\n'
        'Recovery: backup contains original dirty source/docs and Git worktree/index/ref records. '
        'Review runtime_changes.patch and documentation_changes.patch before selective restoration; never reset the whole worktree. '
        'No deletion, push, commit or ref movement. Same-disk backup is not whole-disk disaster protection.\n',encoding='utf-8')
    state.update(status='COMPLETED_FIXED_BUDGET',stage='FINALIZED',episodes=32,adoption_gate=analysis['adoption_gate'],
        source_after=freeze['source_files'],result_path=str(study.PUBLIC.relative_to(study.ROOT)),integrity=integrity)
    study.dump(study.AUDIT/'mission_state.json',state)
    (study.AUDIT/'CONTINUE.md').write_text(
        'Fixed 32-episode budget complete. No pending run. Do not resume to add cases, retune, or replace failed episodes. '
        'Read FINAL_REPORT.md and docs/results/voter_continuity_v1/README.md before deciding any new protocol. '
        'Default remains voter continuity OFF, shared HOLD dwell=1. Paper title unchanged. '
        'Existing user changes are preserved; Git index/HEAD/refs unchanged.\n',encoding='utf-8')
    study.event('finalize',status=state['status'],adoption_gate=analysis['adoption_gate'])
    print(json.dumps(dict(status=state['status'],sizes=sizes,links=len(links),pdfs=pdfs)))


if __name__=='__main__': main()
