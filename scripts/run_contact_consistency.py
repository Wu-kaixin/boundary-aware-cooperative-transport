"""Execute the already frozen comparison with existing bounded tree monitoring."""
import argparse,json,os,sys
from pathlib import Path
import benchmark_workstation as monitor
from audit_contact_consistency import ROOT,MISSION,read,sha
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import source_hashes

def main(dry_run=False):
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    protocol=ROOT/'experiments/ccta2027/contact_consistency_v1.json';plan=read(protocol)
    audit=ROOT/'artifacts/migrations'/MISSION;check=ROOT/'runs/maintenance'/MISSION
    assert sha(protocol)==read(audit/'freeze_receipt.json')['sha256']
    assert encode(source_hashes())==plan['frozen_source_sha256']
    command=[sys.executable,'-m','dbact_sim','study','--protocol',str(protocol),
             '--split','development','--workers',str(plan['budget']['workers']),
             '--out',str(ROOT/plan['run_dir'])]
    if dry_run:
        print(json.dumps(dict(command=command,budget=plan['budget'])));return
    if (ROOT/plan['run_dir']).exists():raise RuntimeError('Batch exists; inspect and preserve it')
    monitor.OUT=check;monitor.AUDIT=audit
    state=read(audit/'mission_state.json');state.update(stage='RUNNING_FIXED_COMPARISON')
    dump(audit/'mission_state.json',state)
    result=monitor.run_command('study',command,plan['budget']['wall_seconds'],os.environ.copy())
    (check/'study.exit.txt').write_text(str(result['exit_code'])+'\n',encoding='utf-8')
    state.update(stage='BATCH_FINISHED',execution=result['status'])
    dump(audit/'mission_state.json',state)
    if result['exit_code']!=0:raise SystemExit(result['exit_code'])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');main(p.parse_args().dry_run)
