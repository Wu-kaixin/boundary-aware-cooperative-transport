"""Run/resume one frozen near or far stage with bounded process-tree monitoring."""
import argparse
import json
import os
from pathlib import Path
import sys
import benchmark_workstation as monitor
from prepare_cargo_discovery import ROOT, MISSION, AUDIT, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import source_hashes

def main(stage,dry_run=False,resume=False):
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    check=ROOT/'runs/maintenance'/MISSION
    receipt=json.loads((AUDIT/'freeze_receipt.json').read_text())[stage]
    protocol=ROOT/receipt['protocol'];plan=json.loads(protocol.read_text())
    assert sha(protocol)==receipt['sha256']
    assert encode(source_hashes())==plan['frozen_source_sha256']
    command=[sys.executable,'-m','dbact_sim','study','--protocol',str(protocol),'--split','development',
             '--workers',str(plan['budget']['workers']),'--out',str(ROOT/plan['run_dir'])]
    if resume:command+=['--resume']
    if dry_run:
        print(json.dumps(dict(command=command,budget=plan['budget'])));return
    if not resume and (ROOT/plan['run_dir']).exists():raise RuntimeError('Existing batch; inspect first')
    monitor.OUT=check;monitor.AUDIT=AUDIT
    state=json.loads((AUDIT/'mission_state.json').read_text())
    state.update(stage='RUNNING_'+stage.upper())
    dump(AUDIT/'mission_state.json',state)
    result=monitor.run_command(stage+('-resume' if resume else ''),command,plan['budget']['wall_seconds'],os.environ.copy())
    (check/f'{stage}.exit.txt').write_text(str(result['exit_code'])+'\n')
    state.update(stage=stage.upper()+'_FINISHED',last_execution=result['status'])
    dump(AUDIT/'mission_state.json',state)
    if result['exit_code']!=0:raise SystemExit(result['exit_code'])

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['near','far']);parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--resume',action='store_true');args=parser.parse_args()
    main(args.stage,args.dry_run,args.resume)
