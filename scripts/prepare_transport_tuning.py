"""Protect this in-place development task before finite parameter screening."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MISSION = '2026-11-02-transport-tuning-v1'
AUDIT = ROOT/'artifacts/migrations'/MISSION
OUT = ROOT/'runs/maintenance'/MISSION

def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def main():
    assert Path.cwd().resolve() == ROOT and Path(sys.prefix).resolve() == ROOT/'.venv'
    if (AUDIT/'mission_state.json').exists():
        raise RuntimeError('Existing mission: inspect state; do not reset')
    assert shutil.disk_usage(ROOT).free > 80*1024**3
    AUDIT.mkdir(parents=True,exist_ok=True)
    OUT.mkdir(parents=True,exist_ok=True)
    (ROOT/'artifacts/research'/MISSION).mkdir(parents=True,exist_ok=True)
    files = list((ROOT/'src').rglob('*.py'))
    files += [ROOT/p for p in ('README.md','docs/DATA_INDEX.md','docs/results/README.md',
                               'experiments/ccta2027/README.md')]
    hashes = {}
    for p in files:
        rel=p.relative_to(ROOT); dest=AUDIT/'backup'/rel
        dest.parent.mkdir(parents=True,exist_ok=True)
        if dest.exists():
            assert sha(dest) == sha(p), 'Partial backup conflicts: preserve and inspect'
        else:
            shutil.copy2(p,dest)
        hashes[rel.as_posix()] = sha(p)
        assert sha(dest) == hashes[rel.as_posix()]
    for name,args in [('worktree.patch',['diff','--no-ext-diff','--no-textconv','--binary']),
                      ('index.patch',['diff','--no-ext-diff','--no-textconv','--cached','--binary']),
                      ('status.txt',['status','--porcelain=v1','--untracked-files=all']),('refs.txt',['show-ref'])]:
        (AUDIT/'backup'/name).write_bytes(subprocess.check_output(['git','--no-optional-locks',*args],cwd=ROOT))
    state=dict(mission=MISSION,status='IN_PROGRESS',stage='BASELINE_AND_OFFLINE_AUDIT',
        head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        index_sha256=sha(ROOT/'.git/index'),
        manuscript_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'manuscript/ccta2027').iterdir() if p.is_file()},
        constraints=['Simulation only; no attachment; allowed local observations only',
                     'Original success, safety, G500 criteria and historical records unchanged',
                     'No Git commits, push or refs changes; no dependency upgrades'],
        budget=dict(maximum_development_cases=72,maximum_physical_seconds_per_case=600,
                    workers=8,internal_threads=1,maximum_wall_seconds_per_stage=14400,
                    rule='Freeze finite parameter settings, cases, selection and stopping before any run; no adaptive retuning'))
    for name,value in [('mission_state.json',state),('backup_manifest.json',hashes)]:
        (AUDIT/name).write_text(json.dumps(value,indent=2),encoding='utf-8')
    (AUDIT/'CONTINUE.md').write_text('No scientific batch started. Read mission_state.json before continuing. Protect dirty source and frozen evidence.\n',encoding='utf-8')
    (AUDIT/'operations.jsonl').write_text(json.dumps(dict(operation='protect',files=len(hashes)))+'\n',encoding='utf-8')
    print(json.dumps(dict(mission=MISSION,protected=len(hashes),free_bytes=shutil.disk_usage(ROOT).free)))

if __name__ == '__main__':
    main()
