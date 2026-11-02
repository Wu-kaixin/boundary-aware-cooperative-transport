"""Protect the current in-place research tree and declare bounded work stages."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MISSION = '2026-11-02-cargo-discovery-v1'
AUDIT = ROOT/'artifacts/migrations'/MISSION

def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def main():
    assert Path.cwd().resolve() == ROOT and Path(sys.prefix).resolve() == ROOT/'.venv'
    if AUDIT.exists():
        raise RuntimeError('Existing mission: read its state, do not reinitialize')
    AUDIT.mkdir(parents=True)
    files = list((ROOT/'src').rglob('*.py')) + list((ROOT/'tests').glob('test_*reference*.py'))
    files += [ROOT/p for p in ('README.md','docs/DATA_INDEX.md','docs/results/README.md',
                               'experiments/ccta2027/README.md')]
    hashes = {}
    for p in files:
        name = p.relative_to(ROOT).as_posix()
        target = AUDIT/'backup'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, target)
        hashes[name] = sha(p)
        assert sha(target) == hashes[name]
    for name, args in [('worktree.patch',['diff','--no-ext-diff','--no-textconv','--binary']),
                       ('index.patch',['diff','--no-ext-diff','--no-textconv','--cached','--binary']),
                       ('status.txt',['status','--porcelain=v1','--untracked-files=all']),
                       ('refs.txt',['show-ref'])]:
        (AUDIT/'backup'/name).write_bytes(subprocess.check_output(['git','--no-optional-locks',*args],cwd=ROOT))
    state = dict(mission=MISSION,status='IN_PROGRESS',stage='OFFLINE_BASELINE_AUDIT',
        head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        manuscript_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'manuscript/ccta2027').iterdir() if p.is_file()},
        index_sha256=sha(ROOT/'.git/index'),
        constraints=['Simulation only; no attachment or truth feedback','Keep title, safety and completion definitions',
                     'Separate unknown shape from initially unknown position','One candidate version; retain all failures'],
        budget=dict(offline_replay_cases=1,offline_wall_seconds=1800,
                    near_feedback_maximum_cases=18,near_maximum_cases=132,far_maximum_cases=24,
                    maximum_physical_seconds_per_case=600,workers=8,internal_threads=1,
                    maximum_wall_seconds_per_batch=14400,
                    rule='Stage protocols must freeze before execution; no outcome-driven addition or retuning'))
    for name, value in [('mission_state.json',state),('backup_manifest.json',hashes)]:
        (AUDIT/name).write_text(json.dumps(value,indent=2),encoding='utf-8')
    for folder in ('runs/maintenance','artifacts/research'):
        (ROOT/folder/MISSION).mkdir(parents=True,exist_ok=False)
    (AUDIT/'CONTINUE.md').write_text('Read mission_state.json and frozen protocols before resuming. No scientific run has started. Preserve pre-existing dirty source and all old results.\n',encoding='utf-8')
    (AUDIT/'operations.jsonl').write_text(json.dumps(dict(operation='protect_source_and_dirty_state',files=len(hashes)))+'\n',encoding='utf-8')
    print(json.dumps(dict(mission=MISSION,protected=len(hashes),free_bytes=shutil.disk_usage(ROOT).free)))

if __name__ == '__main__':
    main()
