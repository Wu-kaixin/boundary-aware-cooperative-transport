"""Protect the in-place checkout before a bounded, simulation-only investigation."""
from pathlib import Path
import hashlib, json, shutil, subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
MISSION = '2026-11-02-contact-consistency-v1'
AUDIT = ROOT/'artifacts/migrations'/MISSION

def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def main():
    assert Path.cwd().resolve() == ROOT and Path(sys.prefix).resolve() == ROOT/'.venv'
    if (AUDIT/'mission_state.json').exists():
        raise RuntimeError('Mission exists; resume without replacing its state')
    AUDIT.mkdir(parents=True, exist_ok=True)
    protected = ['src/dbact/control/controller.py', 'src/dbact/control/phase.py',
        'src/dbact/control/boundary_navigation.py', 'src/dbact/control/observed_geometry.py',
        'src/dbact/control/observed_reference.py', 'src/dbact/control/transport.py',
        'src/dbact/safety/filter.py', 'src/dbact_sim/experiments/deployment.py',
        'src/dbact_sim/experiments/protocol.py', 'README.md', 'docs/results/README.md',
        'docs/DATA_INDEX.md', 'experiments/ccta2027/README.md']
    hashes = {}
    for name in protected:
        dest = AUDIT/'backup'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest.exists():
            raise RuntimeError('Backup collision')
        shutil.copy2(ROOT/name, dest)
        hashes[name] = sha(dest)
        assert sha(ROOT/name) == hashes[name]
    for name,args in [('worktree.patch',['diff','--no-ext-diff','--no-textconv','--binary']),
        ('index.patch',['diff','--no-ext-diff','--no-textconv','--cached','--binary']),
        ('status.txt',['status','--porcelain=v1','--untracked-files=all']),('refs.txt',['show-ref'])]:
        (AUDIT/'backup'/name).write_bytes(subprocess.check_output(['git',*args],cwd=ROOT))
    state = dict(mission=MISSION,status='IN_PROGRESS',stage='TAG_AND_REPLAY_AUDIT',
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        manuscript_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'manuscript/ccta2027').iterdir() if p.is_file()},
        index_sha256=sha(ROOT/'.git/index'),
        constraints=['simulation only','no attachment','own/one-hop observations; existing common supervisor',
            'preserve physics, safety and success definitions','do not change manuscript title or historical results'],
        budget=dict(stage='offline diagnosis first; scientific arms must freeze before execution',
            maximum_new_episodes=18,maximum_physical_seconds=600,versions=1,
            maximum_workers=8,internal_threads=1,maximum_batch_wall_seconds=5400,
            maximum_shadow_replay_cases=2,maximum_shadow_wall_seconds=2400,
            rule='No retuning or added scientific episodes after inspecting outcomes'))
    for name,data in [('mission_state.json',state),('backup_manifest.json',hashes)]:
        (AUDIT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    for category in ['runs/maintenance','artifacts/research']:
        (ROOT/category/MISSION).mkdir(parents=True,exist_ok=False)
    (AUDIT/'CONTINUE.md').write_text('Read mission_state.json. Preserve prior dirty files and all frozen batches. Offline diagnosis precedes a separately frozen simulation protocol. No process started by preparation.\n',encoding='utf-8')
    (AUDIT/'operations.jsonl').write_text(json.dumps(dict(operation='protect_preexisting_files',files=protected))+'\n',encoding='utf-8')
    print(json.dumps(dict(mission=MISSION,protected=len(hashes),free_bytes=shutil.disk_usage(ROOT).free)))

if __name__ == '__main__':
    main()
