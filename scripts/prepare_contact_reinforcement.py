"""Protect the existing checkout and declare one nine-episode development budget."""
from pathlib import Path
import hashlib,json,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-01-contact-reinforcement-v1'
AUDIT=ROOT/'artifacts/migrations'/MISSION
if Path.cwd().resolve()!=ROOT or Path(sys.prefix).resolve()!=ROOT/'.venv':raise RuntimeError('Wrong root/environment')
if (AUDIT/'mission_state.json').exists():raise RuntimeError('Continue existing mission; do not reinitialize')
AUDIT.mkdir(parents=True,exist_ok=True)
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
protected=['src/dbact/control/controller.py','src/dbact/control/phase.py','src/dbact_sim/experiments/deployment.py',
    'README.md','docs/DATA_INDEX.md','docs/results/README.md','experiments/ccta2027/README.md']
hashes={}
for name in protected:
    dest=AUDIT/'backup'/name;dest.parent.mkdir(parents=True,exist_ok=True)
    if dest.exists():raise RuntimeError('Backup collision')
    shutil.copy2(ROOT/name,dest);hashes[name]=sha(dest);assert sha(ROOT/name)==hashes[name]
for name,args in [('worktree.patch',['diff','--no-ext-diff','--no-textconv','--binary']),('index.patch',['diff','--no-ext-diff','--no-textconv','--cached','--binary']),
    ('status.txt',['status','--porcelain=v1','--untracked-files=all']),('refs.txt',['show-ref'])]:
    (AUDIT/'backup'/name).write_bytes(subprocess.check_output(['git',*args],cwd=ROOT))
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
for path in [ROOT/'runs/maintenance'/MISSION,ROOT/'artifacts/research'/MISSION]:path.mkdir(parents=True,exist_ok=False)
state=dict(mission=MISSION,root=str(ROOT),head=head,status='IN_PROGRESS',stage='BASELINE_AND_EXISTING_LOGS',
    manuscript_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'manuscript/ccta2027').iterdir() if p.is_file()},
    budget=dict(new_episodes=9,conditions=['circle','ellipse_parallel','c_open_away'],seed=2,
        arms=['reference','recovery','reinforcement'],physical_seconds=600,frames=12000,workers=8,internal_threads=1,
        versions=1,wall_limit_seconds=3600),
    constraints=['simulation only','no attachment','own observations and one-hop messages; existing common supervisor disclosed',
        'physics, estimator, success, safety and manuscript unchanged','no tuning or extra scientific episodes'])
for name,data in [('mission_state.json',state),('backup_manifest.json',hashes),('design_before_diagnosis.json',state['budget'])]:
    (AUDIT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
(AUDIT/'operations.jsonl').write_text(json.dumps(dict(operation='protected pre-task dirty files',files=protected))+'\n')
print(json.dumps(dict(mission=MISSION,protected=len(protected),head=head)))
