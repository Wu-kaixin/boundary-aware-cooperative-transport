"""Save a pause checkpoint without launching experiments or changing verdicts."""
from pathlib import Path
import hashlib,json,shutil,subprocess,datetime
ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'runs/evidence_closure';OUT=ROOT/'docs/results/evidence_closure/checkpoint_2026-10-07'
OUT.mkdir(parents=True,exist_ok=True)
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
batches={}
for name in ['baseline_static','baseline_transport','study_v1/safety','study_v1/diagnostic','study_v2/diagnostic','study_v3/recovery','study_v3/development','study_v3/ablation','verify_serial','verify_parallel']:
 folder=RUN/name; summary=folder/'summary.json'
 if not summary.exists():batches[name]={'status':'NOT_YET_SUMMARIZED'};continue
 value=json.loads(summary.read_text(encoding='utf-8'))
 dest=OUT/(name.replace('/','_')+'.json');dest.write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8')
 if 'pairs' in value:batches[name]={'status':'COMPLETED','episodes':2*len(value['pairs']),'pairs':len(value['pairs'])}
 else:
  rows=value['results'];planned=value['planned_denominator'];batches[name]={'status':'COMPLETED' if len(rows)==planned else 'PARTIAL','saved_cases':len(rows),'planned_cases':planned,'execution_errors':sum(r['execution_status']!='COMPLETED' for r in rows)}
verification={}
for case in ['verify_c_shape_2_boundary_cvt','verify_c_shape_2_nearest_observed_target']:
 a=RUN/'verify_serial'/case/'case_result.json';b=RUN/'verify_parallel'/case/'case_result.json'
 if a.exists() and b.exists():
  left=json.loads(a.read_text());right=json.loads(b.read_text());lr=left['result'];rr=right['result']
  lr.pop('wall_seconds',None);rr.pop('wall_seconds',None)
  products=['replay.npz','paired_metrics.json','physics_substeps.json','trajectories.csv']
  compared={n:sha(a.parent/n)==sha(b.parent/n) for n in products if (a.parent/n).exists() and (b.parent/n).exists()}
  # NPZ ZIP container times may differ; compare each numerical array.
  import numpy as np
  npz=a.parent/'replay.npz'
  if npz.exists():
   with np.load(npz,allow_pickle=True) as x,np.load(b.parent/'replay.npz',allow_pickle=True) as y:
    arrays={k:bool(np.array_equal(x[k],y[k],equal_nan=True)) if x[k].dtype.kind in 'fc' else bool(np.array_equal(x[k],y[k])) for k in x.files}
   compared['replay_arrays_equal']=all(arrays.values())
  verification[case]={'metrics_equal_except_wall_time':lr==rr,'products':compared}
 else:verification[case]={'status':'PENDING'}
(OUT/'serial_parallel_verification.json').write_text(json.dumps(verification,indent=2),encoding='utf-8')
# Preserve all raw artifacts in place and index their current bytes, including failures.
index=[]
for p in sorted(RUN.rglob('*')):
 if p.is_file():index.append({'path':p.relative_to(ROOT).as_posix(),'bytes':p.stat().st_size,'sha256':sha(p)})
(OUT/'raw_artifact_index.json').write_text(json.dumps(index,indent=2),encoding='utf-8')
state={'saved_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'branch':subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),'source_head_before_checkpoint_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'batches':batches,'raw_files':len(index),'raw_bytes':sum(x['bytes'] for x in index),'test_batch_status':'NOT_STARTED; frozen manifest only','next_steps':['Finish/check outstanding current batches if any; do not rerun completed cases.','Review mechanism and safety quality/stagnation reports; generate paper figures.','Run frozen independent test only when user resumes; no test episodes have been consumed.','Complete final regression after all source changes, evidence report and English professor summary.'],'limitations':['Full 555-pass regression precedes phase recovery and final runner edits; later phase subset passed16. Final full regression still pending.','Early batches captured source hashes but not complete source snapshots; later ablation/verification runner also saves source_snapshot.','Legacy PDFs and Gate6 v2 remain missing.','Main original controller and experimental candidate conclusions remain EMPIRICAL_ONLY.']}
(OUT/'checkpoint.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
print(json.dumps(state,indent=2))
