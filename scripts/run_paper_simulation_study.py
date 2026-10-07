"""Frozen-case outer-process runner; independent outputs, integrity-checked resume."""
from __future__ import annotations
import argparse, hashlib, json, os, platform, subprocess, sys, time, traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['MKL_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1';os.environ['DBACT_CVT_WORKERS']='1'
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
import numpy as np
import scipy, yaml
from run_deployment_pairs import static_arm,transport_arm,encode,dump
from dbact.cpu_budget import limit_blas_threads, ResourceSampler

def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def source_hashes():
    return {p.relative_to(ROOT).as_posix():digest(p) for folder in ('src','scripts','configs') for p in sorted((ROOT/folder).rglob('*')) if p.is_file() and p.suffix in ('.py','.yaml')}
def worker(payload):
    case,folder,identity=payload; out=Path(folder);out.mkdir(parents=True,exist_ok=True)
    limit_blas_threads(1);start=time.perf_counter()
    try:
        if case['gate']==5:
            result=static_arm(case['config'],case['seed'],case['variant'],case['frames'],out)
        else: result=transport_arm(case['config'],case['seed'],case['variant'],case['frames'],out)
        result['execution_status']='COMPLETED'
    except Exception as exc:
        result={'execution_status':'ERROR','error':repr(exc),'traceback':traceback.format_exc(),'task_completed':False,'g500_pass':False,'settled':False}
    result.update(case_id=case['case_id'],shape=case['shape'],variant=case['variant'],seed=case['seed'],stratum=case.get('stratum','near'),wall_seconds=time.perf_counter()-start)
    if case['gate']==6 and result['execution_status']=='COMPLETED':
        # New physical endpoint definition, independent of G500; same task tolerance and settling speed/window.
        tolerance=case['config']['task']['tolerance']
        result['task_completed']=bool(result['settled'] and result['euclidean_endpoint_error']<=tolerance)
        result['task_completed_definition']='settled plus final Euclidean endpoint error <= existing task tolerance'
    products={p.relative_to(out).as_posix():digest(p) for p in out.rglob('*') if p.is_file() and p.name!='case_result.json'}
    dump(out/'case_result.json',{'identity':identity,'products':products,'result':result})
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True);parser.add_argument('--split',required=True)
    parser.add_argument('--workers',type=int,default=3);parser.add_argument('--resume',action='store_true');parser.add_argument('--out',type=Path)
    args=parser.parse_args(); protocol=yaml.safe_load(args.protocol.read_text(encoding='utf-8-sig'))
    source=source_hashes();source_id=encode(source)
    if protocol.get('frozen_source_sha256') and protocol['frozen_source_sha256']!=source_id: raise RuntimeError('source differs from frozen protocol')
    cases=[c for c in protocol['cases'] if c['split']==args.split]
    if not cases: raise ValueError('empty split')
    if len({c['case_id'] for c in cases})!=len(cases):raise ValueError('duplicate case id')
    out=args.out or ROOT/protocol['run_dir']/args.split
    out.mkdir(parents=True,exist_ok=True)
    identity={'protocol_sha256':digest(args.protocol),'source_sha256':source_id,'environment':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__}}
    manifest={'identity':identity,'source_files':source,'git_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'workers':args.workers,'cases':len(cases),'started_utc':__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()}
    old=out/'manifest.json'
    if old.exists() and json.loads(old.read_text())['identity']!=identity:raise RuntimeError('existing run has different identity; use a new directory')
    dump(old,manifest);jobs=[];results=[]
    for c in cases:
        case_identity={**identity,'case_sha256':encode(c)};folder=out/c['case_id'];file=folder/'case_result.json'
        if file.exists():
            if not args.resume:raise RuntimeError('existing case requires --resume or independent directory')
            cached=json.loads(file.read_text())
            if cached['identity']!=case_identity:raise RuntimeError('case identity mismatch')
            if not all((folder/p).is_file() and digest(folder/p)==h for p,h in cached['products'].items()):raise RuntimeError('cached product integrity failure')
            results.append(cached['result']);continue
        if folder.exists() and any(folder.iterdir()):raise RuntimeError('partial case preserved; move it to an archive before retry')
        jobs.append((c,str(folder),case_identity))
    sampler=ResourceSampler(out/'resources.csv');sampler.start();start=time.perf_counter()
    try:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures={pool.submit(worker,j):j for j in jobs}
            for future in as_completed(futures):
                try: result=future.result()
                except Exception as exc:
                    c,folder,ident=futures[future];result={'case_id':c['case_id'],'shape':c['shape'],'variant':c['variant'],'seed':c['seed'],'execution_status':'PROCESS_ERROR','error':repr(exc),'task_completed':False,'g500_pass':False,'settled':False}
                    dump(Path(folder)/'case_result.json',{'identity':ident,'products':{},'result':result})
                results.append(result);dump(out/'summary.json',{'manifest':manifest,'results':results,'completed_denominator':len(results),'planned_denominator':len(cases)})
                print(json.dumps({'case':result['case_id'],'status':result['execution_status'],'seconds':result.get('wall_seconds')}),flush=True)
    finally:sampler.stop()
    dump(out/'summary.json',{'manifest':manifest,'results':sorted(results,key=lambda r:r['case_id']),'planned_denominator':len(cases),'completed_denominator':len(results),'elapsed_seconds':time.perf_counter()-start})
if __name__=='__main__':main()
