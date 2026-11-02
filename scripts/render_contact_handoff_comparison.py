"""Two explicitly post-hoc illustrations from frozen data, using the existing renderer.

Circle is a failure illustration; rectangle-parallel illustrates retained task
completion. Selection is descriptive, not additional experimental evidence.
The full nine-condition tables/trajectories remain the comparison denominator.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import matplotlib.pyplot as plt

from build_ccta_media import video, ensure_mp4_export_available
from dbact_sim.experiments.study import cached_result
from dbact_sim.experiments.deployment import dump
from freeze_contact_handoff_study import ROOT, MISSION, read, sha


def main():
    batch=ROOT/f'runs/research/{MISSION}/main'
    out=ROOT/f'artifacts/research/{MISSION}/media'
    out.mkdir(parents=True,exist_ok=True)
    receipt_path=out/'receipts.json'
    receipts=read(receipt_path) if receipt_path.exists() else []
    ensure_mp4_export_available()
    for condition in ('circle','rectangle_parallel'):
        ids=[f'{condition}_{arm}' for arm in ('legacy','spacing','handoff')]
        for name in ids:
            receipt=read(batch/name/'case_result.json')
            cached_result(batch/name,receipt['identity'])
        path=out/f'{condition}.mp4'
        previous=next((r for r in receipts if r['video']==path.name),None)
        if previous:
            assert sha(path)==previous['sha256']
            assert all(sha(batch/name/'replay.npz')==digest for name,digest in previous['replay_hashes'].items())
        else:
            record=video(batch,ids,path)
            record['selection']='Post-hoc failure/retained-completion illustrations; not extra samples or unbiased selection'
            record['wrapper_sha256']=sha(Path(__file__))
            receipts.append(record)
            dump(receipt_path,receipts)
        gif=path.with_suffix('.gif')
        if not gif.exists():
            subprocess.run([plt.rcParams['animation.ffmpeg_path'],'-n','-i',str(path),
                '-vf','fps=4,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
                '-loop','0',str(gif)],check=True,capture_output=True)
        print(condition,flush=True)
    dump(out/'manifest.json',{'files':{p.name:sha(p) for p in out.iterdir() if p.is_file() and p.name!='manifest.json'},
                             'new_physics_episodes':0,'selection':'Post-hoc illustrations; all nine cases reported elsewhere'})


if __name__=='__main__':
    main()
