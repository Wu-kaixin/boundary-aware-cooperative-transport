"""Publish the selected ghost's verified ancestry, not a new controller trial."""
from __future__ import annotations

import csv
import json
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from research_io import ROOT, guard, read, sha


def main():
    guard()
    raw = ROOT/'runs/research/2026-11-03-map-provenance-v1'
    public = ROOT/'docs/results/map_provenance_v1'
    data = read(raw/'ancestry.json')
    outcome = read(raw/'trace/outcome.json')
    assert outcome['status']=='EXACT_RECONSTRUCTION' and outcome['verified_frames']==933
    assert data['trace_sha256']==outcome['product_sha256']==sha(raw/'trace/map_stages.jsonl.gz')
    assert sha(raw/'frozen_runner.py')==outcome['script_sha256']
    public.mkdir(parents=True, exist_ok=False)
    for source, name in ((raw/'ancestry.json','ancestry.json'),
                         (raw/'trace/outcome.json','trace_outcome.json'),
                         (raw/'trace/frozen_trace.json','trace_identity.json')):
        shutil.copy2(source, public/name)
    with (public/'all_412_shifts.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['frame','time','dx_m','dy_m','active_ancestors','reference_status'])
        writer.writeheader()
        for row in data['steps']:
            writer.writerow(dict(frame=row['frame'],time=row['time'],dx_m=row['registration_translation'][0],
                dy_m=row['registration_translation'][1],active_ancestors=row['active_ancestors'],reference_status=row['reference_status']))
    anchor = np.array(data['weighted_anchor_point'])
    map_path = np.vstack([anchor, anchor+np.cumsum([r['registration_translation'] for r in data['steps']], axis=0)])
    start_reference = np.array(data['steps'][-1]['reference_displacement'])-data['reference_displacement_change']
    reference = np.array([r['reference_displacement'] for r in data['steps']],dtype=float)
    reference_path = anchor+reference-start_reference
    fig, axes = plt.subplots(1,2,figsize=(10.6,4.5),gridspec_kw={'width_ratios':[1.5,1]})
    fig.subplots_adjust(left=.075,right=.98,top=.86,bottom=.25,wspace=.35)
    ax=axes[0]
    ax.plot(map_path[:,0],map_path[:,1],color='#d55e00',label='Stored point: original map transport')
    ax.plot(reference_path[:,0],reference_path[:,1],color='#0072b2',alpha=.65,label='Same anchor: accepted local reference')
    ax.scatter(*anchor,color='black',marker='o',s=25,label='Last fresh support (25.2 s)')
    ax.scatter(*data['selected_point'],color='#d55e00',marker='s',s=40)
    ax.scatter(*data['reference_repositioned_point'],color='#0072b2',marker='s',s=40)
    ax.scatter(*data['true_material_repositioned_point'],color='black',marker='x',s=55,label='True material position at 45.8 s')
    ax.set(xlabel='World x (m)',ylabel='World y (m)',title='412 shifts; no new support, fusion or carving')
    ax.set_aspect('equal',adjustable='datalim')
    ax.legend(fontsize=7,loc='best',frameon=False)
    values=np.array([data['current_true_boundary_distance_m'],data['reference_repositioned_boundary_distance_m']])*1000
    bars=axes[1].bar(['Original map','Reference placement'],values,color=['#d55e00','#0072b2'],width=.55)
    axes[1].bar_label(bars,fmt='%.3f',padding=4,fontsize=9)
    axes[1].set(ylabel='Distance to current true boundary (mm)',ylim=(0,max(values)*1.18),title='One selected historical point')
    for ax in axes:
        ax.grid(alpha=.2);ax.set_axisbelow(True);ax.spines[['top','right']].set_visible(False)
    fig.suptitle('Boundary-map provenance: a retained point follows the wrong motion',fontsize=12)
    fig.text(.5,.10,'Retrospective geometry diagnostic on one known failure. No new physics or validated controller outcome.',ha='center',fontsize=8)
    fig.text(.5,.06,'Boundary distance is not material-point error: reference placement still has 22.973 mm material error.',ha='center',fontsize=8)
    for ext in ('png','pdf'):fig.savefig(public/f'point_ancestry.{ext}',dpi=180)
    plt.close(fig)
    products=['ancestry.json','trace_outcome.json','trace_identity.json','all_412_shifts.csv','point_ancestry.png','point_ancestry.pdf']
    manifest=dict(publisher_sha256=sha(__file__),outputs={name:sha(public/name) for name in products},new_physical_episodes=0)
    (public/'publication.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(published=str(public),trace_frames=933,ancestry_shifts=412,new_physical_episodes=0)))


if __name__=='__main__':main()
