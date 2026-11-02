"""Render all frozen parameter-screen outcomes, including negative results."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch
from prepare_transport_tuning import ROOT, MISSION, sha

BASE=ROOT/'artifacts/research'/MISSION
NAMES={'reference':'Original','slower_reference':'Slower\nreference',
       'lower_effort':'Lower\npress speed','slower_deployment':'Slower\ndeployment',
       'tighter_registration':'Tighter\nmatching','wider_motion_prior':'Wider\nmotion prior'}
CASES={'near_circle':'N: circle','near_ellipse_parallel':'N: ellipse parallel',
       'near_rectangle_parallel':'N: rectangle parallel','near_c_open_away':'N: C, opening away',
       'near_u_tray_s0_r90':'N: U-tray s0 / 90 deg','near_t_load_s0_r90':'N: T-load s0 / 90 deg',
       'far_skew_box_s0_r0_pos0':'F: skew box pos0 (unknown)',
       'far_c_open_away_pos1':'F: C pos1 (initial hint)'}

def read(p):return json.loads(p.read_text(encoding='utf-8'))

def save(fig,name,out):
    for suffix in ('.svg','.png'):
        p=out/(name+suffix)
        if p.exists():raise RuntimeError('Refuse to overwrite figure')
        fig.savefig(p,dpi=220,bbox_inches='tight')
    plt.close(fig)

def main():
    out=ROOT/'docs/results/transport_tuning_v1'
    out.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'svg.fonttype':'none'})
    x=read(BASE/'screen/analysis.json');arms=list(x['totals']);conditions=list(dict.fromkeys(r['condition'] for r in x['episodes']))
    lookup={(r['condition'],r['arm']):r for r in x['episodes']}
    levels=np.zeros((len(conditions),len(arms)))
    colors=['#eeeeee','#b9d9ed','#b2d8c2','#f7c6a3','#ed8d94']
    fig,ax=plt.subplots(figsize=(8.2,5.25))
    for i,c in enumerate(conditions):
        for j,a in enumerate(arms):
            r=lookup[c,a]
            status=4 if r['execution_status']!='COMPLETED' else 3 if r.get('hard_safety') else 2 if r['safe_completed'] and r['g500'] else 1 if r['safe_completed'] else 0
            levels[i,j]=status
            flags=('S' if r['safe_completed'] else 'F')+(' / G' if r['g500'] else ' / -')
            if r.get('hard_safety'):flags='H / -'
            err=r.get('endpoint_error_m')
            label=flags+'\n'+('error' if err is None else f'{err:.3f} m')
            ax.text(j,i,label,ha='center',va='center',fontsize=7.2)
    ax.imshow(levels,cmap=ListedColormap(colors),norm=BoundaryNorm(np.arange(-.5,5.5),5),aspect='auto')
    labels=[]
    for a in arms:
        t=x['totals'][a]
        labels.append(NAMES[a]+f"\nS {t['safe_completed']}/8; G {t['g500']}/8")
    ax.set_xticks(range(len(arms)),labels);ax.xaxis.tick_top();ax.tick_params(axis='x',length=0,pad=6)
    ax.set_yticks(range(len(conditions)),[CASES[c] for c in conditions]);ax.tick_params(axis='y',length=0,pad=6)
    ax.set_xticks(np.arange(-.5,len(arms)),minor=True);ax.set_yticks(np.arange(-.5,len(conditions)),minor=True)
    ax.grid(which='minor',color='white',linewidth=2);ax.tick_params(which='minor',length=0)
    ax.set_title('Fixed parameter development: 8 paired conditions x 6 settings',pad=53,fontsize=10)
    ax.set_xlabel('S: safe task completion; G: G500 pass; F: incomplete; H: hard-safety violation.\nNumbers are final 2-D goal errors. One reused seed; 600 s maximum; no confidence interval.',labelpad=9,fontsize=7.5)
    handles=[Patch(facecolor=colors[i],label=l) for i,l in enumerate(['Incomplete','Safe completion only','Safe completion + G500','Hard-safety failure'])]
    ax.legend(handles=handles,loc='upper center',bbox_to_anchor=(.4,-.16),ncol=2,frameon=False,fontsize=7)
    save(fig,'parameter_screen',out)
    audit=read(BASE/'reference_point_audit.json');data=list(audit['cases'].items())
    fig,ax=plt.subplots(figsize=(7.0,3.25));y=np.arange(len(data));h=.32
    actual=[v['final']['actual_error_m'] for _,v in data]
    ideal=[v['final']['perfect_registration_error_m'] for _,v in data]
    ax.barh(y-h/2,actual,h,label='Recorded readout',color='#0072B2')
    ax.barh(y+h/2,ideal,h,label='Ideal registration at same reference points',color='#E69F00',hatch='//')
    labs=['N: circle','N: ellipse parallel','N: C opening away','N: U-tray s0 / 90 deg',
          'F: skew box pos0 (unknown)','F: C pos1 (initial hint)']
    ax.set_yticks(y,labs);ax.invert_yaxis();ax.set_xscale('log')
    ax.set_xlabel('Final available control-frame displacement error (m; log scale)')
    ax.set_title('Offline reference-point audit of six historical cases',fontsize=10)
    ax.grid(axis='x',which='both',alpha=.15);ax.set_axisbelow(True)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.20),fontsize=7,frameon=False)
    save(fig,'reference_point_error',out)
    manifest=dict(figures=[dict(name='parameter_screen',inputs={'analysis':sha(BASE/'screen/analysis.json')},
        caption='All 48 fixed-budget development episodes. Each condition uses the same initial state across six settings. Cell labels distinguish safe task completion, the unchanged G500 contract, and hard-safety failure; endpoint errors are reported for failures as well. These eight selected conditions and one reused seed do not estimate population reliability.'),
        dict(name='reference_point_error',inputs={'audit':sha(BASE/'reference_point_audit.json')},
        caption='Offline errors at the last available control readout in six historical episodes. The counterfactual uses exact rigid motion of each initialized material reference point and the original accepted voter sets. It is not a new controller or physical trajectory; it separates reference-point semantics from registration residuals. The horizontal axis is logarithmic.')],
        script_sha256=sha(Path(__file__)),artifacts={p.name:sha(p) for p in out.iterdir() if p.suffix in ('.svg','.png')},
        smoothing='None',sampling='All48screenepisodes; lastavailablecontrolreadout for the six preselected offline cases',
        confidence_intervals='Not used: selected one-seed development cases, not independent population samples')
    path=out/'figure_manifest.json'
    if path.exists():raise RuntimeError('Manifest already exists')
    path.write_text(json.dumps(manifest,indent=2),encoding='utf-8')

if __name__=='__main__':main()
