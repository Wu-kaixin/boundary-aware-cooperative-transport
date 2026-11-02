"""Plot only saved same-input readouts; no dynamics, interpolation or smoothing."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_displacement_replay import ROOT, MISSION, sha
from dbact_sim.experiments.deployment import dump


def main():
    folder=ROOT/'artifacts/research'/MISSION/'offline-reference'
    if (folder/'estimation_comparison.pdf').exists():raise RuntimeError('Figure exists; preserve it')
    with np.load(folder/'reference_readout.npz',allow_pickle=False) as z:
        t,truth,old,new=[z[k].copy() for k in ('time','truth','old','shared')]
    plt.rcParams.update({'font.size':8,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axes=plt.subplots(3,1,figsize=(7.16,6.1),layout='constrained',sharex=True)
    for a,ax in enumerate(axes[:2]):
        ax.plot(t,truth[:,a],color='k',lw=1.1,label='Physical displacement (evaluation only)')
        ax.plot(t,old[:,a],color='#009E73',ls='--',lw=.7,label='Accumulated map + gain')
        ax.plot(t,new[:,a],color='#CC79A7',lw=.8,label='Persistent observed reference')
        ax.set_ylabel(('Task-axis' if a==0 else 'Lateral')+' displacement (m)')
    axes[0].legend(fontsize=7,loc='upper left')
    axes[2].plot(t,np.linalg.norm(old-truth,axis=1),color='#009E73',ls='--',lw=.7,label='Original error')
    axes[2].plot(t,np.linalg.norm(new-truth,axis=1),color='#CC79A7',lw=.8,label='Reference error')
    axes[2].set_ylabel('Displacement error norm (m)');axes[2].set_xlabel('Physical time (s)')
    axes[2].legend(fontsize=7)
    for ax in axes:
        ax.axvspan(0,30,color='gray',alpha=.13);ax.grid(alpha=.2)
    fig.suptitle('Same recorded C-away trajectory; one development episode; no closed-loop claim',fontsize=9)
    for ext in ('pdf','svg','png'):fig.savefig(folder/f'estimation_comparison.{ext}',dpi=220)
    plt.close(fig)
    dump(folder/'figure_provenance.json',dict(input_sha256=sha(folder/'reference_readout.npz'),
        script_sha256=sha(Path(__file__)),statistics_unit='One selected historical development episode; frames are dependent',
        processing='All saved frames; no smoothing, interpolation, resampling or missing-value imputation; gray area excluded from frozen MAE window',
        caption='Same-input replay of the recorded C-open-away vector-CVT episode. Black curves are physical displacement used only for evaluation. Green dashed curves show the original shared accumulated-map readout; purple curves show the persistent observed-reference readout on verified reconstructed local observations. The frozen 30--600 s window has mean vector errors of 281.9 and 12.0 mm, respectively. These are within-trajectory descriptive errors, not independent experimental samples or closed-loop completion results.'))


if __name__=='__main__':main()
