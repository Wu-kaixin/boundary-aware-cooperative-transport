"""Read-only affine ancestry of actual map cells; reject unaccounted movement."""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain


def keys(points, objects, voxel, index):
    cells=np.rint(np.asarray(points).reshape(-1,2)/voxel).astype(np.int64)
    codes=np.array([index[str(o)] for o in objects],dtype=np.int64)
    return codes*(1<<42)+(cells[:,0]+(1<<20))*(1<<21)+cells[:,1]+(1<<20)


class Lineage:
    def __init__(self):
        self.nodes=[]
        self.current={}
        self.checked_cells=0

    def add(self, point, parents, delta, frame, stage, **extra):
        self.nodes.append(dict(point=np.asarray(point).tolist(),parents=parents,
            delta=np.asarray(delta).tolist(),frame=frame,stage=stage,**extra))
        return len(self.nodes)-1

    def select(self, state):
        selected={}
        for key,point in zip(state['keys'],state['points']):
            node=self.current[int(key)]
            np.testing.assert_allclose(self.nodes[node]['point'],point,atol=1e-12,rtol=0)
            selected[int(key)]=node
            self.checked_cells+=1
        self.current=selected

    def align(self,before,after,delta,voxel,index,frame):
        self.select(before)
        delta=np.asarray(delta)
        points=np.asarray(before['points']).reshape(-1,2)
        shifted_keys=keys(points+delta,before['objects'],voxel,index)
        weights=np.asarray(before['weights'])
        prior=self.current.copy()
        output={}
        for key,point in zip(after['keys'],after['points']):
            picked=np.flatnonzero(shifted_keys==key)
            assert len(picked)
            shares=weights[picked]/sum(weights[picked])
            expected=np.sum(points[picked]*shares[:,None],axis=0)+delta
            np.testing.assert_allclose(expected,point,atol=1e-12,rtol=0)
            parents=[(prior[int(before['keys'][j])],float(w)) for j,w in zip(picked,shares)]
            output[int(key)]=self.add(point,parents,delta,frame,'alignment')
        self.current=output

    def update(self,before,scan,codes,after,voxel,index,frame,gaps=None):
        self.select(before)
        points=np.asarray(scan['points']).reshape(-1,2)
        codes=np.asarray(codes)
        confidence=np.asarray(scan['confidence'])
        objects=np.asarray(scan['object_ids'])
        # All packets have the same timestamp in this BoundaryView update.
        signature=np.column_stack([codes.astype(float),points])
        _,unique=np.unique(signature,axis=0,return_index=True)
        unique=np.sort(unique)
        cell_keys=keys(points[unique],objects[unique],voxel,index)
        incoming={}
        for key in np.unique(cell_keys):
            picked=unique[cell_keys==key]
            weights=np.maximum(confidence[picked],1e-6)
            total=float(sum(weights))
            point=np.sum(points[picked]*weights[:,None],axis=0)/total
            samples=[dict(point=points[j].tolist(),source_code=int(codes[j]),
                confidence=float(confidence[j]),share=float(w/total),
                true_gap=None if gaps is None else float(gaps[j])) for j,w in zip(picked,weights)]
            incoming[int(key)]=(self.add(point,[],[0.,0.],frame,'observation',samples=samples),total)
        prior_weight=dict(zip(map(int,before['keys']),map(float,before['weights'])))
        for key,(node,total) in incoming.items():
            if key in self.current:
                old=self.current[key]
                old_weight=prior_weight[key]
                share=old_weight/(old_weight+total)
                fresh_share=total/(old_weight+total)
                point=(old_weight*np.asarray(self.nodes[old]['point'])+
                       total*np.asarray(self.nodes[node]['point']))/(old_weight+total)
                node=self.add(point,[(old,share),(node,fresh_share)],[0.,0.],frame,'fusion')
            self.current[key]=node
        self.select(after)

    def explain(self,key):
        root=self.current[key]
        coefficients={root:1.}
        observations,alignments=[],[]
        reconstructed=np.zeros(2)
        translation=np.zeros(2)
        for i in range(root,-1,-1):
            coefficient=coefficients.get(i,0.)
            if coefficient==0.:
                continue
            node=self.nodes[i]
            if node['stage']=='observation':
                observations.append(dict(node_id=i,coefficient=coefficient,**node))
                reconstructed+=coefficient*np.asarray(node['point'])
            else:
                contribution=coefficient*np.asarray(node['delta'])
                translation+=contribution
                reconstructed+=contribution
                if node['stage']=='alignment' and np.any(node['delta']):
                    alignments.append(dict(node_id=i,coefficient=coefficient,**node))
            for parent,share in node['parents']:
                coefficients[parent]=coefficients.get(parent,0.)+coefficient*share
        observations.sort(key=lambda x:x['node_id'])
        alignments.sort(key=lambda x:x['node_id'])
        np.testing.assert_allclose(reconstructed,self.nodes[root]['point'],atol=2e-11,rtol=0)
        assert abs(sum(x['coefficient'] for x in observations)-1.)<1e-12
        return dict(point=self.nodes[root]['point'],reconstructed_point=reconstructed.tolist(),
            translation_contribution=translation.tolist(),observations=observations,alignments=alignments,
            contributing_nodes=len(coefficients),total_nodes=len(self.nodes),checked_cells=self.checked_cells)


def analyze(source,out):
    guard()
    from dbact_sim.experiments.deployment import dump
    outcome=read(source/'outcome.json')
    assert outcome['status']=='EXACT_RECONSTRUCTION' and outcome['verified_frames']==989
    assert sha(source/'map_stages.jsonl.gz')==outcome['product_sha256']
    out=out.resolve()
    assert out.is_relative_to(ROOT/'runs')
    out.mkdir(parents=True,exist_ok=False)
    graph=Lineage()
    events=[]
    frame_summaries={}
    with gzip.open(source/'map_stages.jsonl.gz','rt',encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line)
            frame=row['frame']
            graph.select(row['at_start'])
            if 'alignment' in row:
                delta=np.asarray(row['alignment']['displacement'])-row['prior_displacement']
                graph.align(row['before_align'],row['after_align'],delta,
                    row['voxel_size'],row['object_index'],frame)
                assert row['register_kwargs']=={'apply_motion':False}
                graph.select(row['after_register'])
                if 'after_carve' in row:
                    graph.select(row['after_carve'])
                assert row['update_kwargs']=={'integrate_motion':False}
                graph.update(row['before_update'],row['one_hop_scan'],row['source_codes'],
                    row['after_update'],row['voxel_size'],row['object_index'],frame,
                    row['stage_true_gaps'].get('one_hop_scan'))
            else:
                delta=np.zeros(2)
            graph.select(row['at_end'])
            summary={k:row[k] for k in ('frame','time','phase','acquisition_status','true_center',
                'true_angle','reference_state','fresh_readings','diagnostics')}
            summary['delta']=delta.tolist()
            frame_summaries[frame]=summary
            if np.linalg.norm(delta)>.02:
                events.append(summary)
            if frame%200==0:
                print(json.dumps(dict(checked_frames=frame,nodes=len(graph.nodes))),flush=True)
    witness=read(ROOT/'docs/results/pair_residual_v1/map_point_geometry.json')
    point=np.asarray(witness['selected_map_points'][0])
    points=np.asarray(row['at_end']['points'])
    chosen=int(np.argmin(np.linalg.norm(points-point,axis=1)))
    np.testing.assert_array_equal(points[chosen],point)
    explanation=graph.explain(int(row['at_end']['keys'][chosen]))
    touched={x['frame'] for x in explanation['observations']+explanation['alignments']}
    details={str(k):frame_summaries[k] for k in sorted(touched)}
    result=dict(status='VERIFIED_AFFINE_ANCESTRY',script_sha256=sha(__file__),
        trace_outcome_sha256=sha(source/'outcome.json'),selected_witness_sha256=sha(
            ROOT/'docs/results/pair_residual_v1/map_point_geometry.json'),
        replay_frames=989,new_physical_episodes=0,explanation=explanation,
        selected_cell_timestamp=row['at_end']['timestamps'][chosen],
        selected_cell_confidence=row['at_end']['confidence'][chosen],
        selected_cell_weight=row['at_end']['weights'][chosen],
        selected_cell_true_gap=row['stage_true_gaps']['at_end'][chosen])
    dump(out/'ancestry.json',result)
    dump(out/'contributing_frames.json',details)
    dump(out/'large_alignment_events.json',events)
    dump(out/'outcome.json',dict(status=result['status'],products={name:sha(out/name) for name in
        ('ancestry.json','contributing_frames.json','large_alignment_events.json')}))
    print(json.dumps(plain({k:v for k,v in result.items() if k!='explanation'})),flush=True)
    print(json.dumps(dict(observations=[{k:v for k,v in x.items() if k in ('frame','coefficient','point')}
        for x in explanation['observations']],translation=explanation['translation_contribution'])),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    analyze(args.source,args.out)
