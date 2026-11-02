"""Preflight then freeze shape and location studies separately, without physics."""
import argparse
import copy
import json
import sys
import numpy as np
from shapely.geometry import Polygon
from dbact.cargo import Cargo
from dbact.geometry import polygon_diameter, rotate, signed_distance_to_polygon
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.shapes import CARGO_FAMILIES, cargo_outline
from dbact_sim.experiments.study import source_hashes, environment_versions
from prepare_cargo_discovery import ROOT, MISSION, AUDIT, sha

METHODS = ('boundary_cvt','local_spacing','nearest_observed_target','detection_density_cvt')

def read(p):return json.loads(p.read_text(encoding='utf-8'))

def metadata(cfg, condition, shape, stratum, extra=None):
    env=SimulationEnvironment(copy.deepcopy(cfg),seed=2)
    cargo=env.cargoes[0]
    initial=[a.position.tolist() for a in env.agents]
    polygon=Polygon(cargo.vertices)
    assert polygon.is_valid and polygon.area > 0 and not polygon.interiors
    xmin,xmax,ymin,ymax=env.domain
    assert np.all(cargo.vertices >= [xmin,ymin]) and np.all(cargo.vertices <= [xmax,ymax])
    initial_returns=[len(env.controller.sensor.sense_view(a,env.cargoes,0.)) for a in env.agents]
    if stratum == 'far':assert sum(initial_returns)==0
    task=env.tasks[cargo.object_id].as_dict()
    return dict(condition=condition,shape=shape,stratum=stratum,
        area_m2=cargo.area,mass_kg=cargo.mass,inertia_kg_m2=cargo.inertia,
        perimeter_m=cargo.perimeter,diameter_m=float(polygon_diameter(cargo.vertices)),
        reach_m=float(np.linalg.norm(cargo.local_vertices,axis=1).max()),
        vertex_count=len(cargo.vertices),initial_centroid=cargo.position.tolist(),task=task,
        initial_own_scan_returns=initial_returns,
        minimum_initial_boundary_distance_m=float(np.min(signed_distance_to_polygon(np.asarray(initial),cargo.vertices))),
        initial_tokens='explicit initial position packet' if 'experiment_initial_location_hint' in cfg else 'empty',
        agent_initial_sha256=encode(initial),
        physical_initial_sha256=encode(dict(robots=initial,vertices=cargo.vertices.tolist(),task=task)),
        **(extra or {}))

def case(cfg, meta, method, arm=None):
    arm=arm or method
    return dict(case_id=meta['condition']+'_'+arm,condition_id=meta['condition'],shape=meta['shape'],
                split='development',stratum=meta['stratum'],gate=6,seed=2,frames=12000,
                variant=method,arm=arm,config=copy.deepcopy(cfg),factors=meta,
                expected_initial_state_sha256=meta['agent_initial_sha256'])

def build():
    old=read(ROOT/'experiments/ccta2027/contact_consistency_v1.json')
    originals=[c for c in old['cases'] if c['arm']=='consistent_aggregate']
    assert len(originals)==9
    base=copy.deepcopy(originals[0]['config'])
    assert not base['controller'].get('reference_innovation_trim',False)
    area=Cargo('probe',np.asarray(base['cargoes'][0]['vertices'])).area
    density=base['cargoes'][0]['surface_density']
    outlines={}
    for family in CARGO_FAMILIES:
        for setting in (0,1):
            probe=Cargo('probe',cargo_outline(family,setting))
            outlines[(family,setting)]=probe.local_vertices*np.sqrt(area/probe.area)
    reach=max(float(np.linalg.norm(v,axis=1).max()) for v in outlines.values())
    near=[]; near_meta=[]; configs={}
    for origin in originals:
        cfg=copy.deepcopy(origin['config']); name=origin['condition_id']
        meta=metadata(cfg,name,origin['shape'],'near',dict(cohort='historical_nine',
            origin_case=origin['case_id'],geometry_setting='historical frozen definition'))
        near_meta.append(meta); configs[name]=cfg
        for method in METHODS:near.append(case(cfg,meta,method))
    for (family,setting),outline in outlines.items():
        for angle in (0,90):
            name=f'{family}_s{setting}_r{angle}'
            cfg=copy.deepcopy(base)
            cfg['agents'].update(center=[4.,4.],radius_min=reach+.35,radius_max=reach+1.15)
            cfg['cargoes']=[dict(id='cargo_0',shape='polygon',
                vertices=(rotate(outline,np.deg2rad(angle))+[4.,4.]).tolist(),
                surface_density=density,movable=True)]
            meta=metadata(cfg,name,family,'near',dict(cohort='new_cargo_24',
                geometry_setting=setting,angle_deg=angle,
                normalization='equal polygon area and surface density; inertia/perimeter/span vary',
                geometry_source='src/dbact_sim/experiments/shapes.py'))
            near_meta.append(meta);configs[name]=cfg
            for method in METHODS:near.append(case(cfg,meta,method))
    assert len(near)==132
    far=[];far_meta=[]
    for name in ('circle','skew_box_s0_r0','c_open_away'):
        source_meta=next(m for m in near_meta if m['condition']==name)
        for position_id,center in enumerate(([3.1,5.6],[4.8,5.6])):
            cfg=copy.deepcopy(configs[name])
            cfg['agents']=dict(count=16,layout='grid',center=[4.,.95],spacing=.36,jitter=0.)
            cfg['require_initial_ignorance']=True
            cfg['cargoes'][0]['vertices']=(np.asarray(cfg['cargoes'][0]['vertices'])-[4.,4.]+center).tolist()
            condition=f'{name}_pos{position_id}'
            meta=metadata(cfg,condition,source_meta['shape'],'far',dict(cohort='location_pairs',
                geometry_from_near=name,position_id=position_id,
                normalization='same physical state across location-information arms; geometry remains unknown'))
            far_meta.append(meta)
            far.append(case(cfg,meta,'boundary_cvt','unknown_location'))
            known=copy.deepcopy(cfg)
            known['experiment_initial_location_hint']=dict(object_id='cargo_0',position=center)
            known_meta=metadata(known,condition,source_meta['shape'],'far',dict(cohort='location_pairs',
                geometry_from_near=name,position_id=position_id,
                normalization='one initial location packet, no geometry or ongoing pose oracle'))
            assert known_meta['physical_initial_sha256']==meta['physical_initial_sha256']
            far.append(case(known,known_meta,'boundary_cvt','initial_location_hint'))
    assert len(far)==12
    return dict(near=(near,near_meta),far=(far,far_meta))

def freeze(dry_run=False):
    assert PathGuard()
    plans=build()
    if dry_run:
        print(json.dumps({name:dict(cases=len(cases),conditions=len(meta),
            minimum_initial_distance=min(m['minimum_initial_boundary_distance_m'] for m in meta),
            area_range=[min(m['area_m2'] for m in meta),max(m['area_m2'] for m in meta)])
            for name,(cases,meta) in plans.items()}));return
    for stage in ('baseline','after2','shadow'):
        assert (ROOT/'runs/maintenance'/MISSION/f'{stage}.exit.txt').read_text(encoding='utf-8-sig').strip()=='0'
    shadow=read(ROOT/'artifacts/research'/MISSION/'shadow/analysis.json')
    assert shadow['status']=='FAIL', 'A passed candidate requires the separately frozen 18-case stage first'
    source=source_hashes(); receipts={}
    for name,(cases,meta) in plans.items():
        dest=ROOT/f'experiments/ccta2027/cargo_{name}_v1.json'
        if dest.exists():raise RuntimeError('Existing frozen protocol: do not overwrite')
        plan=dict(version=f'cargo-{name}-v1',status='FROZEN_BEFORE_EXECUTION',cases=cases,conditions=meta,
            run_dir=f'runs/research/{MISSION}/{name}',frozen_source_sha256=encode(source),
            budget=dict(episodes=len(cases),workers=8,internal_threads=1,seed=2,physical_seconds=600,
                        frames=12000,wall_seconds=14400,additional_episodes=0,retuning=False),
            environment=environment_versions(),
            primary='Original complete task AND hard safety; all planned/started cases retained',
            completion='Original HOLD + physical speed<=.005 for40frames, endpoint<=.12m; no new angular requirement',
            secondary=['completion separate from G500','full endpoint vector error','physical duration, censored != completion time',
                       'first actual sensing / CONTACT_READY / actual contact / TRANSPORT / BRAKE / HOLD',
                       'coverage and max gap','robot path length','hard safety and solver fallback'],
            analysis='Paired condition comparisons; one reused development seed. No population success probability, no per-frame sample inflation.',
            information=dict(shape_prior=False,control='own/one-hop sensing; existing common team phase supervisor',
                truth='sensor simulation, physics and independent evaluation only',
                far_ablation='Explicit one-shot initial-location packet in labelled hint arm; TTL unchanged'),
            hypotheses=(dict(deployment='Boundary support may improve deployment and task completion over the other support/target choices',
                decision='Claim a development advantage only if net safe-completion gain >=3/33 against EACH control, no added hard-safety cases; disclose all pairwise losses. Otherwise no uniform superiority claim.')
                if name=='near' else dict(search='Unknown-location sweep+relay can reach detection, contact and completion from zero initial returns',
                decision='Report detection and end-task completion separately; hint arm isolates initial-location information on the identical physical state. No guarantee of general search coverage.')),
            common_controller='Previous consistent-aggregate + observed-reference + radial-pivot + own-scan-distance; failed innovation trim disabled',
            scope='New development evidence; not independent validation or arbitrary-shape reliability',
            prior_shadow_sha256=sha(ROOT/'artifacts/research'/MISSION/'shadow/analysis.json'),
            freeze_script_sha256=sha(Path(__file__)),design_sha256=sha(ROOT/'docs/CARGO_DISCOVERY_RESEARCH_2026-11-02.md'))
        dump(dest,plan);load_protocol(dest,'development',ROOT)
        receipts[name]=dict(protocol=dest.relative_to(ROOT).as_posix(),sha256=sha(dest),cases=len(cases))
    dump(AUDIT/'freeze_receipt.json',receipts)
    state=read(AUDIT/'mission_state.json');state.update(stage='NEAR_AND_FAR_FROZEN',protocols=receipts,
        candidate_decision='Offline gate failed; no candidate physics episodes')
    dump(AUDIT/'mission_state.json',state)
    print(json.dumps(receipts))

def PathGuard():
    from pathlib import Path
    return Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'

from pathlib import Path
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');freeze(p.parse_args().dry_run)
