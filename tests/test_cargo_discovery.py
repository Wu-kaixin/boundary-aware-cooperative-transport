"""Mechanism and information-permission checks, not task-success evidence."""
from types import SimpleNamespace
from dataclasses import replace
import numpy as np
from dbact.control.observed_reference import ObservedReferenceTracker
from dbact.deployment.density import BoundaryAwareDensity, DensityParams
from dbact.deployment.variants import DeploymentVariant, detection_density
from dbact.types import BoundaryView

def test_innovation_trim_preserves_rare_face_explained_by_rigid_translation():
    points=np.vstack([np.c_[np.linspace(-.4,.4,80),np.zeros(80)],
                      np.c_[np.full(8,.5),np.linspace(.1,.4,8)]])
    normals=np.vstack([np.tile([0.,-1.],(80,1)),np.tile([1.,0.],(8,1))])
    scan=BoundaryView(points,normals,np.ones(88),np.ones(88)*.01,np.full(88,'cargo_0'))
    old=ObservedReferenceTracker();new=ObservedReferenceTracker(innovation_trim=True)
    for tracker in (old,new):tracker.initialize(scan,[0.,0.],0.)
    moved=replace(scan,points=points+[.04,0.])
    assert old.update(moved,.2) is None
    result=new.update(moved,.2)
    assert result is not None
    np.testing.assert_allclose(result.displacement,[.04,0.],atol=1e-8)

def test_detection_density_changes_support_not_mass_width_or_background():
    base=BoundaryAwareDensity([[4.,4.],[4.,5.]],[[1.,0.],[1.,0.]],[2.,3.],DensityParams())
    changed=detection_density(base,[[3.,4.],[3.,5.]])
    np.testing.assert_array_equal(changed.targets,[[3.,4.],[3.,5.]])
    assert changed.total_mass == base.total_mass
    assert changed.params.sigma == base.params.sigma
    assert changed.params.base_density == base.params.base_density

def test_detection_variant_excludes_remote_packets_and_preserves_guards():
    from dbact.control.phase import PhaseMonitor
    from dataclasses import dataclass
    @dataclass
    class Cell:
        centroid: np.ndarray
        mass: float = 12.
        redeploy: bool = True
    class Base:
        def __init__(self):self.seen=[]
        def compute(self,i,agents,neighbors,density,domain):
            self.seen.append(density)
            return Cell(np.mean(density.targets,axis=0))
    base=Base()
    controller=SimpleNamespace(cvt=base,phase_monitor=PhaseMonitor(),_fresh_detections={'a','remote'})
    variant=DeploymentVariant(controller,'detection_density_cvt')
    agents=[SimpleNamespace(agent_id=n,position=np.array(p)) for n,p in
            [('a',[1.,2.]),('neighbor',[3.,4.]),('remote',[9.,9.])]]
    density=BoundaryAwareDensity([[2.,3.]],[[0.,1.]],[4.],DensityParams())
    result=variant.compute(0,agents,[1],density,[0,10,0,10])
    np.testing.assert_array_equal(base.seen[1].targets,[[1.,2.]])
    assert result.mass == 12. and result.redeploy
    controller._fresh_detections.clear()
    assert len(detection_density(density,[]).points)==0
    from dbact.control.phase import Phase
    controller.phase_monitor.entered[Phase.CONTACT_READY.label]=1
    controller.phase_monitor.phase=Phase.CONTACT_READY
    before=len(base.seen)
    result=variant.compute(0,agents,[1],density,[0,10,0,10])
    assert len(base.seen)==before+1  # All methods return to the common CVT.
    np.testing.assert_array_equal(result.centroid,[2.,3.135])

def test_cargo_protocol_is_geometry_valid_paired_and_initially_blind(monkeypatch):
    import importlib
    from pathlib import Path
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1]/'scripts'))
    plan=importlib.import_module('freeze_cargo_discovery').build()
    near,meta=plan['near']; far,_=plan['far']
    assert len(near)==132 and len(meta)==33 and len(far)==12
    for i in range(0,len(near),4):
        group=near[i:i+4]
        assert all(c['config']==group[0]['config'] for c in group)
        assert len({c['factors']['physical_initial_sha256'] for c in group})==1
    np.testing.assert_allclose([m['area_m2'] for m in meta],meta[0]['area_m2'],atol=1e-12)
    np.testing.assert_allclose([m['mass_kg'] for m in meta],meta[0]['mass_kg'],atol=1e-12)
    assert len({m['agent_initial_sha256'] for m in meta if m['cohort']=='new_cargo_24'})==1
    for i in range(0,len(far),2):
        unknown,known=far[i:i+2]
        assert sum(unknown['factors']['initial_own_scan_returns'])==0
        assert unknown['factors']['minimum_initial_boundary_distance_m'] > unknown['config']['controller']['sensor_range']
        assert unknown['factors']['physical_initial_sha256']==known['factors']['physical_initial_sha256']
        config=dict(known['config']);config.pop('experiment_initial_location_hint')
        assert config==unknown['config']

def test_location_hint_is_explicit_one_shot_packet_without_geometry():
    from dbact_sim.experiments.deployment import install_initial_location_hint
    controller=SimpleNamespace(tasks={'cargo_0':object()},_tokens={})
    agents=[SimpleNamespace(agent_id='a'),SimpleNamespace(agent_id='b')]
    install_initial_location_hint(controller,agents,None)
    assert controller._tokens=={}
    packet={'object_id':'cargo_0','position':[3.,5.]}
    install_initial_location_hint(controller,agents,packet)
    packet['position'][0]=99.
    for a in agents:
        position,time=controller._tokens[a.agent_id]['cargo_0']
        np.testing.assert_array_equal(position,[3.,5.]);assert time==0.

def test_unseen_cargo_position_and_shape_do_not_change_initial_search_commands():
    import json
    from pathlib import Path
    from dbact_sim.environment import SimulationEnvironment
    path=Path(__file__).parents[1]/'experiments/ccta2027/cargo_far_v1.json'
    plan=json.loads(path.read_text())
    outputs=[]
    for case in plan['cases']:
        if case['arm']!='unknown_location':continue
        env=SimulationEnvironment(case['config'],seed=case['seed'])
        assert not env.controller._tokens
        env.step()  # One engineering frame with zero returns, not a task trial.
        assert not env.controller._fresh_detections
        outputs.append(np.vstack([a.velocity for a in env.agents]))
    for output in outputs[1:]:np.testing.assert_array_equal(output,outputs[0])
