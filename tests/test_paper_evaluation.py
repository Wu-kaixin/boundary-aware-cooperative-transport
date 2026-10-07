import numpy as np
from dbact_sim.paper_evaluation import evaluate_static_hold, progress_errors, event_record, upgrade_legacy_static
SQUARE=np.array([[0.,0.],[1.,0.],[1.,1.],[0.,1.]])
def test_safe_endpoints_can_cross_solid():
    r=evaluate_static_hold([[-1.,.5]],[[3.,0.]],SQUARE,1.,.065)
    assert r['status']=='OBSERVED_VIOLATION'
    assert r['robots'][0]['intersects_solid']
def test_lower_bound_failure_is_not_observed_collision():
    legacy=upgrade_legacy_static({'minimum_true_clearance':.048,'object_safety_violation_frames':8})
    assert legacy['actual_violation_count'] is None
    r=evaluate_static_hold([[-.07,.2]],[[0.,.4]],SQUARE,1.,.065)
    assert r['status']=='CERTIFIED_SAFE'
def test_threshold_is_unresolved_and_inside_is_violation():
    assert evaluate_static_hold([[-.065,.2]],[[0.,0.]],SQUARE,1.,.065)['status']=='UNRESOLVED'
    assert evaluate_static_hold([[.5,.5]],[[0.,0.]],SQUARE,1.,.065)['status']=='OBSERVED_VIOLATION'
def test_signed_errors_do_not_cancel_and_endpoint_is_two_dimensional():
    a=progress_errors([0,0],[.8,.3],[1,0],1)
    b=progress_errors([0,0],[1.2,.3],[1,0],1)
    assert np.isclose(a['absolute_progress_error'],b['absolute_progress_error'])
    assert a['euclidean_endpoint_error']>a['absolute_progress_error']
def test_watchdog_does_not_make_missing_event_zero():
    event=event_record(None,.05,3000)
    assert event['right_censored'] and event['seconds'] is None
    assert event['observation_end_seconds']==150

def test_observers_do_not_change_commands_or_physics(monkeypatch):
    import copy, importlib.util
    from pathlib import Path
    spec=importlib.util.spec_from_file_location('pairs',Path(__file__).resolve().parents[1]/'scripts/run_deployment_pairs.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    from dbact_sim.environment import SimulationEnvironment
    monkeypatch.setenv('DBACT_CVT_WORKERS','1')
    cfg=module.config_for(6,'c_shape')
    left=SimulationEnvironment(copy.deepcopy(cfg),seed=8)
    right=SimulationEnvironment(copy.deepcopy(cfg),seed=8)
    right.engine.model.record_substeps=True
    right.controller.trace_enabled=True
    for _ in range(12):
        left.step();right.step()
        for a,b in zip(left.agents,right.agents):
            np.testing.assert_array_equal(a.position,b.position)
            np.testing.assert_array_equal(a.velocity,b.velocity)
        np.testing.assert_array_equal(left.cargoes[0].position,right.cargoes[0].position)
        assert left.cargoes[0].angle==right.cargoes[0].angle
        assert len(right.engine.model.last_substeps)==right.contact_params.substeps

def test_truth_evaluator_not_imported_by_control():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]/'src/dbact'
    for name in ('controller.py','safety_filter.py','local_cvt.py','phase.py','transport_control.py'):
        assert 'paper_evaluation' not in (root/name).read_text(encoding="utf-8")
