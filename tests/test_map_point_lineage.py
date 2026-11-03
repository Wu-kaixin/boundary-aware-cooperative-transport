"""Provenance must survive weighted rekey collisions, caps and relay duplicates."""
import importlib.util
from pathlib import Path

import numpy as np

from dbact.sensing.boundary_map import LocalBoundaryMap
from dbact.types import BoundaryView


def module(monkeypatch):
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    path=root/'scripts/map_point_lineage.py'
    assert path.exists(), 'Missing lineage reconstruction'
    spec=importlib.util.spec_from_file_location('lineage_test',path)
    m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def scan(points):
    n=len(points)
    return BoundaryView(np.asarray(points,dtype=float),np.tile([0.,1.],(n,1)),
        np.ones(n),np.full(n,.01),np.full(n,'cargo_0',dtype=object))


def state(local):
    return dict(keys=local._keys.copy(),points=local._points.copy(),
        weights=local._weight.copy(),objects=local._objects.copy())


def test_rekey_fusion_lineage_keeps_capped_prior_and_deduplicates_relay(monkeypatch):
    m=module(monkeypatch)
    local=LocalBoundaryMap(voxel_size=1.,max_weight=2.)
    graph=m.Lineage()
    # Two distinct cells become one after +0.3 translation. A fresh cell holds
    # four sample weights; collision fusion caps their combined prior at 2.
    first=scan([[.1,0.],[.2,0.],[.3,0.],[.4,0.],[.6,0.]])
    codes=np.zeros(5,dtype=int)
    before=state(local)
    local.update(first,0.,agent_codes=codes,integrate_motion=False)
    graph.update(before,vars(first),codes,state(local),1.,local._object_index,1)
    before=state(local)
    local.align_translation('cargo_0',np.array([.3,0.]),np.zeros(2))
    graph.align(before,state(local),[.3,0.],1.,local._object_index,2)
    # The merged point is (.25*4+.6)/5+.3 = .62; collision
    # weight caps at 2. A duplicated relay must count as one fresh observation.
    second=scan([[.8,0.],[.8,0.]])
    before=state(local)
    local.update(second,1.,agent_codes=np.zeros(2,dtype=int),integrate_motion=False)
    graph.update(before,vars(second),[0,0],state(local),1.,local._object_index,3)
    result=graph.explain(int(local._keys[0]))
    np.testing.assert_allclose(result['point'],[17/25,0.])
    np.testing.assert_allclose(result['reconstructed_point'],result['point'],atol=1e-14)
    assert sum(x['coefficient'] for x in result['observations']) == 1.
    assert sum(len(x['samples']) for x in result['observations']) == 6
    # New scan owns 1/3, the old collision owns 2/3 after the cap.
    assert result['observations'][-1]['coefficient'] == 1/3
    np.testing.assert_allclose(result['translation_contribution'],[.2,0.])


def test_lineage_rejects_silent_point_movement(monkeypatch):
    m=module(monkeypatch)
    local=LocalBoundaryMap(voxel_size=1.)
    data=scan([[.1,0.]])
    before=state(local)
    local.update(data,0.,agent_codes=np.zeros(1,dtype=int),integrate_motion=False)
    graph=m.Lineage()
    graph.update(before,vars(data),[0],state(local),1.,local._object_index,1)
    changed=state(local)
    changed['points'][0,0]+=.01
    import pytest
    with pytest.raises(AssertionError):
        graph.select(changed)
