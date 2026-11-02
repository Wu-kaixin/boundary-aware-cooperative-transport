"""Analytic examples of the offline static-wrench diagnostic."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest


def test_balanced_and_unbalanced_contacts_are_distinguished():
    path=Path(__file__).resolve().parents[1]/'scripts/diagnose_force_capacity.py'
    import sys
    sys.path.insert(0,str(path.parent))
    try:
        spec=importlib.util.spec_from_file_location('capacity_analysis',path)
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        def check(points,normals):
            return m.capacity(np.array(points,float),np.array(normals,float),np.zeros(2),np.array([1.,0.]),16.,0.)
        assert check([[-1.,0.]],[[-1.,0.]])==pytest.approx(16.)
        assert check([[-1.,1.]],[[-1.,0.]])==pytest.approx(0.)
        assert check([[-1.,1.],[-1.,-1.]],[[-1.,0.],[-1.,0.]])==pytest.approx(32.)
        assert check([],[])==0.
    finally:sys.path.pop(0)
