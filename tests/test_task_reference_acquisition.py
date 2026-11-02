"""The acquisition diagnostic must not equate noisy line support to a contour."""
import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('acquisition_probe',Path(__file__).parents[1]/'scripts/task_reference_acquisition.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_line_with_noisy_normals_is_not_spatial_support():
    x=np.linspace(-1,1,100)
    points=np.c_[x,.002*np.sin(10*x)]
    angles=np.linspace(0,2*np.pi,100)
    rows=np.c_[points,np.cos(angles),np.sin(angles)]
    assert not module.support(rows,.01)['usable']


def test_a_single_visible_side_is_not_surrounding_normal_support():
    x=np.linspace(-1,1,100)
    rows=np.c_[x,x*x,np.zeros(100),np.ones(100)]
    assert not module.support(rows,.01)['usable']


def test_closed_circle_has_geometric_support_but_not_a_task_success_label():
    theta=np.linspace(0,2*np.pi,100,endpoint=False)
    circle=np.c_[np.cos(theta),np.sin(theta)]
    result=module.support(np.c_[circle,circle],.01)
    assert result['usable']
    assert 'completed' not in result and 'center' not in result
