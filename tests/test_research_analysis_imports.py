"""Numerical audit helpers must import without workstation-only dependencies."""
from pathlib import Path
import subprocess
import sys


def test_reference_audit_import_does_not_load_workstation_or_optional_psutil():
    root=Path(__file__).parents[1]
    code='''
import importlib.abc, sys
class NoWorkstation(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname in {'benchmark_workstation','psutil','maintenance.windows_metrics'}:
            raise AssertionError('Numerical audit imported '+fullname)
sys.meta_path.insert(0, NoWorkstation())
sys.path.insert(0, 'scripts')
import audit_reference_motion
assert audit_reference_motion.position_band({'config': {'controller': {'cross_track_deadband': .02}}}) == .02
'''
    result=subprocess.run([sys.executable,'-c',code],cwd=root,capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
