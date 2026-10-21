"""Enforce control/evaluation separation and compatibility of moved modules."""
import ast
import importlib
from pathlib import Path

import pytest


@pytest.mark.parametrize("old,new", [
    ("controller", "control.controller"), ("agent_control", "control.agent_control"),
    ("phase", "control.phase"), ("transport_control", "control.transport"),
    ("perception", "sensing.sensor"), ("boundary_map", "sensing.boundary_map"),
    ("local_boundary_features", "sensing.features"),
    ("boundary_density", "deployment.density"), ("local_cvt", "deployment.cvt"),
    ("deployment_variants", "deployment.variants"), ("theorem_mode", "deployment.theorem"),
    ("safety_filter", "safety.filter"), ("qp2d", "safety.qp2d"),
])
def test_legacy_import_preserves_module_identity(old, new):
    assert importlib.import_module(f"dbact.{old}") is importlib.import_module(f"dbact.{new}")


def test_control_never_imports_simulation_evaluation_or_hardware():
    root = Path(__file__).parents[1] / "src/dbact"
    forbidden = {"dbact_sim", "mas_adapter", "scripts"}
    for path in root.rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            names = ([node.module or ""] if isinstance(node, ast.ImportFrom)
                     else [a.name for a in node.names] if isinstance(node, ast.Import) else [])
            assert not any(name.split(".")[0] in forbidden for name in names), str(path)
