"""Geometry/identity preflight; no additional research episodes."""
import copy
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import config_for
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.shapes import smooth_outline

ROOT = Path(__file__).parents[1]


def load_builder():
    spec = importlib.util.spec_from_file_location("census_builder", ROOT / "scripts/freeze_multishape_census.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("shape,n", [("circle", 64), ("ellipse24", 24)])
def test_smooth_geometry_enters_real_transport_and_protocol(shape, n, tmp_path):
    cfg = config_for(6, shape)
    env = SimulationEnvironment(copy.deepcopy(cfg), seed=2)
    assert len(env.cargoes[0].vertices) == n
    assert "cargo_0" in env.tasks
    assert env.controller.params.task_mode == "transport"
    assert not env.controller.params.theorem_mode
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(cfg))
    protocol_path = tmp_path / "protocol.yaml"
    protocol_path.write_text(yaml.safe_dump({"cases": [{"case_id":shape, "shape":shape,
        "config_ref":"config.yaml", "seed":2, "frames":2, "gate":6,
        "variant":"boundary_cvt", "split":"smoke"}]}))
    _, cases = load_protocol(protocol_path, "smoke", tmp_path)
    assert cases[0]["config"] == cfg


def test_census_pairs_actual_initial_states_and_preserves_common_physics():
    plan = load_builder().build_plan()
    assert len(plan["conditions"]) == 9 and len(plan["cases"]) == 27
    common = config_for(6, "l_shape")
    meta = plan["conditions"]
    np.testing.assert_allclose([m["area_m2"] for m in meta], meta[0]["area_m2"])
    np.testing.assert_allclose([m["mass_kg"] for m in meta], meta[0]["mass_kg"])
    assert len({m["agent_initial_sha256"] for m in meta}) == 1
    for i in range(0,27,3):
        group = plan["cases"][i:i+3]
        assert group[0]["config"] == group[1]["config"] == group[2]["config"]
        for case in group:
            for key in ("controller", "transport", "evaluation", "g500", "dt"):
                assert case["config"][key] == common[key]
            assert case["frames"] == 3000 and case["seed"] == 2
            np.testing.assert_allclose(case["factors"]["initial_centroid"], [4,4])
            assert case["factors"]["task"]["direction"] == [1,0]
    for name in ("ellipse", "rectangle"):
        a,b = [c for c in plan["cases"] if c["case_id"] in
               (f"{name}_parallel_boundary_cvt",f"{name}_perpendicular_boundary_cvt")]
        va,vb = [np.asarray(c["config"]["cargoes"][0]["vertices"]) for c in (a,b)]
        assert np.ptp(va[:,0]) > np.ptp(va[:,1])
        assert np.ptp(vb[:,0]) < np.ptp(vb[:,1])
