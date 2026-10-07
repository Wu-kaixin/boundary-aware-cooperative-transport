"""Scaled cargoes must satisfy the same initial-clearance contract."""
import copy

import numpy as np
import yaml

from dbact.geometry import signed_distance_to_polygon
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import build_cargoes, legal_scatter_radii


def _closed_loop():
    with open("configs/sim/d/l_shape_closed_loop.yaml", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    with open("configs/sim/theorem/static_l_shape_n16_oracle.yaml", encoding="utf-8") as handle:
        geometry = yaml.safe_load(handle)["cargoes"][0]
    geometry["movable"] = True
    cfg["cargoes"] = [geometry]
    return cfg


def test_base_near_layout_is_already_legal_and_is_not_moved():
    cfg = _closed_loop()
    env = SimulationEnvironment(cfg, seed=910072)
    assert env.layout_notes == []
    radii = np.linalg.norm(np.vstack([a.position for a in env.agents]) - np.array([4.0, 4.0]), axis=1)
    assert radii.min() >= 1.50 - 1e-9


def test_scaled_l_shape_raises_the_annulus_instead_of_starting_inside_the_robot_radius():
    cfg = _closed_loop()
    cfg["cargoes"][0]["scale"] = float(cfg["cargoes"][0]["scale"]) * 1.2
    bare = copy.deepcopy(cfg)
    cargoes = build_cargoes(bare, seed=910072)
    radius_min, radius_max, notes = legal_scatter_radii(bare, cargoes, robot_radius=0.13)
    assert radius_min > 1.50
    assert radius_max > radius_min
    assert notes
    env = SimulationEnvironment(cfg, seed=910072)
    assert env.layout_notes
    clearance = signed_distance_to_polygon(
        np.vstack([a.position for a in env.agents]), env.cargoes[0].vertices
    )
    assert float(np.min(clearance)) >= 0.13 - 1e-9
