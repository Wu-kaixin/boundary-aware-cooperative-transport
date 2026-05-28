from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def test_simulation_smoke():
    cfg = load_yaml("configs/sim/circle.yaml")
    env = SimulationEnvironment(cfg)
    env.run(steps=5)
    assert len(env.agents) == cfg["agents"]["count"]
    assert env.log.times[-1] > 0.0
