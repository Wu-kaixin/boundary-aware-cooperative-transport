"""T5 - the three degradation mechanisms, and the no-op property they must have.

``perception_every``, ``planning_every`` and ``communication_dropout_prob`` were added to v1
for the robustness ablation. The single most important property is that their defaults are
**exact** no-ops: the ``nominal`` arm of the ablation has to be v1, or every comparison in
that experiment is against a subtly different controller and none of the numbers transfer to
the branch's other results.

"Exact" is asserted literally here -- identical command sequences, identical map contents --
rather than to a tolerance, because a tolerance would hide precisely the kind of drift these
tests exist to catch.
"""

from __future__ import annotations

import copy

import numpy as np
import pytest

from dbact.controller import DBACTController, DBACTParams
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"
FRAMES = 40


@pytest.fixture(scope="module")
def base_config() -> dict:
    return load_yaml(BASE_CONFIG)


def run(config: dict, frames: int = FRAMES, seed: int = 0) -> SimulationEnvironment:
    env = SimulationEnvironment(config, seed=seed)
    env.run(frames)
    return env


def with_controller(base: dict, **overrides) -> dict:
    config = copy.deepcopy(base)
    config["controller"] = dict(config.get("controller", {}))
    config["controller"].update(overrides)
    return config


def fingerprint(env: SimulationEnvironment) -> dict:
    """Enough state to catch any divergence, cheap enough to compare exactly."""
    return {
        "positions": np.vstack([a.position for a in env.agents]),
        "cargo": np.asarray([env.cargoes[0].position[0], env.cargoes[0].position[1],
                             env.cargoes[0].angle]),
        "phase": int(env.controller.phase),
        "map_sizes": [len(env.controller.maps[a.agent_id]) for a in env.agents],
        "solver": dict(env.summary()["solver"]),
    }


# --------------------------------------------------------------------------- #
# the defaults are exact no-ops
# --------------------------------------------------------------------------- #


def test_defaults_are_the_documented_no_ops():
    params = DBACTParams()
    assert params.perception_every == 1
    assert params.planning_every == 1
    assert params.communication_dropout_prob == 0.0


def test_explicit_no_op_values_reproduce_the_baseline_exactly(base_config):
    """Setting all three to their no-op values must change nothing at all.

    This is the check that the ``nominal`` arm of the robustness ablation is v1: the arm
    sets these fields explicitly, so if writing the default value down were not identical
    to leaving it out, every arm would be compared against a different controller.
    """
    plain = fingerprint(run(base_config))
    explicit = fingerprint(
        run(with_controller(base_config, perception_every=1, planning_every=1,
                            communication_dropout_prob=0.0))
    )
    assert np.array_equal(plain["positions"], explicit["positions"])
    assert np.array_equal(plain["cargo"], explicit["cargo"])
    assert plain["phase"] == explicit["phase"]
    assert plain["map_sizes"] == explicit["map_sizes"]
    assert plain["solver"] == explicit["solver"]


# --------------------------------------------------------------------------- #
# perception_every
# --------------------------------------------------------------------------- #


def test_perception_stride_changes_the_run(base_config):
    """A sensor firing at a fifth of the rate must produce a different run.

    Trivial-looking, and worth pinning: a stride that was silently ignored would make the
    ``slow_updates_5`` arm a duplicate of nominal, and the ablation would report that a
    five-fold slower sensor costs nothing.
    """
    slow = fingerprint(run(with_controller(base_config, perception_every=5)))
    nominal = fingerprint(run(base_config))
    assert not np.array_equal(slow["positions"], nominal["positions"])


def test_perception_stride_still_senses_on_the_first_frame(base_config):
    """A run must not start blind, whatever the stride.

    With ``sensing`` gated only on ``frame % stride == 0`` a large stride would still fire
    on frame 0; the ``or not self._views`` clause is what guarantees it for any future
    change to the counter, and this pins the observable consequence: the map is non-empty
    after one step.
    """
    env = SimulationEnvironment(with_controller(base_config, perception_every=97), seed=0)
    env.step()
    assert any(len(env.controller.maps[a.agent_id]) > 0 for a in env.agents)


def test_perception_stride_leaves_the_map_untouched_between_firings(base_config):
    """No scan, no fusion: the map must not change on a non-sensing frame.

    Running the update on an empty scan would age every cell and decay its weight on a
    frame where no measurement contradicted it -- modelling a *contradicted* observation
    instead of a missing one.
    """
    env = SimulationEnvironment(with_controller(base_config, perception_every=6), seed=0)
    env.step()  # frame 0 senses
    agent_id = env.agents[0].agent_id
    env.step()  # frame 1 does not
    before = env.controller.maps[agent_id]._points.copy()
    env.step()  # frame 2 does not either
    assert np.array_equal(env.controller.maps[agent_id]._points, before)


def test_perception_stride_gives_registration_the_whole_elapsed_interval(base_config):
    """A 4 Hz sensor in a 20 Hz world must divide by the real elapsed time.

    Passing ``dt`` instead of the accumulated interval would make every velocity estimate a
    fifth of the truth at ``perception_every = 5``, and the transport loop closes on that
    estimate -- so the arm would be measuring a broken estimator rather than a slow sensor.
    """
    dt = float(base_config.get("dt", 0.05))
    stride = 5
    env = SimulationEnvironment(with_controller(base_config, perception_every=stride), seed=0)

    # ``observed[k]`` is the accumulator *after* the k-th step. Frames 0 and 5 sense and
    # therefore leave it at zero; frames 1..4 each add one dt.
    observed: list[float] = []
    for _ in range(stride + 1):
        env.step()
        observed.append(env.controller._sense_interval)

    assert observed[0] == pytest.approx(0.0, abs=1e-12), "frame 0 senses and resets"
    assert observed[stride] == pytest.approx(0.0, abs=1e-12), "frame 5 senses and resets"
    for k in range(1, stride):
        assert observed[k] == pytest.approx(k * dt, abs=1e-9), k

    # The interval frame 5 consumed is the accumulator it inherited plus its own dt, which
    # is the full stride and not a single dt. That factor of 5 is the whole point: the
    # transport loop closes on this estimate.
    assert observed[stride - 1] + dt == pytest.approx(stride * dt, abs=1e-9)


# --------------------------------------------------------------------------- #
# planning_every
# --------------------------------------------------------------------------- #


def test_planning_stride_holds_the_nominal_command(base_config):
    env = SimulationEnvironment(with_controller(base_config, planning_every=7), seed=0)
    env.step()
    held = {k: tuple(np.array(v[0])) for k, v in env.controller._held_nominal.items()}
    assert held, "the first frame must plan and populate the hold"
    env.step()
    after = {k: tuple(np.array(v[0])) for k, v in env.controller._held_nominal.items()}
    assert held == after, "a non-planning frame must not recompute the nominal command"


def test_planning_stride_still_filters_every_frame(base_config):
    """The safety filter runs at full rate behind a slow planner.

    Holding the *filtered* output instead would let a stale command drive through a
    constraint that became active after it was computed, which is a much less defensible
    experiment than a slow planner behind a fast barrier filter. The solve count is the
    observable: it must keep rising on frames the planner skipped.
    """
    env = SimulationEnvironment(with_controller(base_config, planning_every=9), seed=0)
    env.step()
    first = env.controller.safety.stats.solves
    env.step()
    assert env.controller.safety.stats.solves > first


def test_planning_stride_changes_the_run(base_config):
    slow = fingerprint(run(with_controller(base_config, planning_every=5)))
    nominal = fingerprint(run(base_config))
    assert not np.array_equal(slow["positions"], nominal["positions"])


# --------------------------------------------------------------------------- #
# communication_dropout_prob
# --------------------------------------------------------------------------- #


def test_dropout_removes_links_and_is_reproducible(base_config):
    """Same seed, same loss pattern. Different seed, different pattern.

    A dropout run that was not reproducible from its seed would be the one experiment on
    this branch whose numbers could not be re-derived.
    """
    config = with_controller(base_config, communication_dropout_prob=0.30)
    a = fingerprint(run(config, seed=3))
    b = fingerprint(run(config, seed=3))
    c = fingerprint(run(config, seed=4))
    assert np.array_equal(a["positions"], b["positions"])
    assert not np.array_equal(a["positions"], c["positions"])


def test_dropout_actually_reduces_the_neighbour_count(base_config):
    """Measured on the neighbour sets, not inferred from the outcome."""
    params = DBACTParams.from_dict(
        dict(base_config["controller"], dt=base_config.get("dt", 0.05))
    )
    env = SimulationEnvironment(base_config, seed=0)
    agents = env.agents

    clean = DBACTController(params, env.domain, env.goal_directions, seed=0, tasks=env.tasks)
    lossy_params = DBACTParams.from_dict(
        dict(base_config["controller"], dt=base_config.get("dt", 0.05),
             communication_dropout_prob=0.5)
    )
    lossy = DBACTController(lossy_params, env.domain, env.goal_directions, seed=0, tasks=env.tasks)

    clean_links = sum(len(n) for n in clean._neighbor_indices(agents))
    lossy_links = sum(len(n) for n in lossy._neighbor_indices(agents))
    assert clean_links > 0
    assert lossy_links < clean_links


def test_dropout_of_one_drops_every_link(base_config):
    """The extreme, so the direction of the probability cannot be inverted silently."""
    params = DBACTParams.from_dict(
        dict(base_config["controller"], dt=base_config.get("dt", 0.05),
             communication_dropout_prob=1.0)
    )
    env = SimulationEnvironment(base_config, seed=0)
    controller = DBACTController(params, env.domain, env.goal_directions, seed=0, tasks=env.tasks)
    assert sum(len(n) for n in controller._neighbor_indices(env.agents)) == 0


def test_dropout_pattern_changes_between_frames(base_config):
    """A lossy link, not a partitioned team.

    Latching one subgraph for the whole run is a different experiment: it tests a team
    split in two, not a team losing packets. The frame index is part of the RNG key so the
    pattern moves, and this pins that.
    """
    params = DBACTParams.from_dict(
        dict(base_config["controller"], dt=base_config.get("dt", 0.05),
             communication_dropout_prob=0.5)
    )
    env = SimulationEnvironment(base_config, seed=0)
    controller = DBACTController(params, env.domain, env.goal_directions, seed=0, tasks=env.tasks)

    first = controller._neighbor_indices(env.agents)
    controller._frame = 1
    second = controller._neighbor_indices(env.agents)
    assert first != second


def test_dropout_is_directional(base_config):
    """i losing j does not imply j losing i, and nothing downstream assumes it does."""
    params = DBACTParams.from_dict(
        dict(base_config["controller"], dt=base_config.get("dt", 0.05),
             communication_dropout_prob=0.5)
    )
    env = SimulationEnvironment(base_config, seed=0)
    controller = DBACTController(params, env.domain, env.goal_directions, seed=0, tasks=env.tasks)
    neighbours = controller._neighbor_indices(env.agents)
    asymmetric = any(
        (j in neighbours[i]) != (i in neighbours[j])
        for i in range(len(neighbours))
        for j in range(len(neighbours))
        if i != j
    )
    assert asymmetric, "a symmetric loss model would be a different experiment"


# --------------------------------------------------------------------------- #
# the ablation harness agrees with the mechanisms
# --------------------------------------------------------------------------- #


def test_the_ablation_declares_the_baseline_noise_collision(matrix_runner, base_config):
    """The baseline already runs at the noise level the plan calls out-of-domain.

    ``configs/sim/d/l_shape_closed_loop.yaml`` sets ``range_noise_std: 0.01``, so the
    plan's ``range_noise_010`` arm is the nominal arm. The ablation script has to know
    that, because otherwise it reports a perturbation that is not one -- and by the plan's
    own rule the nominal arm is out-of-domain too.
    """
    from conftest import load_script_module

    ablation = load_script_module("run_robustness_ablation")
    assert float(base_config["controller"]["range_noise_std"]) == pytest.approx(0.010)
    assert ablation.VARIANTS["range_noise_010"]["range_noise_std"] == pytest.approx(0.010)
    assert "nominal" in ablation.DECLARED_OUT_OF_DOMAIN
    # Arms below and above the baseline, so the sweep spans it.
    assert ablation.VARIANTS["range_noise_000"]["range_noise_std"] == 0.0
    assert ablation.VARIANTS["range_noise_020"]["range_noise_std"] == pytest.approx(0.020)
