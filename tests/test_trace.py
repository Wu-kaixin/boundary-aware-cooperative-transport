from __future__ import annotations

import numpy as np

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml
from dbact_sim.trace import SimulationTrace, VisualizationRecorder


SCENARIO = "configs/sim/v2/l_shape_v2.yaml"


def test_visualization_observer_does_not_change_simulation_numerics():
    baseline = SimulationEnvironment(load_yaml(SCENARIO), seed=7)
    baseline.run(steps=8)

    observed = SimulationEnvironment(load_yaml(SCENARIO), seed=7)
    recorder = VisualizationRecorder(stride=2, sensor_ray_stride=4)
    observed.run(steps=8, on_frame=recorder.capture)

    assert baseline.log.times == observed.log.times
    for agent_id in baseline.log.agent_positions:
        np.testing.assert_array_equal(
            np.vstack(baseline.log.agent_positions[agent_id]),
            np.vstack(observed.log.agent_positions[agent_id]),
        )
    for cargo_id in baseline.log.cargo_vertices:
        np.testing.assert_array_equal(
            np.stack(baseline.log.cargo_vertices[cargo_id]),
            np.stack(observed.log.cargo_vertices[cargo_id]),
        )
    assert baseline.controller.safety.stats.as_dict() == observed.controller.safety.stats.as_dict()


def test_trace_round_trip_is_pickle_free_and_preserves_metrics(tmp_path):
    env = SimulationEnvironment(load_yaml(SCENARIO), seed=3)
    recorder = VisualizationRecorder(stride=2, sensor_ray_stride=4)
    env.run(steps=6, on_frame=recorder.capture)
    trace = SimulationTrace.from_environment(env, recorder)
    trace.save(tmp_path / "trace")

    restored = SimulationTrace.load(tmp_path / "trace")
    assert restored.schema_version == trace.schema_version
    assert restored.agent_ids == trace.agent_ids
    assert restored.cargo_ids == trace.cargo_ids
    assert restored.phase_labels == trace.phase_labels
    np.testing.assert_array_equal(restored.agent_positions, trace.agent_positions)
    np.testing.assert_array_equal(restored.min_distances, trace.min_distances)
    for cargo_id in trace.cargo_ids:
        np.testing.assert_array_equal(
            restored.directional_progress[cargo_id],
            trace.directional_progress[cargo_id],
        )
        np.testing.assert_array_equal(
            restored.max_uncovered_gap[cargo_id],
            trace.max_uncovered_gap[cargo_id],
        )
    assert [item.frame for item in restored.visual_snapshots] == [
        item.frame for item in trace.visual_snapshots
    ]


def test_trace_records_hud_diagnostics_at_every_frame():
    env = SimulationEnvironment(load_yaml(SCENARIO), seed=0)
    env.run(steps=3)
    trace = SimulationTrace.from_environment(env)

    assert trace.frame_count == 4
    assert len(trace.phase_labels) == trace.frame_count
    assert len(trace.contact_ready_agents) == trace.frame_count
    assert len(trace.push_agents) == trace.frame_count
    assert len(trace.qp_status_counts) == trace.frame_count
    assert len(trace.solver_fallbacks) == trace.frame_count
    assert len(trace.solver_infeasible) == trace.frame_count
