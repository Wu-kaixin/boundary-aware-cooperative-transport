from __future__ import annotations

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg", force=True)

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml
from dbact_sim.trace import SimulationTrace, VisualizationRecorder
from dbact_sim.visualization import ResearchVisualizer


@pytest.fixture(scope="module")
def trace():
    env = SimulationEnvironment(load_yaml("configs/sim/v2/l_shape_v2.yaml"), seed=0)
    recorder = VisualizationRecorder(stride=1, sensor_ray_stride=6)
    env.run(steps=4, on_frame=recorder.capture)
    return SimulationTrace.from_environment(env, recorder, simulation_fps=23.5)


@pytest.mark.parametrize("view_mode", ["demo", "paper", "debug"])
def test_research_visualizer_renders_all_view_modes(trace, tmp_path, view_mode):
    visualizer = ResearchVisualizer(trace, view_mode=view_mode)
    output = tmp_path / f"{view_mode}.png"
    visualizer.save_frame(trace.frame_count - 1, output, dpi=70)
    visualizer.close()
    assert output.stat().st_size > 1_000


def test_world_renderer_reuses_core_artists(trace):
    visualizer = ResearchVisualizer(trace, view_mode="demo")
    cargo_ids = {key: id(value) for key, value in visualizer.cargo_patches.items()}
    agent_ids = [id(value) for value in visualizer.agent_patches]
    visualizer.update(1)
    visualizer.update(trace.frame_count - 1)
    assert cargo_ids == {key: id(value) for key, value in visualizer.cargo_patches.items()}
    assert agent_ids == [id(value) for value in visualizer.agent_patches]
    visualizer.close()


def test_hud_phase_path_and_solver_metrics_are_updated(trace):
    visualizer = ResearchVisualizer(trace, view_mode="demo")
    visualizer.update(trace.frame_count - 1, rendering_fps=17.25)
    assert visualizer.hud is not None
    assert set(visualizer.hud.phase_nodes) == {
        "SEARCH",
        "MAP",
        "ENCLOSE",
        "CONTACT_READY",
        "TRANSPORT",
        "BRAKE",
        "HOLD",
    }
    assert "rendering    17.25 FPS" in visualizer.hud.fps_text.get_text()
    assert "fallback" in visualizer.hud.solver_text.get_text()
    visualizer.close()


def test_debug_overlays_are_trace_data_not_simulator_truth(trace):
    visualizer = ResearchVisualizer(trace, view_mode="debug")
    visualizer.update(trace.frame_count - 1)
    snapshot = trace.visual_snapshot(trace.frame_count - 1)
    np.testing.assert_array_equal(visualizer.map_points.get_offsets(), snapshot.mapped_points)
    np.testing.assert_array_equal(visualizer.detected_points.get_offsets(), snapshot.detected_points)
    assert all(line.get_visible() for line in visualizer.truth_lines.values())
    visualizer.close()


def test_contact_and_push_overlays_follow_trace_labels(trace):
    visualizer = ResearchVisualizer(trace, view_mode="demo")
    frame = trace.frame_count - 1
    visualizer.update(frame)
    contacts = set(trace.contact_ready_agents[frame])
    pushers = set(trace.push_agents[frame])
    for index, agent_id in enumerate(trace.agent_ids):
        assert visualizer.contact_rings[index].get_visible() == (
            agent_id in contacts or agent_id in pushers
        )
        assert visualizer.push_arrows[index].get_visible() == (agent_id in pushers)
    assert not any(line.get_visible() for line in visualizer.truth_lines.values())
    visualizer.close()
