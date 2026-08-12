from __future__ import annotations

import matplotlib
import pytest

matplotlib.use("Agg", force=True)

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml
from dbact_sim.trace import SimulationTrace, VisualizationRecorder
from dbact_sim.visualization.animation import render_animation


@pytest.fixture(scope="module")
def trace():
    env = SimulationEnvironment(load_yaml("configs/sim/v2/l_shape_v2.yaml"), seed=2)
    recorder = VisualizationRecorder(stride=1, sensor_ray_stride=8)
    env.run(steps=5, on_frame=recorder.capture)
    return SimulationTrace.from_environment(env, recorder, simulation_fps=21.5)


def test_offline_gif_reports_rendering_and_simulation_fps_separately(trace, tmp_path):
    report = render_animation(
        trace,
        tmp_path / "preview.gif",
        view_mode="demo",
        frame_stride=2,
        fps=8,
        dpi=45,
    )
    assert (tmp_path / "preview.gif").stat().st_size > 1_000
    assert report.source_frames == trace.frame_count
    assert report.rendered_frames == 4
    assert report.rendering_fps > 0.0
    assert report.simulation_fps == pytest.approx(21.5)
    assert report.playback_fps == 8


def test_offline_animation_rejects_unknown_container(trace, tmp_path):
    with pytest.raises(ValueError, match="gif or .mp4"):
        render_animation(trace, tmp_path / "preview.avi")
