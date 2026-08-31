from __future__ import annotations

import matplotlib

matplotlib.use("Agg", force=True)

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml
from dbact_sim.trace import SimulationTrace, VisualizationRecorder
from dbact_sim.visualization.paper_figures import phase_keyframes, write_research_paper_figures


def _short_trace() -> SimulationTrace:
    env = SimulationEnvironment(load_yaml("configs/sim/v2/l_shape_v2.yaml"), seed=0)
    recorder = VisualizationRecorder(stride=1, sensor_ray_stride=8)
    env.run(steps=5, on_frame=recorder.capture)
    return SimulationTrace.from_environment(env, recorder)


def test_phase_keyframes_do_not_fabricate_missing_phases():
    trace = _short_trace()
    frames = phase_keyframes(trace)
    assert set(frames) == {"SEARCH", "MAP", "ENCLOSE", "TRANSPORT", "HOLD"}
    assert frames["HOLD"] is None


def test_paper_figures_write_a_to_g_as_raster_and_vector(tmp_path):
    trace = _short_trace()
    outputs = write_research_paper_figures(
        trace,
        tmp_path,
        formats=("png", "pdf", "svg"),
        dpi=70,
    )
    assert set(outputs) == set("ABCDEFG")
    assert sum(map(len, outputs.values())) == 21
    for paths in outputs.values():
        assert {path.suffix for path in paths} == {".png", ".pdf", ".svg"}
        assert all(path.stat().st_size > 500 for path in paths)
