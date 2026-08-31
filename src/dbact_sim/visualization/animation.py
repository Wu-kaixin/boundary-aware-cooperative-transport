"""Offline GIF/MP4 encoder for immutable simulation traces."""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib.animation as mpl_animation

from dbact_sim.trace import SimulationTrace

from .legacy import _configure_ffmpeg_writer
from .renderer import ResearchVisualizer


@dataclass(frozen=True)
class RenderReport:
    output: str
    view_mode: str
    source_frames: int
    rendered_frames: int
    frame_stride: int
    playback_fps: float
    wall_seconds: float
    rendering_fps: float
    simulation_fps: float | None

    def as_dict(self) -> dict:
        return asdict(self)


def ensure_mp4_export_available() -> None:
    """Fail before simulation when the publication MP4 encoder is unavailable."""
    _configure_ffmpeg_writer()


def render_animation(
    trace: SimulationTrace,
    path: str | Path,
    *,
    view_mode: str = "demo",
    frame_stride: int = 4,
    fps: float = 20.0,
    dpi: int = 110,
    show_ids: bool | None = None,
) -> RenderReport:
    """Encode a trace offline and report encoder throughput separately."""
    output = Path(path)
    suffix = output.suffix.lower()
    if suffix not in {".gif", ".mp4"}:
        raise ValueError("animation output must end in .gif or .mp4")
    if trace.frame_count < 1:
        raise ValueError("cannot render an empty trace")
    if fps <= 0.0:
        raise ValueError("fps must be positive")
    stride = max(1, int(frame_stride))
    frames = list(range(0, trace.frame_count, stride))
    if frames[-1] != trace.frame_count - 1:
        frames.append(trace.frame_count - 1)
    output.parent.mkdir(parents=True, exist_ok=True)

    visualizer = ResearchVisualizer(trace, view_mode=view_mode, show_ids=show_ids)
    started = time.perf_counter()
    callbacks = 0

    def draw(frame: int):
        nonlocal callbacks
        callbacks += 1
        elapsed = time.perf_counter() - started
        live_fps = callbacks / max(elapsed, 1e-9)
        return visualizer.update(frame, rendering_fps=live_fps)

    animation = mpl_animation.FuncAnimation(
        visualizer.fig,
        draw,
        frames=frames,
        interval=1000.0 / float(fps),
        blit=False,
        repeat=False,
        cache_frame_data=False,
    )
    try:
        if suffix == ".mp4":
            _configure_ffmpeg_writer()
            writer = mpl_animation.FFMpegWriter(
                fps=fps,
                codec="h264",
                bitrate=2600,
                extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
            )
        else:
            writer = mpl_animation.PillowWriter(fps=fps)
        animation.save(output, writer=writer, dpi=int(dpi))
    finally:
        visualizer.close()
    elapsed = time.perf_counter() - started
    simulation_fps = float(trace.settings.get("simulation_fps", float("nan")))
    if simulation_fps != simulation_fps:  # NaN
        simulation_fps = None
    return RenderReport(
        output=str(output),
        view_mode=str(view_mode),
        source_frames=trace.frame_count,
        rendered_frames=len(frames),
        frame_stride=stride,
        playback_fps=float(fps),
        wall_seconds=float(elapsed),
        rendering_fps=len(frames) / max(float(elapsed), 1e-9),
        simulation_fps=simulation_fps,
    )


__all__ = ["RenderReport", "ensure_mp4_export_available", "render_animation"]
