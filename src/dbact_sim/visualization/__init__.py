"""Trace-driven research visualization with backwards-compatible helpers."""

from __future__ import annotations

from . import legacy as _legacy
from .legacy import (
    LivePaperViewer,
    animate_simulation,
    plot_coverage_curve,
    plot_paper_frame,
    plot_snapshot,
    plot_trajectories,
    write_paper_figures,
)
from .renderer import DemoVisualizer, ResearchVisualizer
from .styles import MODE_COLORS, PHASE_COLORS, STYLES, VisualStyle, get_style

# Existing callers and tests treat these as module attributes.
animation = _legacy.animation
_configure_ffmpeg_writer = _legacy._configure_ffmpeg_writer

__all__ = [
    "DemoVisualizer",
    "LivePaperViewer",
    "MODE_COLORS",
    "PHASE_COLORS",
    "ResearchVisualizer",
    "STYLES",
    "VisualStyle",
    "animate_simulation",
    "get_style",
    "plot_coverage_curve",
    "plot_paper_frame",
    "plot_snapshot",
    "plot_trajectories",
    "write_paper_figures",
]
