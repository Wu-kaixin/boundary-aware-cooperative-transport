"""Shared conference-demo and paper visual language."""

from __future__ import annotations

from dataclasses import dataclass


PHASE_COLORS = {
    "SEARCH": "#2563eb",
    "MAP": "#0891b2",
    "ENCLOSE": "#16a34a",
    "CONTACT_READY": "#ca8a04",
    "TRANSPORT": "#dc2626",
    "BRAKE": "#ea580c",
    "HOLD": "#475569",
}

MODE_COLORS = {
    "explore": PHASE_COLORS["SEARCH"],
    "search": PHASE_COLORS["SEARCH"],
    "sweep": PHASE_COLORS["SEARCH"],
    "relay": PHASE_COLORS["MAP"],
    "map_boundary": PHASE_COLORS["MAP"],
    "approach": "#22c55e",
    "redeploy": "#84cc16",
    "cage": PHASE_COLORS["ENCLOSE"],
    "push": PHASE_COLORS["TRANSPORT"],
    "convoy": "#7c3aed",
    "brake": PHASE_COLORS["BRAKE"],
    "hold": PHASE_COLORS["HOLD"],
}


@dataclass(frozen=True)
class VisualStyle:
    name: str
    figure_face: str
    world_face: str
    panel_face: str
    text: str
    muted: str
    grid: str
    cargo_face: str
    cargo_edge: str
    goal: str
    target: str
    detected: str
    mapped: str
    cage: str
    trajectory: str
    show_ids: bool
    show_hud: bool
    show_sensor: bool
    show_map: bool
    show_cage: bool
    show_truth_debug: bool
    trail_frames: int | None


STYLES = {
    "demo": VisualStyle(
        name="demo",
        figure_face="#f8fafc",
        world_face="#f8fafc",
        panel_face="#0f172a",
        text="#e2e8f0",
        muted="#94a3b8",
        grid="#cbd5e1",
        cargo_face="#fbbf24",
        cargo_edge="#78350f",
        goal="#dc2626",
        target="#991b1b",
        detected="#06b6d4",
        mapped="#0f766e",
        cage="#16a34a",
        trajectory="#475569",
        show_ids=False,
        show_hud=True,
        show_sensor=True,
        show_map=True,
        show_cage=True,
        show_truth_debug=False,
        trail_frames=160,
    ),
    "paper": VisualStyle(
        name="paper",
        figure_face="#ffffff",
        world_face="#ffffff",
        panel_face="#ffffff",
        text="#111827",
        muted="#6b7280",
        grid="#e5e7eb",
        cargo_face="#f5b642",
        cargo_edge="#3f2a0c",
        goal="#b91c1c",
        target="#7f1d1d",
        detected="#0891b2",
        mapped="#0f766e",
        cage="#15803d",
        trajectory="#374151",
        show_ids=False,
        show_hud=False,
        show_sensor=False,
        show_map=True,
        show_cage=True,
        show_truth_debug=False,
        trail_frames=None,
    ),
    "debug": VisualStyle(
        name="debug",
        figure_face="#f1f5f9",
        world_face="#f8fafc",
        panel_face="#111827",
        text="#f8fafc",
        muted="#9ca3af",
        grid="#cbd5e1",
        cargo_face="#f59e0b",
        cargo_edge="#111827",
        goal="#ef4444",
        target="#991b1b",
        detected="#22d3ee",
        mapped="#0d9488",
        cage="#22c55e",
        trajectory="#64748b",
        show_ids=True,
        show_hud=True,
        show_sensor=True,
        show_map=True,
        show_cage=True,
        show_truth_debug=True,
        trail_frames=None,
    ),
}


def get_style(view_mode: str) -> VisualStyle:
    try:
        return STYLES[str(view_mode).lower()]
    except KeyError as exc:
        raise ValueError(f"unknown view mode {view_mode!r}; choose demo, paper, or debug") from exc


__all__ = ["MODE_COLORS", "PHASE_COLORS", "STYLES", "VisualStyle", "get_style"]
