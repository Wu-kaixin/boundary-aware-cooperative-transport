"""Shared conference-demo and paper visual language."""

from __future__ import annotations

from dataclasses import dataclass


PHASE_COLORS = {
    "SEARCH": "#4aa8d8",
    "MAP": "#168aad",
    "ENCLOSE": "#3a9d5d",
    "CONTACT_READY": "#d69e2e",
    "TRANSPORT": "#d94841",
    "BRAKE": "#dd6b20",
    "HOLD": "#53616f",
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
    agent_face: str
    agent_core: str
    safety_ring: str
    communication_ring: str
    voronoi: str
    color_agents_by_mode: bool
    show_ids: bool
    show_hud: bool
    show_sensor: bool
    show_map: bool
    show_cage: bool
    show_truth_debug: bool
    show_safety: bool
    show_communication: bool
    show_voronoi: bool
    trail_frames: int | None


STYLES = {
    "demo": VisualStyle(
        name="demo",
        figure_face="#eef0f2",
        world_face="#d7dadd",
        panel_face="#1f2937",
        text="#e2e8f0",
        muted="#aab4c3",
        grid="#b8bec4",
        cargo_face="#ff7f0e",
        cargo_edge="#9a4d00",
        goal="#c7352d",
        target="#8f1d17",
        detected="#00a9c6",
        mapped="#0f766e",
        cage="#666666",
        trajectory="#52606d",
        agent_face="#7ec8ee",
        agent_core="#111111",
        safety_ring="#70bde8",
        communication_ring="#8f969d",
        voronoi="#404040",
        color_agents_by_mode=False,
        show_ids=True,
        show_hud=True,
        show_sensor=True,
        show_map=True,
        show_cage=True,
        show_truth_debug=False,
        show_safety=True,
        show_communication=True,
        show_voronoi=True,
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
        agent_face="#7ec8ee",
        agent_core="#111111",
        safety_ring="#70bde8",
        communication_ring="#9ca3af",
        voronoi="#4b5563",
        color_agents_by_mode=False,
        show_ids=False,
        show_hud=False,
        show_sensor=False,
        show_map=True,
        show_cage=True,
        show_truth_debug=False,
        show_safety=False,
        show_communication=False,
        show_voronoi=False,
        trail_frames=None,
    ),
    "debug": VisualStyle(
        name="debug",
        figure_face="#f1f5f9",
        world_face="#d7dadd",
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
        agent_face="#7ec8ee",
        agent_core="#111111",
        safety_ring="#70bde8",
        communication_ring="#8f969d",
        voronoi="#303030",
        color_agents_by_mode=True,
        show_ids=True,
        show_hud=True,
        show_sensor=True,
        show_map=True,
        show_cage=True,
        show_truth_debug=True,
        show_safety=True,
        show_communication=True,
        show_voronoi=True,
        trail_frames=None,
    ),
}


def get_style(view_mode: str) -> VisualStyle:
    try:
        return STYLES[str(view_mode).lower()]
    except KeyError as exc:
        raise ValueError(f"unknown view mode {view_mode!r}; choose demo, paper, or debug") from exc


__all__ = ["MODE_COLORS", "PHASE_COLORS", "STYLES", "VisualStyle", "get_style"]
