"""Simulator ground-truth local boundary sensor (ray casting).

This module is the paper-facing entry point for perception assumptions:
the controller never sees the full cargo polygon—only ray-cast visible
boundary measurements with estimated normals.
"""

from __future__ import annotations

from dbact.local_sensing import LocalBoundarySensor

# Canonical alias used in docs / paper-grade wiring.
RayCastBoundarySensor = LocalBoundarySensor

__all__ = ["RayCastBoundarySensor", "LocalBoundarySensor"]
