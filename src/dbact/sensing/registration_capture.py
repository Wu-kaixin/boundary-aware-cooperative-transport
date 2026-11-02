"""Opt-in copies of actual translation-solver inputs; no evaluator dependencies.

This research observer wraps one map instance, not the global map class. The
original matching, trimming, registration, fusion and motion commits run once.
"""

from __future__ import annotations

import numpy as np


class RegistrationCapture:
    def __init__(self, local_map, frame):
        self.map = local_map
        self.frame = frame
        self.epoch = -1
        self.records = {}
        self.pre = {}
        self.fusion = {}
        self.valid_matches = 0
        register = local_map.register
        match = local_map._match_rows
        solve = local_map._solve_translation
        fusion = local_map._fusion_drift

        def register_observed(scan, dt):
            self.epoch = int(frame())
            self.records = {}
            self.fusion = {}
            names = set(local_map.displacement) | set(map(str, scan.object_ids))
            self.pre = {n: local_map.object_displacement(n) for n in names}
            return register(scan, dt)

        def match_observed(*args):
            rows, valid = match(*args)
            self.valid_matches = int(np.count_nonzero(valid))
            return rows, valid

        def solve_observed(object_id, n, residual, dt, reference):
            # These are exactly the post-MAD-trim arguments, before map mutation.
            record = {
                "frame": self.epoch,
                "dt": float(dt),
                "A": n.copy(),
                "b": residual.copy(),
                "reference": reference.copy(),
                "m_pre": self.pre.get(object_id, np.zeros(2)).copy(),
                "matched_before_trim": self.valid_matches,
            }
            result = solve(object_id, n, residual, dt, reference)
            record.update(
                translation=result.translation.copy(),
                matches=result.matches,
                quality=result.quality,
                clamped=result.clamped,
                conditioning=result.conditioning,
                rms=result.residual_rms,
            )
            self.records[object_id] = record
            return result

        def fusion_observed(rows, delta):
            # Fusion uses its own POST-fusion normals; it is not registration A.
            parts = {}
            for name in np.unique(local_map._objects[rows]):
                selected = local_map._objects[rows] == name
                normals = local_map._normals[rows[selected]].copy()
                parts[str(name)] = {
                    "A": normals,
                    "b": np.einsum("ij,ij->i", normals, delta[selected]),
                    "cells": int(np.count_nonzero(selected)),
                }
            result = fusion(rows, delta)
            for name, data in parts.items():
                data["translation"] = result.get(name, np.zeros(2)).copy()
                data["admitted"] = name in result
            self.fusion = parts
            return result

        local_map.register = register_observed
        local_map._match_rows = match_observed
        local_map._solve_translation = solve_observed
        local_map._fusion_drift = fusion_observed

    def current(self, object_id):
        if self.epoch != int(self.frame()):
            return None
        return self.records.get(object_id)

    def current_fusion(self, object_id):
        return self.fusion.get(object_id) if self.epoch == int(self.frame()) else None
