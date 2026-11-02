"""Experimental initial-reference fit under an explicit translation hypothesis.

Fitted zero yaw is NOT measured zero physical rotation. A rotating noncircular
object can bias this readout. No center-error or registration certificate is
provided; the evaluator must retain rotation/model-mismatch failures.
"""
from __future__ import annotations

import numpy as np

from dbact.control.initial_task_reference import TaskReferenceTracker


class TranslationTaskReferenceTracker(TaskReferenceTracker):
    """Use existing bootstrap, correspondence, support, speed and filter rules."""

    def _motion_increment(self, normals, lever, residual):
        translation = np.linalg.lstsq(normals, residual, rcond=1e-3)[0]
        return np.r_[translation, 0.]

    def _geometry_rejection(self, normals, matched_points):
        # No yaw parameter is estimated. Translation rank and matched spatial
        # support are still checked by the inherited update; do not forge yaw.
        return None

    def initialization_record(self):
        record = super().initialization_record()
        record.update(reference_kind='translation_from_initial_observed_geometry',
            rotation_model='translation_hypothesis_not_measured_zero_yaw',
            registration_certified=False)
        return record
