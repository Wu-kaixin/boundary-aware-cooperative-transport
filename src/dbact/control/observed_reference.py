"""Experimental displacement readout against a fixed, locally observed reference.

The reference contains own/one-hop range returns, not prior geometry. Its
centroid is a material reference point, NOT a known center of mass. The initial
offset is inherited from the old map estimate; bootstrap bias is not corrected.
This readout never changes the safety map or its boundary velocities.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree

from dbact.types import BoundaryView


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


@dataclass(frozen=True)
class ReferenceReading:
    displacement: np.ndarray
    velocity: np.ndarray
    matches: int
    residual_rms: float
    conditioning: float
    angle: float


class ObservedReferenceTracker:
    """Five-step point-to-plane SE(2) registration to an immutable observed scan.

    A failed match abstains; it does not return a stale position as a fresh vote.
    Unobservable yaw (e.g. an ideal circle) is handled by the least-squares
    pseudoinverse, while translation needs the old full-vector rank threshold.
    Residual/rank checks are empirical admission checks, not error certificates.
    """

    def __init__(self, *, gate=.12, normal_cosine=.5, max_speed=.6,
                 max_yaw_rate=.8, velocity_filter=.35, radial_readout=False,
                 range_noise_std=.01, innovation_trim=False, support_gate=False):
        values = [gate, normal_cosine, max_speed, max_yaw_rate, velocity_filter]
        if (not np.isfinite(values).all() or gate <= 0 or max_speed <= 0
                or max_yaw_rate <= 0 or not -1 <= normal_cosine <= 1
                or not 0 <= velocity_filter <= 1):
            raise ValueError("Invalid reference tracker parameters")
        self.gate, self.normal_cosine = gate, normal_cosine
        self.max_speed, self.max_yaw_rate = max_speed, max_yaw_rate
        self.velocity_filter = velocity_filter
        self.radial_readout = radial_readout
        self.range_noise_std = range_noise_std
        self.innovation_trim = bool(innovation_trim)
        self.support_gate = bool(support_gate)
        self.support_minor_rms = float("nan")
        if support_gate and (not np.isfinite(range_noise_std) or range_noise_std < 0):
            raise ValueError("Support gate requires a finite nonnegative range noise scale")
        if radial_readout and (not np.isfinite(range_noise_std) or range_noise_std <= 0):
            raise ValueError("Radial readout requires a positive observed range noise scale")
        self.pivot_status = "not_enabled"
        self.pivot_rms = float("nan")
        self.pivot_shift = np.zeros(2)
        self.initialized = False
        self.initial_time = None
        self.last_reading = None
        self.reason = "not_initialized"

    @staticmethod
    def _conditioning(normals):
        eigenvalues = np.linalg.eigvalsh(normals.T @ normals)
        return float(max(eigenvalues[0], 0.) / max(eigenvalues[-1], 1e-12))

    def initialize(self, scan: BoundaryView, bootstrap_displacement, timestamp):
        if self.initialized:
            raise RuntimeError("An observed reference must not be silently replaced")
        if len(scan) < 8 or self._conditioning(scan.normals) <= 1e-3:
            self.reason = "bootstrap_unobservable"
            return None
        offset = np.asarray(bootstrap_displacement, dtype=float).reshape(2)
        if not np.isfinite(offset).all() or not np.isfinite(timestamp):
            raise ValueError("Non-finite bootstrap")
        self.anchor = scan.points.mean(axis=0)
        self.reference = scan.points.copy() - self.anchor
        if self.radial_readout:
            from dbact.control.observed_geometry import radial_pivot
            pivot, self.pivot_status, self.pivot_rms = radial_pivot(scan.points, self.range_noise_std)
            if pivot is not None:
                self.pivot_shift = pivot-self.anchor
        self.normals = scan.normals.copy()
        self.tree = cKDTree(self.reference)
        self.offset = offset.copy()
        self.translation = np.zeros(2)
        self.angle = 0.
        self.velocity = np.zeros(2)
        self.last_time = float(timestamp)
        self.initial_time = float(timestamp)
        self.initialized = True
        self.reason = "bootstrap_only"
        # Initializing position does not measure velocity, so no stopping vote.
        return None

    def initialization_record(self) -> dict:
        """Record the actual immutable permitted reference, without evaluator truth."""
        if not self.initialized:
            return {"initialized": False}
        return dict(initialized=True, time=self.initial_time, anchor=self.anchor.tolist(),
                    offset=self.offset.tolist(), pivot_shift=self.pivot_shift.tolist(),
                    pivot_status=self.pivot_status, reference_points_relative=self.reference.tolist(),
                    reference_normals=self.normals.tolist())

    def _rows(self, scan, translation, angle):
        matrix = rotation(angle)
        local = (scan.points-self.anchor-translation) @ matrix
        distance, index = self.tree.query(local, k=1)
        normals = self.normals[index] @ matrix.T
        lever = self.reference[index] @ matrix.T
        predicted = self.anchor + translation + lever
        aligned = np.einsum('ij,ij->i', normals, scan.normals) >= self.normal_cosine
        keep = (distance <= self.gate) & aligned
        n, lever = normals[keep], lever[keep]
        residual = np.einsum('ij,ij->i', n, scan.points[keep]-predicted[keep])
        if len(residual) >= 8:
            score = residual
            if self.innovation_trim:
                # Different surface normals give different residuals for the
                # same rigid motion. Trim unexplained innovation, not that
                # coherent motion. This remains an empirical ICP candidate;
                # a small fitted residual is not a correspondence certificate.
                turn = n[:,1]*lever[:,0]-n[:,0]*lever[:,1]
                design = np.column_stack([n, turn])
                score = residual-design @ np.linalg.lstsq(design, residual, rcond=1e-3)[0]
            center = np.median(score)
            trim = np.abs(score-center) <= 2.5*(np.median(np.abs(score-center))+1e-6)
            if np.count_nonzero(trim) >= 8:
                n, lever, residual = n[trim], lever[trim], residual[trim]
        return n, lever, residual

    def update(self, scan: BoundaryView, timestamp):
        self.last_reading = None
        self.support_minor_rms = float("nan")
        if not self.initialized:
            self.reason = 'not_initialized'
            return None
        elapsed = float(timestamp)-self.last_time
        if not np.isfinite(timestamp) or elapsed <= 0:
            raise ValueError('Fresh increasing observation timestamps required')
        if len(scan) < 8:
            self.reason = 'insufficient_returns'
            return None
        translation, angle = self.translation.copy(), self.angle
        for _ in range(5):
            n, lever, residual = self._rows(scan, translation, angle)
            if len(n) < 8 or self._conditioning(n) <= 1e-3:
                self.reason = 'translation_unobservable'
                return None
            turn = n[:,1]*lever[:,0]-n[:,0]*lever[:,1]
            design = np.column_stack([n, turn])
            delta = np.linalg.lstsq(design, residual, rcond=1e-3)[0]
            translation += delta[:2]
            angle += float(delta[2])
            if np.linalg.norm(delta) <= 1e-6:
                break
        n, matched_points, residual = self._rows(scan, translation, angle)
        conditioning = self._conditioning(n) if len(n) else 0.
        if len(n) < 8 or conditioning <= 1e-3:
            self.reason = 'final_match_unobservable'
            return None
        if self.support_gate:
            # Noisy local PCA normals can manufacture apparent full rank on a
            # single flat patch. Inspect the *matched spatial support* as well.
            # Centering makes this empirical gate rigid-frame invariant; the
            # three-noise-scale margin is not an error/confidence certificate.
            centered = matched_points - matched_points.mean(axis=0)
            eigenvalues = np.linalg.eigvalsh(centered.T @ centered / len(centered))
            self.support_minor_rms = float(np.sqrt(max(eigenvalues[0], 0.)))
            if self.support_minor_rms <= 3 * self.range_noise_std + 1e-12:
                self.reason = 'support_nearly_linear'
                return None
        motion = translation-self.translation
        if (np.linalg.norm(motion) > self.max_speed*elapsed + 1e-10
                or abs(angle-self.angle) > self.max_yaw_rate*elapsed + 1e-10):
            self.reason = 'motion_bound_exceeded'
            return None
        rms = float(np.sqrt(np.mean(residual**2)))
        if not np.isfinite(rms) or rms > self.gate:
            self.reason = 'residual_rejected'
            return None
        a = self.velocity_filter
        readout = translation
        readout_motion = motion
        if self.pivot_status == "accepted":
            # Change reference point, not the registration fit or admission.
            # An unobservable circular yaw must not move its estimated center.
            readout = translation + (rotation(angle)-np.eye(2)) @ self.pivot_shift
            readout_motion = motion + (rotation(angle)-rotation(self.angle)) @ self.pivot_shift
        self.velocity = (1-a)*self.velocity + a*readout_motion/elapsed
        self.translation, self.angle, self.last_time = translation, angle, float(timestamp)
        self.last_reading = ReferenceReading(self.offset+readout, self.velocity.copy(),
                                             len(n), rms, conditioning, angle)
        self.reason = 'accepted'
        return self.last_reading
