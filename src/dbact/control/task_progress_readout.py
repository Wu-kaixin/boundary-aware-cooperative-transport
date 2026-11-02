"""Experimental map-anchored scalar readout; no truth or physical-model access.

It never integrates a second correction into the map. The original gain and
quality/velocity/neighbor logic are preserved. Default controllers do not use it.
"""

from __future__ import annotations

import numpy as np

from dbact.sensing.registration_capture import RegistrationCapture
from dbact.sensing.task_observability import directional_certificate

ARMS = ("baseline", "certificate_only", "anchored_readout")


def anchored_candidate(original, record, direction, gain, quality, speed, yaw=False):
    """Return a scalar candidate plus explicit admission reason, without mutation."""
    report = {
        "admitted": False,
        "reason": "NO_FRESH_REGISTRATION",
        "candidate": original,
    }
    if record is None:
        return report
    if yaw:
        report["reason"] = "YAW_UNSUPPORTED"
        return report
    a, b, pre = record["A"], record["b"], record["m_pre"]
    if not (np.isfinite(a).all() and np.isfinite(b).all() and np.isfinite(pre).all()):
        report["reason"] = "NONFINITE"
        return report
    cert = directional_certificate(a, direction)
    report.update({k: v for k, v in cert.items() if k != "weights"})
    if not cert["identifiable"]:
        report["reason"] = "DIRECTION_UNIDENTIFIABLE"
    elif quality == "unavailable":
        report["reason"] = "ORIGINAL_QUALITY_UNAVAILABLE"
    elif record["clamped"]:
        report["reason"] = "REGISTRATION_CLAMPED"
    else:
        step = float(cert["weights"] @ b)
        report["relative_map_scalar"] = step
        # This is a conservative admission bound on map residual, not a claim
        # that residual equals physical frame displacement. Do not clip it.
        if not np.isfinite(step) or abs(step) > speed * record["dt"]:
            report["reason"] = "MAP_RESIDUAL_OUTSIDE_EXISTING_STEP_BOUND"
        else:
            report.update(
                admitted=True,
                reason="ADMITTED",
                candidate=float(gain * (np.dot(direction, pre) + step)),
            )
    return report


class ProgressReadout:
    """Opt-in instance adapter around the original progress feedback method."""

    def __init__(self, controller, agents, arm):
        if arm not in ARMS:
            raise ValueError(f"Unknown progress arm: {arm}")
        controller._ensure_state(agents)
        self.controller = controller
        self.arm = arm
        self.captures = {
            aid: RegistrationCapture(m, lambda: controller._frame)
            for aid, m in controller.maps.items()
        }
        self.readouts = {}
        original = controller._credible_task_progress

        def read(agent_id, object_id, local_map, direction):
            value = original(agent_id, object_id, local_map, direction)
            capture = self.captures[agent_id]
            record = capture.current(object_id)
            quality = controller._progress_quality[agent_id][object_id]
            report = {"admitted": False, "reason": "BASELINE", "candidate": value}
            if arm != "baseline":
                report = anchored_candidate(
                    value,
                    record,
                    direction,
                    controller.params.progress_estimate_gain,
                    quality,
                    local_map.max_object_speed,
                    local_map.estimate_yaw,
                )
            returned = report["candidate"] if arm == "anchored_readout" else value
            if arm == "anchored_readout" and report["admitted"]:
                # Preserve the existing hold rule using this arm's last accepted
                # scalar. A later unavailable observation must not jump back to
                # an unselected readout. Maps/quality/velocity stay untouched.
                controller._credible_progress[agent_id][object_id] = returned
            self.readouts[(agent_id, object_id)] = {
                "frame": controller._frame,
                "original": value,
                "returned": returned,
                "quality": quality,
                "decision": report,
                "m_post": local_map.object_displacement(object_id),
                "velocity": local_map.object_velocity(object_id),
                "gain": controller.params.progress_estimate_gain,
            }
            return returned

        controller._credible_task_progress = read
