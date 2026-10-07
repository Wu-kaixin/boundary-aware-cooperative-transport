"""D2 - the closed-loop supervisor.

    SEARCH -> DISCOVER -> ENCLOSE -> CONTACT_READY -> TRANSPORT -> BRAKE -> HOLD

Every transition is a guard on a measured quantity, never a frame number. The
500-frame budget is a deadline the whole episode is scored against, not a
schedule the phases are cut to; a run that encloses in 80 frames spends the
saving on transport, and a run that has not enclosed by frame 300 fails that
deadline rather than being marched into a phase it is not ready for.

Two properties are enforced structurally rather than checked afterwards.

**Monotonicity.** The supervisor never returns to an earlier phase. Enclosure
quality dips every time the cargo breaks loose, and a machine that fell back to
ENCLOSE on each dip would re-arm the dwell timer and chatter between two states
at the stick-slip frequency. Progress being one-way is what makes "transport was
activated after enclosure" a fact about the run rather than about which frame you
sampled.

**Dwell.** CONTACT_READY needs a quorum held for ``contact_dwell`` consecutive
steps. A single step of quorum is a transient: robots swing through the contact
band on their way to the cage ring, and a machine that armed on the instantaneous
count started pushing while the enclosure was still open on the far side.

Every guard reads decentralised quantities -- how many robots hold enough
boundary in their own maps, how many report themselves in the contact band, how
much of the merged map's angular extent is covered, what the robots' own
registration says the cargo has travelled. None of them is a simulator pose.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class Phase(IntEnum):
    """Ordered so that ``>=`` means "at least this far along"."""

    SEARCH = 0
    DISCOVER = 1
    ENCLOSE = 2
    CONTACT_READY = 3
    TRANSPORT = 4
    BRAKE = 5
    HOLD = 6

    @property
    def label(self) -> str:
        return self.name


@dataclass
class PhaseGates:
    """Guard thresholds. Each one is a quantity the robots can measure."""

    informed_fraction: float = 0.55
    map_coverage_min: float = 0.70
    contact_quorum: int = 4
    contact_dwell: int = 20
    transport_quorum: int = 3
    brake_fraction: float = 0.80
    hold_tolerance: float = 0.0
    hold_speed: float = 0.02
    hold_dwell: int = 1
    # Below this push-side count the transport push is treated as lost. It is not
    # the count required to start transport, and it is not a brake permission.
    sustain_contacts: int = 1
    contact_loss_dwell: int = 20
    contact_regain_dwell: int = 20
    pause_limit: int = 80
    estimate_loss_dwell: int = 20
    recover_on_brake_progress: bool = False


@dataclass
class PhaseSignals:
    """One step of supervisor input, all of it estimated on board."""

    agent_count: int = 0
    informed_agents: int = 0
    map_coverage: float = 0.0
    contact_ready: int = 0
    transport_active: int = 0
    progress: float = 0.0
    target_distance: float = 1.0
    estimated_speed: float = 0.0
    stopping_distance: float = 0.0
    estimate_quality: str = "credible"


@dataclass
class PhaseMonitor:
    """Monotone event-driven phase machine with a dwell on the contact quorum."""

    gates: PhaseGates = field(default_factory=PhaseGates)
    phase: Phase = Phase.SEARCH
    frame: int = 0
    quorum_streak: int = 0
    entered: dict[str, int] = field(default_factory=dict)
    history: list[int] = field(default_factory=list)
    settle_streak: int = 0
    loss_streak: int = 0
    regain_streak: int = 0
    unavailable_streak: int = 0
    pause_frames: int = 0
    push_suspended: bool = False
    brake_reason: str | None = None
    overshoot: bool = False
    short_of_target: bool = False
    contact_recovery_failed: bool = False
    estimate_stopped: bool = False

    def __post_init__(self) -> None:
        self.entered.setdefault(Phase.SEARCH.label, 0)

    def update(self, signals: PhaseSignals, frame: int) -> Phase:
        self.frame = int(frame)

        if signals.contact_ready >= self.gates.contact_quorum:
            self.quorum_streak += 1
        else:
            self.quorum_streak = 0
        self._update_response_clocks(signals)

        while True:
            nxt = self._next(signals)
            if nxt is None:
                break
            if nxt is Phase.BRAKE and self.phase is not Phase.BRAKE:
                self._mark_brake(signals)
            self.phase = nxt
            self.entered.setdefault(nxt.label, self.frame)
        self.history.append(int(self.phase))
        return self.phase

    def _update_response_clocks(self, signals: PhaseSignals) -> None:
        gates = self.gates
        if signals.transport_active < gates.sustain_contacts:
            self.loss_streak += 1
            self.regain_streak = 0
        elif signals.transport_active >= gates.transport_quorum:
            self.regain_streak += 1
            self.loss_streak = 0
        else:
            # Between the loss floor and the start quorum: neither trip nor resume.
            self.loss_streak = 0
            self.regain_streak = 0
        if signals.estimate_quality == "unavailable":
            self.unavailable_streak += 1
        else:
            self.unavailable_streak = 0
        if self.phase is Phase.BRAKE and self._hold_ready(signals):
            self.settle_streak += 1
        elif self.phase is not Phase.BRAKE:
            self.settle_streak = 0
        if self.phase is Phase.TRANSPORT and self.loss_streak >= gates.contact_loss_dwell:
            if not self.push_suspended:
                self.push_suspended = True
                self.pause_frames = 0
            else:
                self.pause_frames += 1
        elif (
            self.push_suspended
            and self.regain_streak >= gates.contact_regain_dwell
            and not self._brake_due(signals)
        ):
            self.push_suspended = False
            self.pause_frames = 0
        elif self.push_suspended:
            self.pause_frames += 1

    def _brake_due(self, signals: PhaseSignals) -> bool:
        """Estimated remaining distance is inside the declared stop.

        ``stopping_distance`` is computed by the controller from its estimated
        speed and the transport stop model. The brake fraction remains as the
        minimum lead when that distance is not supplied. Neither input is a
        simulator pose.
        """
        remaining = float(signals.target_distance) - float(signals.progress)
        fraction_lead = (1.0 - self.gates.brake_fraction) * float(signals.target_distance)
        return remaining <= max(float(signals.stopping_distance), fraction_lead)

    def _hold_ready(self, signals: PhaseSignals) -> bool:
        arrived = signals.progress >= signals.target_distance - self.gates.hold_tolerance
        slow = abs(signals.estimated_speed) <= self.gates.hold_speed
        known = signals.estimate_quality != "unavailable"
        return arrived and slow and known

    def _mark_brake(self, signals: PhaseSignals) -> None:
        tolerance = self.gates.hold_tolerance
        self.overshoot = signals.progress > signals.target_distance + tolerance
        self.short_of_target = signals.progress < signals.target_distance - tolerance
        self.push_suspended = False

    def _stop_for_estimate(self, signals: PhaseSignals) -> bool:
        return (
            signals.estimate_quality == "unavailable"
            and self.unavailable_streak >= self.gates.estimate_loss_dwell
            and signals.progress > 0.0
        )

    def _next(self, signals: PhaseSignals) -> Phase | None:
        g = self.gates
        if self.phase == Phase.SEARCH:
            return Phase.DISCOVER if signals.informed_agents >= 1 else None
        if self.phase == Phase.DISCOVER:
            enough = signals.informed_agents >= max(1, int(round(g.informed_fraction * max(signals.agent_count, 1))))
            return Phase.ENCLOSE if enough and signals.map_coverage >= g.map_coverage_min else None
        if self.phase == Phase.ENCLOSE:
            return Phase.CONTACT_READY if self.quorum_streak >= g.contact_dwell else None
        if self.phase == Phase.CONTACT_READY:
            # contact_quorum armed this phase. transport_quorum may arm the push.
            # Neither count is allowed to withhold braking once the estimated
            # stop point has been reached or the motion estimate has failed.
            if self._brake_due(signals):
                self.brake_reason = "estimated_stop_from_contact_ready"
                return Phase.BRAKE
            if self._stop_for_estimate(signals):
                self.brake_reason = "estimate_unavailable"
                self.estimate_stopped = True
                return Phase.BRAKE
            if signals.transport_active >= self.gates.transport_quorum:
                return Phase.TRANSPORT
            return None
        if self.phase == Phase.TRANSPORT:
            if self._brake_due(signals):
                self.brake_reason = "estimated_stop"
                return Phase.BRAKE
            if self._stop_for_estimate(signals):
                self.brake_reason = "estimate_unavailable"
                self.estimate_stopped = True
                return Phase.BRAKE
            if self.push_suspended and self.pause_frames >= self.gates.pause_limit:
                self.brake_reason = "contact_recovery_failed"
                self.contact_recovery_failed = True
                return Phase.BRAKE
            return None
        if self.phase == Phase.BRAKE:
            if self._hold_ready(signals) and self.settle_streak >= self.gates.hold_dwell:
                return Phase.HOLD
            return None
        return None

    # ------------------------------------------------------------------ #

    def frame_of(self, phase: Phase) -> int | None:
        return self.entered.get(phase.label)

    def reached(self, phase: Phase) -> bool:
        return self.phase >= phase

    def as_dict(self) -> dict:
        return {
            "final_phase": self.phase.label,
            "entered": dict(self.entered),
            "first_detection_frame": self.entered.get(Phase.DISCOVER.label),
            "enclosure_frame": self.entered.get(Phase.ENCLOSE.label),
            "contact_ready_frame": self.entered.get(Phase.CONTACT_READY.label),
            "transport_frame": self.entered.get(Phase.TRANSPORT.label),
            "brake_frame": self.entered.get(Phase.BRAKE.label),
            "hold_frame": self.entered.get(Phase.HOLD.label),
            "brake_reason": self.brake_reason,
            "push_suspended": self.push_suspended,
            "overshoot": self.overshoot,
            "short_of_target": self.short_of_target,
            "contact_recovery_failed": self.contact_recovery_failed,
            "estimate_stopped": self.estimate_stopped,
        }


__all__ = ["Phase", "PhaseGates", "PhaseSignals", "PhaseMonitor"]
