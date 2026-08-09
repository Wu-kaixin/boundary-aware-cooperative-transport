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


@dataclass
class PhaseMonitor:
    """Monotone event-driven phase machine with a dwell on the contact quorum."""

    gates: PhaseGates = field(default_factory=PhaseGates)
    phase: Phase = Phase.SEARCH
    frame: int = 0
    quorum_streak: int = 0
    entered: dict[str, int] = field(default_factory=dict)
    history: list[int] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.entered.setdefault(Phase.SEARCH.label, 0)

    def update(self, signals: PhaseSignals, frame: int) -> Phase:
        self.frame = int(frame)

        if signals.contact_ready >= self.gates.contact_quorum:
            self.quorum_streak += 1
        else:
            self.quorum_streak = 0

        while True:
            nxt = self._next(signals)
            if nxt is None:
                break
            self.phase = nxt
            self.entered.setdefault(nxt.label, self.frame)
        self.history.append(int(self.phase))
        return self.phase

    def _next(self, s: PhaseSignals) -> Phase | None:
        g = self.gates
        if self.phase == Phase.SEARCH:
            return Phase.DISCOVER if s.informed_agents >= 1 else None
        if self.phase == Phase.DISCOVER:
            enough = s.informed_agents >= max(1, int(round(g.informed_fraction * max(s.agent_count, 1))))
            return Phase.ENCLOSE if enough and s.map_coverage >= g.map_coverage_min else None
        if self.phase == Phase.ENCLOSE:
            return Phase.CONTACT_READY if self.quorum_streak >= g.contact_dwell else None
        if self.phase == Phase.CONTACT_READY:
            return Phase.TRANSPORT if s.transport_active >= g.transport_quorum else None
        if self.phase == Phase.TRANSPORT:
            return Phase.BRAKE if s.progress >= g.brake_fraction * s.target_distance else None
        if self.phase == Phase.BRAKE:
            return Phase.HOLD if s.progress >= s.target_distance - g.hold_tolerance else None
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
        }


__all__ = ["Phase", "PhaseGates", "PhaseSignals", "PhaseMonitor"]
