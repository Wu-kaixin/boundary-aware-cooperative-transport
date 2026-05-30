from src.optitrack.natnet_adapter import NatNetRigidBody
from src.optitrack.tracking_validator import TrackingValidator


def body(x: float, timestamp: float = 1.0, tracked: bool = True) -> NatNetRigidBody:
    return NatNetRigidBody(
        name="Rigid Body 001",
        position=(x, 0.0, 0.0),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        tracked=tracked,
        timestamp=timestamp,
        rigid_body_id=1,
    )


def test_tracking_validator_disabled_passes_frames_through():
    bodies = [body(0.0)]
    assert TrackingValidator({"enabled": False}).apply(bodies) is bodies


def test_tracking_validator_rejects_position_jump_when_enabled():
    validator = TrackingValidator(
        {
            "enabled": True,
            "reject_position_jump": True,
            "max_position_jump_m": 0.5,
            "publish_untracked": True,
        }
    )

    first = validator.apply([body(0.0, timestamp=1.0)])[0]
    jumped = validator.apply([body(2.0, timestamp=2.0)])[0]

    assert first.tracked is True
    assert jumped.tracked is False
    assert jumped.position == first.position


def test_tracking_validator_can_drop_rejected_untracked_body():
    validator = TrackingValidator(
        {
            "enabled": True,
            "reject_position_jump": True,
            "max_position_jump_m": 0.5,
            "publish_untracked": False,
        }
    )

    validator.apply([body(0.0, timestamp=1.0)])

    assert validator.apply([body(2.0, timestamp=2.0)]) == []


def test_tracking_validator_publishes_timed_out_last_pose(monkeypatch):
    times = iter([1.0, 1.3])
    monkeypatch.setattr("src.optitrack.tracking_validator.now_s", lambda: next(times))
    validator = TrackingValidator(
        {
            "enabled": True,
            "tracking_timeout_enabled": True,
            "tracking_timeout_ms": 200,
            "publish_untracked": True,
        }
    )

    validator.apply([body(0.0, timestamp=1.0)])
    timed_out = validator.apply([])[0]

    assert timed_out.tracked is False
    assert timed_out.position == (0.0, 0.0, 0.0)

