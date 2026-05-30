import pytest

from src.robot.robot_module import RobotModule


class FakePublisher:
    def __init__(self):
        self.messages = []
        self.closed = False

    def publish(self, topic, message):
        self.messages.append((topic, message))

    def close(self):
        self.closed = True


class FakeSubscriber:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class FailingAdapter:
    def __init__(self):
        self.stop_all_called = False
        self.close_called = False

    def connect_all(self, *_args, **_kwargs):
        raise RuntimeError("connect failed")

    def stop_all(self):
        self.stop_all_called = True

    def close(self):
        self.close_called = True


def test_robot_module_cleans_up_when_connect_fails():
    module = RobotModule.__new__(RobotModule)
    module.running = False
    module.stopping = False
    module.robots_config = {
        "connection": {
            "conn_type": "sta",
            "proto_type": "udp",
            "retry_count": 0,
            "require_sn": True,
        },
        "chassis": {"stop_on_exit": True},
    }
    module.registry = type("Registry", (), {"robots": []})()
    module.adapter = FailingAdapter()
    module.publisher = FakePublisher()
    module.subscriber = FakeSubscriber()
    module.logger = type(
        "Logger",
        (),
        {
            "exception": lambda *args, **kwargs: None,
            "info": lambda *args, **kwargs: None,
        },
    )()

    with pytest.raises(RuntimeError, match="connect failed"):
        module.run()

    statuses = [message.status for _topic, message in module.publisher.messages]
    assert "starting" in statuses
    assert "error" in statuses
    assert "stopped" in statuses
    assert module.adapter.stop_all_called is True
    assert module.adapter.close_called is True
    assert module.publisher.closed is True
    assert module.subscriber.closed is True
