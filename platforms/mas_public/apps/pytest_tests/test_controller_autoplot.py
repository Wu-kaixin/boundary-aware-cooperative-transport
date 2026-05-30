from pathlib import Path

from src.controller.controller_module import ControllerModule


class FakePublisher:
    def publish(self, *_args, **_kwargs):
        pass

    def close(self):
        pass


class FakeSubscriber:
    def close(self):
        pass


class FakeRecorder:
    def __init__(self, experiment_dir: Path):
        self.experiment_dir = experiment_dir
        self.closed = False
        self.commands = []
        self.module_statuses = []

    def record_control_command(self, command):
        self.commands.append(command)

    def record_module_status(self, status, *_args, **_kwargs):
        self.module_statuses.append(status)

    def flush(self):
        pass

    def close(self):
        self.closed = True


def make_module(tmp_path, enabled: bool) -> ControllerModule:
    module = ControllerModule.__new__(ControllerModule)
    module.running = True
    module.robot_ids = ["robot_1"]
    module.controller_config = {"plot": {"enable_after_experiment": enabled}}
    module.controller = type("Controller", (), {"robot_mode": "chassis_lead"})()
    module.publisher = FakePublisher()
    module.subscriber = FakeSubscriber()
    module.robot_status_subscriber = FakeSubscriber()
    module.recorder = FakeRecorder(tmp_path)
    module.task_failed = False
    module.task_completed = False
    module.failure_message = ""
    module.safety_event_messages = []
    module.interrupted = False
    module.logger = type(
        "Logger",
        (),
        {
            "info": lambda *args, **kwargs: None,
            "warning": lambda *args, **kwargs: None,
            "error": lambda *args, **kwargs: None,
            "exception": lambda *args, **kwargs: None,
        },
    )()
    return module


def test_shutdown_runs_autoplot_then_check_when_enabled(tmp_path, monkeypatch):
    plot_calls = []
    check_calls = []

    class FakePlotter:
        def __init__(self, experiment_dir):
            self.experiment_dir = experiment_dir

        def plot_all(self):
            plot_calls.append(self.experiment_dir)
            return []

    def fake_check(experiment_dir):
        check_calls.append(experiment_dir)
        return type("Report", (), {"warnings": [], "errors": [], "ok": True})()

    monkeypatch.setattr("src.controller.controller_module.ExperimentPlotter", FakePlotter)
    monkeypatch.setattr("apps.check_experiment.check_experiment", fake_check)
    module = make_module(tmp_path, enabled=True)
    module.shutdown()
    assert module.recorder.closed is True
    assert module.recorder.commands[-1].commands[0].controller_mode == "shutdown"
    assert plot_calls == [tmp_path]
    assert check_calls == [tmp_path]


def test_shutdown_skips_autoplot_but_still_checks_when_disabled(tmp_path, monkeypatch):
    plot_calls = []
    check_calls = []

    class FakePlotter:
        def __init__(self, experiment_dir):
            self.experiment_dir = experiment_dir

        def plot_all(self):
            plot_calls.append(self.experiment_dir)
            return []

    def fake_check(experiment_dir):
        check_calls.append(experiment_dir)
        return type("Report", (), {"warnings": [], "errors": [], "ok": True})()

    monkeypatch.setattr("src.controller.controller_module.ExperimentPlotter", FakePlotter)
    monkeypatch.setattr("apps.check_experiment.check_experiment", fake_check)
    module = make_module(tmp_path, enabled=False)
    module.shutdown()
    assert module.recorder.closed is True
    assert plot_calls == []
    assert check_calls == [tmp_path]


def test_shutdown_skips_autoplot_and_check_after_interrupt(tmp_path, monkeypatch):
    plot_calls = []
    check_calls = []

    class FakePlotter:
        def __init__(self, experiment_dir):
            self.experiment_dir = experiment_dir

        def plot_all(self):
            plot_calls.append(self.experiment_dir)
            return []

    def fake_check(experiment_dir):
        check_calls.append(experiment_dir)
        return type("Report", (), {"warnings": [], "errors": [], "ok": True})()

    monkeypatch.setattr("src.controller.controller_module.ExperimentPlotter", FakePlotter)
    monkeypatch.setattr("apps.check_experiment.check_experiment", fake_check)
    module = make_module(tmp_path, enabled=True)
    module.interrupted = True
    module.shutdown()

    assert module.recorder.closed is True
    assert plot_calls == []
    assert check_calls == []


def test_failed_shutdown_records_prior_safety_events(tmp_path, monkeypatch):
    monkeypatch.setattr("src.controller.controller_module.ExperimentPlotter", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        "apps.check_experiment.check_experiment",
        lambda *_args, **_kwargs: type("Report", (), {"warnings": [], "errors": [], "ok": True})(),
    )
    module = make_module(tmp_path, enabled=False)
    module.task_failed = True
    module.failure_message = "tracking_lost: untracked robots=['robot_3']"
    module.safety_event_messages = [
        "world_out_of_bounds: robots=['robot_1', 'robot_2']",
        "tracking_lost: untracked robots=['robot_3']",
    ]

    module.shutdown()

    assert module.recorder.module_statuses[-1].status == "failed"
    assert module.recorder.module_statuses[-1].message == (
        "tracking_lost: untracked robots=['robot_3']; "
        "prior_events=[\"world_out_of_bounds: robots=['robot_1', 'robot_2']\"]"
    )

