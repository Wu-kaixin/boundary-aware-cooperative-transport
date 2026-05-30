from src.controller.experiment_logger import ExperimentLogger


def test_experiment_logger_snapshots_nested_controller_configs(tmp_path, monkeypatch):
    config_dir = tmp_path / "configs"
    (config_dir / "controllers").mkdir(parents=True)
    (config_dir / "system.yaml").write_text("system:\n  name: test\n", encoding="utf-8")
    (config_dir / "controller.yaml").write_text(
        "controller:\n  type: manual\n  robot_mode: free\n", encoding="utf-8"
    )
    (config_dir / "controllers" / "manual.yaml").write_text("chassis_vx: 0.0\n", encoding="utf-8")
    (config_dir / "controllers" / "point.yaml").write_text("kp_x: 1.0\n", encoding="utf-8")

    monkeypatch.setattr("src.controller.experiment_logger.CONFIG_DIR", config_dir)
    monkeypatch.setattr("src.controller.experiment_logger.timestamp_for_dir", lambda: "20260519_120000")
    monkeypatch.setattr("src.controller.experiment_logger.get_project_root", lambda: tmp_path)

    experiment_dir = ExperimentLogger("manual", "data/experiments").create()

    assert (experiment_dir / "config_snapshot" / "system.yaml").exists()
    assert (experiment_dir / "config_snapshot" / "controller.yaml").exists()
    assert (experiment_dir / "config_snapshot" / "controllers" / "manual.yaml").exists()
    assert not (experiment_dir / "config_snapshot" / "controllers" / "point.yaml").exists()
