import pandas as pd
import pytest
import yaml

from src.controller.plotting.experiment_plotter import ExperimentPlotter


@pytest.fixture
def world_csv(tmp_path):
    world = pd.DataFrame(
        [
            {
                "timestamp": 1.0,
                "frame_id": 1,
                "robot_id": "robot_1",
                "tracked": True,
                "x": -0.5,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "wz": 0.0,
            },
            {
                "timestamp": 1.0,
                "frame_id": 1,
                "robot_id": "robot_2",
                "tracked": True,
                "x": 1.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "wz": 0.0,
            },
        ]
    )
    world.to_csv(tmp_path / "world_state.csv", index=False)
    return tmp_path


def write_snapshot(experiment_dir, controller_type: str):
    snapshot_dir = experiment_dir / "config_snapshot"
    snapshot_dir.mkdir(exist_ok=True)
    (snapshot_dir / "controller.yaml").write_text(
        yaml.safe_dump(
            {
                "controller": {"type": controller_type},
                "plot": {
                    "plot_trajectory": True,
                    "plot_pose_time_series": True,
                    "plot_command_time_series": True,
                    "save_every_n_frames": 2,
                },
            }
        ),
        encoding="utf-8",
    )
    (snapshot_dir / "system.yaml").write_text(
        yaml.safe_dump(
            {
                "world": {
                    "x_min": -1.0,
                    "x_max": 1.0,
                    "y_min": -1.0,
                    "y_max": 1.0,
                    "z_min": -0.1,
                    "z_max": 1.0,
                }
            }
        ),
        encoding="utf-8",
    )


def test_plotter_generates_cvt_voronoi(world_csv):
    write_snapshot(world_csv, "cvt")
    outputs = ExperimentPlotter(world_csv).plot_all()
    output_names = {output.name for output in outputs}
    assert "cvt_voronoi.png" in output_names
    assert (world_csv / "plots" / "cvt" / "cvt_voronoi.png").exists()


def test_plotter_skips_cvt_voronoi_for_non_cvt_controller(world_csv):
    write_snapshot(world_csv, "point")
    outputs = ExperimentPlotter(world_csv).plot_all()
    output_names = {output.name for output in outputs}
    assert "cvt_voronoi.png" not in output_names
    assert not (world_csv / "plots" / "cvt" / "cvt_voronoi.png").exists()


def test_plotter_prefers_experiment_config_snapshot(world_csv):
    write_snapshot(world_csv, "point")

    outputs = ExperimentPlotter(world_csv).plot_all()
    output_names = {output.name for output in outputs}

    assert "cvt_voronoi.png" not in output_names


def test_plotter_generates_cvt_voronoi_sequence(tmp_path):
    rows = []
    for frame_id in range(1, 6):
        rows.extend(
            [
                {
                    "timestamp": float(frame_id),
                    "frame_id": frame_id,
                    "robot_id": "robot_1",
                    "tracked": True,
                    "x": -0.5 + frame_id * 0.01,
                    "y": 0.0,
                    "z": 0.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "vx": 0.0,
                    "vy": 0.0,
                    "wz": 0.0,
                },
                {
                    "timestamp": float(frame_id),
                    "frame_id": frame_id,
                    "robot_id": "robot_2",
                    "tracked": True,
                    "x": 0.5 - frame_id * 0.01,
                    "y": 0.0,
                    "z": 0.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "vx": 0.0,
                    "vy": 0.0,
                    "wz": 0.0,
                },
            ]
        )
    pd.DataFrame(rows).to_csv(tmp_path / "world_state.csv", index=False)
    write_snapshot(tmp_path, "cvt")

    plotter = ExperimentPlotter(tmp_path)
    outputs = plotter.plot_all()
    output_names = {output.name for output in outputs}

    assert "cvt_voronoi_step_000000.png" in output_names
    assert "cvt_voronoi_step_000002.png" in output_names
    assert "cvt_voronoi_step_000004.png" in output_names
    assert (tmp_path / "plots" / "cvt" / "cvt_voronoi_step_000000.png").exists()
    assert (tmp_path / "plots" / "cvt" / "cvt_voronoi_step_000002.png").exists()
    assert (tmp_path / "plots" / "cvt" / "cvt_voronoi_step_000004.png").exists()

