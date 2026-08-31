from __future__ import annotations

import sys
from types import SimpleNamespace

import matplotlib

from dbact_sim import visualization


def test_configure_ffmpeg_writer_uses_imageio_binary(tmp_path, monkeypatch):
    ffmpeg = tmp_path / "ffmpeg.exe"
    ffmpeg.write_bytes(b"test executable placeholder")
    availability = iter([False, True])
    monkeypatch.setattr(
        visualization.animation.FFMpegWriter,
        "isAvailable",
        lambda *args: next(availability),
    )
    monkeypatch.setitem(
        sys.modules,
        "imageio_ffmpeg",
        SimpleNamespace(get_ffmpeg_exe=lambda: str(ffmpeg)),
    )
    previous = matplotlib.rcParams["animation.ffmpeg_path"]
    try:
        visualization._configure_ffmpeg_writer()
        assert matplotlib.rcParams["animation.ffmpeg_path"] == str(ffmpeg)
    finally:
        matplotlib.rcParams["animation.ffmpeg_path"] = previous


def test_configure_ffmpeg_writer_honours_standard_environment_variable(
    tmp_path,
    monkeypatch,
):
    ffmpeg = tmp_path / "conda-ffmpeg.exe"
    ffmpeg.write_bytes(b"test executable placeholder")
    availability = iter([False, True])
    monkeypatch.setattr(
        visualization.animation.FFMpegWriter,
        "isAvailable",
        lambda *args: next(availability),
    )
    monkeypatch.setenv("IMAGEIO_FFMPEG_EXE", str(ffmpeg))
    previous = matplotlib.rcParams["animation.ffmpeg_path"]
    try:
        visualization._configure_ffmpeg_writer()
        assert matplotlib.rcParams["animation.ffmpeg_path"] == str(ffmpeg)
    finally:
        matplotlib.rcParams["animation.ffmpeg_path"] = previous
