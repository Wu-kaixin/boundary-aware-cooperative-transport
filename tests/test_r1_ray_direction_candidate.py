"""Analytic geometry checks only; no simulation or controller replay."""

import math
from pathlib import Path

import pytest
from conftest import load_script_module


@pytest.fixture(scope="module")
def candidate():
    with pytest.MonkeyPatch.context() as patch:
        patch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
        yield load_script_module("analyze_r1_ray_direction_candidate")


def test_own_symmetric_chord_attains_angle_bound(candidate):
    angle, radius, noise = math.pi / 72, 0.1, 0.03
    start = [radius * math.cos(angle), -radius * math.sin(angle)]
    end = [start[0], -start[1]]
    result = candidate.directional_lower(
        [0, 0], [-1, 0], start, end, [0, 0], 0.004, noise, 0.065
    )
    assert result["gain"] == pytest.approx(noise * (1 - math.cos(angle)), abs=1e-14)
    assert 0 < result["gain"] < 0.000029
    assert result["noise"] == noise


def test_own_endpoint_projection_cannot_gain(candidate):
    angle = math.pi / 36
    result = candidate.directional_lower(
        [0, 0],
        [-1, 0],
        [0.1, 0],
        [0.2 * math.cos(angle), 0.2 * math.sin(angle)],
        [0, 0],
        0.01,
        0.03,
        0.065,
    )
    assert result["gain"] == pytest.approx(0, abs=1e-14)
    assert result["new_h"] == pytest.approx(-0.005)


def test_neighbor_radial_error_can_be_tangential_to_query_normal(candidate):
    result = candidate.directional_lower(
        [0, 0.1], [0, 1], [0, 0], [0, 0], [-0.2, 0], 0.01, 0.03, 0.065
    )
    assert result["effective_noise"] == pytest.approx(0, abs=1e-14)
    assert result["gain"] == pytest.approx(0.03)
    assert result["new_h"] == pytest.approx(0.025)
