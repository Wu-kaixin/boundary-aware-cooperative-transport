import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from build_paper_evidence import episode_row, pair_rows, paired_bootstrap, wilson_interval


def test_wilson_empty_and_bounds():
    empty = wilson_interval(0, 0)
    assert empty["rate"] is None
    interval = wilson_interval(0, 9)
    assert interval["low"] == 0
    assert 0 < interval["high"] < 1


def test_censored_readiness_is_not_a_zero_difference():
    left = episode_row(
        {
            "case_id": "a",
            "shape": "c_shape",
            "seed": 8,
            "variant": "boundary_cvt",
            "stratum": "near",
            "execution_status": "COMPLETED",
            "events": {"CONTACT_READY": {"seconds": None, "right_censored": True}},
            "physical_contact_ready": {"seconds": None, "right_censored": True},
        },
        "development",
    )
    right = episode_row(
        {
            "case_id": "b",
            "shape": "c_shape",
            "seed": 8,
            "variant": "nearest_observed_target",
            "stratum": "near",
            "execution_status": "COMPLETED",
            "events": {"CONTACT_READY": {"seconds": 6.0, "right_censored": False}},
            "physical_contact_ready": {"seconds": 7.0, "right_censored": False},
        },
        "development",
    )
    pair = pair_rows([left, right], "boundary_cvt", "nearest_observed_target")[0]
    assert pair["ready_difference_seconds"] is None
    assert pair["ready_pair_status"] == "censored"
    assert paired_bootstrap([])["n"] == 0
