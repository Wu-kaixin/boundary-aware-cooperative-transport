from __future__ import annotations

import json

import matplotlib
import pytest

matplotlib.use("Agg", force=True)

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.publication import (
    build_selection_manifest,
    compare_rerun,
    load_episode_rows,
    select_publication_cases,
    verify_rerun,
)
from dbact_sim.scenarios import load_yaml
from dbact_sim.trace import SimulationTrace
from dbact_sim.visualization import write_outcome_comparison


EPISODES = "docs/results/v2_shape_matrix/episodes.csv"
MATRIX_MANIFEST = "docs/results/v2_shape_matrix/manifest.json"


def test_publication_selection_is_predeclared_and_deterministic():
    rows = load_episode_rows(EPISODES)
    selection = select_publication_cases(rows)
    assert selection["median_alpha"] == pytest.approx(0.4)
    assert selection["cases"]["representative_success"]["case_id"] == (
        "u_shape__a0.40__seed000"
    )
    assert selection["cases"]["high_concavity_failure"]["case_id"] == (
        "star10__a0.40__seed001"
    )
    assert selection["success_pool_size"] == 4
    assert selection["failure_max_concavity_pool_size"] == 3


def test_manifest_keeps_the_complete_matrix_denominator():
    rows = load_episode_rows(EPISODES)
    selection = select_publication_cases(rows)
    manifest = build_selection_manifest(EPISODES, MATRIX_MANIFEST, rows, selection)
    assert manifest["denominator"]["episodes"] == 180
    assert manifest["denominator"]["successes"] == 54
    assert manifest["denominator"]["P_success"] == pytest.approx(0.3)
    source = json.loads(open(MATRIX_MANIFEST, encoding="utf-8").read())
    assert manifest["source"]["research_git_sha"] == source["git_sha"]


def test_rerun_verification_fails_closed_on_metric_drift():
    rows = load_episode_rows(EPISODES)
    source = next(row for row in rows if row["case_id"] == "u_shape__a0.40__seed000")
    observed = dict(source)
    assert verify_rerun(source, observed)["passed"]
    observed["J"] += 1e-4
    comparison = compare_rerun(source, observed)
    assert comparison["passed"] is False
    assert comparison["mismatches"][0]["field"] == "J"
    with pytest.raises(RuntimeError, match="drifted"):
        verify_rerun(source, observed)


def test_outcome_comparison_writes_raster_and_vector(tmp_path):
    env = SimulationEnvironment(load_yaml("configs/sim/v2/l_shape_v2.yaml"), seed=0)
    env.run(steps=4)
    trace = SimulationTrace.from_environment(env)
    record = {
        "case_id": "test__a0.40__seed000",
        "shape": "l_shape",
        "seed": 0,
        "concavity_ratio": 0.2,
        "J_over_target": 0.8,
        "efficiency": 0.9,
        "frames_run": 4,
        "solver_fallbacks": 0,
        "solver_infeasible": 0,
        "final_phase": "SEARCH",
        "failure_class": "SUCCESS",
    }
    failure = dict(record, case_id="test_failure", failure_class="TRANSPORT_STALL")
    paths = write_outcome_comparison(
        trace,
        trace,
        tmp_path,
        success_record=record,
        failure_record=failure,
        formats=("png", "svg"),
        dpi=70,
    )
    assert {path.suffix for path in paths} == {".png", ".svg"}
    assert all(path.stat().st_size > 1_000 for path in paths)
