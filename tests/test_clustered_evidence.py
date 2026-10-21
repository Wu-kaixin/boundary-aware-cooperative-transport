import pytest

from dbact_sim.experiments.evidence import _bool_count, seed_cluster_bootstrap


def test_many_repeated_conditions_do_not_create_independent_seed_replication():
    pairs = [{"seed": seed, "ready_difference_seconds": float(seed)} for seed in (1, 2) for _ in range(30)]
    result = seed_cluster_bootstrap(pairs)
    assert result["n_pairs"] == 60
    assert result["n_seed_clusters"] == 2
    assert result["low"] is result["high"] is None
    assert result["mean"] == pytest.approx(1.5)


def test_clustered_interval_is_reproducible_and_keeps_censoring_missing():
    pairs = [{"seed": seed, "ready_difference_seconds": -float(seed)} for seed in range(8)]
    pairs.append({"seed": 99, "ready_difference_seconds": None})
    a, b = seed_cluster_bootstrap(pairs), seed_cluster_bootstrap(pairs)
    assert a == b
    assert a["n_pairs"] == a["n_seed_clusters"] == 8
    assert a["low"] <= a["mean"] <= a["high"]


def test_reused_seed_rates_keep_denominators_without_independence_intervals():
    rows = [{"seed": 2, "task_completed": True}, {"seed": 2, "task_completed": False}]
    result = _bool_count(rows, "task_completed")
    assert result["k"] == 1 and result["n"] == 2 and result["rate"] == 0.5
    assert result["low"] is result["high"] is None
    rows[1]["seed"] = 5
    assert _bool_count(rows, "task_completed")["high"] is not None
