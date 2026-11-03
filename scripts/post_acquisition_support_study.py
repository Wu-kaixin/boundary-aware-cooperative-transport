"""Prospective eight-episode acceptance gates; no physical protocol is frozen yet.

This publication checkpoint provides the evaluator only. It must not be treated
as a study runner or evidence that the planned physical episodes have executed.
"""
from __future__ import annotations

import math


ARMS = ('map_bridge_control', 'post_support_candidate')
CONDITIONS = ('near_circle', 'near_l_reference', 'far_c_open_away_pos0',
              'far_skew_box_s0_r0_pos0')


def evaluate(rows, reproductions, conditions):
    """Fail closed on missing/duplicate episodes, then keep both gate levels."""
    conditions = tuple(conditions)
    expected = {(condition, arm) for condition in CONDITIONS for arm in ARMS}
    keys = [(row.get('condition'), row.get('arm')) for row in rows]
    complete = (len(conditions) == len(CONDITIONS)
                and set(conditions) == set(CONDITIONS)
                and len(keys) == len(expected) and set(keys) == expected
                and all(row.get('execution') == 'COMPLETED' for row in rows))
    lookup = dict(zip(keys, rows))
    candidates = [lookup.get((condition, ARMS[1]), {}) for condition in CONDITIONS]
    pairs = [(lookup.get((condition, ARMS[0]), {}), candidate)
             for condition, candidate in zip(CONDITIONS, candidates)]

    def scaling(row, stage):
        value = (row.get('scaling') or {}).get(stage)
        return value if type(value) is int and value >= 0 else None

    def not_increased(control, candidate, stage):
        before, after = scaling(control, stage), scaling(candidate, stage)
        return before is not None and after is not None and after <= before

    def audit(row):
        return row.get('post_support_audit') or {}

    skew_control, skew_candidate = pairs[-1]
    before = scaling(skew_control, 'post_acquisition')
    after = scaling(skew_candidate, 'post_acquisition')
    c_before, c_after = [r.get('maximum_absolute_rotation_deg') for r in pairs[2]]
    rotation_valid = all(isinstance(v, (int, float)) and math.isfinite(v)
                         and v >= 0 for v in (c_before, c_after))
    mechanism = dict(
        complete_eight=complete,
        original_controls_exact=complete and set(reproductions) == set(CONDITIONS)
            and all(reproductions[c].get('exact') is True for c in CONDITIONS),
        safe_completion=complete and all(r.get('safe_completed') is True for r in candidates),
        zero_epoch_valid=complete and all(r.get('zero_epoch_valid') is True for r in candidates),
        no_false_hold=complete and all(r.get('false_hold') is False for r in candidates),
        no_hard_safety_failure=complete and all(r.get('hard_safety') == [] for r in candidates),
        no_budget_cutoff=complete and all(r.get('budget_terminated') is False for r in candidates),
        matched_g500_preserved=complete and all(
            type(control.get('g500')) is bool and type(candidate.get('g500')) is bool
            and (not control['g500'] or candidate['g500']) for control, candidate in pairs),
        scaling_not_increased=complete and all(not_increased(control, candidate, stage)
            for control, candidate in pairs for stage in ('acquisition', 'post_acquisition')),
        skew_post_acquisition_improved=complete and before is not None and after is not None and after < before,
        c_rotation_not_increased=complete and rotation_valid and c_after <= c_before + 1e-12,
        input_and_row_audits_verified=complete and all(audit(r).get('verified') is True for r in candidates),
        candidate_path_exercised=complete and any(audit(r).get('applied', 0) > 0 for r in candidates),
        admitted_hard_rows_satisfied=complete and all(audit(r).get('hard_violations') == 0 for r in candidates),
    )
    full = dict(
        mechanism_pass=all(mechanism.values()),
        zero_candidate_scaling=complete and all(scaling(r, stage) == 0
            for r in candidates for stage in ('acquisition', 'post_acquisition')),
        g500_all_four=complete and all(r.get('g500') is True for r in candidates),
        formal_transport_all_four=complete and all(
            (r.get('phase_response') or {}).get('transport_frame') is not None for r in candidates),
        uncapped_admitted_object_rows_satisfied=complete and all(
            audit(r).get('uncapped_object_violations') == 0 for r in candidates),
    )
    return mechanism, full
