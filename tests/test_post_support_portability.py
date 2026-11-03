"""Keep historical Windows identity distinct from portable paired invariants."""
import copy
import importlib
import json
from pathlib import Path

import numpy as np
import pytest


def local_fixture(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root / 'scripts'))
    module = importlib.import_module('post_acquisition_support_study')
    old_path = root / 'experiments/ccta2027/map_reference_bridge_v1.json'
    original_bytes = old_path.read_bytes()
    old = json.loads(original_bytes)
    windows = json.loads((root / 'tests/fixtures/post_support_windows_initial_states.json').read_text())
    assert module.sha(old_path) == windows['source_protocol_sha256']
    for case in old['cases']:
        if case['arm'] != 'map_bridge_candidate':
            continue
        frozen = windows['states'][case['condition_id']]
        assert module.encode(frozen['positions']) == frozen['sha256'] == case['expected_initial_state_sha256']
        env = module.SimulationEnvironment(copy.deepcopy(case['config']), seed=case['seed'])
        positions = np.vstack([a.position for a in env.agents])
        # This bound is for the portable test fixture only. Production still
        # requires exact hashes and must refuse cross-platform historical drift.
        np.testing.assert_allclose(positions, frozen['positions'], rtol=0., atol=1e-12)
        case['expected_initial_state_sha256'] = module.encode(positions.tolist())
    return module, old, old_path, original_bytes


def test_portable_cases_keep_all_parameters_and_paired_initial_states(monkeypatch):
    module, old, path, before = local_fixture(monkeypatch)
    cases, preflight = module.build_cases(old)
    assert len(cases) == len(preflight) == 8
    for case in cases:
        original = next(c for c in old['cases'] if c['condition_id'] == case['condition_id']
                        and c['arm'] == 'map_bridge_candidate')
        config = copy.deepcopy(case['config'])
        assert config['controller'].pop('post_acquisition_finite_support', False) is (case['arm'] == module.ARMS[1])
        assert config == original['config']
        for key in ('seed', 'frames', 'expected_initial_state_sha256'):
            assert case[key] == original[key]
        baseline = module.SimulationEnvironment(original['config'], seed=original['seed'])
        candidate = module.SimulationEnvironment(case['config'], seed=case['seed'])
        np.testing.assert_array_equal([a.position for a in candidate.agents], [a.position for a in baseline.agents])
    assert path.read_bytes() == before


@pytest.mark.parametrize('mutation', ['initial_hash', 'numeric_parameter'])
def test_portable_fixture_still_rejects_identity_or_parameter_changes(monkeypatch, mutation):
    module, old, path, before = local_fixture(monkeypatch)
    for case in old['cases']:
        if case['arm'] == 'map_bridge_candidate':
            if mutation == 'initial_hash':
                case['expected_initial_state_sha256'] = '0' * 64
            else:
                case['config']['controller']['max_speed'] = .36
    with pytest.raises(AssertionError):
        module.build_cases(old)
    assert path.read_bytes() == before
