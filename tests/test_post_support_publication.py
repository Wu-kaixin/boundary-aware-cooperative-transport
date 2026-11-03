"""Execution-error receipts must remain publishable without invented metrics."""
import csv
import importlib
import json
from pathlib import Path


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def error_fixture(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    study = importlib.import_module('post_acquisition_support_study')
    base, out = tmp_path / 'base', tmp_path / 'base/run'
    protocol = tmp_path / 'protocol.json'
    write(protocol, {'fixture': True})
    cases = [dict(case_id=f'{condition}_{arm}', condition_id=condition, arm=arm)
             for condition in study.CONDITIONS for arm in study.ARMS]
    plan = dict(cases=cases, conditions=list(study.CONDITIONS), frozen_source_sha256='synthetic',
                tests={}, canonical_sha256='synthetic', environment={}, installed_locked_packages={},
                source_files={}, scripts={}, preflight=[])
    identity = {'synthetic': True}
    write(out / 'manifest.json', {'identity': identity})
    write(base / 'execution_outcome.json', {'status': 'COMPLETED_WITH_ERRORS', 'execution_errors': 8})
    results, rows = {}, []
    for case in cases:
        folder = out / case['case_id']
        result = {'execution_status': 'ERROR', 'error': 'synthetic engine failure'}
        write(folder / 'case_result.json', {'result': result})
        results[folder.name] = result
        # Real episode() error schema, rather than a mock metric row.
        rows.append(study.episode(case, folder, result))
    for name, value in dict(BASE=base, OUT=out, PROTOCOL=protocol).items():
        monkeypatch.setattr(study, name, value)
    monkeypatch.setattr(study, 'check', lambda: plan)
    monkeypatch.setattr(study, 'identity', lambda *args: identity)
    return study, plan, base, protocol, results, rows


def assert_errors_retained(public, csv_name):
    rows = list(csv.DictReader((public / csv_name).open(encoding='utf-8')))
    assert len(rows) == 8
    assert all(r['execution'] == 'ERROR' and r['seconds'] == '' for r in rows)
    assert all(r['execution_error'] == 'synthetic engine failure' for r in rows)
    assert all(r.get('endpoint_m', r.get('endpoint_mm')) == '' for r in rows)


def test_terminal_receipt_export_retains_all_execution_errors(monkeypatch, tmp_path):
    study, plan, base, protocol, results, rows = error_fixture(monkeypatch, tmp_path)
    module = importlib.import_module('export_post_support_receipts')
    public = tmp_path / 'public'
    public.mkdir()
    monkeypatch.setattr(module, 'PUBLIC', public)
    monkeypatch.setattr(module, 'guard', lambda: None)
    monkeypatch.setattr(module, 'cached_result', lambda folder, identity: results[folder.name])
    monkeypatch.setattr('sys.argv', ['export_post_support_receipts.py'])
    module.main()
    assert_errors_retained(public, 'all_eight_receipts.csv')
    report = json.loads((public / 'terminal_receipt_summary.json').read_text())
    assert report['terminal_episode_receipts'] == 8
    assert all(r['error'] == 'synthetic engine failure' for r in report['episodes'])


def test_full_publisher_retains_errors_and_marks_missing_plot_metrics(monkeypatch, tmp_path):
    study, plan, base, protocol, results, rows = error_fixture(monkeypatch, tmp_path)
    module = importlib.import_module('publish_post_acquisition_support')
    public = tmp_path / 'public'
    for name, value in dict(BASE=base, PROTOCOL=protocol, ROOT=tmp_path, PUBLIC=public).items():
        monkeypatch.setattr(module, name, value)
    monkeypatch.setattr(module, 'guard', lambda: None)
    monkeypatch.setattr(module, 'check', lambda: plan)
    report = dict(episodes=rows, protocol_sha256=module.sha(protocol), original_reproduction={},
        mechanism_gates={'execution': False}, full_gates={'execution': False},
        mechanism_gate='FAIL', full_gate='FAIL')
    write(base / 'analysis.json', report)
    receipt = dict(status='VERIFIED', episodes=8, analysis_sha256=module.sha(base / 'analysis.json'))
    write(base / 'analysis_outcome.json', receipt)
    write(base / 'analysis_adapter_v1/outcome.json', receipt)
    labels = []
    original = module.plt.Axes.annotate

    def capture(self, text, *args, **kwargs):
        labels.append(text)
        return original(self, text, *args, **kwargs)

    monkeypatch.setattr(module.plt.Axes, 'annotate', capture)
    module.main()
    assert_errors_retained(public, 'all_eight.csv')
    assert labels.count('NA') == 32  # Eight explicitly missing values on each of four panels.
    assert (public / 'comparison.png').is_file() and (public / 'comparison.pdf').is_file()
