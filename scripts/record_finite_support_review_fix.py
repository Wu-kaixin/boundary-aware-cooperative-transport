"""Record engineering-only review closure without relabeling frozen physics."""
from datetime import datetime, timezone
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import source_hashes, environment_versions, verify_source_snapshot


def counts(path):
    suite = ET.parse(path).getroot().find('testsuite').attrib
    result = {k:int(suite[k]) for k in ('tests', 'failures', 'errors', 'skipped')}
    result['passed'] = result['tests']-result['failures']-result['errors']-result['skipped']
    return result


def main():
    guard()
    checks = ROOT/'runs/maintenance/2026-11-03-finite-acquisition-support-v1'
    raw = ROOT/'runs/research/2026-11-03-finite-acquisition-support-v1'
    public = ROOT/'docs/results/finite_acquisition_support_v1'
    protocol = ROOT/'experiments/ccta2027/finite_acquisition_support_v1.json'
    plan = read(protocol)
    assert sha(protocol) == sha(raw/'freeze/protocol.json')
    assert sha(ROOT/'experiments/ccta2027/finite_acquisition_support_physical_v1_design.md') == plan['design_sha256']
    verify_source_snapshot(raw/'freeze', {**plan['source_files'], **plan['scripts']})
    verify_source_snapshot(raw/'run', plan['source_files'])
    publication = read(public/'publication.json')
    for name, digest in publication['outputs'].items():
        assert sha(public/name) == digest, name
    assert sha(raw/'analysis.json') == sha(public/'analysis.json')
    assert sha(ROOT/'configs/research/best_known_transport_v1.yaml') == plan['canonical_sha256']
    assert environment_versions() == plan['environment']
    source = source_hashes()
    changed = [name for name in set(source) | set(plan['source_files'])
               if source.get(name) != plan['source_files'].get(name)]
    assert changed == ['src/dbact/sensing/finite_support.py']
    red, green, core = (counts(checks/name) for name in ('review-red.xml', 'review-green.xml', 'review-core.xml'))
    assert red['failures'] == 4 and red['errors'] == 0 and red['passed'] == 1
    assert green['failures'] == green['errors'] == core['failures'] == core['errors'] == 0
    destination = checks/'review_fix'
    destination.mkdir(exist_ok=False)
    extra = ['tests/test_finite_support_review_fix.py',
             'experiments/ccta2027/finite_acquisition_support_review_fix_v1.md',
             'scripts/record_finite_support_review_fix.py']
    snapshot = {**source, **{name:sha(ROOT/name) for name in extra}}
    for name in snapshot:
        target = destination/'source_snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, target)
    verify_source_snapshot(destination, snapshot)
    for name in ('review-red.xml', 'review-green.xml', 'review-core.xml'):
        shutil.copy2(checks/name, destination/name)
    output = public/'post_review_checks.json'
    if output.exists():
        raise RuntimeError('Preserve existing review closure')
    report = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
        base_git_sha=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        corrected_runtime_source_sha256=encode(source), source_files=source,
        corrected_snapshot=destination.relative_to(ROOT).as_posix(), snapshot_files=snapshot,
        original_physical_source_sha256=plan['frozen_source_sha256'],
        original_protocol_sha256=sha(protocol), changed_source_files=changed,
        original_publication_sha256=sha(public/'publication.json'),
        original_physical_and_publication_integrity='PASS', canonical_sha256=plan['canonical_sha256'],
        red=red, targeted=green, core=core,
        test_receipts={name:sha(checks/name) for name in ('review-red.xml', 'review-green.xml', 'review-core.xml')},
        environment=environment_versions(), new_research_physical_episodes=0,
        corrected_runtime_physical_validation='NOT_RUN', candidate_adopted=False,
        deferred_minor='Baseline original_row_robot_steps=0 in the original CSV is an absent-counter placeholder, not a measured zero; corrected derivative deferred.',
        scope='Review-fix engineering checkpoint only; the original eight episodes validate neither this runtime nor arbitrary unseen geometry')
    dump(output, report)
    dump(destination/'identity.json', report)
    print(dict(corrected_source_sha256=report['corrected_runtime_source_sha256'],
               core=core, targeted=green, original_integrity='PASS', new_research_physical_episodes=0), flush=True)


if __name__ == '__main__':
    main()
