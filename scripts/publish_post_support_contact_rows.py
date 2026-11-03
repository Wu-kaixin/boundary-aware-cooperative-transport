"""Publish every fixed contact state and both conditional residual diagnostics."""
from __future__ import annotations

import csv

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump


DECOMPOSITION = ROOT / 'runs/research/2026-11-03-post-support-contact-row-decomposition-v1'
ENVELOPE = ROOT / 'runs/research/2026-11-03-post-support-contact-motion-envelope-v1'
PUBLIC = ROOT / 'docs/results/post_support_contact_row_decomposition_v1'


def main():
    guard()
    decomposition = read(DECOMPOSITION / 'analysis.json')
    envelope = read(ENVELOPE / 'analysis.json')
    for folder, status in ((DECOMPOSITION, 'VERIFIED_ROW_DECOMPOSITION'), (ENVELOPE, 'VERIFIED_MOTION_ENVELOPE')):
        outcome = read(folder / 'outcome.json')
        assert outcome['status'] == status and outcome['analysis_sha256'] == sha(folder / 'analysis.json')
    assert envelope['decomposition_sha256'] == sha(DECOMPOSITION / 'analysis.json')
    lookup = {(s['condition'], s['frame'], s['agent_id']): s for s in envelope['rows']}
    assert len(lookup) == len(decomposition['rows']) == 128
    rows = []
    for source in decomposition['rows']:
        bound = lookup[source['condition'], source['frame'], source['agent_id']]
        for key in ('positive_force', 'declared_pusher', 'admitted'):
            assert source[key] == bound[key]
        rows.append(dict(condition=source['condition'], frame=source['frame'], agent_id=source['agent_id'],
            admitted=source['admitted'], guard=source['guard'], positive_force=source['positive_force'],
            declared_pusher=source['declared_pusher'], limiting_row=source.get('limiting_row'),
            minimum_uncapped_residual=source.get('minimum_uncapped_residual'),
            residual_without_motion=source.get('minimum_residual_without_motion'),
            still_conflicting_without_motion=source.get('still_conflicting_without_motion'),
            maximum_residual_over_configured_motion=bound.get('maximum_row_residual'),
            original_command_rejected_for_all_bounded_motion=bound.get('original_command_rejected_for_all_bounded_motion'),
            limiting_h=source.get('limiting_h'), command_term=source.get('command_term'),
            geometric_term=source.get('geometric_term'), margin_term=source.get('margin_term'), motion_term=source.get('motion_term'),
            r_safe=source.get('r_safe'), gamma_obj=source.get('gamma_obj'), rho=source.get('rho'),
            configured_motion_bound=bound.get('configured_motion_bound')))
    PUBLIC.mkdir(parents=True, exist_ok=False)
    for name, subset in (('all_128_states.csv', rows), ('all_17_positive_force_states.csv', [r for r in rows if r['positive_force']])):
        with (PUBLIC / name).open('x', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(subset)
    dump(PUBLIC / 'summary.json', dict(decomposition_counts=decomposition['counts'], motion_envelope_counts=envelope['counts'],
        new_physical_episodes=0, limiting_row_h_note='negative_h counts refer to the selected minimum-residual row, not every row.',
        inference_limit='Conditional witnesses exclude the same original commands within the configured motion ball. They do not exclude every other pushing command or certify true object motion.'))
    dump(PUBLIC / 'provenance.json', dict(decomposition_directory=DECOMPOSITION.relative_to(ROOT).as_posix(),
        envelope_directory=ENVELOPE.relative_to(ROOT).as_posix(),
        decomposition_sha256=sha(DECOMPOSITION / 'analysis.json'), envelope_sha256=sha(ENVELOPE / 'analysis.json'),
        contact_analysis_sha256=envelope['contact_analysis_sha256'], runtime_source_sha256=envelope['runtime_source_sha256'],
        protocol_sha256=envelope['protocol_sha256'], publisher_sha256=sha(__file__), new_physical_episodes=0))
    dump(PUBLIC / 'publication.json', dict(products={p.name: sha(p) for p in PUBLIC.iterdir() if p.is_file()}, new_physical_episodes=0))
    print(dict(all_states=len(rows), positive_force_states=sum(r['positive_force'] for r in rows)), flush=True)


if __name__ == '__main__':
    main()
