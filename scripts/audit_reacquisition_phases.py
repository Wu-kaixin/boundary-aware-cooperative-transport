"""Reconstruct original phase decisions from all four saved candidate episodes."""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import json
import shutil
import sys

from research_io import ROOT, guard, read, sha


def main():
    guard()
    assert not any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules)
    protocol = ROOT/'experiments/ccta2027/reference_reacquisition_v1.json'
    design = ROOT/'experiments/ccta2027/acquisition_normal_audit_v1_design.md'
    plan = read(protocol)
    batch = ROOT/plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch/'source_snapshot'/name) == digest
    sys.path.insert(0, str(batch/'source_snapshot/src'))
    from dbact.control.phase import PhaseSignals
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, environment_versions
    assert environment_versions() == plan['environment']
    manifest = read(batch/'manifest.json')
    assert manifest['identity']['protocol_sha256'] == sha(protocol)
    out = ROOT/'runs/research/2026-11-03-acquisition-normal-audit-v1/phases'
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, out/'audit_reacquisition_phases.py')
    shutil.copy2(design, out/'design.md')
    shutil.copy2(ROOT/'scripts/research_io.py', out/'research_io.py')
    identity = dict(source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(protocol),
        script_sha256=sha(__file__), design_sha256=sha(design),
        new_physical_episodes=0, new_controller_replay_frames=0,
        scope='Original phase monitor only, driven by original saved signals; not a new controller trajectory')
    dump(out/'identity.json', identity)
    episodes, verified, current = [], 0, None
    try:
        for case in plan['cases']:
            if case['arm'] != 'reference_reacquisition':
                continue
            current = case['case_id']
            folder = batch/current
            result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
            assert result['execution_status'] == 'COMPLETED'
            rows = read(folder/'paired_metrics.json')['steps']
            env = SimulationEnvironment(case['config'], seed=case['seed'])
            monitor = env.controller.phase_monitor
            phases, events, previous, last_phase = {}, {}, [], 'SEARCH'
            counts = Counter()
            for k, row in enumerate(rows):
                assert row['frame'] == k+1
                signals = PhaseSignals(**row['phase_guards'])
                assert signals.transport_active == sum(bool(d['push_side']) for d in previous)
                previous = row['diagnostics']
                before = monitor.phase.name
                will_brake = monitor._brake_due(signals)
                monitor.update(signals, k)
                assert monitor.phase.name == row['phase'], (current, k, 'phase')
                assert monitor.as_dict() == row['phase_response'], (current, k, 'response')
                phase = row['phase']
                group = phases.setdefault(phase, dict(frames=0, maximum_active=0, quorum_frames=0,
                    first_quorum_frame=None, minimum_remaining_m=None))
                group['frames'] += 1
                group['maximum_active'] = max(group['maximum_active'], signals.transport_active)
                group['quorum_frames'] += int(signals.transport_active >= monitor.gates.transport_quorum)
                remaining = signals.target_distance-signals.progress
                group['minimum_remaining_m'] = remaining if group['minimum_remaining_m'] is None else min(group['minimum_remaining_m'], remaining)
                if signals.transport_active >= monitor.gates.transport_quorum:
                    group['first_quorum_frame'] = group['first_quorum_frame'] or row['frame']
                    events.setdefault('first_quorum', dict(frame=row['frame'], time=k*env.dt, phase=phase,
                        active=signals.transport_active))
                if before == 'CONTACT_READY':
                    counts['entered_update_in_CONTACT_READY'] += 1
                    counts['CONTACT_READY_quorum_met'] += int(signals.transport_active >= monitor.gates.transport_quorum)
                    counts['CONTACT_READY_brake_due'] += int(will_brake)
                if phase != last_phase:
                    events[phase] = dict(frame=row['frame'], monitor_frame=k, time=k*env.dt,
                        from_phase=last_phase, signals=asdict(signals),
                        brake_due=will_brake,
                        brake_lead_m=max(signals.stopping_distance, (1.-monitor.gates.brake_fraction)*signals.target_distance),
                        response=monitor.as_dict())
                last_phase = phase
                verified += 1
            episodes.append(dict(case_id=current, verified_frames=len(rows), gates=asdict(monitor.gates),
                receipt_sha256=sha(folder/'case_result.json'), by_phase=phases, events=events,
                contact_ready_decisions=dict(counts), final_response=monitor.as_dict()))
            print(json.dumps(dict(case=current, frames=len(rows), phase_exact=True,
                contact_ready_maximum_active=phases.get('CONTACT_READY', {}).get('maximum_active'),
                first_quorum=events.get('first_quorum'))), flush=True)
        assert len(episodes) == 4
        report = dict(**identity, episodes=episodes, phase_reconstruction_frames=verified,
            status='EXACT_PHASE_RECONSTRUCTION', active_from_previous_diagnostics_verified=True,
            frame_convention='Saved frame is one-based; monitor frame and control time are zero-based')
        dump(out/'analysis.json', report)
        dump(out/'outcome.json', dict(**identity, status=report['status'], cases=4,
            phase_reconstruction_frames=verified, analysis_sha256=sha(out/'analysis.json')))
    except Exception as exc:
        dump(out/'failure.json', dict(**identity, status='INVALID_PHASE_RECONSTRUCTION',
            case_id=current, verified_frames=verified, error=repr(exc)))
        raise


if __name__ == '__main__':
    main()
