"""Analysis-only identity and exact packet serialization corrections."""
from __future__ import annotations

import copy
from pathlib import Path
import shutil
import traceback

import post_acquisition_support_study as study
import post_acquisition_support_audit as support_audit_module
from research_io import guard,read,sha
from trace_boundary_map import plain
from task_translation_physical_study_v2 import reproduction as strict_reproduction
from dbact_sim.experiments.deployment import dump


def control_reproduction(old_folder,new_folder,old_result,new_result,old_case,new_case):
    """Remove only the exact planned arm label from the scientific comparison."""
    if not (old_case['arm']=='map_bridge_candidate' and new_case['arm']=='map_bridge_control'
            and old_case['condition_id']==new_case['condition_id']
            and old_result['arm']==old_case['arm'] and new_result['arm']==new_case['arm']
            and old_result['case_id']==old_case['case_id'] and new_result['case_id']==new_case['case_id']):
        raise ValueError('Unexpected control identity; only the frozen arm rename is permitted')
    normalized=copy.deepcopy(new_result)
    normalized['arm']=old_result['arm']
    result=strict_reproduction(old_folder,new_folder,old_result,normalized)
    result['execution_metadata']['arm']=[old_result['arm'],new_result['arm']]
    result['planned_arm_identity_verified']=True
    return result


def audit_with_exact_packets(folder, case, **kwargs):
    """Compare packet arrays as exact lists, retaining every audit assertion."""
    original = support_audit_module.recorded
    try:
        support_audit_module.recorded = plain
        return support_audit_module.audit(folder, case, **kwargs)
    finally:
        support_audit_module.recorded = original


def main():
    guard()
    plan=study.check()
    execution=read(study.BASE/'execution_outcome.json')
    assert execution['status'] in ('COMPLETED','COMPLETED_WITH_ERRORS')
    target=study.BASE/'analysis_adapter_v1'
    target.mkdir(exist_ok=False)
    source=Path(__file__).resolve()
    design=study.ROOT/'experiments/ccta2027/post_acquisition_finite_support_v1_analysis_addendum.md'
    test=study.ROOT/'tests/test_post_support_analysis.py'
    serializer=study.ROOT/'scripts/trace_boundary_map.py'
    identity=dict(protocol_sha256=sha(study.PROTOCOL),runtime_source_sha256=plan['frozen_source_sha256'],
        adapter_sha256=sha(source),design_sha256=sha(design),test_sha256=sha(test),
        original_comparator_sha256=sha(study.ROOT/'scripts/task_translation_physical_study_v2.py'),
        original_auditor_sha256=sha(study.ROOT/'scripts/post_acquisition_support_audit.py'),
        serialization_helper_sha256=sha(serializer),
        precheck_sha256=sha(study.CHECKS/'control_reproduction_precheck_v1.json'),
        new_physical_episodes=0,scope='Analysis only: verify the planned result.arm rename and compare packet arrays as exact lists. Every scientific value and audit assertion is retained.')
    for path in (source,design,test,serializer):
        shutil.copy2(path,target/path.name)
    dump(target/'started.json',identity)
    original=study.reproduction
    original_support=study.support_audit

    def compare(old_folder,new_folder,old_result,new_result):
        case=next(c for c in plan['cases'] if c['case_id']==new_folder.name)
        old_case=plan['originals'][case['condition_id']]['case']
        return control_reproduction(old_folder,new_folder,old_result,new_result,old_case,case)

    try:
        study.reproduction=compare
        study.support_audit=audit_with_exact_packets
        study.analyze()
        dump(target/'outcome.json',dict(**identity,status='VERIFIED',analysis_sha256=sha(study.BASE/'analysis.json')))
    except Exception:
        dump(target/'failure.json',dict(**identity,status='FAILED',traceback=traceback.format_exc()))
        raise
    finally:
        study.reproduction=original
        study.support_audit=original_support


if __name__=='__main__':
    main()
