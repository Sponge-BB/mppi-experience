#!/usr/bin/env python3
"""Post-run provenance checks, no ROS/control or optimizer calls."""
import hashlib
import json
from pathlib import Path
import sys

TOOLS=Path(__file__).resolve().parent
OUT=TOOLS.parent
sys.path.insert(0,str(TOOLS))
import build_state_handoff as builder


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    m=builder.check()
    plan=json.loads((OUT/'plan.json').read_text())
    complete=json.loads((OUT/'complete.json').read_text())
    assert len(complete['completed'])==len(plan['jobs'])==8
    assert sha(builder.OUT/'build_manifest.json')==plan['build_manifest_sha256']
    assert sha(TOOLS/'run_state_ab.py')==plan['runner_sha256']
    common_sources=None;rows=[]
    for job in plan['jobs']:
        trial=OUT/job['trial']
        manifest=json.loads((trial/'manifest.json').read_text())
        status=json.loads((trial/'run_status.json').read_text())
        audit=json.loads((trial/'trial_audit.json').read_text())
        assert audit['valid_experiment'] and not audit.get('errors')
        assert status['production_hashes_unchanged'] and status['build_manifest_unchanged']
        assert not status['runner_error'] and not status['stop_errors']
        assert manifest['native_binaries_sha256']==m['native_binaries_sha256']
        sources=manifest['experiment_sources_sha256']
        if common_sources is None:common_sources=sources
        assert sources==common_sources
        runtime_count=0;external_count=0
        for name,digest in sources.items():
            frozen=trial/'runtime'/name
            if frozen.is_file():runtime_count+=1
            else:
                # Preparation inventories C++ diagnostic helpers as declared
                # experiment inputs; only Python is copied into runtime.
                assert Path(name).suffix!='.py',name
                frozen=builder.HERE/name;external_count+=1
            assert sha(frozen)==digest,name
        assert sha(trial/'coupling_parameters.json')==manifest['parameters_sha256']
        assert manifest['state_handoff_ab']['group']==job['group']
        rows.append(dict(trial=trial.name,runtime_sources_verified=runtime_count,
            declared_external_sources_verified=external_count,
            valid_interface=True,production_unchanged=True,build_unchanged=True))
    parent_fsm=(builder.BASE/'source/remani_replan_fsm.cpp').read_text()
    child_fsm=(builder.OUT/'source/remani_replan_fsm.cpp').read_text()
    def callback(text):
        return text.split('  void REMANIReplanFSM::mmCarOdomCallback(',1)[1].split(
            '  void REMANIReplanFSM::mmManiOdomCallback(',1)[0]
    assert callback(child_fsm).replace('    applied_raw_odom_=msg;\n','')==callback(parent_fsm)
    record=dict(all_eight_valid=True,same_binary=True,same_runtime_sources=True,
        original_odom_processing_byte_identical_except_message_retention=True,
        original_mppi_library_unchanged=sha(builder.OUT/'build/libonline_mppi.so')==sha(builder.BASE/'build/libonline_mppi.so'),
        build_manifest_sha256=sha(builder.OUT/'build_manifest.json'),trials=rows,
        tools_sha256={p.name:sha(p) for p in sorted(TOOLS.iterdir()) if p.is_file()},
        github_upload_still_paused=True,production_deployment_changed=False)
    (OUT/'verification.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
