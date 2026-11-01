#!/usr/bin/env python3
"""Continue only the two unmeasured crossing trials after a report-only drift."""
import json
from pathlib import Path
import shutil

import run_delivery_next_online as run
from run_native_collision_3d import trial_kwargs

ROOT=run.DEFAULT


def main():
    run.register_scenes()
    plan=json.loads((ROOT/'plan.json').read_text())
    completed=json.loads((ROOT/'progress.json').read_text())
    if len(completed)!=2 or any(r['scene']!='static' or not r['valid_interface'] for r in completed):
        raise ValueError('Expected exactly the original two valid static results')
    if (ROOT/'complete.json').exists():raise ValueError('Already complete')
    build=Path(plan['build']);manifest=run.builder.check(build)
    if run.lifecycle.sha(build/'build_manifest.json')!=plan['build_manifest_sha256']:
        raise ValueError('Build changed')
    static=ROOT/completed[0]['trial']
    baseline=json.loads((static/'manifest.json').read_text())['experiment_sources_sha256']
    unstarted=ROOT/plan['jobs'][2]['trial']
    archived=unstarted.with_name(unstarted.name+'_not_started_report_hash_guard')
    if (unstarted/'run_status.json').exists() or (unstarted/'owned_pids.txt').exists():
        raise ValueError('Crossing run may already have started')
    prepared=json.loads(((archived if archived.exists() else unstarted)/'manifest.json').read_text())['experiment_sources_sha256']
    differences=[k for k in set(baseline)|set(prepared) if baseline.get(k)!=prepared.get(k)]
    if differences!=['early_delivery_report_native.py']:
        raise ValueError('Unexpected drift: '+str(differences))
    if not archived.exists():shutil.move(str(unstarted),str(archived))
    runtime=None
    operational=(*run.lifecycle.RUNTIME_CONTRACT,'native_collision_runtime.py','delivery_next_capture.py')
    evidence=dict(original_plan_sha256=run.lifecycle.sha(ROOT/'plan.json'),
        original_stop_sha256=run.lifecycle.sha(ROOT/'stopped.json'),
        report_only_drift=differences,control_sources_verified=list(operational),
        original_static_not_rerun=True,unstarted_directory_preserved=True)
    (ROOT/'resume_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    for job in plan['jobs'][2:]:
        def prepare(out,settings):
            run.patch_trial(out,settings,job['scene'],job['early_exit'])
            hashes=json.loads((out/'manifest.json').read_text())['experiment_sources_sha256']
            for key in operational:
                if hashes[key]!=baseline[key]:raise ValueError('Actual control source changed: '+key)
        with run.clean_environment():
            trial=ROOT/job['trial']
            valid=run.lifecycle.run_trial(trial,build,'sphere_timing_'+job['scene'],'engineering',11,
                job['duration_s'],0.,0.,False,plan['build_manifest_sha256'],expected_runtime_sources=runtime,
                reserve_bytes=run.RESERVE+run.TRIAL_ALLOWANCE,prepare_hook=prepare,**trial_kwargs(manifest))
        runtime=json.loads((trial/'manifest.json').read_text())['experiment_sources_sha256']
        result=json.loads((trial/'result.json').read_text())
        completed.append(dict(**job,valid_interface=valid,reason=result['reason']))
        (ROOT/'progress.json').write_text(json.dumps(completed,indent=2)+'\n')
        if not valid:raise RuntimeError('Invalid run retained')
    (ROOT/'complete.json').write_text(json.dumps(dict(complete=True,completed=completed,
        resumed_after_report_only_drift=True),indent=2)+'\n')
    print('COMPLETE',ROOT,flush=True)


if __name__=='__main__':main()
