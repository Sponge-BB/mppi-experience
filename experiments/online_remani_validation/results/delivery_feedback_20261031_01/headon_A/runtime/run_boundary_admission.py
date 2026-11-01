#!/usr/bin/env python3
"""Small static start-admission A/B; all deferrals remain failed requests."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path

from run_coupled_objective import run_trial,check_build,check_space,sha


def validate_direction_proof(path,base):
    proof=json.loads(path.read_text())
    if (proof.get('schema')!='directed_low_speed_verdict_v1' or
        proof.get('verified_signed_negative_native_chain') is not True or
        proof.get('same_fixture') is not True or proof.get('same_runtime') is not True):
        raise ValueError('Directed signed-state proof is missing/incomplete; no admission experiment')
    if len(proof.get('trials',[]))!=2:raise ValueError('Incomplete directed pair')
    for trial in proof['trials']:
        manifest_path=Path(trial['source'])/'manifest.json'
        if sha(manifest_path)!=trial.get('manifest_sha256'):raise ValueError('Directed proof input changed')
        manifest=json.loads(manifest_path.read_text())
        if manifest.get('build')!=str(base) or manifest.get('build_manifest_sha256')!=sha(base/'build_manifest.json'):
            raise ValueError('Directed proof used a different base build')
    return proof


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--direction-proof',type=Path,required=True)
    args=parser.parse_args();build=args.build.resolve();out=args.out.resolve();proof=args.direction_proof.resolve()
    manifest,_=check_build(build)
    if not manifest.get('boundary_admission_experiment') or manifest.get('boundary_admission_prepared_only'):
        raise ValueError('Requires compiled boundary-admission build')
    validate_direction_proof(proof,Path(manifest['boundary_admission_parent']))
    check_space(out.parent,2*1024**3);out.mkdir(parents=True,exist_ok=False)
    # Alternate the order in the second seed. These are online trajectories,
    # not frozen complete-native-input paired solves or a significance study.
    jobs=[dict(seed=11,admission=False),dict(seed=11,admission=True),
          dict(seed=12,admission=True),dict(seed=12,admission=False)]
    plan=dict(created=datetime.now(timezone.utc).isoformat(),build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'),jobs=jobs,scene='static',group='coupled',duration_s=48.,
        direction_proof=str(proof),direction_proof_sha256=sha(proof),
        signed_low_speed=True,domain_recovery=True,clearance_recovery=False,
        single_factor='static actual-fixed-boundary admission OFF/ON; defer is not planning success',
        keep_all_waiting_in_elapsed=True,minimum_free_bytes=2*1024**3,
        reference_policy_unchanged=True,geometry_and_margin_unchanged=True,
        budgets_and_native_gates_unchanged=True,MPPI_unchanged=True,
        paired_complete_native_input=False,statistical_performance_claim=False,
        scope='fixed-arm native ROS numerical SIL; not Gazebo/hardware',
        deadline=args.__dict__.get('deadline','runner honors REMANI_EXPERIMENT_DEADLINE if supplied'))
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');done=[];runtime=None
    try:
        for job in jobs:
            if sha(proof)!=plan['direction_proof_sha256']:raise RuntimeError('Direction prerequisite changed during batch')
            seed=job['seed'];mode=job['admission']
            name='static_coupled_seed%d_admission_%s'%(seed,'on' if mode else 'off')
            valid=run_trial(out/name,build,'static','coupled',seed,48.,1.,1.,False,
                plan['build_manifest_sha256'],expected_runtime_sources=runtime,reserve_bytes=2*1024**3,
                diagnostic_capture=True,domain_recovery=True,state_contract=True,signed_low_speed=True,
                boundary_admission=mode)
            current=json.loads((out/name/'manifest.json').read_text())['experiment_sources_sha256']
            if runtime is None:runtime=current
            done.append(dict(trial=name,seed=seed,admission=mode,valid=valid))
            (out/'progress.json').write_text(json.dumps(done,indent=2)+'\n')
            if not valid:raise RuntimeError('Admission interface/audit failure; retain all outcomes, no rerun')
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(completed=done,reason=str(exc)),indent=2)+'\n');raise
    (out/'complete.json').write_text(json.dumps(dict(complete=True,completed=done),indent=2)+'\n')
    print('BOUNDARY_ADMISSION_PAIR_COMPLETE',out,flush=True)


if __name__=='__main__':main()
