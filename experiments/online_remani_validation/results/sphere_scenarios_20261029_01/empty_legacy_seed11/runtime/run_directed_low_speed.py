#!/usr/bin/env python3
"""Directed native state-chain probe; not a moving-plant performance test."""
import argparse
from datetime import datetime,timezone
import json
import math
from pathlib import Path

from prepare_directed_low_speed import apply_fixture
from run_coupled_objective import run_trial,check_build,check_space,sha
from report_domain_recovery import read_records,assert_plot_quiet
from report_state_contract import cpp_identity,identity_key,match_observer,snapshot_evidence,backend_scope_evidence


def single(rows,event):
    found=[r for r in rows if r.get('event')==event]
    return found[0] if len(found)==1 else None


def fixed_fixture_identity_proof(rows):
    """No missing/mixed raw message or substituted boundary can pass a witness."""
    stages=('selected_boundary','desired_boundary','manager_input','frontend_init','backend_fixed')
    records=[single(rows,'state_contract_'+name) for name in stages]
    if any(record is None for record in records):return False
    keys=[identity_key(cpp_identity(record)) for record in records]
    if keys[0] is None or any(key!=keys[0] for key in keys):return False
    for record in records:
        p=record.get('planner_p');v=record.get('planner_v');yaw=record.get('planner_yaw')
        if (record.get('raw_odom_finite') is not True or record.get('raw_odom_processed_finite') is not True or
            not isinstance(p,list) or len(p)<2 or p[:2]!=[0.,0.] or
            not isinstance(v,list) or len(v)<2 or v[1]!=0. or
            yaw!=0. or record.get('raw_odom_x')!=0. or record.get('raw_odom_y')!=0. or
            record.get('raw_odom_yaw')!=0. or record.get('raw_odom_body_vx')!=-.04):return False
    backend=records[-1];states=backend.get('initial_states')
    if not states or len(states[0])<2 or any(len(row)<2 for row in states[0][:2]):return False
    return states[0][0][0]==0. and states[0][1][0]==0. and states[0][1][1]==0.


def analyze_trial(path):
    manifest=json.loads((path/'manifest.json').read_text())
    status=json.loads((path/'run_status.json').read_text())
    audit=json.loads((path/'trial_audit.json').read_text())
    outcome=json.loads((path/'result.json').read_text())
    native,_=read_records(path,'dual_timeline.jsonl')
    odom,_=read_records(path,'state_odom_events.jsonl')
    grouped={}
    for row in native or []:
        if row.get('request_id',0)>0:grouped.setdefault((row.get('pid'),row['request_id']),[]).append(row)
    requests=[]
    for (pid,request),rows in sorted(grouped.items()):
        selected=single(rows,'state_contract_selected_boundary')
        root=single(rows,'state_contract_native_astar_root')
        heads=[r for r in rows if r.get('event')=='state_contract_native_minco_head' and r.get('block_index')==0]
        head=heads[0] if len(heads)==1 else None
        backend=single(rows,'state_contract_backend_fixed')
        desired=single(rows,'state_contract_desired_boundary')
        end=single(rows,'state_contract_request_end')
        exact=match_observer(cpp_identity(selected or {}),odom)
        frame=snapshot_evidence(path,backend,selected) if backend else None
        static=(frame or {}).get('independent_fixed_static') or {}
        raw=selected.get('raw_odom_body_vx') if selected else None
        backend_states=backend.get('initial_states') if backend else None
        first=backend_states[0] if backend_states else None
        processed=selected.get('raw_odom_processed_vx') if selected else None
        minco_v=head['head_state'][0][1] if head else None
        backend_v=first[0][1] if first else None
        identity_proof=fixed_fixture_identity_proof(rows)
        frame_proof=(frame is not None and frame.get('payload_hash_verified') is True and
                     frame.get('schema')=='delivery_snapshot_v1' and frame.get('margin_is_0_10m') is True)
        actual_direction=(identity_proof and frame_proof and exact['exact_message_observed'] is True and raw==-.04 and
            selected is not None and selected.get('raw_odom_processed_gear')==-1 and
            selected.get('planner_gear')==-1 and processed is not None and processed<0 and
            desired is not None and desired['planner_gear']==-1 and desired['planner_v'][0]<0 and
            root is not None and root['native_root_gear']==-1 and root['native_signed_speed']<-.05 and
            head is not None and head['native_first_gear']==-1 and minco_v<0 and
            backend is not None and backend.get('fixed_gear')==-1 and backend_v<0 and
            static.get('category')=='safe' and (frame or {}).get('fixed_online_agreement') is True and
            backend_scope_evidence(rows,backend)['complete'] is True)
        requests.append(dict(pid=pid,request_id=request,raw_v=raw,processed_v=processed,
            selected_v=(selected or {}).get('planner_v'),desired_v=(desired or {}).get('planner_v'),
            processed_gear=(selected or {}).get('raw_odom_processed_gear'),
            native_root_v=(root or {}).get('native_signed_speed'),native_root_gear=(root or {}).get('native_root_gear'),
            first_minco_v=minco_v,first_minco_gear=(head or {}).get('native_first_gear'),
            backend_v=backend_v,backend_gear=(backend or {}).get('fixed_gear'),
            exact_raw_message=exact,frozen_static=static,request_success=(end or {}).get('success'),
            fixed_fixture_identity_proof=identity_proof,frame_identity_proof=frame_proof,
            negative_direction_native_chain_observed=bool(actual_direction)))
    fixture=manifest.get('directed_low_speed_probe') or {}
    return dict(trial=path.name,source=str(path),run_status=status,audit=audit,outcome=outcome,
        signed_low_speed=manifest['coupling_experiment']['signed_low_speed'],fixture=fixture,
        runtime_sha256=manifest.get('experiment_sources_sha256'),requests=requests,
        negative_direction_native_chain_observed=any(r['negative_direction_native_chain_observed'] for r in requests),
        interface_valid=audit.get('valid_experiment') is True,
        manifest_sha256=sha(path/'manifest.json'))


def write_verdict(out):
    paths=[out/('empty_coupled_seed11_signed_'+mode) for mode in ('off','on')]
    trials=[analyze_trial(path) for path in paths]
    assert_plot_quiet(dict(trials=trials))
    on=trials[1]
    same_fixture=trials[0]['fixture']==trials[1]['fixture']
    same_runtime=trials[0]['runtime_sha256']==trials[1]['runtime_sha256']
    passed=all(t['interface_valid'] for t in trials) and same_fixture and same_runtime and on['negative_direction_native_chain_observed']
    result=dict(schema='directed_low_speed_verdict_v1',verified_signed_negative_native_chain=passed,
        same_fixture=same_fixture,same_runtime=same_runtime,trials=trials,
        scope='native ROS fixed-state input probe; pose derivative is intentionally inconsistent with twist; NOT physical motion, navigation, real-world safety or speed comparison',
        complete_native_map_frozen_replay=False,reporter_sha256=sha(Path(__file__)))
    (out/'verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    lines=['低速倒退：原生链路定向状态输入验证',result['scope'],
        'ON负方向完整传入原生搜索/MINCO/backend='+str(passed),
        '相同fixture='+str(same_fixture)+'；相同runtime='+str(same_runtime),'']
    for trial in trials:
        lines.append(trial['trial']+'：接口审计='+str(trial['interface_valid']))
        for r in trial['requests']:
            lines.append('  请求%d raw=%s -> processed=%s -> root=%s(gear %s) -> MINCO=%s -> backend=%s；场内起点=%s'%(
                r['request_id'],r['raw_v'],r['processed_v'],r['native_root_v'],r['native_root_gear'],
                r['first_minco_v'],r['backend_v'],r['frozen_static'].get('category')))
    lines.extend(['','不将probe_complete计为到达目标，不根据此输入probe宣称避障效果提升。'])
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')
    return passed


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();build=args.build.resolve();out=args.out.resolve()
    m,_=check_build(build)
    if not m.get('state_contract_experiment') or m.get('boundary_admission_experiment'):
        raise ValueError('Requires the frozen original state-contract planner')
    check_space(out.parent,2*1024**3);out.mkdir(parents=True,exist_ok=False)
    plan=dict(created=datetime.now(timezone.utc).isoformat(),build=str(build),build_manifest_sha256=sha(build/'build_manifest.json'),
        scene='empty',group='coupled',seed=11,duration_s=18.,modes=[False,True],state=[0.,0.,0.,-.04,0.],goal=[-.8,0.,0.],
        performance_comparison=False,physical_motion_test=False,full_native_map_frozen=False)
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');done=[];runtime=None
    try:
        for mode in plan['modes']:
            name='empty_coupled_seed11_signed_'+('on' if mode else 'off')
            valid=run_trial(out/name,build,'empty','coupled',11,18.,1.,1.,False,
                plan['build_manifest_sha256'],expected_runtime_sources=runtime,reserve_bytes=2*1024**3,
                diagnostic_capture=True,domain_recovery=True,state_contract=True,signed_low_speed=mode,
                prepare_hook=apply_fixture)
            current=json.loads((out/name/'manifest.json').read_text())['experiment_sources_sha256']
            if runtime is None:runtime=current
            done.append(dict(trial=name,valid=valid));(out/'progress.json').write_text(json.dumps(done,indent=2)+'\n')
            if not valid:raise RuntimeError('Directed probe interface failure; retained without rerun')
        (out/'complete.json').write_text(json.dumps(dict(complete=True,completed=done),indent=2)+'\n')
        if not write_verdict(out):raise RuntimeError('Signed native chain not proven; do not proceed to admission experiment')
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(completed=done,reason=str(exc)),indent=2)+'\n');raise
    print('DIRECTED_LOW_SPEED_VERIFIED',out,flush=True)


if __name__=='__main__':main()
