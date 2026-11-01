#!/usr/bin/env python3
"""Independent read-only audit of the completed four-trial delivery batch."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

HERE=Path(__file__).resolve().parent
BATCH=HERE/'results/delivery_next_20261031_01'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def collision_detail(detail,body,resolution):
    points=np.asarray(detail['snapshot_points_xyz_velocity_track'],dtype=float)
    def gap(pose,time):
        x,y,yaw=pose;co,si=np.cos(yaw),np.sin(yaw)
        centers=body[:,:3].copy()
        centers[:,:2]=centers[:,:2]@np.array([[co,si],[-si,co]])+[x,y]
        predicted=points[:,:3]+(time-detail['snapshot_stamp'])*points[:,3:6]
        distance=np.linalg.norm(centers[:,None,:]-predicted[None,:,:],axis=2)
        hard=distance-body[:,3,None]
        # Exactly the native experiment's evaluation-only physical-sample
        # convention: remove one map voxel from BASE hard radii, not ARM radii.
        physical=distance-(body[:,3]-(body[:,6]==0)*resolution)[:,None]
        at=np.unravel_index(hard.argmin(),hard.shape)
        return dict(hard_gap_m=float(hard.min()),physical_sample_gap_m=float(physical.min()),
            closest_sphere=int(at[0]),closest_point=int(at[1]),sphere_type=int(body[at[0],6]))
    candidate=[detail['candidate_'+key] for key in ('x','y','yaw')]
    measured=[detail['measured_'+key] for key in ('x','y','yaw')]
    original_clock=detail['candidate_start']+detail['local_t']/detail['execution_scale']
    return dict(request_id=detail['request_id'],branch=detail['branch'],local_t=detail['local_t'],
        wall_ahead=detail['wall_ahead'],execution_scale=detail['execution_scale'],
        query_valid=detail['query_valid'],candidate_age_s=detail['candidate_age'],
        candidate_pose=candidate,measured_pose=measured,
        candidate_at_gate_clock=gap(candidate,detail['query_stamp']),
        candidate_at_unchanged_absolute_start_clock=gap(candidate,original_clock),
        measured_at_gate_forecast_clock=gap(measured,detail['query_stamp']),
        gate_query_time=detail['query_stamp'],alternate_absolute_query_time=original_clock,
        method='same logged CV points/velocities, same saved fixed-arm 67-sphere centres/radii; numerical geometry reconstruction, not a native intervention',
        interpretation='native hard-boundary overlap at the tested FUTURE pose/time; not hold/t0 rejection. Alternate clock tests sensitivity only and does not establish the correct fast-layer execution clock.')


def audit_trial(path):
    result=json.loads((path/'result.json').read_text())
    events=rows(path/'dual_timeline.jsonl');phases=rows(path/'cusp_phase_events.jsonl')
    cycles=rows(path/'state_mppi_cycles.jsonl');receipts=rows(path/'reference_events.jsonl')
    solver=rows(path/'solver_trace.jsonl');state=np.genfromtxt(path/'state.csv',delimiter=',',names=True)
    chains=[]
    for pub in (r for r in events if r['event']=='reference_publish'):
        receipt=next(r for r in receipts if r['mono']>=pub['mono'])
        revision=receipt['revision'];epoch=receipt['epoch']
        same=[r for r in cycles if r.get('guide_revision')==revision and r.get('guide_epoch')==epoch]
        kernel=next(r for r in same if r.get('first_kernel_cycle_for_reference'))
        validated=next(r for r in same if r.get('first_validated_command_cycle_for_reference'))
        if not validated['validated_command_published'] or validated['reason']!='ok':
            raise ValueError('First claimed validated command has no valid publication')
        trigger=next((r for r in events if r['event']=='refresh_trigger' and r['mono']<pub['mono']),None)
        if pub['request_id']==1:trigger=None
        request=next(r for r in events if r['event']=='request_begin' and r['request_id']==pub['request_id'])
        chains.append(dict(request_id=pub['request_id'],epoch=epoch,revision=revision,
            request_begin_mono=request['mono'],publish_mono=pub['mono'],receipt_mono=receipt['mono'],
            first_kernel_mono=kernel['online_step_entry_monotonic'],
            first_validated_publish_mono=validated['command_publish_after_monotonic'],
            publish_to_kernel_s=kernel['online_step_entry_monotonic']-pub['mono'],
            publish_to_validated_s=validated['command_publish_after_monotonic']-pub['mono'],
            trigger_to_kernel_s=kernel['online_step_entry_monotonic']-trigger['mono'] if trigger else None,
            trigger_to_validated_s=validated['command_publish_after_monotonic']-trigger['mono'] if trigger else None,
            publish_reference_age_sim_s=pub['sim']-pub['guide_start'],
            first_kernel_reason=kernel['reason'],first_validated_reason=validated['reason']))
    body_record=json.loads((path/'native_collision_3d_body.json').read_text())
    body=np.asarray(body_record['body']);resolution=body_record['header'][11]
    parameters=yaml.safe_load((path/'parameters.yaml').read_text())
    uncertainty={k:v for k,v in parameters.items() if k.startswith('/remani_planner_node/dynamic_point_flow/') and
                 any(word in k for word in ('uncertainty','acceleration_bound'))}
    details=[]
    for d in (r for r in events if r['event']=='latest_prediction_reject_detail'):
        if any(float(v)!=0 for v in uncertainty.values()):
            raise ValueError('Zero-uncertainty reconstruction does not apply')
        details.append(collision_detail(d,body,resolution))
    opt=[r for r in solver if r.get('event')=='end' and r.get('stage')=='optimizer']
    selected=[r for r in events if r['event']=='early_delivery_check' and r.get('selected_for_exit')]
    for r in selected:
        if not r['native_pass'] or r['budget_expired'] or r['elapsed_end_ms']>=1200.:
            raise ValueError('Claimed live candidate was not natively checked inside budget')
    terminal=state[state['t']>=state['t'][-1]-2.]
    far=(np.hypot(terminal['x']-6.,terminal['y'])>.35)
    stopped=(np.abs(terminal['v'])<.02)&(np.abs(terminal['w'])<.06)
    with gzip.open(path/'mppi_diagnostics.json.gz','rt') as stream:diagnostics=json.load(stream)
    capture=json.loads((path/'delivery_event_capture.json').read_text())
    capture_verified=None
    if capture['recorded']:
        metadata=json.loads((path/'native3d_event_input.json').read_text())
        if sha(path/'native3d_event_input.npz')!=metadata['npz_sha256']:
            raise ValueError('Saved complete frame hash mismatch')
        with np.load(path/'native3d_event_input.npz',allow_pickle=False) as archive:
            header=archive['payload'][:32]
            if header[20]!=1. or header[12]!=67.:
                raise ValueError('Capture does not contain complete expected native geometry/map')
            capture_verified=dict(event=metadata['event'],cycle_id=metadata['cycle_id'],
                body_count=int(header[12]),full_esdf=bool(header[20]),snapshot_sequence=int(header[2]),
                state=archive['state'].tolist(),npz_sha256=metadata['npz_sha256'])
    return dict(trial=path.name,result_reason=result['reason'],
        sampled_physical_collision=result['native3d_truth_monitor']['sample_collision'],
        min_physical_sample_gap_m=result['native3d_truth_monitor']['min_physical_sample_gap'],
        physical_truth_checks=result['native3d_truth_monitor']['checks'],
        truth_scope=result['native3d_truth_monitor']['scope'],
        final_state=result['final_state'],final_goal_distance_m=float(np.linalg.norm(np.asarray(result['final_state'][:2])-[6.,0.])),
        terminal_two_seconds_non_goal_stopped_samples=int(np.sum(far&stopped)),
        phase_events=phases,actual_phase_transition_count=sum(bool(r.get('measured_phase_transition')) for r in phases),
        maximum_seen_reference_phase_count=max(r['debug'][2] for r in phases),
        native_backend_passes=sum(bool(r['success']) for r in opt),native_backend_requests=len(opt),
        early_selected=len(selected),early_selected_end_elapsed_ms=[r['elapsed_end_ms'] for r in selected],
        latest_rejections=[r for r in events if r['event']=='acceptance_reject'],rejection_reconstruction=details,
        delivery_chains=chains,capture=capture,capture_verified=capture_verified,
        control_reasons=dict(Counter(r['reason'] for r in diagnostics)),
        source_sha256={name:sha(path/name) for name in ('dual_timeline.jsonl','solver_trace.jsonl','reference_events.jsonl',
            'state_mppi_cycles.jsonl','cusp_phase_events.jsonl','state.csv','result.json','native_collision_3d_body.json')})


def main():
    complete=json.loads((BATCH/'complete.json').read_text())
    if not complete.get('complete'):raise ValueError('Batch incomplete')
    trials=[audit_trial(BATCH/job['trial']) for job in complete['completed']]
    out=BATCH/'report';out.mkdir(exist_ok=True)
    report=dict(scope='independent read-only source-event and geometry audit; no simulation/replanning/rendering',
        complete_sha256=sha(BATCH/'complete.json'),script_sha256=sha(Path(__file__)),trials=trials,
        limitations=['one trial per scene/group; closed-loop request inputs differ',
            'both groups share cusp association; only late fixed-input A/B isolates that fix',
            'rejected future pose tested at two clocks, not an executed collision or proof which timing policy is correct',
            'physical sphere/cylinder sampling is not continuous full-mesh collision proof'])
    (out/'independent_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    text=['独立核对：轨迹交付与换向',
        '1. 四次都到达目标；原生后端验收共9次全部通过，但其中1条被下游最新动态预测拒收，所以实际发布8条。没有把“优化器通过”误记成“机器人已经用上”。四次物理球采样监测均未发现碰撞，最小采样间隙约8.70～29.26厘米；这不是整机网格连续碰撞证明。',
        '2. 没再出现旧的终点前约36厘米换向卡死。动态A实际经历前进→倒车→前进两次换向，并到达目标；其余三次新参考只有一个前进段，不能拿它们证明复杂换向已普遍解决。动态A留有一次完整67球＋ESDF事件输入，文件哈希已核对。',
        '3. 首次重规划触发到MPPI首次读取新参考／首次发布获准指令：']
    for t in trials:
        refresh=t['delivery_chains'][-1]
        text.append('   %s：%.3f秒／%.3f秒。'%(t['trial'],refresh['trigger_to_kernel_s'],refresh['trigger_to_validated_s']))
    text.extend([
        '   已逐条用发布事件→接收revision/epoch→控制周期核对，未用重复guide_id或日志写出时间代替采用时间。静态A第一次读取后未通过执行检查，下一周期才发出获准指令，两种时间没有混写。',
        '4. 动态A第2次请求的拒收已定位到未来轨迹检查，不是t=0保持检查：候选相对时间2.5秒、执行比例1、有效CV查询，底盘硬边界间隙约-5.83毫米。去掉原底盘5厘米体素附加量后，球采样物理间隙仍约+4.42厘米，因此不能说这里已经发生物理碰撞。',
        '   此候选发布前已老了约1.008秒。保持候选姿态不变，仅将查询时间从“现在+2.5秒”换成“候选原起始时间+2.5秒”，硬间隙变成+9.74厘米，说明验收对时间锚点敏感。但MPPI并不严格按原绝对时刻执行，尚不能据此认定最新检查错了，更不能直接删除安全门。应把快层实际采用的进度/时序与慢层验收假设对齐。',
        '5. 这轮说明提前检查并交付合格候选有实际收益，同时换向关联在一个真实三段参考中正常推进；不代表双时间尺度协同已全部解决，也不代表消除了绕障中的短暂停车。两组各一次且闭环输入已经不同，不能把所有时延和路线差别都归功于一个改动。'])
    (out/'独立核对.txt').write_text('\n\n'.join(text)+'\n')
    print(json.dumps([dict(trial=t['trial'],goal=t['result_reason'],native='%s/%s'%(t['native_backend_passes'],t['native_backend_requests']),
        rejections=len(t['latest_rejections']),transitions=t['actual_phase_transition_count'],
        collision=t['sampled_physical_collision'],refresh=t['delivery_chains'][-1]) for t in trials],indent=2))


if __name__=='__main__':main()
