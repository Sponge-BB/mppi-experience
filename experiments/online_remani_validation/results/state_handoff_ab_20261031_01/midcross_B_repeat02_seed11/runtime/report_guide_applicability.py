#!/usr/bin/env python3
"""Offline first-use applicability of the exact sampled guide given to MPPI.

Polyline projection is a diagnostic neighborhood, NOT proof of the kernel's
chosen segment, a true splice, a feasible connection, or native-polynomial
tracking. No ROS/model work or file reads occur on import. The CLI checks the
completed batch's recorded PIDs/port before analysis as well as before plotting.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct

from report_domain_recovery import assert_plot_quiet, digest, read_json, read_records
from report_state_contract import (conjunction, finite, integer, match_observer,
    observer_index, publish_bounds, reference_key, reference_link, state_matches_identity)
from state_contract_records import record_with_validity


PARAMETERS = dict(gear_speed_epsilon_mps=1e-6, branch_distance_tolerance_m=.02,
                  zero_command_epsilon=1e-6, measured_low_v_mps=.02, measured_low_w_radps=.05)
LIMITATIONS = [
    '使用首次实际online_step调用时复制的状态及精确transaction/refSHA/odom链，不用p0残差或最近CSV代替。',
    '投影对象是实际记录的MPPI sampled_reference数组，不是原生多项式；数组内线性/角度插值只是离线诊断。',
    '最近有序same-gear片段不等于kernel实际选中片段、真实拼接、可行连接或控制授权；交叉/多分支保留歧义。',
    '零速/极低速实测状态不能证明当前gear；不继承历史方向，不用角度相似偷偷确定gear。',
    'MPPI使用整条几何guide；本报告不把native epoch推算成实际消费相位，也不丢弃负/过期epoch部分。',
    'command变化、零命令原因、实测低速及恢复只是时序关联；命令非零不等于物理恢复，缺失时钟不补50ms。',
    'OFF/ON是不同闭环请求，不是同一输入因果配对；诊断I/O可改变后续时序，不作速度或安全改善结论。',
]


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def gear(speed, epsilon=PARAMETERS['gear_speed_epsilon_mps']):
    return (1 if speed>epsilon else -1 if speed < -epsilon else None) if finite(speed) else None


def guide_token(row):
    """Native guide identity is available even on a gated/no-kernel cycle."""
    generation=row.get('goal_generation');revision=row.get('guide_revision');epoch=row.get('guide_epoch')
    hashes=row.get('message_hashes')
    if not integer(generation) or not integer(revision) or not integer(epoch) or revision<1 or epoch<1:
        return None
    if not isinstance(hashes,list) or not hashes or not all(isinstance(v,str) and len(v)==64 for v in hashes):return None
    return generation,revision,epoch,tuple(hashes)


def sampled_payload(record):
    rows=record.get('reference') if isinstance(record,dict) else None
    times=record.get('times') if isinstance(record,dict) else None
    result=dict(representation='recorded MPPI sampled_reference [x,y,body_yaw,signed_speed]',
        payload_sha256=None,payload_sha256_verified=None,ordered_times_valid=None,
        native_polynomial_evaluation=None,
        native_polynomial_scope='Native polynomial coefficients elsewhere are not joined/evaluated by this report.')
    if not isinstance(rows,list) or len(rows)<2 or not all(isinstance(r,list) and len(r)==4 and all(finite(v) for v in r) for r in rows):
        result['error']='missing_or_invalid_finite_Nx4_sampled_array';return result,None,None
    raw=b''.join(struct.pack('<4d',*row) for row in rows)
    actual=hashlib.sha256(raw).hexdigest();result['payload_sha256']=actual
    result['payload_sha256_verified']=actual==record.get('sha256')
    result['ordered_times_valid']=isinstance(times,list) and len(times)==len(rows) and all(finite(t) for t in times) and all(b>=a for a,b in zip(times,times[1:]))
    if result['payload_sha256_verified'] is not True or result['ordered_times_valid'] is not True:
        result['error']='unverified_array_hash_or_missing_ordered_sample_times';return result,None,None
    return result,rows,times


def sampled_neighborhood(state, rows, times, parameters=None):
    """Project onto adjacent ordered samples, retaining disjoint near-ties."""
    p=dict(PARAMETERS,**(parameters or {}));current=gear(state[3],p['gear_speed_epsilon_mps']) if isinstance(state,list) and len(state)==5 else None
    answer=dict(status=None,current_measured_gear=current,diagnostic_only=True,true_splice_proven=False,
        temporal_phase_window_applied=False,nearest_errors=None,candidate_branches=[],eligible_segments=None,
        unavailable_reason=None)
    if not isinstance(state,list) or len(state)!=5 or not all(finite(v) for v in state):
        answer.update(status='unknown',unavailable_reason='missing_finite_copied_state');return answer
    if (not isinstance(rows,list) or len(rows)<2 or not isinstance(times,list) or len(times)!=len(rows)
            or not all(isinstance(r,list) and len(r)==4 and all(finite(v) for v in r) for r in rows)
            or not all(finite(t) for t in times) or not all(b>=a for a,b in zip(times,times[1:]))):
        answer.update(status='unknown',unavailable_reason='unverified_sampled_array_or_order');return answer
    candidates=[]
    for index,(a,b) in enumerate(zip(rows,rows[1:])):
        if not times[index+1]>=times[index]:continue
        ga,gb=gear(a[3],p['gear_speed_epsilon_mps']),gear(b[3],p['gear_speed_epsilon_mps'])
        segment_gear=ga if ga==gb else (ga if gb is None else gb if ga is None else None)
        if segment_gear is None or current is not None and segment_gear!=current:continue
        dx=b[0]-a[0];dy=b[1]-a[1];length2=dx*dx+dy*dy
        # Repeated native segment endpoints can share a time. Never invent a
        # spatial connector that takes zero recorded reference time.
        if times[index+1]==times[index] and length2>1e-20:continue
        fraction=max(0.,min(1.,((state[0]-a[0])*dx+(state[1]-a[1])*dy)/length2)) if length2>1e-20 else 0.
        x=a[0]+fraction*dx;y=a[1]+fraction*dy;yaw=wrap(a[2]+fraction*wrap(b[2]-a[2]))
        speed=a[3]+fraction*(b[3]-a[3]);ex=state[0]-x;ey=state[1]-y
        candidates.append(dict(segment_index=index,gear=segment_gear,fraction=fraction,
            reference_time=times[index]+fraction*(times[index+1]-times[index]),
            projected_sampled_pose=[x,y,yaw],reference_signed_speed=speed,
            distance_m=math.hypot(ex,ey),lateral_error_m=-math.sin(yaw)*ex+math.cos(yaw)*ey,
            heading_error_rad=wrap(state[2]-yaw),speed_error_mps=state[3]-speed,
            ordered_tangent_cross_track_m=(dx*ey-dy*ex)/math.sqrt(length2) if length2>1e-20 else None,
            ordered_tangent_residual_m=(dx*ex+dy*ey)/math.sqrt(length2) if length2>1e-20 else None,
            degenerate_sample_segment=length2<=1e-20))
    answer['eligible_segments']=len(candidates)
    if not candidates:
        answer.update(status='no_same_gear_fragment' if current is not None else 'current_gear_unknown',
                      unavailable_reason='no_ordered_segment_with_provable_compatible_gear');return answer
    best=min(row['distance_m'] for row in candidates)
    near=[row for row in candidates if row['distance_m']<=best+p['branch_distance_tolerance_m']]
    branches=[]
    for row in near:
        if not branches or row['segment_index']!=branches[-1][-1]['segment_index']+1 or row['gear']!=branches[-1][-1]['gear']:
            branches.append([])
        branches[-1].append(row)
    for branch in branches:
        nearest=min(branch,key=lambda row:(row['distance_m'],row['segment_index']))
        answer['candidate_branches'].append(dict(sample_segment_range=[branch[0]['segment_index'],branch[-1]['segment_index']],
            nearby_segment_count=len(branch),gear=nearest['gear'],same_gear_proven=current is not None,
            representative=nearest,heading_error_range_rad=[min(r['heading_error_rad'] for r in branch),max(r['heading_error_rad'] for r in branch)],
            speed_error_range_mps=[min(r['speed_error_mps'] for r in branch),max(r['speed_error_mps'] for r in branch)]))
    if current is None:answer['status']='current_gear_unknown'
    elif len(branches)>1:answer['status']='ambiguous_sampled_branches'
    else:
        answer['status']='unique_sampled_same_gear_neighborhood'
        answer['nearest_errors']=answer['candidate_branches'][0]['representative']
    return answer


def command_class(row, parameters=None):
    p=dict(PARAMETERS,**(parameters or {}));v=row.get('command_v');w=row.get('command_w')
    return abs(v)<=p['zero_command_epsilon'] and abs(w)<=p['zero_command_epsilon'] if finite(v) and finite(w) else None


def cycle_time(row):
    bounds=publish_bounds(row)
    return sum(bounds)/2. if bounds else None


def duration_bounds(start,end):
    a=publish_bounds(start);b=publish_bounds(end)
    if a is None or b is None or b[0]<a[1]:return None
    return [b[0]-a[1],b[1]-a[0]]


def compact_cycle(row):
    return dict(cycle_id=row.get('cycle_id'),guide_token=list(guide_token(row)) if guide_token(row) else None,
        publish_ros_bounds=publish_bounds(row),command_v=row.get('command_v'),command_w=row.get('command_w'),
        reason=row.get('reason'),validated_command_published=row.get('validated_command_published'),
        validation_state=row.get('validation_state'),command_zero=command_class(row))


def contiguous(a,b):
    return integer(a.get('cycle_id')) and integer(b.get('cycle_id')) and b['cycle_id']==a['cycle_id']+1


def classification_run(rows,index,classifier,stop=None):
    """A run crossing first-use can predate adoption; gaps censor, never fill."""
    if not 0<=index<len(rows) or classifier(rows[index]) is not True:return None
    start=index;end=index
    while start>0 and contiguous(rows[start-1],rows[start]) and classifier(rows[start-1]) is True:start-=1
    limit=len(rows) if stop is None else stop
    while end+1<limit and contiguous(rows[end],rows[end+1]) and classifier(rows[end+1]) is True:end+=1
    following=rows[end+1] if end+1<limit and contiguous(rows[end],rows[end+1]) else None
    recovered=following is not None and classifier(following) is False
    return dict(start_cycle_id=rows[start].get('cycle_id'),last_classified_cycle_id=rows[end].get('cycle_id'),
        started_before_first_kernel=start<index,
        left_censored=start==0 or start>0 and (not contiguous(rows[start-1],rows[start]) or classifier(rows[start-1]) is None),
        right_censored=not recovered,recorded_cycle_count=end-start+1,
        publication_duration_bounds_s=duration_bounds(rows[start],following) if recovered else None,
        recovery_cycle=compact_cycle(following) if recovered else None,
        reasons=dict(Counter(row.get('reason','<missing>') for row in rows[start:end+1])))


def transition_context(rows,index,odom,parameters=None):
    p=dict(PARAMETERS,**(parameters or {}));first=rows[index];token=guide_token(first);end=index+1
    while end<len(rows) and contiguous(rows[end-1],rows[end]) and guide_token(rows[end])==token:end+=1
    before=rows[index-1] if index>0 and contiguous(rows[index-1],first) else None
    tenure=rows[index:end];validated=next((r for r in tenure if r.get('validated_command_published') is True),None)
    nonzero=next((r for r in tenure if command_class(r,p) is False),None)
    after=tenure[1] if len(tenure)>1 else None
    def measured_low(row):
        state=row.get('validation_state');identity=row.get('validation_odom_identity')
        match=match_observer(identity,odom)
        if match['exact_message_observed'] is not True or state_matches_identity(state,identity) is not True:return None
        return abs(state[3])<p['measured_low_v_mps'] and abs(state[4])<p['measured_low_w_radps']
    zero_runs=[];i=index
    while i<end:
        if command_class(rows[i],p) is True:
            run=classification_run(rows,i,lambda row:command_class(row,p),end);zero_runs.append(run)
            while i+1<end and command_class(rows[i+1],p) is True:i+=1
        i+=1
    delta=[first['command_v']-before['command_v'],first['command_w']-before['command_w']] if before is not None and all(finite(row.get(k)) for row in (before,first) for k in ('command_v','command_w')) else None
    return dict(before=compact_cycle(before) if before else None,first=compact_cycle(first),
        next=compact_cycle(after) if after else None,command_delta_v_w=delta,
        same_guide_last_cycle_id=tenure[-1].get('cycle_id'),same_guide_observed_cycles=len(tenure),
        first_validated_command=compact_cycle(validated) if validated else None,
        first_nonzero_command=compact_cycle(nonzero) if nonzero else None,
        first_kernel_to_nonzero_command_bounds_s=duration_bounds(first,nonzero) if nonzero is not None and nonzero is not first else ([0.,0.] if nonzero is first and publish_bounds(first) else None),
        measured_motion_observed_at_first_nonzero_command=measured_low(nonzero) is False if nonzero is not None and measured_low(nonzero) is not None else None,
        zero_command_run_at_first_kernel=classification_run(rows,index,lambda row:command_class(row,p),end),
        measured_low_motion_run_at_first_kernel=classification_run(rows,index,measured_low,end),
        zero_command_runs=zero_runs,reason_counts=dict(Counter(r.get('reason','<missing>') for r in tenure)),
        zero_command_reasons=dict(Counter(r.get('reason','<missing>') for r in tenure if command_class(r,p) is True)),
        interval_semantics='contiguous publication-cycle order until this native guide changes; durations are local publish-call bounds, not actuator execution')


def analyze_guides(cycles,odom,consumed,commits,observer,parameters=None):
    rows=cycles or [];ids=Counter(row.get('cycle_id') for row in rows);groups=defaultdict(list)
    ordered=bool(rows) and all(integer(row.get('cycle_id')) and row['cycle_id']>0 for row in rows) and all(b['cycle_id']>a['cycle_id'] for a,b in zip(rows,rows[1:]))
    for index,row in enumerate(rows):
        token=guide_token(row)
        if token is not None and row.get('online_step_invoked') is True and reference_key(row) is not None:groups[token].append(index)
    output=[];index=observer_index(odom)
    for token,indices in groups.items():
        first_index=indices[0];first=rows[first_index];refkey=reference_key(first)
        refs=[ref for ref in consumed or [] if reference_key(ref,True)==refkey]
        link=reference_link(first,consumed,commits,observer)
        payload,sampled,times=sampled_payload(refs[0] if len(refs)==1 else None)
        match=match_observer(first.get('solve_odom_identity'),odom,index)
        copied_matches=state_matches_identity(first.get('solve_state'),first.get('solve_odom_identity'))
        unique=integer(first.get('cycle_id')) and ids[first['cycle_id']]==1
        first_flag=first.get('first_kernel_cycle_for_reference') is True if isinstance(first.get('first_kernel_cycle_for_reference'),bool) else None
        exact=conjunction((ordered,unique,first_flag,link['complete_exact_reference_chain'],payload['payload_sha256_verified'],
                           match['exact_message_observed'],copied_matches))
        projection=sampled_neighborhood(first.get('solve_state'),sampled,times,parameters) if exact is True else None
        output.append(dict(goal_generation=token[0],guide_revision=token[1],guide_epoch=token[2],message_hashes=list(token[3]),
            first_kernel_cycle_id=first.get('cycle_id'),online_step_entry_ros_time=first.get('online_step_entry_ros_time'),
            state_copy_ros_time=first.get('solve_ros_time'),copied_solve_state=first.get('solve_state'),
            solve_odom_identity=first.get('solve_odom_identity'),solve_observer_match=match,state_equals_identity=copied_matches,
            unique_first_cycle_id=unique,recorded_first_kernel_flag=first.get('first_kernel_cycle_for_reference'),
            first_flag_consistent=first_flag,recorded_cycle_order_valid=ordered,
            reference=link,sampled_payload=payload,exact_association=exact,applicability=projection,
            kernel_code=first.get('kernel_code'),online_step_succeeded=first.get('online_step_succeeded'),
            transition=transition_context(rows,first_index,odom,parameters)))
    return dict(present=cycles is not None,observed_cycle_count=len(rows) if cycles is not None else None,
                first_use_guides=len(output) if cycles is not None else None,records=output,
                recorded_cycle_order_valid=ordered if rows else None,
                any_exact_guide=any(row['exact_association'] is True for row in output) if output else None)


def trial_report(trial):
    trial=Path(trial);names=('manifest.json','run_status.json','trial_audit.json','result.json')
    docs={name:read_json(trial/name) for name in names};manifest=docs['manifest.json']
    settings=manifest.get('coupling_experiment') or {};status=docs['run_status.json']
    if status.get('returncode') is None or status.get('result_written') is not True or not isinstance(settings.get('signed_low_speed'),bool):
        raise ValueError('Need explicitly finished signed OFF/ON trial: '+str(trial))
    records={};hashes={name:digest(trial/name) for name in names}
    for name in ('state_mppi_cycles.jsonl','state_odom_events.jsonl','consumed_references.jsonl','reference_events.jsonl','observer_events.jsonl'):
        records[name],path=read_records(trial,name)
        if path is not None:hashes[path.name]=digest(path)
    guides=analyze_guides(*(records[name] for name in ('state_mppi_cycles.jsonl','state_odom_events.jsonl','consumed_references.jsonl','reference_events.jsonl','observer_events.jsonl')))
    return dict(trial=trial.name,source=str(trial),signed_low_speed=settings['signed_low_speed'],run_status=status,
        audit=docs['trial_audit.json'],outcome=docs['result.json'],input_sha256=hashes,guides=guides,
        runtime_sha256=manifest.get('experiment_sources_sha256'))


def build_report(root):
    root=Path(root).resolve();complete=read_json(root/'complete.json');plan=read_json(root/'plan.json');jobs=complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs,list) or len(jobs)!=2:raise ValueError('Two explicitly completed trials required')
    names=[job.get('trial') for job in jobs]
    if any(not isinstance(name,str) or Path(name).name!=name or name in ('.','..') for name in names) or len(set(names))!=2:raise ValueError('Invalid trial names')
    # Before reading large logs, verify the same recorded-process/port contract
    # used by the state report. No signal or ROS connection is ever sent.
    assert_plot_quiet(dict(trials=[dict(run_status=read_json(root/name/'run_status.json')) for name in names]))
    trials=sorted([trial_report(root/name) for name in names],key=lambda trial:trial['signed_low_speed'])
    if {trial['signed_low_speed'] for trial in trials}!={False,True}:raise ValueError('Distinct OFF/ON trials required')
    return record_with_validity(dict(schema='guide_applicability_v1',created=datetime.now(timezone.utc).isoformat(),
        root=str(root),diagnostic_only=True,true_splice_proven=False,performance_comparison=False,paired_same_input=False,
        parameters=PARAMETERS,limitations=LIMITATIONS,plan=plan,complete=complete,trials=trials,
        root_input_sha256={name:digest(root/name) for name in ('plan.json','complete.json')},
        conclusion='精确定位guide首次实际调用；局部sampled片段相容性与command时序不是实际拼接证明或OFF/ON因果改善。'))


def show(value):return '未知' if value is None else ('%.4g'%value if finite(value) else str(value))


def report_text(report):
    lines=['Guide实际接入诊断（不是p0残差/真实拼接证明）',report['conclusion']]
    for trial in report['trials']:
        lines.extend(['','%s：%s'%('ON' if trial['signed_low_speed'] else 'OFF',trial['trial'])])
        for guide in trial['guides']['records']:
            app=guide.get('applicability');errors=app.get('nearest_errors') if isinstance(app,dict) else None
            transition=guide['transition'];recovery=transition['first_nonzero_command']
            lines.append('rev=%s first cycle=%s exact=%s status=%s；横向/朝向/速度误差=%s/%s/%s；Δcmd(v,w)=%s；first非零cmd=%s；零命令原因=%s'%(
                guide['guide_revision'],guide['first_kernel_cycle_id'],show(guide['exact_association']),app.get('status') if app else '未知',
                show(errors.get('lateral_error_m') if errors else None),show(errors.get('heading_error_rad') if errors else None),
                show(errors.get('speed_error_mps') if errors else None),show(transition['command_delta_v_w']),
                recovery.get('cycle_id') if recovery else '未知',json.dumps(transition['zero_command_reasons'],ensure_ascii=False)))
            stop=transition['zero_command_run_at_first_kernel'];measured=transition['measured_low_motion_run_at_first_kernel']
            lines.append('  首用时zero-command停顿界=%s秒；是否先于guide首用=%s；右删失=%s；实测低速段恢复cycle=%s（命令非零不是运动证明）'%(
                show(stop.get('publication_duration_bounds_s') if stop else None),show(stop.get('started_before_first_kernel') if stop else None),
                show(stop.get('right_censored') if stop else None),
                measured['recovery_cycle'].get('cycle_id') if measured and measured.get('recovery_cycle') else '未知'))
    return '\n'.join(lines+['','解释边界：']+LIMITATIONS)+'\n'


def plot_report(report,path):
    assert_plot_quiet(report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(11.,7.),squeeze=False)
    for column,trial in enumerate(report['trials']):
        guides=trial['guides']['records']
        for row,(field,label) in enumerate((('lateral_error_m','Lateral error (m)'),('heading_error_rad','Heading error (rad)'),('speed_error_mps','Signed speed error (m/s)'))):
            ax=axes[row,column];known=[]
            for index,guide in enumerate(guides):
                app=guide.get('applicability');errors=app.get('nearest_errors') if isinstance(app,dict) else None
                value=errors.get(field) if isinstance(errors,dict) else None
                if finite(value):known.append((index,value))
                else:ax.text(index,.08,'ambig' if app and app.get('status')=='ambiguous_sampled_branches' else 'unknown',rotation=90,fontsize=6,ha='center',transform=ax.get_xaxis_transform())
            if known:ax.scatter([v[0] for v in known],[v[1] for v in known],s=22)
            ax.axhline(0.,color='.7',linewidth=.5);ax.grid(alpha=.2);ax.set_ylabel(label)
            ax.set_xticks(range(len(guides)));ax.set_xticklabels([str(g['guide_revision']) for g in guides],fontsize=7);ax.set_xlabel('Guide revision (not paired requests)')
        axes[0,column].set_title('ON' if trial['signed_low_speed'] else 'OFF')
    fig.suptitle('First actual kernel call vs ordered same-gear sampled neighborhood — not a true splice',fontsize=10)
    fig.tight_layout(rect=(0,0,1,.96));fig.savefig(path,dpi=130);plt.close(fig)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--plot',action='store_true');args=parser.parse_args(argv)
    root=args.root.resolve();out=args.out.resolve()
    if out.exists() or out==root or out in root.parents:parser.error('--out must be a fresh directory, never input or an ancestor')
    report=build_report(root);out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    (out/'实验结论.txt').write_text(report_text(report),encoding='utf-8')
    if args.plot:plot_report(report,out/'first_use_applicability.png')
    print(json.dumps(dict(out=str(out),conclusion=report['conclusion']),ensure_ascii=False));return 0


if __name__=='__main__':raise SystemExit(main())
