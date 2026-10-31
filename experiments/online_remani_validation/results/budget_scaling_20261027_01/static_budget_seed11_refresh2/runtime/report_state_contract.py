#!/usr/bin/env python3
"""Post-run, read-only exact state/reference-chain diagnostic, never a benchmark.

Importing this module does not contact ROS, run a model, read trial logs, or
plot. Reports require two explicitly completed trials. Plotting additionally
uses report_domain_recovery.assert_plot_quiet to check recorded PIDs and port.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import re
import json

from report_domain_recovery import assert_plot_quiet, digest, read_json, read_records
from state_contract_records import IDENTITY_ALGORITHM, fnv1a64, record_with_validity


LIMITATIONS = [
    '单seed静态诊断SIL；OFF/ON闭环请求可能不同，不是同一冻结输入的因果配对。',
    '同步诊断写入/flush会扰动后续周期时序；本实验不是速度benchmark，也不证明动态或硬件安全。',
    '只以序列化hash+seq+纳秒stamp+frames关联状态；禁止最近CSV/近时刻替代精确身份。',
    'callback-entry不是网络到达时刻；capture_mono/capture_sim才是C++事件发生时刻，mono/sim可能是延迟flush时刻。',
    '缺失或歧义为null/未知，空证据不算完整链；同一payload重复回调不推断唯一收包时刻。',
    'kernel返回0、验证成功、最终命令发布与非零物理运动是不同事实；有效性字段不是安全认证。',
    'backend快照/已有静态10cm评估仅作同一冻结输入诊断；报告不重跑规划器或改变碰撞门限。',
]
SHA256 = re.compile(r'^[0-9a-f]{64}$')
CPP_STAGES = ('selected_boundary', 'desired_boundary', 'manager_input', 'frontend_init',
              'native_astar_input', 'native_astar_root', 'native_astar_result',
              'native_minco_head', 'backend_fixed', 'solver_return', 'epoch_chosen',
              'candidate_ready', 'reference_publish', 'request_end','backend_scope_link')
CRITICAL_STAGES = ('selected_boundary','desired_boundary','manager_input','backend_fixed',
                   'solver_return','epoch_chosen','candidate_ready','reference_publish','request_end')
RAW_STAGES = ('selected_boundary','desired_boundary','manager_input','backend_fixed','candidate_ready','reference_publish')


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def nonempty_hashes(value):
    return isinstance(value, list) and bool(value) and all(isinstance(v, str) and SHA256.fullmatch(v) for v in value)


def identity_key(identity):
    """No fallback to a stamp, position, nearest sample, or incomplete identity."""
    if not isinstance(identity, dict) or identity.get('algorithm') != IDENTITY_ALGORITHM:
        return None
    value = identity.get('message_hash')
    if (not isinstance(value, str) or not value.isascii() or not value.isdecimal()
            or str(int(value)) != value or not 0 <= int(value) <= 0xffffffffffffffff):
        return None
    seq, sec, nsec = [identity.get(k) for k in ('header_seq', 'stamp_sec', 'stamp_nsec')]
    if not all(integer(v) for v in (seq, sec, nsec)) or not 0 <= seq <= 0xffffffff or not 0 <= sec <= 0xffffffff or not 0 <= nsec < 1000000000:
        return None
    if not all(isinstance(identity.get(k), str) for k in ('frame_id', 'child_frame_id')):
        return None
    if isinstance(identity.get('validity'), dict) and identity['validity'].get('serialized') is False:
        return None
    return (IDENTITY_ALGORITHM, value, seq, sec, nsec, identity['frame_id'], identity['child_frame_id'])


def cpp_identity(row):
    if row.get('raw_odom_missing') is not False:
        return None
    return dict(algorithm=row.get('raw_odom_message_hash_algorithm'), message_hash=row.get('raw_odom_message_fnv64'),
                header_seq=row.get('raw_odom_header_seq'), stamp_sec=row.get('raw_odom_stamp_sec'),
                stamp_nsec=row.get('raw_odom_stamp_nsec'), frame_id=row.get('raw_odom_frame_id'),
                child_frame_id=row.get('raw_odom_child_frame_id'))


def state_matches_identity(state, identity):
    measured = identity.get('state') if isinstance(identity, dict) else None
    if not isinstance(state, list) or not isinstance(measured, list) or len(state) != 5 or len(measured) != 5:
        return None
    if not all(finite(v) for v in state + measured):
        return None
    return state == measured


def observer_index(rows):
    index = defaultdict(list)
    for row in rows or []:
        key = identity_key(row.get('odom_identity'))
        if row.get('event') == 'odom_callback' and key is not None:
            index[key].append(row)
    return index


def match_observer(identity, rows, index=None):
    key = identity_key(identity)
    matches = (index if index is not None else observer_index(rows)).get(key, []) if key is not None else []
    known = bool(rows) and key is not None
    return dict(identity_complete=key is not None, exact_message_observed=bool(matches) if known else None,
                matching_receipts=len(matches) if rows is not None and key is not None else None,
                unique_callback_ros_time=matches[0].get('callback_ros_time') if len(matches) == 1 else None,
                unique_callback_monotonic=matches[0].get('callback_monotonic') if len(matches) == 1 else None)


def reference_key(row, consumed=False):
    revision = row.get('revision' if consumed else 'guide_revision')
    epoch = row.get('epoch' if consumed else 'guide_epoch')
    hashes = row.get('message_hashes')
    array_hash = row.get('sha256' if consumed else 'reference_array_sha256')
    if not integer(revision) or revision < 1 or not integer(epoch) or epoch < 1 or not nonempty_hashes(hashes):
        return None
    if not isinstance(array_hash, str) or not SHA256.fullmatch(array_hash):
        return None
    return revision, epoch, tuple(hashes), array_hash


def reference_link(cycle, consumed, commits, observer):
    """Complete reference identity, including goal-keyed transaction receipts."""
    key = reference_key(cycle)
    refs = [r for r in consumed or [] if key is not None and reference_key(r, True) == key]
    commit = [r for r in commits or [] if key is not None and r.get('event') == 'replace' and
              (r.get('revision'), r.get('epoch'), tuple(r.get('message_hashes') or [])) == key[:3]]
    receipt = []
    if len(commit) == 1 and isinstance(commit[0].get('goal_key'), str) and commit[0]['goal_key']:
        goal_hash = hashlib.sha256(commit[0]['goal_key'].encode()).hexdigest()
        receipt = [r for r in observer or [] if r.get('event') == 'reference_observer_receipt' and
                   r.get('valid_framing') is True and r.get('goal_sha256') == goal_hash and
                   tuple(r.get('message_hashes') or []) == key[2] and
                   isinstance(r.get('transaction_sha256'), str) and SHA256.fullmatch(r['transaction_sha256'])]
    transactions = {r['transaction_sha256'] for r in receipt}
    exact = (len(refs) == 1 and len(commit) == 1 and len(transactions) == 1) if key is not None and consumed and commits and observer else None
    return dict(identity_complete=key is not None, consumed_matches=len(refs) if consumed is not None else None,
                commit_matches=len(commit) if commits is not None else None,
                transaction_receipts=len(receipt) if observer is not None else None,
                transaction_sha256=next(iter(transactions)) if len(transactions) == 1 else None,
                transaction_fnv64=receipt[0].get('transaction_fnv64') if len(transactions) == 1 and receipt and
                    len({r.get('transaction_fnv64') for r in receipt}) == 1 else None,
                complete_exact_reference_chain=exact,
                consumed_ros_time=refs[0]['t'] + 1000. if len(refs) == 1 and finite(refs[0].get('t')) else None)


def conjunction(values):
    values = list(values)
    if not values:
        return None
    if any(v is False for v in values):
        return False
    return True if all(v is True for v in values) else None


def difference(later, earlier):
    return later-earlier if finite(later) and finite(earlier) else None


def source_stamp(identity):
    return identity['stamp_sec']+identity['stamp_nsec']*1e-9 if identity_key(identity) is not None else None


def publish_bounds(row):
    before=row.get('command_publish_before_ros_time');after=row.get('command_publish_after_ros_time')
    return [before,after] if finite(before) and finite(after) and before<=after else None


def analyze_cycles(cycles, odom, consumed, commits, observer):
    index = observer_index(odom); ids = Counter(r.get('cycle_id') for r in cycles or [])
    result = []; seen_kernel = set(); seen_validated = set()
    for row in cycles or []:
        cycle_id = row.get('cycle_id')
        solve = match_observer(row.get('solve_odom_identity'), odom, index)
        validate = match_observer(row.get('validation_odom_identity'), odom, index)
        solve['state_equals_identity'] = state_matches_identity(row.get('solve_state'), row.get('solve_odom_identity'))
        validate['state_equals_identity'] = state_matches_identity(row.get('validation_state'), row.get('validation_odom_identity'))
        ref = reference_link(row, consumed, commits, observer)
        invoked = row.get('online_step_invoked'); kernel_code = row.get('kernel_code')
        kernel_success = invoked is True and row.get('online_step_succeeded') is True and kernel_code == 0 if isinstance(invoked, bool) and integer(kernel_code) else None
        validated = conjunction((row.get('validation_invoked'), row.get('validated_command_published'),
                                 row.get('reason') == 'ok' if isinstance(row.get('reason'), str) else None,
                                 row.get('validation_code') == 0 if integer(row.get('validation_code')) else None))
        generation = row.get('goal_generation'); key = reference_key(row)
        first_key = (generation, key[:3]) if integer(generation) and key is not None else None
        expected_kernel = invoked is True and first_key not in seen_kernel if first_key is not None and isinstance(invoked, bool) else None
        expected_validated = validated is True and first_key not in seen_validated if first_key is not None and isinstance(validated, bool) else None
        if first_key is not None and invoked is True:seen_kernel.add(first_key)
        if first_key is not None and validated is True:seen_validated.add(first_key)
        flags = conjunction((row.get('first_kernel_cycle_for_reference') == expected_kernel if isinstance(row.get('first_kernel_cycle_for_reference'), bool) and expected_kernel is not None else None,
                             row.get('first_validated_command_cycle_for_reference') == expected_validated if isinstance(row.get('first_validated_command_cycle_for_reference'), bool) and expected_validated is not None else None))
        unique = integer(cycle_id) and cycle_id > 0 and ids[cycle_id] == 1
        complete = conjunction((unique, solve['exact_message_observed'], solve['state_equals_identity'],
                                validate['exact_message_observed'], validate['state_equals_identity'],
                                ref['complete_exact_reference_chain'], kernel_success, validated, flags))
        result.append(dict(cycle_id=cycle_id, unique_cycle_id=unique, solve=solve, validation=validate,
            reference=ref, online_step_invoked=invoked, actual_kernel_success=kernel_success,
            actual_validated_command=validated, first_flags_consistent=flags, complete_exact_python_chain=complete,
            goal_generation=generation, guide_epoch=row.get('guide_epoch'), guide_revision=row.get('guide_revision'),
            message_hashes=row.get('message_hashes'), reference_array_sha256=row.get('reference_array_sha256'),
            solve_ros_time=row.get('solve_ros_time'), validation_ros_time=row.get('validation_ros_time'),
            command_ros_time=row.get('command_ros_time'), command_v=row.get('command_v'), command_w=row.get('command_w'),
            checkpoint_semantics=row.get('checkpoint_semantics'),
            online_step_entry_ros_time=row.get('online_step_entry_ros_time'),
            online_step_entry_monotonic=row.get('online_step_entry_monotonic'),
            command_publish_ros_bounds=publish_bounds(row),
            entry_odom_age_s=difference(row.get('online_step_entry_ros_time'),source_stamp(row.get('solve_odom_identity')))
                if solve['exact_message_observed'] is True else None,
            reason=row.get('reason'), kernel_code=kernel_code, validation_code=row.get('validation_code')))
    return dict(present=cycles is not None, observed_cycles=len(cycles) if cycles is not None else None,
                all_cycles_exact=conjunction(r['complete_exact_python_chain'] for r in result),
                exact_validated_cycles=sum(r['complete_exact_python_chain'] is True for r in result) if cycles else None,
                actual_kernel_successes=sum(r['actual_kernel_success'] is True for r in result) if cycles else None,
                actual_validated_commands=sum(r['actual_validated_command'] is True for r in result) if cycles else None,
                reasons=dict(Counter(r.get('reason') for r in cycles or [])) if cycles is not None else None,
                rows=result)


def event_stage(row):
    event = row.get('event')
    return event[len('state_contract_'):] if isinstance(event, str) and event.startswith('state_contract_') else None


def snapshot_evidence(trial, row, selected=None):
    """Inspect only the explicitly recorded frozen backend file, never nearest."""
    frame = row.get('backend_frame')
    result = dict(frame=frame, present=None, sha256=None, schema=None,payload_hash_verified=None,
                  recorded_raw_static=row.get('raw_static'), recorded_fixed_static=row.get('fixed_static'),
                  independent_raw_static=None,independent_selected_static=None,independent_fixed_static=None,
                  raw_online_agreement=None,fixed_online_agreement=None)
    reference=frame.get('reference',frame) if isinstance(frame,dict) else None
    relative = reference.get('snapshot') if isinstance(reference,dict) else None
    if not isinstance(relative, str) or not relative:
        return result
    base = (Path(trial) / 'delivery_diagnostics').resolve(); path = (base / relative).resolve()
    if base not in path.parents:
        result.update(present=False, error='snapshot_path_outside_diagnostic_directory');return result
    if not path.is_file():
        result.update(present=False, error='recorded_snapshot_missing');return result
    payload = read_json(path);expected=reference.get('snapshot_payload_hash');algorithm=reference.get('hash_algorithm')
    payload_hash=format(int(fnv1a64(path.read_bytes())),'016x')
    verified=payload_hash==expected if algorithm=='fnv1a64_json' and isinstance(expected,str) and re.fullmatch('[0-9a-f]{16}',expected) else None
    result.update(present=True, sha256=digest(path), schema=payload.get('schema'),
                  snapshot_payload_hash=expected,hash_algorithm=algorithm,payload_hash_verified=verified)
    if payload.get('schema')!='delivery_snapshot_v1' or verified is not True:return result
    # Independent endpoint-only arithmetic: no trajectory/optimizer replay.
    from recheck_delivery_diag import evaluate_pose, snapshot_arrays
    try:
        snapshot=snapshot_arrays(payload)
        result['recorded_margin_m']=payload.get('margin')
        result['margin_is_0_10m']=abs(payload['margin']-.1)<=1e-12 if finite(payload.get('margin')) else None
        poses={'raw': [row.get('raw_odom_x'),row.get('raw_odom_y'),row.get('raw_odom_yaw')],
               'selected': (selected.get('planner_p') or [])[:2]+[selected.get('planner_yaw')] if selected else None,
               'fixed': (row.get('planner_p') or [])[:2]+[row.get('fixed_yaw')]}
        for name,pose in poses.items():
            if isinstance(pose,list) and len(pose)==3 and all(finite(v) for v in pose):
                result['independent_'+name+'_static']=evaluate_pose(snapshot,*pose,0.,dynamic=False)
        for name in ('raw','fixed'):
            online=row.get(name+'_static');offline=result.get('independent_'+name+'_static')
            if isinstance(online,dict) and isinstance(offline,dict) and online.get('valid') is True and all(finite(v) for v in (online.get('physical'),online.get('gap'),offline.get('physical'),offline.get('gap'))):
                result[name+'_online_agreement']=abs(online['physical']-offline['physical'])<=1e-8 and abs(online['gap']-offline['gap'])<=1e-8
    except (ValueError,TypeError,KeyError,OverflowError) as exc:
        result['independent_recheck_error']=type(exc).__name__+': '+str(exc)
    return result


def boundary_velocity(row, yaw):
    velocity=row.get('planner_v') if isinstance(row,dict) else None
    return velocity[0]*math.cos(yaw)+velocity[1]*math.sin(yaw) if isinstance(velocity,list) and len(velocity)>=2 and all(finite(v) for v in velocity[:2]) and finite(yaw) else None


def matrix_velocity(matrix, yaw):
    if not isinstance(matrix,list) or len(matrix)<2 or not all(isinstance(r,list) and len(r)>=2 for r in matrix[:2]):return None
    return boundary_velocity({'planner_v':[matrix[0][1],matrix[1][1]]},yaw)


def one(by_stage,stage):
    rows=by_stage[stage]
    return rows[0] if len(rows)==1 else {}


def request_metrics(by_stage, matches):
    selected=one(by_stage,'selected_boundary');desired=one(by_stage,'desired_boundary')
    root=one(by_stage,'native_astar_root');backend=one(by_stage,'backend_fixed')
    publication=one(by_stage,'reference_publish');epoch=one(by_stage,'epoch_chosen')
    heads=[r for r in by_stage['native_minco_head'] if r.get('block_index')==0]
    head=heads[0] if len(heads)==1 else {};yaw=selected.get('raw_odom_yaw')
    initial=backend.get('initial_states');first=initial[0] if isinstance(initial,list) and initial else None
    invoked=sorted([r for r in matches if r.get('online_step_invoked') is True and r.get('unique_cycle_id') is True],key=lambda r:r['cycle_id'])
    first_cycle=invoked[0] if invoked else None
    pubtime=publication.get('capture_sim');start=publication.get('actual_start_epoch')
    marker_to_entry=difference(first_cycle.get('online_step_entry_ros_time'),pubtime) if first_cycle else None
    bounds=first_cycle.get('command_publish_ros_bounds') if first_cycle else None
    return dict(selected_raw_odom_age_s=difference(selected.get('capture_sim'),source_stamp(cpp_identity(selected))),
        first_kernel_cycle_id=first_cycle.get('cycle_id') if first_cycle else None,
        first_kernel_entry_odom_age_s=first_cycle.get('entry_odom_age_s') if first_cycle else None,
        native_publish_marker_ros_time=pubtime,publish_lateness_s=difference(pubtime,start),
        chosen_minus_requested_epoch_s=difference(epoch.get('chosen_start_epoch'),epoch.get('requested_start_epoch')),
        publish_marker_to_kernel_entry_s=marker_to_entry,
        publish_marker_to_command_call_bounds_s=[difference(v,pubtime) for v in bounds] if bounds and finite(pubtime) else None,
        clock_semantics='native capture marker is post-publish bookkeeping; Python kernel entry and command call bounds are local invocation checkpoints, not network arrival or physical execution',
        signed_speed=dict(reference_heading='selected raw odometry yaw',reference_yaw=yaw,
            raw_body_vx=selected.get('raw_odom_body_vx'),
            processed=boundary_velocity({'planner_v':[selected.get('raw_odom_processed_vx'),selected.get('raw_odom_processed_vy')]},yaw),
            selected=boundary_velocity(selected,yaw),desired=boundary_velocity(desired,yaw),
            native_root=root.get('native_signed_speed'),minco_head=matrix_velocity(head.get('head_state'),yaw),
            backend_fixed=matrix_velocity(first,yaw),candidate=boundary_velocity(one(by_stage,'candidate_ready'),yaw),
            published=boundary_velocity(publication,yaw)),
        gears=dict(processed=selected.get('raw_odom_processed_gear'),selected=selected.get('planner_gear'),
            desired=desired.get('planner_gear'),native_root=root.get('native_root_gear'),minco_head=head.get('native_first_gear'),
            backend_fixed=backend.get('fixed_gear'),candidate=one(by_stage,'candidate_ready').get('planner_gear'),published=publication.get('planner_gear')))


def unsigned_identity(value):
    if integer(value) and 0<=value<=0xffffffffffffffff:return str(value)
    if isinstance(value,str) and value.isascii() and value.isdecimal() and str(int(value))==value and int(value)<=0xffffffffffffffff:return value
    return None


def backend_scope_evidence(rows, backend):
    backend_id=backend.get('state_contract_backend_id');input_hash=unsigned_identity(backend.get('input_hash'))
    if backend.get('state_contract_backend_id_valid') is not True or not integer(backend_id) or backend_id<1 or input_hash is None:
        return dict(complete=None,scope_id=None,reason='missing_backend_identity')
    links=[r for r in rows if event_stage(r)=='backend_scope_link' and r.get('state_contract_backend_id')==backend_id]
    if len(links)!=1:return dict(complete=False,scope_id=None,reason='missing_or_duplicate_backend_scope_link')
    link=links[0];scope=link.get('scope_id')
    coupling=[r for r in rows if r.get('event')=='coupling_scope' and r.get('scope_id')==scope]
    complete=(integer(scope) and scope>0 and unsigned_identity(link.get('input_hash'))==input_hash and
              len(coupling)==1 and unsigned_identity(coupling[0].get('input_hash'))==input_hash)
    return dict(complete=complete,scope_id=scope,reason='exact_backend_id+scope_id+input_hash' if complete else 'scope_or_hash_disagreement')


def analyze_native(native, odom, python, trial):
    groups = defaultdict(list); errors = []
    for row in native or []:
        stage = event_stage(row)
        if stage == 'trace_error':errors.append(row)
        if stage is None and row.get('event') not in ('stage_begin','reference_publish','coupling_scope'):continue
        pid, request = row.get('pid'), row.get('request_id')
        if integer(pid) and pid > 1 and integer(request) and request > 0:
            groups[(pid, request)].append(row)
    index = observer_index(odom); requests = []
    publication_counts=Counter(r.get('transaction_fnv64') for r in native or [] if event_stage(r)=='reference_publish' and isinstance(r.get('transaction_fnv64'),str))
    for (pid, request), rows in sorted(groups.items()):
        by_stage = {stage:[r for r in rows if event_stage(r) == stage] for stage in CPP_STAGES}
        selected = by_stage['selected_boundary']; identity = cpp_identity(selected[0]) if len(selected) == 1 else None
        observed = match_observer(identity, odom, index)
        selected_key = identity_key(identity)
        raw_rows=[r for r in rows if 'raw_odom_message_fnv64' in r or event_stage(r) in RAW_STAGES]
        identity_consistency = conjunction(identity_key(cpp_identity(r)) == selected_key if selected_key is not None and identity_key(cpp_identity(r)) is not None else None for r in raw_rows)
        publications = by_stage['reference_publish']; matches = []
        if len(publications) == 1:
            tx = publications[0].get('transaction_fnv64')
            if isinstance(tx, str) and tx and publication_counts[tx]==1:
                matches = [r for r in python['rows'] if r['reference'].get('transaction_fnv64') == tx and
                           r['reference'].get('complete_exact_reference_chain') is True]
        captures = [r.get('capture_mono') for r in rows if event_stage(r) in CPP_STAGES]
        clocks = all(finite(v) for v in captures) and len(captures) > 0 and all(finite(r.get('capture_sim')) for r in rows if event_stage(r) in CPP_STAGES)
        stages = {stage:dict(count=len(items), record=items[0] if len(items) == 1 else None)
                  for stage, items in by_stage.items()}
        backend = by_stage['backend_fixed'];snapshot=snapshot_evidence(trial,backend[0],selected[0] if len(selected)==1 else None) if len(backend)==1 else None
        critical=all(len(by_stage[stage])==1 for stage in CRITICAL_STAGES)
        searches=by_stage['native_astar_input']
        initflags=[r.get('poly_init') for r in rows if r.get('event')=='stage_begin' and r.get('stage')=='initialization']
        frontend_branch='native_astar' if searches else ('warm' if initflags==[False] else None)
        frontend_complete=(len(searches)==1 and len(by_stage['native_astar_root'])==1 and len(by_stage['native_astar_result'])==1 and
            len([r for r in by_stage['native_minco_head'] if r.get('block_index')==0])==1) if frontend_branch=='native_astar' else (True if frontend_branch=='warm' else None)
        raw_complete=all(len(by_stage[stage])==1 and identity_key(cpp_identity(by_stage[stage][0])) is not None for stage in RAW_STAGES)
        success=conjunction(one(by_stage,stage).get('success') for stage in ('solver_return','request_end'))
        scope=backend_scope_evidence(rows,backend[0]) if len(backend)==1 else dict(complete=None,scope_id=None,reason='missing_unique_backend')
        requests.append(dict(pid=pid, request_id=request, raw_odom_identity=identity, raw_observer_match=observed,
            boundary_raw_identity_consistent=identity_consistency, capture_clocks_present=clocks,
            critical_stages_complete=critical,raw_bearing_stages_complete=raw_complete,
            frontend_branch=frontend_branch,frontend_chain_complete=frontend_complete,
            stages=stages,backend_snapshot=snapshot,backend_scope=scope,metrics=request_metrics(by_stage,matches),
            exact_reference_linked_cycles=[r['cycle_id'] for r in matches] if len(publications) == 1 else None,
            exact_validated_command_cycles=[r['cycle_id'] for r in matches if r['complete_exact_python_chain'] is True] if len(publications) == 1 else None,
            complete_native_to_validated_command=conjunction((critical,raw_complete,frontend_complete,success,
                observed['exact_message_observed'],identity_consistency, clocks,
                snapshot.get('payload_hash_verified') if isinstance(snapshot,dict) else None,
                snapshot.get('schema')=='delivery_snapshot_v1' if isinstance(snapshot,dict) and snapshot.get('schema') is not None else None,
                scope['complete'],
                len(publications) == 1, bool(matches),
                any(r['complete_exact_python_chain'] is True for r in matches))) if rows else None))
    return dict(present=native is not None, observed_requests=len(requests) if native is not None else None,
                complete_native_to_validated_commands=sum(r['complete_native_to_validated_command'] is True for r in requests) if requests else None,
                trace_errors=errors if native is not None else None, requests=requests)


def summarize_trial(trial):
    trial = Path(trial)
    documents = {name:read_json(trial / name) for name in ('manifest.json','run_status.json','trial_audit.json','result.json')}
    manifest=documents['manifest.json'];status=documents['run_status.json'];settings=manifest.get('coupling_experiment') or {}
    if status.get('returncode') is None or status.get('result_written') is not True:
        raise ValueError('Trial is not explicitly finished: ' + str(trial))
    if not isinstance(settings.get('signed_low_speed'), bool) or settings.get('state_contract_capture') is not True:
        raise ValueError('Missing explicit signed OFF/ON and state capture: ' + str(trial))
    inputs={}; hashes={name:digest(trial / name) for name in documents}
    for name in ('state_odom_events.jsonl','state_mppi_cycles.jsonl','consumed_references.jsonl',
                 'reference_events.jsonl','observer_events.jsonl','dual_timeline.jsonl'):
        rows,path=read_records(trial,name)
        inputs[name]=rows
        if path is not None:hashes[path.name]=digest(path)
    python=analyze_cycles(inputs['state_mppi_cycles.jsonl'],inputs['state_odom_events.jsonl'],
        inputs['consumed_references.jsonl'],inputs['reference_events.jsonl'],inputs['observer_events.jsonl'])
    native=analyze_native(inputs['dual_timeline.jsonl'],inputs['state_odom_events.jsonl'],python,trial)
    return dict(trial=trial.name,source=str(trial),signed_low_speed=settings['signed_low_speed'],
        scene=manifest.get('scene'),seed=settings.get('seed'),build=manifest.get('build'),
        build_manifest_sha256=manifest.get('build_manifest_sha256'),runtime_sha256=manifest.get('experiment_sources_sha256'),
        coupling_experiment=settings,run_status=status,audit=documents['trial_audit.json'],outcome=documents['result.json'],
        input_sha256=hashes,python_chain=python,native_chain=native,
        observer_odom_records=len(inputs['state_odom_events.jsonl']) if inputs['state_odom_events.jsonl'] is not None else None)


def build_report(root):
    root=Path(root).resolve();complete=read_json(root/'complete.json');plan=read_json(root/'plan.json')
    jobs=complete.get('completed')
    if complete.get('complete') is not True or not isinstance(jobs,list) or len(jobs)!=2:
        raise ValueError('Exactly two completed trials required; never report a live/incomplete batch')
    names=[job.get('trial') for job in jobs]
    if any(not isinstance(name,str) or Path(name).name!=name or name in ('.','..') for name in names) or len(set(names))!=2:
        raise ValueError('Invalid or duplicate completed trial names')
    trials=[summarize_trial(root/name) for name in names]
    if {trial['signed_low_speed'] for trial in trials}!={False,True}:
        raise ValueError('One explicit signed OFF and one ON trial required')
    trials.sort(key=lambda trial:trial['signed_low_speed']);pair={}
    for key in ('scene','seed','build','build_manifest_sha256','runtime_sha256'):
        values=[trial.get(key) for trial in trials]
        pair[key]=values[0]==values[1] if all(value is not None and value!={} and value!='' for value in values) else None
    settings=[{k:v for k,v in trial['coupling_experiment'].items() if k!='signed_low_speed'} for trial in trials]
    pair['other_recorded_settings_equal']=settings[0]==settings[1] if all(settings) else None
    return record_with_validity(dict(schema='state_contract_report_v1',created=datetime.now(timezone.utc).isoformat(),
        root=str(root),diagnostic_only=True,performance_comparison=False,paired_same_input=False,
        plan=plan,complete=complete,pair_consistency=pair,limitations=LIMITATIONS,trials=trials,
        root_input_sha256={name:digest(root/name) for name in ('plan.json','complete.json')},
        conclusion='逐组报告精确状态/引用链；OFF/ON请求不视为同一输入，不能据此宣称速度因果改善。'))


def show(value):return '未知' if value is None else str(value)


def report_text(report):
    lines=['状态合同：单seed静态诊断（非速度benchmark）',report['conclusion']]
    for trial in report['trials']:
        python=trial['python_chain'];native=trial['native_chain']
        lines.extend(['','%s: %s'%('ON' if trial['signed_low_speed'] else 'OFF',trial['trial']),
            '结果=%s；保守碰撞=%s；审计有效=%s'%(show(trial['outcome'].get('reason')),show(trial['outcome'].get('collision_conservative_projection')),show(trial['audit'].get('valid_experiment'))),
            'odom记录=%s；MPPI周期=%s；成功kernel=%s；实际验证命令=%s；精确完整周期链=%s'%tuple(show(v) for v in (trial['observer_odom_records'],python['observed_cycles'],python['actual_kernel_successes'],python['actual_validated_commands'],python['exact_validated_cycles'])),
            'native请求=%s；精确连至验证命令=%s'%(show(native['observed_requests']),show(native['complete_native_to_validated_commands']))])
    lines.extend(['','配对条件='+json.dumps(report['pair_consistency'],ensure_ascii=False),'','解释边界：']+LIMITATIONS)
    return '\n'.join(lines)+'\n'


def plot_report(report, path):
    # The guard is inside this entry point too; direct callers cannot bypass it.
    assert_plot_quiet(report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(12.,8.),squeeze=False)
    for column,trial in enumerate(report['trials']):
        requests=trial['native_chain']['requests']
        metrics=[row['metrics'] for row in requests]
        series=[(('selected_raw_odom_age_s','selected snapshot'),('first_kernel_entry_odom_age_s','first actual kernel entry')),
                tuple((name,name) for name in ('raw_body_vx','processed','selected','desired','native_root','minco_head','backend_fixed','published')),
                (('publish_lateness_s','publish marker - actual epoch'),('chosen_minus_requested_epoch_s','chosen - requested epoch'))]
        for number,entries in enumerate(series):
            ax=axes[number,column];plotted=False
            for key,label in entries:
                values=[metric['signed_speed'].get(key) if number==1 else metric.get(key) for metric in metrics]
                points=[(index,value) for index,value in enumerate(values) if finite(value)]
                if points:
                    ax.scatter([p[0] for p in points],[p[1] for p in points],s=19,label=label);plotted=True
            if not plotted:ax.text(.5,.5,'unknown / no exact evidence',ha='center',transform=ax.transAxes)
            else:ax.legend(fontsize=6,loc='best',ncol=2 if number==1 else 1)
            ax.axhline(0.,color='.7',linewidth=.5);ax.grid(alpha=.2)
            ax.set_ylabel(('Odom age (s)','Signed speed along raw heading (m/s)','Epoch difference (s)')[number])
            ax.set_xticks(range(len(requests)));ax.set_xticklabels([str(r['request_id']) for r in requests],fontsize=7)
            ax.set_xlabel('Recorded request ID (groups are not paired)')
        axes[0,column].set_title(('ON' if trial['signed_low_speed'] else 'OFF')+' — '+trial['trial'],fontsize=9)
    fig.suptitle('Exact state / native root / MINCO / epoch checkpoints — not speed or safety evidence',fontsize=11)
    fig.tight_layout(rect=(0,0,1,.96));fig.savefig(path,dpi=130);plt.close(fig)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--plot',action='store_true',help='One PNG, only after recorded ROS PIDs exit and port closes')
    args=parser.parse_args(argv);root=args.root.resolve();out=args.out.resolve()
    if out.exists() or out==root or out in root.parents:
        parser.error('--out must be a fresh directory, never an input directory or its ancestor')
    report=build_report(root)
    if args.plot:assert_plot_quiet(report)
    out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    (out/'实验结论.txt').write_text(report_text(report),encoding='utf-8')
    if args.plot:plot_report(report,out/'state_chain_checkpoints.png')
    print(json.dumps(dict(out=str(out),conclusion=report['conclusion']),ensure_ascii=False));return 0


if __name__=='__main__':raise SystemExit(main())
