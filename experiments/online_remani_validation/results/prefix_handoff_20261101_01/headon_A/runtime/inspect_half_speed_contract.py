#!/usr/bin/env python3
"""Read historical half-speed rejections and the actual fast-layer contract.

No ROS, optimization, simulator or control publication. A failed candidate was
not executed: nearby MPPI motion belongs to the consumed reference, not to that
candidate. Frozen-CV dynamic checks do not replace the full native gate.
"""
import argparse
import ast
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from progress import project
from render_coupled_objective import SourceReader, sha
from replay_temporal_collision import Candidate
from report_candidate_collision_repair import dynamic_check


HERE = Path(__file__).resolve().parent
RESULTS = HERE / 'results'
DEFAULT = RESULTS / 'half_speed_contract_20261101_01' / 'replay'
SOURCES = ('window_continuity_20261101_01', 'early_collision_repair_20261101_01')
ORIGIN = 1000.


def stats(values):
    a = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if not len(a):
        return dict(count=0, minimum=None, p10=None, median=None, p90=None, maximum=None)
    q = np.quantile(a, (0., .1, .5, .9, 1.))
    return dict(count=len(a), **dict(zip(('minimum', 'p10', 'median', 'p90', 'maximum'), map(float, q))))


def phase_evidence(state, refs):
    """Actual published feedback plus independent geometric association.

    state.virtual_time/time_scale record MPPI's progress topic. The independent
    projection is only association with the currently consumed guide; its
    derivative during a detour is not the trajectory's actual execution speed.
    """
    result = []
    for index, ref in enumerate(refs):
        end = refs[index+1]['t'] if index+1 < len(refs) else float(state['t'][-1])+1e-6
        part = state[(state['t'] >= ref['t']) & (state['t'] < end)]
        reference, times = np.asarray(ref['reference']), np.asarray(ref['times'])
        previous, projections = 0., []
        for row in part:
            previous, scale, residual = project(
                np.array([row['x'], row['y'], row['yaw'], row['v'], row['w']]),
                reference, times, previous)
            projections.append(dict(t=float(row['t']), phase_local_s=previous,
                velocity_tangent_scale=scale, association_residual_m=residual))
        rates = [(b['phase_local_s']-a['phase_local_s'])/(b['t']-a['t'])
                 for a, b in zip(projections, projections[1:]) if b['t']-a['t'] > 1e-8]
        stamped = part[np.abs(part['trajectory_start']-ref['stamp']) < 1e-6]
        result.append(dict(revision=ref['revision'], reference_sha256=ref['sha256'],
            consumed_t_s=ref['t'], guide_epoch_s=ref['stamp'], interval_end_s=end,
            same_guide_feedback_rows=len(stamped),
            recorded_feedback_scale=stats(stamped['time_scale']),
            recorded_feedback_scale_equal_half_count=int(np.sum(np.abs(stamped['time_scale']-.5) < 1e-6)),
            recorded_feedback_scale_below_half_count=int(np.sum(stamped['time_scale'] < .5)),
            geometric_association_rate=stats(rates),
            geometric_association_residual_m=stats([p['association_residual_m'] for p in projections]),
            projection_rows=projections,
            scope='Only the consumed reference is associated. Feedback scale is clipped tangential velocity/reference speed, not a committed future time scaling. Geometric phase can advance during a detour without executing the polynomial.'))
    return result


def nearby_execution(state, commands, executions, manifest, result, body, scenario, truth_gap, request, end):
    begin = request['sim']-ORIGIN
    if end <= begin:
        end = begin+3.
    selected = [c for c in commands if begin <= c['command_ros_time']-ORIGIN <= end]
    valid = [c for c in selected if c.get('validated_command_published')]
    rollouts = [r for r in executions if begin <= r['t'] <= end and r.get('rollout')
                and r.get('reason') == 'ok']
    rows = state[(state['t'] >= begin) & (state['t'] <= end)]
    anchor = result.get('motion_anchor') if manifest.get('scenario_start') == 'motion' else result.get('goal_sent')
    gaps = []
    if anchor is not None:
        for row in rows:
            elapsed = float(row['t'])-float(anchor)-scenario['appearance_s']
            actors = [(a['x']+elapsed*a['vx'], a['y']+elapsed*a['vy'], a['r'],
                       a['id'], a['vx'], a['vy']) for a in scenario['actors']] if elapsed >= 0 else []
            gap = truth_gap(np.array([row['x'], row['y'], row['yaw']]),
                np.asarray(body['body']), float(body['header'][11]), actors)
            gaps.append(dict(t=float(row['t']), physical_sample_gap_m=gap))
    lease = [r['lease_s'] for r in rollouts if r.get('lease_s') is not None]
    return dict(interval_elapsed_s=[begin, end],
        currently_consumed_revisions=sorted(set(c['guide_revision'] for c in valid)),
        cycle_count=len(selected), validated_command_count=len(valid),
        reason_counts=dict(Counter(c['reason'] for c in selected)),
        command_linear_mps=stats([c['command_v'] for c in valid]),
        command_angular_rps=stats([c['command_w'] for c in valid]),
        recorded_feedback_scale=stats(rows['time_scale']),
        recorded_feedback_scale_equal_half_count=int(np.sum(np.abs(rows['time_scale']-.5) < 1e-6)),
        approved_rollout_count=len(rollouts),
        approved_selected_rollout_native_model_min_gap_m=stats([r['validation'][1] for r in rollouts]),
        approved_brake_rollout_native_model_min_gap_m=stats([r['validation'][2] for r in rollouts]),
        approved_rollout_horizon_s=stats([r['rollout'][-1][0] for r in rollouts]),
        command_lease_s=stats(lease),
        independent_physical_sample_minimum=min(gaps, key=lambda g:g['physical_sample_gap_m']) if gaps else None,
        scope='Actual previously consumed guide and repeatedly validated MPPI rollouts nearby in time. A failed candidate was never executed, so this does not certify that candidate. Native model gaps and physical sphere-to-synthetic-truth gaps have different boundaries.')


def inspect(source):
    reader = SourceReader(source)
    captures = reader.jsonl('temporal_collision.jsonl')
    timeline = reader.jsonl('dual_timeline.jsonl')
    state = reader.csv('state.csv', required=True)
    commands = reader.jsonl('state_mppi_cycles.jsonl')
    executions = reader.jsonl('execution_rollouts.jsonl')
    refs = sorted(reader.jsonl('consumed_references.jsonl'), key=lambda r:r['t'])
    manifest = reader.json('manifest.json', required=True)
    result = reader.json('result.json', required=True)
    body = reader.json('native_collision_3d_body.json', required=True)
    scenario = reader.json('scenario.json', required=True)
    # Reuse the recorded trial's physical-gap function: this scene extends the
    # x wall to 7.5 m. Importing today's default helper would use a 4.5 m wall.
    truth_source = reader.locate('runtime/native_collision_runtime.py', required=True)
    nodes = ast.parse(truth_source.read_text()).body
    function = next(n for n in nodes if isinstance(n, ast.FunctionDef)
                    and n.name == 'sampled_truth_clearance')
    namespace = {'np': np}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(truth_source), 'exec'), namespace)
    truth_gap = namespace['sampled_truth_clearance']
    plans = {r['plan_ordinal']:r for r in captures if r['stage'] == 'planning'}
    requests = {r['request_id']:r for r in timeline if r['event'] == 'request_begin'}
    failures = []
    for capture in captures:
        if capture['stage'] != 'final_candidate' or capture.get('native_success') is not False:
            continue
        plan = plans[capture['plan_ordinal']]
        candidate = Candidate(capture['candidate'], plan)
        checks = [dynamic_check(candidate, plan, scale) for scale in (1., .5)]
        epoch = plan['requested_epoch']
        consumed = [r for r in refs if abs(r['stamp']-epoch) < 1e-6]
        publications = [r for r in timeline if r['event'] == 'reference_publish'
                        and r['request_id'] == capture['request_id']]
        next_ref = next((r for r in refs if r['t'] >= capture['capture_sim']-ORIGIN), None)
        end = next_ref['t'] if next_ref else min(float(state['t'][-1]), epoch-ORIGIN+3.)
        failures.append(dict(request_id=capture['request_id'], plan_ordinal=capture['plan_ordinal'],
            capture_t_s=capture['capture_sim']-ORIGIN, requested_epoch_s=epoch,
            candidate_sha256=candidate.geometry_hash(), candidate_duration_s=candidate.duration,
            frozen_cv_source_sha256=checks[0].get('source_sha256'),
            recorded_native_success=capture['native_success'], recorded_native_reason=capture['native_reason'],
            full_speed_and_half_speed_dynamic_checks=checks,
            nominal_dynamic_pass=(checks[0]['available'] and checks[0]['first_collision'] is None
                                  and checks[0]['valid_sample_count'] > 0),
            half_dynamic_pass=(checks[1]['available'] and checks[1]['first_collision'] is None
                               and checks[1]['valid_sample_count'] > 0),
            same_request_reference_publish_count=len(publications),
            same_request_consumed_reference_count=len(consumed),
            failed_candidate_was_executed=False if not publications and not consumed else None,
            full_only_native_gate_pass=None,
            native_gate_scope='Python checks only the dynamic branch. Static/self/wheel/full native full-only verdict is not inferred.',
            nearby_actual_execution=nearby_execution(state,commands,executions,
                manifest,result,body,scenario,truth_gap,requests[capture['request_id']],end)))
    runtime = {}
    for name in ('local_execution.py', 'progress.py', 'simulator.py'):
        path = reader.locate('runtime/'+name, required=True)
        runtime[name] = dict(path=str(path), sha256=sha(path))
    reader.verify()
    return dict(source=str(Path(source).resolve()), failed_candidates=failures,
        consumed_reference_phase_evidence=phase_evidence(state,refs),
        actual_contract=dict(authority='Repeated native-3D selected/brake rollout validation before MPPI command publication',
            progress_source='runtime/local_execution.py progress.project; state.csv trajectory_start/virtual_time/time_scale records the MPPI progress topic',
            feedback_scale_definition='clip(measured velocity dot reference tangent / max(0.02, abs(reference speed)), 0, 1)',
            fixed_half_speed_execution_promise=False,
            future_rollout_is_commitment=False,
            native_tracker_progress_topic='/sim/native_controller_progress (remapped away from MPPI progress topic)'),
        recorded_result=dict(reason=result['reason'],native3d_truth_monitor=result.get('native3d_truth_monitor')),
        runtime_sources=runtime, source_sha256=reader.hashes)


def narrative(data):
    lines=['半速检查与实际执行：历史记录核对','',
        '要核实：REMANI候选为什么只在半速动态分支失败；MPPI实际上有没有固定按半速执行。',
        '本脚本只读历史记录，不重新规划、不启动ROS，也不修改原始实验。','']
    for row in data['trials']:
        lines.append(Path(row['source']).parent.name+' / '+Path(row['source']).name)
        for c in row['failed_candidates']:
            full,half=c['full_speed_and_half_speed_dynamic_checks']
            gap=lambda r:100*r['minimum']['minimum_model_gap_m'] if r.get('minimum') else float('nan')
            actual=c['nearby_actual_execution']
            scale=actual['recorded_feedback_scale']
            lines.append('请求%d：同一候选、同一冻结CV，常速最小检查间隙%+.3f厘米；半速%+.3f厘米。' % (c['request_id'],gap(full),gap(half)))
            lines.append('该候选发布/采用%d/%d次；不能拿附近机器人的实际绕行，证明这条未执行候选安全。' % (c['same_request_reference_publish_count'],c['same_request_consumed_reference_count']))
            lines.append('同期实际使用参考版本%s；有效MPPI控制%d/%d次；反馈进度比例最小/中位/最大=%.3f/%.3f/%.3f。' % (actual['currently_consumed_revisions'],actual['validated_command_count'],actual['cycle_count'],scale['minimum'],scale['median'],scale['maximum']))
            if actual['independent_physical_sample_minimum']:
                lines.append('同期实际运动最小物理球采样间隙%.3f厘米。此项描述实际旧参考下的运动，不给失败候选作安全背书。' % (100*actual['independent_physical_sample_minimum']['physical_sample_gap_m']))
        lines.append('')
    lines += ['可以确认：上述失败候选常速动态分支通过、半速动态分支碰撞；实际MPPI采用按当前状态反复验证的局部展开，反馈比例会变化，没有固定半速执行承诺。',
        '不能确认：仅凭这些记录不能认定删除半速检查一定安全，不能认定它是所有交付慢的唯一原因，也不能把常速动态分支通过当作完整原生验收通过。',
        '下一项有针对性的验证：正常不暂停闭环只切换是否要求半速动态分支，保留静态、自碰撞、轮约束、常速和快层执行验收；结束后用新候选的冻结CV复核旧半速门是否会挡住它。',
        'JSON还保留每条已采用参考的实际反馈、独立几何投影、输入哈希和局部展开验收数据。几何投影进度不等于机器人承诺按该速度执行轨迹。']
    return '\n'.join(lines)+'\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT)
    args=parser.parse_args()
    data=dict(schema='half_speed_contract_history_v1',
        script_sha256=sha(Path(__file__)),
        scope='Read-only fixed-candidate dynamic checks and actual consumed-reference evidence; no new experiment.',
        trials=[inspect(RESULTS/name/'headon_B') for name in SOURCES])
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'历史核对.json').write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (args.out/'历史核对.txt').write_text(narrative(data))
    print(narrative(data))


if __name__=='__main__':
    main()
