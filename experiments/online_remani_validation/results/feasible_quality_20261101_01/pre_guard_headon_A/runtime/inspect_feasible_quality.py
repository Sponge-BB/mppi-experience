#!/usr/bin/env python3
"""Post-run B q2 curve quality audit; no ROS, optimizer or map substitution."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from audit_backend_kinematics import CHANNELS, native_times, scan_wheels
from backend_base_feasibility_breakdown import (
    CHANNELS as COST_CHANNELS, piece_nodes, point_costs, source_contract)


HERE = Path(__file__).resolve().parent
TRIAL = HERE/'results/prefix_handoff_20261101_01/headon_B'
OUT = HERE/'results/feasible_quality_20261101_01/replay'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def snap_cost(b, T):
    """Native getTrajJerkCost: its actual quantity is integral squared snap.

    Same ten polynomial terms, all recorded 8 coordinates; no refitting.
    """
    c4, c5, c6, c7 = b[4:8]
    return float(576.*(c4@c4)*T + 2880.*(c4@c5)*T**2 +
        4800.*(c5@c5)*T**3 + 5760.*(c4@c6)*T**3 +
        21600.*(c5@c6)*T**4 + 10080.*(c4@c7)*T**4 +
        25920.*(c6@c6)*T**5 + 40320.*(c5@c7)*T**5 +
        100800.*(c6@c7)*T**6 + 100800.*(c7@c7)*T**7)


def parameters(trial, manifest):
    loaded = yaml.safe_load((trial/'parameters.yaml').read_text())
    baseline = json.loads((trial/'effective_real_params.json').read_text())
    overrides = manifest.get('parameter_overrides', {})
    prefix = '/remani_planner_node/'
    names = ('mm/mobile_base_wheel_base', 'mm/mobile_base_wheel_radius',
        'mm/mobile_base_max_wheel_omega', 'mm/mobile_base_max_wheel_alpha',
        'mm/mobile_base_non_singul_vel', 'optimization/weight_base_feasibility',
        'optimization/weight_time', 'optimization/constrain_points_perPiece',
        'optimization/dense_sample_resolution')
    for name in names:
        key = prefix+name
        expected = overrides.get(key, baseline.get(key))
        if loaded[key] != expected:
            raise ValueError('Loaded parameter differs from recorded baseline/override: '+key)
    v = lambda name: float(loaded[prefix+name])
    wheel = dict(wheel_base_m=v(names[0]), wheel_radius_m=v(names[1]),
        wheel_speed_limit_rad_s=v(names[2]), wheel_acceleration_limit_rad_s2=v(names[3]))
    numeric = np.array([v(names[5]), v(names[2])*v(names[1]), v(names[4]),
        v(names[0]), v(names[1]), v(names[2]), v(names[3])])
    return wheel, numeric, v(names[6]), int(v(names[7])), int(v(names[8])), {
        'loaded': {prefix+name: loaded[prefix+name] for name in names},
        'baseline': {prefix+name: baseline.get(prefix+name) for name in names},
        'manifest_overrides': {prefix+name: overrides[prefix+name] for name in names if prefix+name in overrides},
        'interpretation': 'parameters.yaml is the loaded trial configuration; effective_real_params is the original deployment baseline, with recorded trial overrides applied.'}


def analyze(record, wheel, numeric, time_weight, K, D):
    parts = record['parts']
    duration = sum(piece['T'] for part in parts for piece in part['pieces'])
    candidate = dict(parts=parts)
    cache = {}
    native = scan_wheels(candidate, list(native_times(duration)), wheel, cache)
    groups = {}
    for t, value in cache.items():
        groups.setdefault((value['part'], value['piece']), []).append(t)
    pieces, total = [], np.zeros(6)
    start = 0.
    for block, part in enumerate(parts):
        for index, piece in enumerate(part['pieces']):
            T = float(piece['T'])
            # Captured coefficients are dimension-by-descending-power.
            b = np.asarray(piece['coeffs'], dtype=float)[:, ::-1].T.copy()
            nodes = piece_nodes(b, T, K, D)
            costs = np.zeros(6)
            for kind in ('primary', 'dense'):
                v, a, j, weights = nodes[kind]
                if len(v):
                    costs += np.sum(weights[:, None]*point_costs(v,a,j,numeric,part['gear'],kind=='dense'),axis=0)
            total += costs
            times = np.linspace(0., T, max(2, int(np.ceil(T/.01))+1))
            xy = np.polynomial.polynomial.polyval(times, b[:, :2]).T
            length = float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
            motion = scan_wheels(candidate, groups.get((block,index), []), wheel, cache)
            samples = [cache[t] for t in groups.get((block,index), [])]
            slowest = min(samples, key=lambda s: np.linalg.norm(s['velocity'])) if samples else None
            minimum_speed = (dict(global_t_s=slowest['t'], local_t_s=slowest['local_t'],
                speed_mps=float(np.linalg.norm(slowest['velocity']))) if slowest else None)
            pieces.append(dict(block=block,piece_index_1based=index+1,
                global_begin_s=start,global_end_s=start+T,duration_s=T,
                length_m=length,average_geometric_speed_mps=length/T,
                start_xy=xy[0].tolist(),end_xy=xy[-1].tolist(),
                wheel_native_grid=motion,
                minimum_native_grid_speed=minimum_speed,
                native_smooth_snap_cost=snap_cost(b,T),native_time_cost=time_weight*T,
                native_base_feasibility_costs=dict(zip(COST_CHANNELS,map(float,costs))),
                native_base_feasibility_sum=float(costs.sum()),
                primary_samples=nodes['primary_count'],dense_samples=nodes['dense_node_count']))
            start += T
    return dict(stage=record['stage'],capture_sim=record['capture_sim'],
        duration_s=duration,length_m=sum(p['length_m'] for p in pieces),
        native_smooth_snap_cost=sum(p['native_smooth_snap_cost'] for p in pieces),
        native_time_cost=time_weight*duration,
        native_base_feasibility_costs=dict(zip(COST_CHANNELS,map(float,total))),
        native_base_feasibility_sum=float(total.sum()),native_grid= native,
        pieces=pieces,map_related_cost=None,map_related_gradient=None,
        full_objective=None,full_gradient=None,
        scope='Same recorded polynomial. Native 10ms hard-wheel grid and source-guarded primary+dense feasibility soft quadrature. Smooth/time are cost-only; no MINCO reverse pass or complete native map objective replay.')


def narrative(data):
    lines=['首条可行轨迹质量：B q2轻量核对','',
        '只重算已保存初值和返回曲线，不启动ROS、不重新优化、不用后来的局部地图冒充原完整地图。',
        '轮速标称限制4.1rad/s，轮加速度4.25rad/s²；原生验收另有5%容差。',
        '软成本使用标称限制；表中每段峰值使用原生全轨迹0.01秒采样。',
        '平滑项虽然函数名含Jerk，实际是四阶导数平方积分；只算该成本，没有反传完整梯度。','']
    for value in data['curves']:
        lines.append(('初始曲线' if value['stage']=='minco_before_optimizer' else '返回曲线')+
            '：时长%.3f秒，长度%.3f米；平滑成本%.6f，时间成本%.6f，底盘可行性软成本%.6f。' %
            (value['duration_s'],value['length_m'],value['native_smooth_snap_cost'],value['native_time_cost'],value['native_base_feasibility_sum']))
        lines.append('各项底盘软成本：'+json.dumps(value['native_base_feasibility_costs'],ensure_ascii=False))
        lines.append('段号 / 时长s / 长度m / 左右轮速峰rad/s / 左右轮加速度峰rad/s² / 底盘软成本')
        for piece in value['pieces']:
            peaks=piece['wheel_native_grid']['peaks']
            vals=[peaks[name]['abs_value'] if peaks[name] else float('nan') for name in CHANNELS]
            lines.append('%2d / %7.3f / %.3f / %.3f %.3f / %.3f %.3f / %.6f' %
                (piece['piece_index_1based'],piece['duration_s'],piece['length_m'],*vals,piece['native_base_feasibility_sum']))
        lines.append('')
    lines += ['已知边界',
        '本轮只有初值与最终曲线，没有中间每步x/g和完整原生ESDF；地图相关成本及完整时间梯度写未知。',
        '几何长度是0.01秒折线估计；轮峰是采样最大值，不是连续时间极值证明。',
        '某一长段远离轮边界，能说明这条返回曲线的该段不是一直被轮限制顶住；不能单凭这一点证明压缩该段仍满足全部碰撞/平滑/边界条件。',
        '前26次已接受候选未通过轮预检，第27次首次轮和原生检查通过即提前退出。这说明先得到了可行曲线，尚未专门验证后续质量改进；不能推断继续迭代一定会变短。']
    return '\n'.join(lines)+'\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,default=TRIAL)
    parser.add_argument('--out',type=Path,default=OUT)
    args=parser.parse_args();trial=args.trial.resolve()
    manifest=json.loads((trial/'manifest.json').read_text())
    proof=source_contract(Path(manifest['build']))
    wheel,numeric,tw,K,D,parameter_proof=parameters(trial,manifest)
    trace=records(trial/'static_width_trace.jsonl')
    selected=[next(r for r in trace if r['request_id']==2 and r['stage']==stage)
              for stage in ('minco_before_optimizer','optimizer_return')]
    timeline=records(trial/'dual_timeline.jsonl')
    sources=[Path(__file__),trial/'static_width_trace.jsonl',trial/'parameters.yaml',
        trial/'effective_real_params.json',trial/'manifest.json',trial/'dual_timeline.jsonl',
        trial/'failure_trace.jsonl',HERE/'backend_base_feasibility_breakdown.py',
        HERE/'audit_backend_kinematics.py']
    hashes={str(p):sha(p) for p in sources}
    result=dict(schema='feasible_quality_partial_cost_v1',source_sha256=hashes,
        trial=str(trial),request_id=2,parameters=parameter_proof,source_contract=proof,
        quadrature=dict(primary_intervals_per_piece=K,dense_resolution=D),
        curves=[analyze(r,wheel,numeric,tw,K,D) for r in selected],
        recorded_events=[r for r in timeline if r.get('request_id')==2 and r['event'] in
            ('early_delivery_summary','domain_solver_status','cheap_scaling_summary')],
        recorded_totals=[r for r in records(trial/'failure_trace.jsonl') if r.get('sim',0)>1015.],
        full_native_objective_replayed=False,gradients_replayed=False)
    if any(sha(Path(path))!=value for path,value in hashes.items()):
        raise RuntimeError('Input changed during audit')
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'初值与返回曲线核对.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (args.out/'初值与返回曲线核对.txt').write_text(narrative(result))
    print(narrative(result))


if __name__=='__main__':
    main()
