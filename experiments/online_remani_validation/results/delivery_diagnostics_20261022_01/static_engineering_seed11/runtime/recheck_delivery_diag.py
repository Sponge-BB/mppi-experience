#!/usr/bin/env python3
"""Independent, read-only recheck of frozen delivery candidates.

No optimizer, ROS graph, trajectory retiming, or acceptance change is performed.
The arithmetic below deliberately does not call the online collision kernel.
It checks the SAME recorded disk-union projection against the SAME recorded
bilinear distance field: agreement is not independent physical ground truth.
"""
import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np

EPS = 1e-8


def load_json(path):
    with Path(path).open(encoding='utf-8') as stream:
        return json.load(stream)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve_snapshot(root, frame):
    reference = frame.get('reference', frame) if frame else None
    value = reference.get('snapshot') if reference else None
    if value is None:
        return None
    path = (root / value).resolve()
    if root.resolve() not in path.parents:
        raise ValueError('snapshot path escapes diagnostic directory')
    return load_json(path)


def snapshot_arrays(raw):
    """Validate transport geometry without inheriting its evaluation logic."""
    if raw is None:
        return None
    m = dict(raw)
    nx, ny = int(m['nx']), int(m['ny'])
    if nx < 2 or ny < 2 or m['res'] <= 0:
        raise ValueError('invalid grid metadata')
    m['field'] = np.asarray(m['field'], dtype=float).reshape(ny, nx)
    m['body'] = np.asarray(m['body'], dtype=float).reshape(-1, 3)
    m['obstacles'] = np.asarray(m['obstacles'], dtype=float).reshape(-1, 6)
    if len(m['body']) == 0 or not np.all(np.isfinite(m['body'])) or np.any(m['body'][:, 2] <= 0):
        raise ValueError('invalid body geometry')
    if m['margin'] < 0 or not math.isfinite(m['margin']):
        raise ValueError('invalid margin')
    return m


def evaluate_pose(m, x, y, yaw, forecast, dynamic=False):
    """Return a geometric category, keeping unknown distinct from collision."""
    if m is None:
        return dict(category='invalid', reason='missing_snapshot', physical=None, gap=None)
    if not all(math.isfinite(v) for v in (x, y, yaw, forecast)):
        return dict(category='invalid', reason='nonfinite_query', physical=None, gap=None)
    if dynamic and (forecast < 0 or forecast > m['max_prediction']):
        return dict(category='unknown', reason='outside_prediction', physical=None, gap=None)
    co, si = math.cos(yaw), math.sin(yaw)
    body = m['body']
    centers = np.column_stack((x + co*body[:, 0] - si*body[:, 1],
                               y + si*body[:, 0] + co*body[:, 1]))
    if dynamic:
        obstacles = m['obstacles']
        if not len(obstacles):
            return dict(category='safe', reason='no_obstacles', physical=None, gap=None)
        if not np.all(np.isfinite(obstacles)) or np.any(obstacles[:, 4] <= 0) or np.any(~np.isin(obstacles[:, 5], [0., 1.])):
            return dict(category='invalid', reason='invalid_obstacle', physical=None, gap=None)
        predicted = obstacles[:, :2] + forecast*obstacles[:, 2:4]*obstacles[:, 5, None]
        clearances = np.linalg.norm(centers[:, None, :] - predicted[None, :, :], axis=2) - body[:, 2, None] - obstacles[None, :, 4]
        bi, oi = np.unravel_index(int(np.argmin(clearances)), clearances.shape)
        physical = float(clearances[bi, oi])
        detail = dict(body_index=int(bi), obstacle_index=int(oi))
    else:
        grid = (centers - [m['gx'], m['gy']])/m['res']
        bad = ~np.all(np.isfinite(grid), axis=1) | np.any(grid < 0., axis=1) | (grid[:, 0] >= m['nx']-1) | (grid[:, 1] >= m['ny']-1)
        if np.any(bad):
            return dict(category='invalid', reason='body_outside_grid', body_index=int(np.flatnonzero(bad)[0]), physical=None, gap=None)
        ij = np.floor(grid).astype(np.int64)
        a, b = (grid - ij).T
        ix, iy = ij.T
        f = m['field']
        corners = np.column_stack((f[iy, ix], f[iy, ix+1], f[iy+1, ix], f[iy+1, ix+1]))
        if not np.all(np.isfinite(corners)) or np.any(corners < 0):
            return dict(category='invalid', reason='invalid_distance_cell', physical=None, gap=None)
        distances = (1-b)*((1-a)*corners[:, 0]+a*corners[:, 1]) + b*((1-a)*corners[:, 2]+a*corners[:, 3])
        clearances = distances-body[:, 2]
        bi = int(np.argmin(clearances))
        physical = float(clearances[bi])
        detail = dict(body_index=bi, obstacle_index=None)
    gap = physical - m['margin']
    category = 'collision' if physical < -EPS else ('margin_only' if gap < -EPS else 'safe')
    return dict(category=category, reason='geometric_clearance', physical=physical, gap=gap, **detail)


def polynomial(piece, t):
    c = np.asarray(piece['coeffs'], dtype=float)
    if c.ndim != 2 or c.shape[0] < 2 or c.shape[1] != 8 or not np.all(np.isfinite(c)):
        raise ValueError('invalid coefficient matrix')
    if piece.get('basis', 'descending') != 'descending':
        raise ValueError('unsupported polynomial coefficient order')
    # Match native ascending-power accumulation to avoid signed-zero changes
    # from Horner evaluation near an exactly stationary boundary.
    p, v = np.zeros(c.shape[0]), np.zeros(c.shape[0])
    tn = 1.
    for power in range(8):
        p += tn*c[:, 7-power]
        tn *= t
    tn = 1.
    for power in range(1, 8):
        v += (power*tn)*c[:, 7-power]
        tn *= t
    return p, v


def candidate_samples(candidate, mode='dense', dt=.02):
    elapsed = 0.
    for part_index, part in enumerate(candidate['parts']):
        pieces = part['pieces']
        times = np.asarray([piece['T'] for piece in pieces], dtype=float)
        if not len(times) or not np.all(np.isfinite(times)) or np.any(times <= 0):
            raise ValueError('invalid candidate durations')
        duration = float(part['duration'])
        if abs(float(np.sum(times))-duration) > 1e-7*max(1., duration):
            raise ValueError('part duration does not match polynomial pieces')
        gear = int(part['gear'])
        if gear not in (-1, 1):
            raise ValueError('invalid gear')
        if mode == 'K8':
            samples = [(i, q, float(np.sum(times[:i]))+q) for i, T in enumerate(times) for q in np.linspace(0., T, 9)]
        elif mode == 'dense':
            n = max(1, int(math.ceil(duration/dt)))
            samples = []
            for k in range(n+1):
                q = duration*k/n
                i, local = 0, q
                while i < len(times) and local > times[i]:
                    local -= float(times[i])
                    i += 1
                if i == len(times):
                    i -= 1
                    local += float(times[i])
                samples.append((i, local, float(q)))
        else:
            raise ValueError(mode)
        for sample_index, (i, local, q) in enumerate(samples):
            p, v = polynomial(pieces[i], local)
            yaw = math.atan2(gear*float(v[1]), gear*float(v[0]))
            yield dict(part=part_index, piece=i, sample=sample_index, q=q, local_t=local,
                       t=elapsed+q, query_time=float(candidate['start_time'])+elapsed+q,
                       x=float(p[0]), y=float(p[1]), vx=float(v[0]), vy=float(v[1]),
                       speed=float(np.linalg.norm(v[:2])), yaw=yaw, gear=gear)
        elapsed += duration


def fresh(m, now):
    return m is not None and bool(m['healthy']) and -.05 <= now-m['stamp'] <= m['max_age']


def audit_candidate(root, path, dt=.02):
    candidate = load_json(path)
    if candidate.get('schema') != 'delivery_candidate_v1':
        raise ValueError('unknown candidate schema: '+str(path))
    records, summaries = [], []
    for frame_name in ('solve_frame', 'accept_frame'):
        frame = candidate.get(frame_name, {})
        m = snapshot_arrays(resolve_snapshot(root, frame))
        now = float(frame.get('now', float('nan')))
        for mode in ('K8', 'dense'):
            categories = Counter()
            first_failure, min_static, min_dynamic = None, None, None
            low_speed = 0
            for sample in candidate_samples(candidate, mode, dt):
                low_speed += sample['speed'] < 1e-5
                stat = evaluate_pose(m, sample['x'], sample['y'], sample['yaw'], 0.)
                dyn = dict(category='past', reason='past_candidate_time', physical=None, gap=None)
                if sample['query_time'] >= now:
                    dyn = evaluate_pose(m, sample['x'], sample['y'], sample['yaw'], sample['query_time']-m['stamp'] if m else 0., True)
                for layer, value in (('static', stat), ('dynamic', dyn)):
                    categories[layer+'_'+value['category']] += 1
                    if value['gap'] is not None:
                        if layer == 'static':
                            min_static = value['gap'] if min_static is None else min(min_static, value['gap'])
                        else:
                            min_dynamic = value['gap'] if min_dynamic is None else min(min_dynamic, value['gap'])
                    if first_failure is None and value['category'] in ('invalid', 'collision', 'margin_only'):
                        first_failure = dict(layer=layer, **sample, **value)
                records.append(dict(candidate=path.name, frame=frame_name, sampling=mode, **sample,
                                    static_category=stat['category'], static_physical=stat['physical'], static_gap=stat['gap'],
                                    dynamic_category=dyn['category'], dynamic_physical=dyn['physical'], dynamic_gap=dyn['gap']))
            summaries.append(dict(candidate=path.name, frame=frame_name, sampling=mode,
                                  snapshot_sequence=m['sequence'] if m else None,
                                  snapshot_age=now-m['stamp'] if m else None,
                                  snapshot_fresh=fresh(m, now), counts=dict(categories),
                                  min_static_gap=min_static, min_dynamic_gap=min_dynamic,
                                  low_speed_samples=low_speed, first_failure=first_failure,
                                  gate_ok=fresh(m, now) and first_failure is None))
    expected = next(s for s in summaries if s['frame']=='accept_frame' and s['sampling']=='dense')
    recorded_failure = candidate.get('failure')
    recorded_accepted = candidate.get('accepted', recorded_failure is None)
    agreement = bool(candidate.get('finished', True)) and expected['gate_ok'] == recorded_accepted
    first = expected['first_failure']
    detailed = None
    if first is not None and recorded_failure is not None:
        deltas = {}
        for key in ('query_time', 'yaw', 'physical', 'gap'):
            a, b = first.get(key), recorded_failure.get(key)
            if a is not None and b is not None and math.isfinite(a) and math.isfinite(b):
                deltas[key] = abs(a-b)
        detailed = dict(part_matches=first.get('part')==recorded_failure.get('part'),
                        absolute_differences=deltas)
    return dict(path=str(path), sha256=digest(path), recorded_failure=recorded_failure,
                recorded_finished=candidate.get('finished'), recorded_accepted=recorded_accepted,
                replay_gate_status_agrees=agreement, first_failure_comparison=detailed,
                request_id=candidate.get('request_id'), scope_id=candidate.get('scope_id'),
                summaries=summaries), records


def audit_geometry_error(root, path):
    error = load_json(path)
    if error.get('schema') != 'delivery_geometry_error_v1':
        raise ValueError('unknown geometry error schema')
    frame = error['frame']
    m = snapshot_arrays(resolve_snapshot(root, frame))
    p, v = np.asarray(error['p'], dtype=float), np.asarray(error['v'], dtype=float)
    block = error['blocks'][error['block']]
    if block.get('basis') != 'ascending_rows_per_8':
        raise ValueError('unsupported geometry error coefficient order')
    i = error['piece']
    c = np.asarray(block['coeffs'], dtype=float)[8*i:8*(i+1), :]
    s = float(error['s'])
    b0 = np.array([s**k for k in range(8)])
    b1 = np.array([k*s**(k-1) if k else 0. for k in range(8)])
    cp, cv = c.T@b0, c.T@b1
    deltas = {}
    if np.all(np.isfinite(cp)) and np.all(np.isfinite(p)):
        deltas['p_max_abs'] = float(np.max(np.abs(cp[:len(p)]-p)))
    if np.all(np.isfinite(cv)) and np.all(np.isfinite(v)):
        deltas['v_max_abs'] = float(np.max(np.abs(cv[:len(v)]-v)))
    if error.get('yaw') is None:
        yaw = float('nan')
    else:
        yaw = float(error['yaw'])
    forecast = float(error['forecast']) if error.get('forecast') is not None else float('nan')
    stat = evaluate_pose(m, float(p[0]), float(p[1]), yaw, 0.)
    dyn = evaluate_pose(m, float(p[0]), float(p[1]), yaw, forecast, True)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(v)):
        cause = 'nonfinite_position_or_velocity'
    elif stat['category']=='invalid':
        cause = stat['reason']
    elif not math.isfinite(forecast) or forecast < 0.:
        cause = 'negative_or_nonfinite_forecast'
    elif dyn['category']=='invalid':
        cause = dyn['reason']
    else:
        cause = 'geometry_valid_gradient_or_aggregate_requires_native_inspection'
    return dict(path=str(path), sha256=digest(path), request_id=error.get('request_id'),
                scope_id=error.get('scope_id'), block=error['block'], piece=i,
                sample=error['sample'], forecast=forecast if math.isfinite(forecast) else None,
                stored_leaf=error.get('leaf'), independent_cause=cause,
                reconstructed_parameter_deltas=deltas,
                snapshot_fresh=fresh(m, frame['now']),
                static=stat, dynamic=dyn)


def figure_candidate(root, audit, rows, target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    candidate = load_json(Path(audit['path']))
    m = snapshot_arrays(resolve_snapshot(root, candidate['accept_frame']))
    if m is None:
        return
    dense = [r for r in rows if r['frame']=='accept_frame' and r['sampling']=='dense']
    if not dense:
        return
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    ax = axes[0]
    f = m['field']
    xx = m['gx']+np.arange(m['nx'])*m['res']
    yy = m['gy']+np.arange(m['ny'])*m['res']
    ax.contourf(xx, yy, f, levels=[-.001, .05], colors=['#bdbdbd'], alpha=.8)
    ax.plot([r['x'] for r in dense], [r['y'] for r in dense], color='#2670a7', label='Frozen candidate')
    failure = next(s['first_failure'] for s in audit['summaries'] if s['frame']=='accept_frame' and s['sampling']=='dense')
    selected = failure or min(dense, key=lambda r: r['static_gap'] if r['static_gap'] is not None else float('inf'))
    co, si = math.cos(selected['yaw']), math.sin(selected['yaw'])
    for bx, by, radius in m['body']:
        cx = selected['x']+co*bx-si*by
        cy = selected['y']+si*bx+co*by
        ax.add_patch(Circle((cx, cy), radius, fill=False, lw=1.5, color='#d95f02'))
        ax.add_patch(Circle((cx, cy), radius+m['margin'], fill=False, lw=.7, ls='--', color='#d95f02', alpha=.65))
    ax.scatter([selected['x']], [selected['y']], marker='x', color='#d95f02', label='Checked projection')
    xs, ys = [r['x'] for r in dense], [r['y'] for r in dense]
    ax.set(xlim=(min(xs)-.8, max(xs)+.8), ylim=(min(ys)-.8, max(ys)+.8), xlabel='x [m]', ylabel='y [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.legend(fontsize=8)
    ax.set_title('Actual recorded body, not a point')
    ax = axes[1]
    for frame, color, style in (('solve_frame', '#2670a7', '-'), ('accept_frame', '#d95f02', '--')):
        current = [r for r in rows if r['frame']==frame and r['sampling']=='dense']
        values = [np.nan if r['static_gap'] is None else r['static_gap'] for r in current]
        ax.plot([r['t'] for r in current], values, color=color, ls=style, label=frame.replace('_frame', '')+' static gap')
    ax.axhline(0., color='black', lw=.8)
    ax.axhline(-m['margin'], color='gray', ls=':', lw=.8, label='Physical contact')
    ax.set(xlabel='Candidate time [s]', ylabel='Clearance minus margin [m]')
    ax.legend(fontsize=8)
    ax.set_title('Same candidate; no re-optimization')
    fig.suptitle(Path(audit['path']).name)
    fig.savefig(target, dpi=150)
    plt.close(fig)


def report(root, out, dt=.02, plots=2):
    root, out = Path(root).resolve(), Path(out).resolve()
    if out.exists():
        raise FileExistsError('report directory already exists: '+str(out))
    paths = sorted((root/'candidates').glob('*.json'))
    errors = sorted((root/'geometry_errors').glob('*.json'))
    if not paths and not errors:
        raise ValueError('no recorded candidates or geometry errors')
    all_audits, all_rows = [], []
    for path in paths:
        audit, rows = audit_candidate(root, path, dt)
        all_audits.append(audit)
        all_rows.extend(rows)
    error_audits = [audit_geometry_error(root, path) for path in errors]
    out.mkdir(parents=True)
    output = dict(schema='delivery_independent_recheck_v1', source=str(root), dt=dt,
                  method='Independent NumPy bilinear field + recorded disk-union; no optimizer, retiming or ROS',
                  limitations=['Recorded geometry is not independent physical ground truth.',
                               'K8 is per piece; dense uses original whole-block gate sample times.',
                               'Both samplers keep candidate.start_time+offset. K8 is only a sampling-density sensitivity, NOT full optimizer-objective replay: the online added dynamic objective uses solve_frame.now+offset.',
                               'Unknown dynamic horizon is not collision and is not a safety certificate.',
                               'Same candidate only; differences do not estimate closed-loop policy performance.',
                               'Low-speed heading remains the native velocity-derived convention; no explicit endpoint yaw is fabricated.'],
                  source_snapshot_sha256={str(p.relative_to(root)): digest(p) for p in sorted((root/'snapshots').glob('*.json'))},
                  candidate_count=len(paths), audits=all_audits, geometry_error_audits=error_audits)
    (out/'recheck.json').write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    if all_rows:
        with (out/'samples.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
            writer.writeheader()
            writer.writerows(all_rows)
    lines = ['同候选离线重查（没有重新优化、改变轨迹时钟或放宽验收）',
             'physical<0：投影外形相交；physical>=0且gap<0：仅安全余量不足；invalid：查询无效；unknown：动态预测时域外。',
             '独立重写了插值和投影计算，但使用同一记录几何，不等于真实物理碰撞测量。', '']
    lines.insert(3, 'K8仅用于同候选、同candidate.start_time时钟下的采样密度对比，不是原目标函数重放；原动态软代价使用solve_frame.now+offset，不能据本表认定动态软代价数值或时域漏检。')
    for audit in all_audits:
        lines.append(Path(audit['path']).name+': 与在线验收结论'+('一致' if audit['replay_gate_status_agrees'] else '不一致（需检查，不能直接归因算法）'))
        for s in audit['summaries']:
            failure = s['first_failure']
            detail = ('无不合格样本' if failure is None else '{} {} t={:.6f}s physical={} gap={}'.format(failure['layer'], failure['category'], failure['t'], failure['physical'], failure['gap']))
            lines.append('  {} / {} / fresh={} / {} / 最小静态gap={}'.format(s['frame'], s['sampling'], s['snapshot_fresh'], detail, s['min_static_gap']))
    lines.extend(['', '记录的优化几何异常：{}次；原始文件保持不变。'.format(len(errors))])
    for error in error_audits:
        lines.append(Path(error['path']).name+': '+json.dumps(error, ensure_ascii=False))
    (out/'重查结论.txt').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    rejected = [a for a in all_audits if a['recorded_failure'] is not None]
    selected = (rejected+ [a for a in all_audits if a['recorded_failure'] is None])[:plots]
    for i, audit in enumerate(selected, 1):
        rows = [r for r in all_rows if r['candidate']==Path(audit['path']).name]
        figure_candidate(root, audit, rows, out/('candidate_%02d.png'%i))
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnostics', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--dt', type=float, default=.02)
    parser.add_argument('--plots', type=int, default=2)
    args = parser.parse_args()
    if not math.isfinite(args.dt) or args.dt <= 0 or args.plots not in (0, 1, 2):
        parser.error('dt must be positive; plots must be 0, 1 or 2')
    result = report(args.diagnostics, args.out, args.dt, args.plots)
    print(json.dumps(dict(candidates=result['candidate_count'], out=str(args.out)), ensure_ascii=False))
