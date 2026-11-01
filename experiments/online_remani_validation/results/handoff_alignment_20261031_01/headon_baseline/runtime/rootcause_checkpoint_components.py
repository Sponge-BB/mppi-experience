#!/usr/bin/env python3
"""Decompose an attested full-native checkpoint using the existing MINCO probe."""
import json
from pathlib import Path
import numpy as np
import rootcause_components as rc

HERE = Path(__file__).resolve().parent
BASE = HERE/'results/root_cause_20261030_01'


def main():
    evidence = rc.attest()
    raw = [json.loads(line) for line in (BASE/'native_run_03/results.jsonl').open()]
    check = next(r for r in raw if r.get('label') == 'original_converged_collision_checkpoint')
    inp = json.loads((BASE/'native_run_03/input.json').read_text())
    source = next(r for r in inp['checks'] if r['label'] == check['label'])
    run = dict(source['input'], stages=[source['stage']])
    loaded = json.loads((rc.TRIAL/'effective_real_params.json').read_text())
    ctx = rc.context(run, loaded)
    probe = rc.Probe(evidence['library'])
    x = np.asarray(source['stage']['x'])
    value = probe.evaluate(x, ctx)
    native = check['components']
    cost_errors = {name: abs(value['costs'][name]-native[key]) for name, key in
                   [('smooth', 'smooth'), ('time', 'time'), ('wheel', 'base_feasibility')]}
    if max(cost_errors.values()) > 1e-8:
        raise RuntimeError('Checkpoint component costs do not match native evaluation')
    original = next(json.loads(line) for line in (BASE/'native_run_02/results.jsonl').open()
                    if json.loads(line).get('label') == 'original_initial_300')
    if not np.array_equal(x, np.asarray(original['stages'][-1]['x'])):
        raise RuntimeError('Checkpoint coordinates differ from the converged recorded state')
    full = np.asarray(check['gradient'])
    gradients = {k: np.asarray(value['gradients'][k]) for k in ('smooth', 'wheel', 'time')}
    gradients['time_self'] = gradients['time']*(1+native['self_collision']/native['time'])
    gradients['base_obstacle_residual'] = full-gradients['smooth']-gradients['wheel']-gradients['time_self']
    gradients['full'] = full
    groups = dict(first_point_xy=[0, 1], later_point_xy=ctx.groups['point_xy'][2:],
                  time=ctx.groups['time'], gear_xy=ctx.groups['gear_xy'], gear_angle=ctx.groups['gear_angle'])
    stats = {}
    for k, g in gradients.items():
        stats[k] = dict(norm=float(np.linalg.norm(g)), first_point_fraction_sq=float(np.sum(g[:2]**2)/max(1e-30, g@g)),
            groups={name: float(np.linalg.norm(g[ids])) for name, ids in groups.items()},
            top=[dict(index=int(i), value=float(g[i])) for i in np.argsort(abs(g))[-8:][::-1]], gradient=g.tolist())
    branches = rc.channels(value, ctx)
    directions = []
    for key in ('base_obstacle_residual', 'wheel', 'smooth', 'full'):
        g = gradients[key]
        d = -g/max(float(np.linalg.norm(g)), 1e-30)
        row = dict(name=key, first_point_direction=d[:2].tolist(),
            derivatives={k: float(v@d) for k, v in gradients.items()}, steps=[])
        for h in (1e-6, 1e-5, 1e-4):
            changed = probe.evaluate(x+h*d, ctx)
            bc = rc.channels(changed, ctx)
            row['steps'].append(dict(step=h, smooth=changed['costs']['smooth'],
                wheel=changed['costs']['wheel'], base_channels=bc,
                native_wheel_peaks=rc.hard_peak(changed, ctx),
                collision_cost_not_replayed=True))
        directions.append(row)
    result = dict(schema='rootcause_checkpoint_components_v1', checkpoint=check['label'],
        full_native_cost=check['cost'], native_cost_components=native, component_cost_absolute_errors=cost_errors,
        checkpoint_x_bitwise_equal_run02_final=True,
        canonical_scope='Existing attested native MINCO probe uses the identical x, effective PVAJ and gear layout. Native checkpoint coefficients were not separately exported.',
        provenance=evidence, stats=stats, base_channels=branches, original_native_wheel_peaks=rc.hard_peak(value,ctx),
        directions=directions,
        residual_scope='Full native gradient minus exact native smooth, base-feasibility and fixed-arm constant-self/time gradients. Recorded arm obstacle, joint and other costs are zero; not an independent replay of the 3-D collision gradient.')
    dest=BASE/'checkpoint_components.json'
    dest.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(dest)
    print(json.dumps(dict(stats={k:{a:b for a,b in v.items() if a!='gradient'} for k,v in stats.items()},
        direction_derivatives=[dict(name=v['name'],derivatives=v['derivatives'],steps=[dict(step=s['step'],wheel=s['wheel'],peaks=s['native_wheel_peaks']['ratios']) for s in v['steps']]) for v in directions]),ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
