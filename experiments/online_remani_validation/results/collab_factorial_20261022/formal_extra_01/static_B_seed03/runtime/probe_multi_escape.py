#!/usr/bin/env python3
"""Offline bounded-command coverage probe; NEVER changes controller actions."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from suite_multi_dynamic import clearance


def probe(out):
    config = json.loads((out/'fixture/limits.json').read_text())
    manifest = json.loads((out/'manifest.json').read_text())
    base = np.array(manifest['cases']['04_four_dense']); appearance = manifest['appearance_s']
    def obs(t):
        a = base.copy(); a[:, :2] += (t-appearance)*a[:, 2:4]; return a
    details = []
    for path in sorted((out/'04_four_dense/data').glob('seed*.json')):
        r = json.loads(path.read_text()); body = np.array(r['body'])
        first = next((v for v in r['trace'] if v['unverified_brake']), None)
        if first is None: continue
        trials = []; h = config['track']/2
        for v in np.linspace(-.22, .22, 23):
            for w in np.linspace(-.8, .8, 17):
                if max(abs(v-h*w), abs(v+h*w)) > config['wheel_v']+1e-9: continue
                u = np.array([v, w]); s = np.array(first['state']); gap = clearance(s, obs(first['t']), body); positions = [s[:2].tolist()]
                for k in range(380): # 1.9 s at 5 ms, same observed CV validity.
                    a = (u-s[3:])/.12
                    a /= max(1, abs(a[0])/config['accel'], abs(a[1])/config['angular_accel'],
                             abs(a[0]-h*a[1])/config['wheel_a'], abs(a[0]+h*a[1])/config['wheel_a'])
                    new = s[3:]+.005*a; vv, ww = (s[3:]+new)/2; theta = s[2]+.0025*ww
                    s[:2] += .005*vv*np.array([np.cos(theta), np.sin(theta)])
                    s[2] = math.atan2(math.sin(s[2]+.005*ww), math.cos(s[2]+.005*ww)); s[3:] = new
                    gap = min(gap, clearance(s, obs(first['t']+(k+1)*.005), body))
                    if k % 20 == 19: positions.append(s[:2].tolist())
                trials.append({'v': float(v), 'w': float(w), 'min_gap_m': float(gap), 'positions': positions})
        feasible = [v for v in trials if v['min_gap_m'] >= config['margin']]
        best = max(trials, key=lambda v: v['min_gap_m'])
        details.append({'seed': r['seed'], 'first_unverified_brake_s': first['t'], 'tested': len(trials),
                        'feasible_1_9s': len(feasible), 'largest_clearance_trial': best,
                        'zero_command': next(v for v in trials if abs(v['v']) < 1e-9 and abs(v['w']) < 1e-9)})
    result = {'scope': 'offline constant-command coverage probe, 1.9s, dense ideal plant; NOT controller fix, recursive safety, or proof of goal reachability', 'results': details}
    (out/'escape_coverage_probe.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({**result, 'results': [dict({k:v for k,v in row.items() if k not in ('largest_clearance_trial', 'zero_command')}, best_v=row['largest_clearance_trial']['v'], best_w=row['largest_clearance_trial']['w'], best_gap=row['largest_clearance_trial']['min_gap_m']) for row in details]}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('out', type=Path); a = p.parse_args(); probe(a.out)
