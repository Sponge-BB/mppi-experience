#!/usr/bin/env python3
"""Read-only input/geometry/search provenance audit of a declared V12 suite.

Writes one derived JSON report; task success is deliberately separate from
contract consistency, since a collision trial may retain a valid contract.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from dynamic_scenarios import actors_at, body_clearance, CASES

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(out):
    suite = json.loads((out/'suite_manifest.json').read_text())
    expected_body = np.atleast_2d(np.loadtxt(
        HERE/'results/final_report/empty_mppi/geometry.csv', delimiter=','))
    records, failures = [], []
    for spec in suite['specs']:
        source = out/spec['key']
        faults = []
        if not (source/'result.json').exists():
            failures.append(spec['key']+': missing result')
            continue
        result = json.loads((source/'result.json').read_text())
        limits = json.loads((source/'limits.json').read_text())
        manifest = json.loads((source/'manifest.json').read_text())
        body = np.atleast_2d(np.loadtxt(source/'geometry.csv', delimiter=','))
        if not np.array_equal(body, expected_body):
            faults.append('body geometry differs from baseline')
        if limits['max_prediction'] != spec['prediction']:
            faults.append('prediction differs from declared trial')
        if any(limits[k] != 0 for k in ('pos_unc', 'vel_unc', 'acc_unc')):
            faults.append('undeclared uncertainty radius')
        build = Path(spec['build'])
        if digest(build/'build/libonline_mppi.so') != suite['kernels'][str(build)]:
            faults.append('build library changed after declaration')
        if manifest['port'] != 11441:
            faults.append('ROS master is not isolated port')
        status = json.loads((source/'run_status.json').read_text())
        if status['required_process_failed']:
            faults.append('required native process failed')
        for name, key in (('reference_audit', 'reference_contract_pass'),
                          ('execution_audit', 'execution_contract_pass')):
            f = source/(name+'.json')
            if not f.exists() or not json.loads(f.read_text())[key]:
                faults.append(name+' did not pass')
        truth = np.atleast_1d(np.genfromtxt(source/'obstacles.csv', delimiter=',', names=True))
        anchor = result['motion_anchor'] or result['goal_sent'] or 0.
        if spec['scene'] in CASES:
            for stamp in np.unique(truth['t']):
                observed = truth[truth['t'] == stamp]
                declared = actors_at(spec['scene'], stamp-anchor)
                if len(observed) != len(declared):
                    faults.append('missing truth actor in an appearance frame')
                    break
                for actor in declared:
                    matches = observed[observed['id'] == actor[3]]
                    if len(matches) != 1 or not np.allclose(
                            [matches[0][k] for k in ('x', 'y', 'r', 'vx', 'vy')],
                            [actor[k] for k in (0, 1, 2, 4, 5)], atol=1e-8, rtol=0):
                        faults.append('truth motion differs from declared scene')
                        break
        snapshots = sorted((source/'search').glob('*.npz'))
        maximum_horizon = 0.
        for f in snapshots:
            with np.load(f, allow_pickle=False) as data:
                rows = data['rows']
                if rows.size:
                    horizon = float(rows[:, 1].max())
                    maximum_horizon = max(maximum_horizon, horizon)
                    if len(data['obstacles']) and horizon+float(data['obstacle_age']) > limits['max_prediction']+1e-6:
                        faults.append('search extends past measured CV validity')
                        break
        visual = source/'visualization/visualization_audit.json'
        gif_counts = {}
        if not visual.exists():
            faults.append('missing visualization audit')
        else:
            v = json.loads(visual.read_text())
            for name, expected in v['source_sha256'].items():
                if digest(source/name) != expected:
                    faults.append('visualization input changed: '+name)
            for name in ('overview', 'search_zoom'):
                with Image.open(source/'visualization'/(name+'.gif')) as gif:
                    count = gif.n_frames
                    for i in range(count):
                        gif.seek(i)
                        gif.load()
                    gif_counts[name] = count
        records.append(dict(key=spec['key'], result=result['reason'],
                            contract_pass=not faults, failures=sorted(set(faults)),
                            snapshots=len(snapshots), max_search_horizon_s=maximum_horizon,
                            decoded_gif_frames=gif_counts))
        failures += [spec['key']+': '+f for f in sorted(set(faults))]
    production = {}
    for name in suite['kernels']:
        build_manifest = json.loads((Path(name)/'build_manifest.json').read_text())
        for relative, expected in build_manifest['production_sources_sha256'].items():
            production[relative] = digest(ROOT/relative) == expected
    if not all(production.values()):
        failures.append('production sources differ from build baseline')
    report = dict(contract_pass=not failures, failures=failures, trials=records,
                  production_source_count=len(production), production_sources_unchanged=all(production.values()),
                  scope='Provenance and discrete-model contracts, not statistical advantage '
                        'or physical-robot safety; collisions remain failed tasks.')
    (out/'suite_audit.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return report['contract_pass']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    raise SystemExit(0 if audit(args.out.resolve()) else 1)
