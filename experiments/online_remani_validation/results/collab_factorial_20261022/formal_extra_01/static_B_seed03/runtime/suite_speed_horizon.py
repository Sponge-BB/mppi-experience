#!/usr/bin/env python3
"""Predeclared speed/horizon experiment; no ROS, no installation, no CBF."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shutil
import time

import numpy as np
from suite_v8 import multi, sha, HERE
from test_mppi_v2 import Controller, ptr, DP

PROFILES = {
    'base': dict(vmax=.30, wheel_v=.2296, reference_speed=.18),
    'fast': dict(vmax=.35, wheel_v=.32, reference_speed=.24),
    'capacity': dict(vmax=.35, wheel_v=.32, reference_speed=.18),
}
CASES = ('05_head_on_only', '04_four_dense')


def contracts(build, fixture, validity):
    c = Controller(build, fixture)
    lib = c.lib
    lib.online_search_enable.argtypes = [ct.c_void_p, ct.c_int]
    lib.online_search_trace.argtypes = [ct.c_void_p, DP, ct.c_int]
    lib.online_search_trace.restype = ct.c_int
    lib.online_recovery_debug.argtypes = [ct.c_void_p, DP]
    lib.online_avoidance_debug.argtypes = [ct.c_void_p, DP]
    x = np.linspace(0, 3, 301)
    ref = np.ascontiguousarray(np.column_stack((x, x*0, x*0, x*0+.18)))
    s = np.array([1.2, 0., 0., .18, 0.])
    obs = np.array([[2.3, .3, -.18, 0., .2, 1.]])
    lib.online_search_enable(c.handle, 1)
    c.step(s, ref, obs)
    snap = multi.search_snapshot(c, 0.)
    horizon = .1 * int(np.floor((validity-.04-.01)/.1))
    ends = [v['points'][-1][0] for v in snap['candidates']]
    assert ends and all(abs(t-horizon) < 1e-8 for t in ends), (validity, ends)
    assert max(ends)+.04 <= validity
    # Finite-difference prediction is still age-bounded; invalid calls clear trace.
    result = np.zeros(8)
    for age, cbf in ((.36, 0), (.04, 1)):
        code = lib.online_step(c.handle, ptr(s), ptr(ref), len(ref), ptr(obs), len(obs), age,
                               ptr(c.field), 300, 200, -7.5, -5., .05, cbf, ptr(result))
        assert code != 0 and lib.online_search_trace(c.handle, None, 0) == 0
    lib.online_reset(c.handle)
    c.field[:] = 0
    u = c.step(s, ref, obs)
    d = np.zeros(10); lib.online_recovery_debug(c.handle, ptr(d))
    assert np.max(abs(u)) == 0 and d[9] == 1 and d[3] == 0
    c.close()
    return dict(validity_s=validity, rollout_s=horizon, candidates=len(ends),
                stale_rejected=True, cbf_rejected=True, fully_occupied_stop_unverified=True)


def audit_record(r, limits, validity):
    rows = r['trace']
    s = np.array([x['state'] for x in rows]); ns = np.array([x['next_state'] for x in rows])
    u = np.array([x['command'] for x in rows]); h = limits['track']/2
    assert np.isfinite(s).all() and np.isfinite(ns).all() and np.isfinite(u).all()
    assert np.array_equal(s[1:], ns[:-1])
    for x in (u, ns[:, 3:]):
        assert np.max(abs(x[:, 0])) <= limits['vmax']+1e-9
        assert np.max(abs(x[:, 1])) <= limits['wmax']+1e-9
        assert np.max(abs(x[:, 0])+h*abs(x[:, 1])) <= limits['wheel_v']+1e-9
    acc = (ns[:, 3:]-s[:, 3:])/.05
    assert np.max(abs(acc[:, 0])) <= limits['accel']+1e-8
    assert np.max(abs(acc[:, 1])) <= limits['angular_accel']+1e-8
    assert np.max(abs(acc[:, 0])+h*abs(acc[:, 1])) <= limits['wheel_a']+1e-8
    for row in rows:
        assert np.allclose(np.array(row['obstacles']).reshape(-1, 6), multi.obstacles(r['case'], row['t']))
    for snap in r['searches']:
        selected = [c for c in snap['candidates'] if c['selected']]
        assert len(selected) <= 1 and all(c['feasible'] for c in selected)
        for c in snap['candidates']:
            points = np.array(c['points']); assert np.isfinite(points).all()
            # Empty-scene V5 retains the original 3s horizon; dynamic CV does not.
            if snap['t'] >= 2: assert points[-1, 0]+.04 <= validity+1e-8
            assert c['feasible'] == (c['gap'] >= 0)
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--build', type=Path, required=True); p.add_argument('--out', type=Path, required=True)
    a = p.parse_args(); out = a.out.resolve(); build = a.build.resolve()
    out.mkdir(parents=True, exist_ok=False); (out/'source_snapshot').mkdir()
    bm = json.loads((build/'build_manifest.json').read_text())
    assert sha(build/'build/libonline_mppi.so') == bm['mppi_library_sha256']
    for name, value in bm['production_sources_sha256'].items(): assert sha(HERE.parents[1]/name) == value
    for name in ('build_v8_horizon.py', 'suite_speed_horizon.py', 'suite_multi_dynamic.py', 'suite_v8.py', 'test_mppi_v2.py'):
        shutil.copy2(HERE/name, out/'source_snapshot'/name)
    original = HERE/'results/mppi_v8/validation_03'
    fixture = original/'fixture'
    configs = {}
    for profile in ('base', 'fast', 'capacity'):
        for validity in ((2., 3., 4.) if profile != 'capacity' else (2.,)):
            key = f'{profile}_h{int(validity)}'; dest = out/key/'fixture'; dest.mkdir(parents=True)
            limits = json.loads((fixture/'limits.json').read_text())
            limits.update({k: v for k, v in PROFILES[profile].items() if k != 'reference_speed'})
            limits['max_prediction'] = validity
            # Zero uncertainty is also enforced by Controller's no_uncertainty profile.
            limits.update(pos_unc=0., vel_unc=0., acc_unc=0.)
            (dest/'limits.json').write_text(json.dumps(limits, indent=2)+'\n')
            shutil.copy2(fixture/'geometry.csv', dest/'geometry.csv')
            configs[key] = dict(profile=profile, validity_s=validity, limits=limits,
                                reference_speed=PROFILES[profile]['reference_speed'],
                                cases=list(CASES if profile != 'capacity' else CASES[:1]))
    manifest = dict(scope='fixed-reference ideal-CV SIL, no real control; measured compute delay NOT applied to plant',
                    build=str(build), build_manifest=bm, configs=configs, seeds=[1, 2, 3],
                    cases={case: multi.CASES[case] for case in CASES},
                    snapshots={f.name: sha(f) for f in (out/'source_snapshot').iterdir()},
                    baseline_library_sha256=sha(HERE/'builds/mppi_v8_09/build/libonline_mppi.so'),
                    timing='Serial trials; seed1 trace enabled at 5Hz; kernel excludes export and plant/map construction.',
                    hypotheses='Longer horizon and/or higher speed may improve retreat, but collision/timeout/latency regressions are retained.',
                    inherited_limitations='Same fixed action speeds and .8s switching; same costs; BaseMPPI inverse-horizon sample budget; grid proposal risk horizon 1.5s.')
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    checks = [contracts(build, out/f'base_h{h}'/'fixture', float(h)) for h in (2, 3, 4)]
    (out/'contracts.json').write_text(json.dumps(checks, indent=2)+'\n')
    summary = {}; exact = []
    # Paired seed-major execution distributes machine-load drift across variants.
    for seed in (1, 2, 3):
        for case in CASES:
            for key, config in configs.items():
                if case not in config['cases']: continue
                print('START', seed, case, key, flush=True); start = time.monotonic()
                r = multi.trial(build, out/key/'fixture', case, seed, capture=seed == 1,
                                reference_speed=config['reference_speed'])
                r.update(version=key, config=config)
                audit_record(r, config['limits'], config['validity_s'])
                if key == 'base_h2':
                    old = json.loads((original/case/'data'/f'v8_seed{seed}.json').read_text())
                    assert len(old['trace']) == len(r['trace'])
                    for x, y in zip(old['trace'], r['trace']):
                        for field in ('state', 'next_state', 'command', 'gap'): assert x[field] == y[field], (case, seed, field)
                    exact.append(dict(case=case, seed=seed, exact_match=True))
                dest = out/key/case/'data'; dest.mkdir(parents=True, exist_ok=True)
                (dest/f'seed{seed}.json').write_text(json.dumps(r, separators=(',', ':'))+'\n')
                summary.setdefault(key, {}).setdefault(case, []).append(dict(seed=seed, **r['metrics']))
                (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
                print('DONE', key, case, seed, r['metrics'], 'wall_s', round(time.monotonic()-start, 1), flush=True)
    for name, value in bm['production_sources_sha256'].items(): assert sha(HERE.parents[1]/name) == value
    for name, value in manifest['snapshots'].items(): assert sha(out/'source_snapshot'/name) == value
    (out/'audit.json').write_text(json.dumps(dict(integrity_passed=True, trials=sum(len(rs) for c in summary.values() for rs in c.values()),
        production_files_unchanged=len(bm['production_sources_sha256']), baseline_exact=exact,
        note='Integrity and actuator constraints only; safety outcomes are recorded, not asserted successful.'), indent=2)+'\n')
    print('COMPLETE', out, flush=True)


if __name__ == '__main__':
    main()
