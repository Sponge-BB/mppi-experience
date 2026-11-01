#!/usr/bin/env python3
"""Read-only search API contracts; no robot/ROS connections."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller, DP, ptr
from suite_multi_dynamic import search_snapshot, obstacles, plant


def run(build, fixture, out):
    c = Controller(build, fixture); lib = c.lib
    lib.online_search_enable.argtypes = [ct.c_void_p, ct.c_int]
    lib.online_search_trace.argtypes = [ct.c_void_p, DP, ct.c_int]
    lib.online_search_trace.restype = ct.c_int
    x = np.linspace(0, 3, 301); ref = np.ascontiguousarray(np.column_stack((x, x*0, x*0, x*0+.18)))
    s = np.array([.5, 0., 0., .18, 0.]); obs = np.array([[1.4, -.5, 0., .26, .23, 1.], [2.5, 1., 0., -.2, .23, 1.]])
    lib.online_search_enable(c.handle, 1); c.step(s, ref, obs)
    first = search_snapshot(c, 0.)
    assert first['candidates']
    assert sum(v['selected'] for v in first['candidates']) == 1
    for v in first['candidates']:
        a = np.array(v['points']); assert np.isfinite(a).all()
        assert np.array_equal(a[0, 1:], s[:2]) and a[0, 0] == 0
        assert np.allclose(np.diff(a[:, 0]), .1)
        assert a[-1, 0]+.04 <= 2.
        assert v['feasible'] == (v['gap'] >= 0)
    assert search_snapshot(c, 0.) == first, 'Getter must not mutate controller'
    # Short buffers must not overwrite memory; return value remains total size.
    n = lib.online_search_trace(c.handle, None, 0); guard = np.full((3, 8), 123456.)
    assert lib.online_search_trace(c.handle, ptr(guard), 1) == n
    assert np.all(guard[1:] == 123456.)
    lib.online_search_enable(c.handle, 0); assert lib.online_search_trace(c.handle, None, 0) == 0
    c.step(s, ref, obs); assert lib.online_search_trace(c.handle, None, 0) == 0
    lib.online_search_enable(c.handle, 1); c.step(s, ref, obs)
    result = np.zeros(8)
    code = lib.online_step(c.handle, ptr(s), ptr(ref), len(ref), ptr(obs), len(obs), .36,
                          ptr(c.field), 300, 200, -7.5, -5., .05, 0, ptr(result))
    assert code != 0 and lib.online_search_trace(c.handle, None, 0) == 0
    c.step(s, ref, obs); lib.online_reset(c.handle)
    assert lib.online_search_trace(c.handle, None, 0) == 0
    # The extra geometry audit must preserve the original lagged plant exactly.
    body = np.atleast_2d(np.loadtxt(fixture/'geometry.csv', delimiter=','))
    s1 = np.array([.3, .2, -.4, .1, -.2]); s2 = s1.copy(); u = np.array([.16, .3])
    c.plant(s1, u); plant(c.c, s2, u, '04_four_dense', 3., body)
    assert np.array_equal(s1, s2)
    for case, count in [('01_single', 1), ('02_two_crossing', 2), ('03_three_sequential', 3), ('04_four_dense', 4)]:
        assert len(obstacles(case, 1.99)) == 0
        assert len(obstacles(case, 2.)) == count and len(obstacles(case, 44.95)) == count
        assert np.allclose(obstacles(case, 20.)[:, :2], obstacles(case, 2.)[:, :2]+18*obstacles(case, 2.)[:, 2:4])
    c.close()
    checks = ['finite actual candidates and unique selected rollout', 'same start pose and valid CV horizon',
              'getter is read-only and capacity-bounded', 'disable/reset/rejected input clear traces',
              'dense-audit plant identical to prior plant', 'all actors persist under declared CV law']
    out.write_text(json.dumps({'passed': True, 'checks': checks, 'candidate_count': len(first['candidates'])}, indent=2)+'\n')
    print('PASS', out)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--build', type=Path, required=True)
    p.add_argument('--fixture', type=Path, required=True); p.add_argument('--out', type=Path, required=True)
    a = p.parse_args(); run(a.build, a.fixture, a.out)
