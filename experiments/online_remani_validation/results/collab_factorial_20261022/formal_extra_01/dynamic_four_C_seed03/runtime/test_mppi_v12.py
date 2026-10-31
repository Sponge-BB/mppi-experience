#!/usr/bin/env python3
"""V12 controller contract tests; isolated C API, no ROS or robot publishers."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller, DP, ptr


def configured(build, fixture, prediction):
    ctrl = Controller(build, fixture)
    ctrl.lib.online_destroy(ctrl.handle)
    ctrl.c.update(max_prediction=prediction, pos_unc=0., vel_unc=0., acc_unc=0.)
    params = np.asarray(list(ctrl.c.values()), dtype=float)
    body = np.ascontiguousarray(np.atleast_2d(np.loadtxt(fixture/'geometry.csv', delimiter=',')))
    ctrl.handle = ctrl.lib.online_create(ptr(params), ptr(body), len(body), 1)
    assert ctrl.handle
    ctrl.lib.online_selected_controls.argtypes = [ct.c_void_p, DP, ct.c_int]
    ctrl.lib.online_selected_controls.restype = ct.c_int
    ctrl.lib.online_search_trace.argtypes = [ct.c_void_p, DP, ct.c_int]
    ctrl.lib.online_search_trace.restype = ct.c_int
    ctrl.lib.online_decision_debug.argtypes = [ct.c_void_p, DP]
    return ctrl


def run(build, fixture, out):
    x = np.linspace(0., 3., 301)
    reference = np.ascontiguousarray(np.column_stack((x, x*0., x*0., np.minimum(.18, .8*(3.-x)))))
    checks = []
    horizon_results = []
    for prediction in (2., 3., 4.):
        ctrl = configured(build, fixture, prediction)
        try:
            state = np.array([.5, 0., 0., .18, 0.])
            obstacles = np.array([[2.1, 0., -.18, 0., .20, 1.]])
            command = ctrl.step(state, reference, obstacles)
            count = ctrl.lib.online_selected_controls(ctrl.handle, None, 0)
            expected = int(np.floor((prediction-.04-.01)/.1))
            assert count == expected, (prediction, count, expected)
            n = ctrl.lib.online_search_trace(ctrl.handle, None, 0)
            trace = np.zeros((n, 8))
            assert ctrl.lib.online_search_trace(ctrl.handle, ptr(trace), n) == n
            selected = trace[trace[:, 7] > .5]
            assert len(selected) == count+1 and np.all(selected[:, 6] > .5)
            assert selected[:, 1].max() <= prediction-.04
            horizon_results.append(dict(prediction_s=prediction, rollout_s=count*.1,
                                        command=command.tolist(), kernel_ms=ctrl.ms[-1]))
        finally:
            ctrl.close()
    checks.append('2/3/4s CV validity produces 1.9/2.9/3.9s actual control rollouts, with selected feasible search trace')
    ctrl = configured(build, fixture, 3.)
    try:
        # Remote motion outside the reference corridor cannot activate a turn
        # merely because its dynamic track exists.
        state = np.array([.5, 0., 0., .18, 0.])
        command = ctrl.step(state, reference, np.array([[2.4, 2., -.18, 0., .20, 1.]]))
        decision = np.zeros(12)
        ctrl.lib.online_decision_debug(ctrl.handle, ptr(decision))
        assert command[0] > 0 and abs(command[1]) < .05 and decision[6] == 0, (command, decision)
        checks.append('remote nonconflicting dynamic track keeps positive straight reference tracking')
        ctrl.lib.online_reset(ctrl.handle)
        reverse=reference.copy();reverse[:,0]*=-1;reverse[:,3]*=-1
        backward=ctrl.step(np.array([-.5,0.,0.,-.18,0.]),reverse,
                           np.array([[-2.4,2.,.18,0.,.20,1.]]))
        assert backward[0]<0 and abs(backward[1])<.05,backward
        checks.append('legitimate reverse reference remains admitted with remote dynamic targets')
        ctrl.lib.online_reset(ctrl.handle)
        # No candidate can make an occupied full-body map safe. The controller
        # must reject instead of forcing a side-pass or reducing the margin.
        ctrl.field.fill(0.)
        command = ctrl.step(state, reference, np.array([[2.1, 0., -.18, 0., .20, 1.]]))
        decision = np.zeros(12)
        ctrl.lib.online_decision_debug(ctrl.handle, ptr(decision))
        assert np.max(np.abs(command)) == 0. and decision[0] == 4, (command, decision)
        checks.append('fully occupied map rejects all dynamic passing proposals without forced motion')
    finally:
        ctrl.close()
    report = dict(passed=True, checks=checks, horizon_results=horizon_results,
                  scope='Controller contracts only; not a native REMANI or physical-robot safety validation.')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.build, args.fixture, args.out)
