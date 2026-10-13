#!/usr/bin/env python3
"""Deterministic model regressions with the exact publication validator.

This is NOT an online perception/planner experiment. Declared CV actors are
deliberately supplied as ideal measurements to isolate controller regressions.
Online performance evidence must come from validate_native_v12.py instead.
"""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from dynamic_scenarios import actors_at, body_clearance
from test_mppi_v12 import configured
from test_mppi_v2 import DP, ptr

HERE = Path(__file__).resolve().parent


def trial(build, fixture, scene, initial=None, dynamic_episode=False, recorded_reference=None):
    controller = configured(build, fixture, 3.)
    controller.lib.online_validate_execution.argtypes = [
        ct.c_void_p, DP, DP, ct.c_int, DP, ct.c_int, ct.c_double,
        ct.c_double, DP, ct.c_int, ct.c_int, ct.c_double, ct.c_double,
        ct.c_double, DP, ct.c_int, DP]
    yy = -5. + np.arange(200)[:, None] * .05
    xx = -7.5 + np.arange(300)[None, :] * .05
    controller.field = np.ascontiguousarray(
        np.minimum(2-abs(yy), np.minimum(xx+1.5, 4.5-xx))-.071)
    x = np.linspace(0., 3., 301)
    reference = np.ascontiguousarray(np.column_stack(
        (x, x*0., x*0., np.minimum(.18, .8*(3.-x)))))
    if recorded_reference is not None:
        messages = [json.loads(line) for line in
                    (recorded_reference/'consumed_references.jsonl').read_text().splitlines()]
        reference = np.ascontiguousarray(messages[-1]['reference'], dtype=float)
    body = np.atleast_2d(np.loadtxt(fixture/'geometry.csv', delimiter=','))
    state = np.zeros(5) if initial is None else np.asarray(initial, dtype=float)
    minimum_gap, lateral, rejections = 10., 0., {}
    settled = 0
    try:
        if dynamic_episode:
            # Exercise rejoin/endpoint after a dynamic episode, when the CV
            # stream has cleared. This remote track does not block the robot.
            controller.step(state, reference, np.array([[4., 1.5, -.18, 0., .20, 1.]]))
        for k in range(1000):
            t = k*.05
            actors = actors_at(scene, t)
            observations = np.ascontiguousarray([
                [o[0]-.04*o[4], o[1]-.04*o[5], o[4], o[5], o[2], 1.]
                for o in actors], dtype=float).reshape(-1, 6)
            command = controller.step(state, reference, observations)
            n = controller.lib.online_selected_controls(controller.handle, None, 0)
            controls = np.zeros((n, 2))
            controller.lib.online_selected_controls(controller.handle, ptr(controls), n)
            rows, report = np.zeros((650, 8)), np.zeros(6)
            code = controller.lib.online_validate_execution(
                controller.handle, ptr(state), ptr(controls), n, ptr(observations),
                len(observations), .04, 0., ptr(controller.field), 300, 200,
                -7.5, -5., .05, ptr(rows), len(rows), ptr(report))
            if code:
                command = np.zeros(2)
                controller.lib.online_feedback(controller.handle, 0., 0.)
                rejections[str(code)] = rejections.get(str(code), 0)+1
            controller.plant(state, command)
            minimum_gap = min(minimum_gap, body_clearance(
                state, body, actors_at(scene, t+.05)))
            lateral = max(lateral, abs(state[1]))
            at_goal = (np.linalg.norm(state[:2]-[3., 0.]) < .05
                       and abs(state[2]) < .1 and abs(state[3]) < .05)
            settled = settled+1 if at_goal else 0
            if minimum_gap <= 0 or settled >= 20:
                break
        return dict(scene=scene, passed=bool(minimum_gap > 0 and settled >= 20),
                    duration_s=(k+1)*.05, min_gap_m=minimum_gap,
                    max_lateral_m=lateral, rejections=rejections,
                    final_state=state.tolist(), initial=initial,
                    dynamic_episode=dynamic_episode,
                    recorded_reference=str(recorded_reference) if recorded_reference else None)
    finally:
        controller.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--fixture', type=Path, default=HERE/'results/final_report/empty_mppi')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    checks = [trial(args.build, args.fixture, scene)
              for scene in ('dynamic_single', 'dynamic_head_on_center', 'dynamic_four')]
    checks.append(trial(args.build, args.fixture, 'empty',
                        [3.13, .0, 2., .0, .0], dynamic_episode=True))
    stalled = HERE/'results/native_v12/validation_02/05_head_on_only_h3'
    if (stalled/'result.json').exists():
        initial = json.loads((stalled/'result.json').read_text())['final_state']
        checks.append(trial(args.build, args.fixture, 'empty', initial,
                            dynamic_episode=True, recorded_reference=stalled))
        for initial in ([3.13, .03, .9, 0., 0.], [3.19, .03, -.9, 0., 0.]):
            checks.append(trial(args.build, args.fixture, 'empty', initial,
                                dynamic_episode=True, recorded_reference=stalled))
    checks[0]['passed'] = bool(checks[0]['passed'] and checks[0]['max_lateral_m'] < .35)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    report = dict(passed=all(c['passed'] for c in checks), checks=checks,
                  scope='Ideal measured CV and fixed synthetic reference, differential plant, '
                        'exact publication validator; no ROS, no REMANI replanning, no detector. '
                        '50ms truth sampling is a regression check, not a safety proof.')
    encoded = json.dumps(report, indent=2)
    with args.out.open('x') as f:
        f.write(encoded+'\n')
    print(encoded, flush=True)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
