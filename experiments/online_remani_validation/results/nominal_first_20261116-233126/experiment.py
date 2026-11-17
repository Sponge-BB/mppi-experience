#!/usr/bin/python3
"""Isolated nominal-first algorithm benchmark. Never writes src or devel."""
import copy
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path
import resource
import statistics
import sys
import time

import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray

CASE = Path(__file__).resolve().parent
ROOT = CASE.parents[3]
SOURCE = ROOT/'src/mm_remani/scripts/mppi/active_arm_mppi.py'
TESTS = ROOT/'src/mm_remani/test'
NAV = ROOT/'experiments/online_remani_validation/results/dynamic_crossing_20261116-195203/navigation_segment'
sys.path[:0] = [str(SOURCE.parent), str(TESTS)]
import active_arm_mppi as deployed
from active_arm_model import PiperArmModel
import test_mppi_navigation_priority as fixtures


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copies():
    before = SOURCE.read_text()
    old = '''        cost,valid,gaps,states = self.evaluate(state,us,reference,arm_reference,snapshot,now,tracking=tracking)
        self.record_visual_rollouts(states, valid, reset=True)
        external_gaps=self.last_external_gaps.copy()
        intervention=not valid[0] or external_gaps[0]<.12
'''
    new = '''        cost,valid,gaps,states = self.evaluate(state,us[:1],reference,arm_reference,snapshot,now,tracking=tracking)
        external_gaps=self.last_external_gaps.copy()
        intervention=not valid[0] or external_gaps[0]<.12
        if intervention and len(us)>1:
            first_diagnostics=self.last_evaluation_diagnostics
            rc,rv,rg,rs=self.evaluate(state,us[1:],reference,arm_reference,snapshot,now,tracking=tracking)
            cost=np.r_[cost,rc];valid=np.r_[valid,rv];gaps=np.r_[gaps,rg]
            states=np.concatenate((states,rs),axis=0)
            external_gaps=np.r_[external_gaps,self.last_external_gaps]
            self.last_external_gaps=external_gaps
            self.last_evaluation_diagnostics={key:first_diagnostics[key]+value
                for key,value in self.last_evaluation_diagnostics.items()}
        self.record_visual_rollouts(states, valid, reset=True)
'''
    assert before.count(old) == 1
    after = before.replace(old, new)
    marker = "return (us[0],states[0]),{'feasible':int(valid.sum()),'candidates':len(us),"
    assert after.count(marker) == 1
    after = after.replace(marker, "return (us[0],states[0]),{'feasible':1,'candidates':1,")
    (CASE/'before.py').write_text(before)
    (CASE/'after.py').write_text(after)
    (CASE/'nominal_first.patch').write_text(''.join(difflib.unified_diff(
        before.splitlines(True), after.splitlines(True), fromfile=str(SOURCE), tofile=str(SOURCE))))
    result = []
    for label in ('before', 'after'):
        spec = importlib.util.spec_from_file_location('isolated_nominal_'+label, CASE/(label+'.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        # Both independent modules use the identical deployed optional native
        # library. Their experiment paths must not change library discovery.
        module._load_position_servo_library = deployed._load_position_servo_library
        module._load_legacy_velocity_library = deployed._load_legacy_velocity_library
        result.append(module)
    return result


def compare(a, b, path='', differences=None):
    differences = [] if differences is None else differences
    if isinstance(a, dict) and isinstance(b, dict):
        if a.keys() != b.keys():
            differences.append(dict(path=path, difference='keys', before=list(a), after=list(b)))
        for key in a.keys() & b.keys():
            compare(a[key], b[key], path+'/'+str(key), differences)
    elif isinstance(a, np.ndarray) and isinstance(b, np.ndarray):
        x, y = np.asarray(a), np.asarray(b)
        if x.shape != y.shape:
            differences.append(dict(path=path, difference='shape', before=list(x.shape), after=list(y.shape)))
        elif x.dtype.kind in 'fiu' and y.dtype.kind in 'fiu':
            if not np.array_equal(x, y):
                differences.append(dict(path=path, difference='numeric', max_abs=float(np.max(np.abs(x-y)))))
        elif not np.array_equal(x, y):
            differences.append(dict(path=path, difference='value'))
    elif isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            differences.append(dict(path=path, difference='length', before=len(a), after=len(b)))
        else:
            for index, (left, right) in enumerate(zip(a, b)):
                compare(left, right, path+'/'+str(index), differences)
    elif a != b:
        differences.append(dict(path=path, difference='value', before=a, after=b))
    return differences


def tracking_for(state, horizon, dt, speed):
    steps = np.arange(horizon+1)*dt
    samples = np.zeros((horizon+1, 6))
    samples[:, :2] = state[:2]+steps[:, None]*speed*np.array([np.cos(state[2]), np.sin(state[2])])
    samples[:, 2:4] = [state[2], speed]
    samples[:, 5] = abs(speed)
    return dict(identity=(100., 1), gear=1, base_samples=samples,
                resume_distance=.05, yaw_hold_speed=.04)


def analytic(module, servo, seed, kind):
    limits = dict(track=.4, vmax=.5, wmax=1., wheel_v=.8,
                  accel=1., angular_accel=2., wheel_a=1.5)
    kernel = module.WholeBodyMPPI(fixtures.TwoSphereRobot(), limits,
        (np.full(6, -1.), np.ones(6), np.full(6, .8), np.full(6, 3.)),
        [[-.6, 0., 0., 0., 0., 0.]], horizon=12, step=.1,
        random_samples=8, seed=seed, arm_position_servo=servo)
    state = np.zeros(17); state[3] = .16
    reference = np.c_[np.linspace(0., 1., 51), np.zeros((51, 2)), np.full(51, .16)]
    obstacle = fixtures.MovingSphere(10., .5)
    if kind == 'arm_only':
        # Separate geometric fixtures exercise the same branch in each
        # execution model; each before/after pair receives identical inputs.
        obstacle = fixtures.MovingSphere(.28, .57) if servo else fixtures.MovingSphere(.3, .5)
    if kind == 'fallback': obstacle = fixtures.MovingSphere(.12, 0.)
    if kind == 'no_feasible':
        obstacle = fixtures.MovingSphere(0., 0.); obstacle.initial[:] = 0.
    if kind == 'random_rescue':
        kernel.postures = np.empty((0, 6))
        kernel.limits['vmax'] = kernel.limits['wmax'] = .0001
        state[3] = 0.
        obstacle = fixtures.MovingSphere(.2, .57999)
    tracking = tracking_for(state, kernel.H, kernel.dt, .16) if kind == 'timed_tracking' else None
    return kernel, (state, reference, np.zeros(6), obstacle, 100.), dict(tracking=tracking)


def equivalence(modules):
    records = []
    for servo in (False, True):
        for seed in (11, 12, 13):
            for kind in ('clear', 'arm_only', 'fallback', 'no_feasible', 'random_rescue', 'timed_tracking'):
                pair = [analytic(m, servo, seed, kind) for m in modules]
                for round_id in range(2):
                    outputs = [k.plan(*args, **kw) for k, args, kw in pair]
                    a, b = outputs
                    if kind == 'arm_only':
                        assert a[1]['mode'] == 'arm_only' and a[0] is not None
                    if kind == 'fallback':
                        assert a[1]['mode'] == 'whole_body_fallback' and a[0] is not None
                    if kind == 'no_feasible':
                        assert a[0] is None
                    differences = compare(a, b)
                    extra = compare(pair[0][0].rng.get_state(), pair[1][0].rng.get_state(), '/rng')
                    extra += compare(pair[0][0].previous, pair[1][0].previous, '/previous')
                    for key in ('_tracking_capture', '_tracking_key', '_tracking_gear'):
                        extra += compare(getattr(pair[0][0], key), getattr(pair[1][0], key), '/'+key)
                    expected = {'/1/feasible', '/1/candidates'} if a[1]['mode'] == 'nominal' else set()
                    unexpected = [d for d in differences if d['path'] not in expected]+extra
                    record = dict(servo=servo, seed=seed, scenario=kind, round=round_id,
                                  mode=a[1]['mode'], feasible=a[1].get('feasible'),
                                  expected_metadata_differences=differences,
                                  unexpected_differences=unexpected)
                    records.append(record)
                    if unexpected:
                        (CASE/'equivalence_failure.json').write_text(json.dumps(record, indent=2))
                        raise AssertionError(record)
    return records


def real_case(module, model, config, state, snapshot, servo, now, seed=11):
    kernel = module.WholeBodyMPPI(model, dict(config['base_limits']),
        (model.lower, model.upper, config['arm_velocity'], config['arm_acceleration']),
        config['postures'], horizon=config['horizon'], step=1./config['rate_hz'],
        random_samples=config['random_samples'], seed=seed, arm_position_servo=servo)
    axis = np.array([np.cos(state[2]), np.sin(state[2])])
    reference = np.c_[state[:2]+np.linspace(0., 1., 51)[:, None]*axis,
                      np.full(51, state[2]), np.full(51, .12)]
    tracking = tracking_for(state, kernel.H, kernel.dt, .12)
    return kernel, (state, reference, state[5:11].copy(), snapshot, now), dict(tracking=tracking)


def benchmark(modules):
    selected = json.loads((CASE/'recorded_nominal_input.json').read_text())
    parameters_path = NAV/'latest_parameters.json'
    parameters = json.loads(parameters_path.read_text())['parameters']
    config = parameters['whole_body_mppi']
    model = PiperArmModel.from_parameters(parameters['remani_planner_node'])
    model.lower[5], model.upper[5] = -.35, .35
    raw_path = Path(selected['map_path'])
    snapshot = deployed.Snapshot.decode(numpy_msg(Float64MultiArray)().deserialize(raw_path.read_bytes()).data)
    current_path = raw_path.with_name(raw_path.name.replace('_full.msg', '_packet.msg'))
    snapshot = deployed.Snapshot.decode(numpy_msg(Float64MultiArray)().deserialize(current_path.read_bytes()).data, snapshot)
    state = np.asarray(selected['inputs']['measured_state'])
    report = []
    for servo in (False, True):
        warm = [real_case(m, model, config, state, snapshot, servo, selected['source']) for m in modules]
        native_loaded = [bool(k._servo_native if servo else k._legacy_native) for k, _, _ in warm]
        assert all(native_loaded), 'Native integration must not silently fall back in benchmark'
        outputs = [k.plan(*args, **kw) for k, args, kw in warm]
        differences = compare(*outputs)
        unexpected = [d for d in differences if d['path'] not in ('/1/feasible', '/1/candidates')]
        assert not unexpected, unexpected
        times = []
        for round_id in range(7):
            kernels = [real_case(m, model, config, state, snapshot, servo, selected['source']) for m in modules]
            elapsed = [None, None]
            order = (0, 1) if round_id % 2 == 0 else (1, 0)
            observed = [None, None]
            for index in order:
                kernel, args, kwargs = kernels[index]
                began = time.perf_counter()
                observed[index] = kernel.plan(*args, **kwargs)
                elapsed[index] = (time.perf_counter()-began)*1000.
            assert compare(*observed) == differences
            times.append(dict(round=round_id, order=['before' if i == 0 else 'after' for i in order],
                              before_ms=elapsed[0], after_ms=elapsed[1], saved_ms=elapsed[0]-elapsed[1]))
        before_ms = statistics.median(p['before_ms'] for p in times)
        after_ms = statistics.median(p['after_ms'] for p in times)
        report.append(dict(servo=servo, mode=outputs[0][1]['mode'],
            output_metadata_differences=differences,
            nominal_info_before=outputs[0][1], nominal_info_after=outputs[1][1],
            native_integrators_loaded=native_loaded,
            pairs=times, before_median_ms=before_ms, after_median_ms=after_ms,
            median_pair_saved_ms=statistics.median(p['saved_ms'] for p in times),
            ratio_after_before=after_ms/before_ms))
    return dict(input_state=state.tolist(), recorded_cycle_state=selected['state'],
        recorded_cycle_mode=selected['inputs']['mode'], recorded_plan_time=selected['source'],
        map=dict(path=str(raw_path), sha256=digest(raw_path), packet_path=str(current_path),
                 packet_sha256=digest(current_path), shape=list(snapshot.distance.shape),
                 field=float(snapshot.header[14]), fusion_stamp=float(snapshot.header[3])),
        parameters_path=str(parameters_path), parameters_sha256=digest(parameters_path),
        configuration=config, native_library_path=str(ROOT/'devel/lib/libmppi_position_servo.so'),
        native_library_sha256=digest(ROOT/'devel/lib/libmppi_position_servo.so'),
        saved_snapshot_header_equals_status_header=bool(np.array_equal(snapshot.header,
            np.asarray(selected['inputs']['snapshot_header']))), tests=report,
        boundary='Recorded measured state and actual saved map with actual runtime parameters; the straight reference and timed tracking samples are constructed. This is an algorithm benchmark, not exact-reference replay or closed-loop/robot evidence.')


def dynamic_benchmark(modules):
    records = []
    for servo in (False, True):
        for kind in ('arm_only', 'fallback', 'no_feasible'):
            for module in modules:
                kernel, args, kwargs = analytic(module, servo, 11, kind)
                kernel.plan(*args, **kwargs)
            pairs = []
            for round_id in range(5):
                kernels = [analytic(module, servo, 11, kind) for module in modules]
                times, outputs = [None, None], [None, None]
                order = (0, 1) if round_id % 2 == 0 else (1, 0)
                for index in order:
                    kernel, args, kwargs = kernels[index]
                    began = time.perf_counter()
                    outputs[index] = kernel.plan(*args, **kwargs)
                    times[index] = (time.perf_counter()-began)*1000.
                assert not compare(*outputs)
                pairs.append(dict(round=round_id, order=['before' if i == 0 else 'after' for i in order],
                                  before_ms=times[0], after_ms=times[1], added_ms=times[1]-times[0]))
            before_ms = statistics.median(p['before_ms'] for p in pairs)
            after_ms = statistics.median(p['after_ms'] for p in pairs)
            records.append(dict(scenario=kind, servo=servo, pairs=pairs,
                before_median_ms=before_ms, after_median_ms=after_ms,
                median_pair_added_ms=statistics.median(p['added_ms'] for p in pairs),
                ratio_after_before=after_ms/before_ms))
    return dict(boundary='Analytic two-sphere branch fixtures, not actual-map or robot latency.', tests=records)


def main():
    start_hash = digest(SOURCE)
    modules = copies()
    cases = equivalence(modules)
    real = benchmark(modules)
    dynamic = dynamic_benchmark(modules)
    assert digest(SOURCE) == start_hash
    summary = dict(result='PASS', production_unchanged=True, source_sha256=start_hash,
        before_sha256=digest(CASE/'before.py'), after_sha256=digest(CASE/'after.py'),
        equivalence=cases, real_input_benchmark=real, dynamic_branch_benchmark=dynamic,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
        limitations=['Nominal candidate/feasible counters and optional visual rollout counts intentionally describe only the one actually evaluated candidate.',
                     'Finite fixture/seed equivalence is regression evidence, not a proof over all robot configurations.',
                     'No ROS master, actuator I/O, production edit, devel build or closed-loop experiment was performed.'])
    (CASE/'result.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(result='PASS', equivalence_cases=len(cases),
        real_perf=[{key:item[key] for key in ('servo', 'mode', 'before_median_ms',
            'after_median_ms', 'median_pair_saved_ms', 'ratio_after_before')} for item in real['tests']],
        dynamic_perf=[{key:item[key] for key in ('servo', 'scenario', 'before_median_ms',
            'after_median_ms', 'median_pair_added_ms', 'ratio_after_before')} for item in dynamic['tests']],
        peak_rss_mib=summary['peak_rss_mib']), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
