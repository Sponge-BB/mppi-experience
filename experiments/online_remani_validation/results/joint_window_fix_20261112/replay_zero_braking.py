#!/usr/bin/env python3
"""Five recorded states/maps, one identical zero-intent candidate per state.

No ROS init/master, bag scan, control publication or runtime-file changes.
This is a counterfactual velocity-input comparison, NOT replay of the original
navigation reference, candidate set, previous control or complete execution.
"""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import ast
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import resource
import sys
import time

import numpy as np
from std_msgs.msg import Float64MultiArray

ROOT = Path('/home/l/remani_ws')
OUT = Path(__file__).resolve().parent
SOURCE = OUT.parent / 'drive_session_20261112-120325'
sys.path.insert(0, str(ROOT/'src/mm_remani/scripts/mppi'))
from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot, WholeBodyMPPI
from standby_whole_body_mppi import StandbyWholeBodyMPPI


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_estimator():
    """Execute the actual pure class without importing the ROS node module."""
    path = ROOT/'src/mm_remani/scripts/filter_joint_states.py'
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                and n.name == 'SourceTimeVelocityWindow')
    namespace = {'math': math, 'deque': deque}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[node.name]


def snapshot_for(seq, header):
    full = next((SOURCE/'map_events').glob('*_seq%d_*_full.msg' % seq))
    data = Float64MultiArray().deserialize(full.read_bytes()).data
    snapshot = Snapshot.decode(data)
    packet = list((SOURCE/'map_events').glob('*_seq%d_*_packet.msg' % seq))
    if packet:
        packet = packet[0]
        snapshot = Snapshot.decode(Float64MultiArray().deserialize(packet.read_bytes()).data,
                                   snapshot)
    assert np.array_equal(snapshot.header, header), 'Exact failed-cycle header required'
    assert not snapshot.groups, 'These five recorded events have no dynamic groups'
    return snapshot, [str(p.relative_to(SOURCE)) for p in ([full]+([packet] if packet else []))]


def evaluate(kernel, state, snapshot, now):
    # Candidate is all-zero velocity intent for both base and arm. Navigation
    # reference is a stationary placeholder and changes cost, not acceptance.
    reference = np.tile(np.r_[state[:3], 0.], (kernel.H+1, 1))
    controls = np.zeros((1, kernel.H, 8))
    _, valid, gaps, predicted = kernel.evaluate(
        state, controls, reference, state[5:11].copy(), snapshot, now)
    return dict(valid=bool(valid[0]), min_gap_m=float(gaps[0]),
                external_gap_m=float(kernel.last_external_gaps[0]),
                horizon_final_q=predicted[0, -1, 5:11].tolist(),
                **{k: v[0] for k, v in kernel.last_evaluation_diagnostics.items()})


def main():
    started = time.monotonic()
    dq_doc = json.loads((SOURCE/'dq_root_cause_analysis.json').read_text())
    original = json.loads((SOURCE/'original_geometry_failure_audit.json').read_text())
    events = {e['seq']: e for e in original['events']}
    params = json.loads((SOURCE/'runtime_parameters.json').read_text())
    mppi = params['/whole_body_mppi']
    model = PiperArmModel.from_parameters(params['/remani_planner_node'])
    model.lower[5] = max(model.lower[5], -.35)
    model.upper[5] = min(model.upper[5], .35)
    estimator_type = load_estimator()
    kernels = {}
    for phase, cls in [('standby', StandbyWholeBodyMPPI), ('navigation', WholeBodyMPPI)]:
        kernels[phase] = cls(model, mppi['base_limits'],
            (model.lower, model.upper, mppi['arm_velocity'], mppi['arm_acceleration']),
            mppi['postures'], enabled=True, seed=11, horizon=mppi['horizon'],
            step=1./mppi['rate_hz'], random_samples=mppi['random_samples'],
            arm_position_servo=False)
    rows = []
    for event in dq_doc['events']:
        seq = event['seq']
        recorded = events[seq]['diagnostics']
        state = np.asarray(recorded['measured_state'], dtype=float)
        matched = event['matched_sample']
        assert np.array_equal(state[5:11], matched['q'])
        assert np.array_equal(state[11:17], matched['dq'])
        estimator = estimator_type(.05, .15)
        result = None
        supplied = []
        for sample in sorted(event['observed_samples'], key=lambda s: s['source_ros']):
            if sample['source_ros'] > matched['source_ros']:
                break
            supplied.append(sample)
            value = estimator.update(sample['source_ros'], sample['q'])
            if sample['source_ros'] == matched['source_ros']:
                result = value
        assert result is not None, 'A complete causal window must exist'
        new_state = state.copy()
        new_state[11:17] = result
        snapshot, files = snapshot_for(seq, recorded['snapshot_header'])
        phase = 'standby' if event['phase'] == 'standby' else 'navigation'
        kernel = kernels[phase]
        rows.append(dict(seq=seq, phase=phase, estimator_stamp=matched['source_ros'],
            same_q=True, same_base=True, same_map_header=True, map_files=files,
            observed_samples_before_or_at_event=len(supplied),
            max_observed_source_gap_s=max(b['source_ros']-a['source_ros']
                for a, b in zip(supplied, supplied[1:])),
            original_dq=state[11:17].tolist(), window_dq=result,
            original=evaluate(kernel, state, snapshot, recorded['plan_time']),
            window=evaluate(kernel, new_state, snapshot, recorded['plan_time'])))
    # Sustained actual-rate fixture ends at the same recorded navigation pose;
    # 1.2 rad/s is not clipped to the 0.15 rad/s commanded-velocity limit.
    recorded = events[10468]['diagnostics']
    state = np.array(recorded['measured_state'], dtype=float)
    estimator = estimator_type(.05, .15)
    epoch = recorded['plan_time']
    end = epoch+.30
    for index in range(61):
        stamp = epoch+.005*index
        q = state[5:11].copy()
        q[3] += 1.2*(stamp-end)
        dq = estimator.update(stamp, q.tolist())
    assert abs(dq[3]-1.2) < 1e-5
    sustained = state.copy()
    sustained[11:17] = dq
    snapshot, _ = snapshot_for(10468, recorded['snapshot_header'])
    synthetic = dict(kind='synthetic constant joint4 speed ending at recorded pose',
        expected_dq_rad_s=1.2, estimated_dq=dq,
        evaluation=evaluate(kernels['navigation'], sustained, snapshot, recorded['plan_time']))
    assert not synthetic['evaluation']['valid']
    assert not synthetic['evaluation']['braking_stopped']
    paths = ['src/mm_remani/scripts/filter_joint_states.py',
        'src/mm_remani/scripts/mppi/active_arm_model.py',
        'src/mm_remani/scripts/mppi/active_arm_mppi.py',
        'src/mm_remani/scripts/mppi/standby_whole_body_mppi.py',
        'src/my_description/robot/mm.urdf',
        'src/mm_remani/config/mppi_shoulder_wrist_proxy.json',
        'devel/lib/libmppi_position_servo.so']
    report = dict(scope=__doc__, estimator_window_s=.05,
        acceptance='Actual phase-specific evaluate method, actual recorded map and state; only dq changes.',
        limitations=[
            'Not an original full MPPI candidate/reference/previous-command replay.',
            'Recorded observed_samples may omit messages; interpolation uses only saved source-time positions.',
            'Conditioned velocity is an estimate, not proof of true physical speed.',
            'No wall-clock freshness gates or hardware execution exercised.',
            'Stationary reference affects cost only; no cost improvement claim.'],
        events=rows, synthetic_sustained_speed=synthetic,
        summary=dict(original_valid=sum(e['original']['valid'] for e in rows),
                     window_valid=sum(e['window']['valid'] for e in rows),
                     cases=len(rows)),
        parameters=dict(horizon=mppi['horizon'], step=1./mppi['rate_hz'],
                        qmin=model.lower.tolist(), qmax=model.upper.tolist(),
                        arm_acceleration=mppi['arm_acceleration'],
                        arm_command_velocity=mppi['arm_velocity'],
                        arm_position_servo=False),
        code_sha256={p: sha(ROOT/p) for p in paths},
        input_sha256={p: sha(SOURCE/p) for p in ('dq_root_cause_analysis.json',
                        'original_geometry_failure_audit.json', 'runtime_parameters.json')},
        wall_seconds=time.monotonic()-started,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.)
    (OUT/'zero_braking_counterfactual.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(summary=report['summary'], rows=[dict(seq=e['seq'],
        old_max_dq=max(map(abs,e['original_dq'])), new_max_dq=max(map(abs,e['window_dq'])),
        old=e['original'], new=e['window']) for e in rows],
        synthetic=synthetic, wall_seconds=report['wall_seconds'], peak_rss_mib=report['peak_rss_mib']),
        indent=2))


if __name__ == '__main__':
    main()
