#!/usr/bin/env python3
"""Offline controller-layer comparison. Never initializes ROS or publishes commands."""
import argparse
import copy
import ctypes as ct
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import time
import types

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SCENES = 'empty,curve,reverse,new_static,crossing,delayed_crossing,blocked,late_intrusion,lag_mismatch'
DP = ct.POINTER(ct.c_double)
CALLBACK = ct.CFUNCTYPE(None, ct.c_int, ct.c_double, DP, DP)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Stamp:
    clock = 1000.0

    def __init__(self, value=0):
        self.value = float(value)

    @classmethod
    def now(cls):
        return cls(cls.clock)

    def to_sec(self):
        return self.value

    def is_zero(self):
        return self.value == 0

    def __sub__(self, other):
        return Stamp(self.value - other.value)


class MemoryPublisher:
    def __init__(self, *args, **kwargs):
        self.last = None

    def publish(self, message):
        self.last = copy.deepcopy(message)


class TrackerAdapter:
    """Execute the real Tracker class with a fake rospy module, no ROS transport.

    Real generated ROS message classes are only in-memory data containers.
    Reference generation and common obstacle guard belong to this experiment;
    neither is presented as production REMANI's FSM/replanning pipeline.
    """
    def __init__(self, effective):
        self.errors = []
        fake = types.ModuleType('rospy')
        fake.Time = Stamp
        fake.Duration = Stamp
        fake.Publisher = MemoryPublisher
        fake.get_param = lambda key, default=None: effective.get('/real_base_tracker/' + key.lstrip('~'), default)
        for name in ('init_node', 'Subscriber', 'Timer', 'on_shutdown', 'spin',
                     'loginfo', 'logwarn', 'logerr', 'logwarn_throttle', 'loginfo_throttle'):
            setattr(fake, name, lambda *args, **kwargs: None)
        fake.ROSInterruptException = RuntimeError
        previous = sys.modules.get('rospy')
        try:
            sys.modules['rospy'] = fake
            self.module = load_module('offline_original_tracker', ROOT/'src/mm_remani/scripts/real_base_tracker.py')
        finally:
            if previous is None:
                sys.modules.pop('rospy', None)
            else:
                sys.modules['rospy'] = previous
        assert self.module.rospy is fake
        self.tracker = None
        self.callback = CALLBACK(self.call)

    def call(self, reset, t, inputs, outputs):
        try:
            Stamp.clock = 1000.0 + t
            if reset:
                self.tracker = self.module.RealBaseTracker()
                outputs[0] = outputs[1] = outputs[2] = 0
                return
            x, y, yaw, v, w, rx, ry, ryaw, rv, rw = [inputs[i] for i in range(10)]
            odom = self.module.Odometry()
            odom.header.stamp = Stamp.now()
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z = math.sin(yaw/2)
            odom.pose.pose.orientation.w = math.cos(yaw/2)
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
            self.tracker.odom_cb(odom)
            reference = self.module.Twist()
            reference.linear.x, reference.linear.y, reference.linear.z = rx, ry, ryaw
            reference.angular.x, reference.angular.y = rv*math.cos(ryaw), rv*math.sin(ryaw)
            reference.angular.z = rw
            self.tracker.car_cmd_cb(reference)
            self.tracker.control_loop(None)
            cmd = self.tracker.cmd_pub.last
            outputs[0], outputs[1] = cmd.linear.x, cmd.angular.z
            outputs[2] = float(self.tracker.tracking_fault)
        except Exception as error:
            self.errors.append(repr(error))
            outputs[0] = outputs[1] = float('nan')
            outputs[2] = -999


def export_configuration(out):
    import roslaunch
    # This parses XML/YAML only. It does not launch nodes or contact ROS master.
    launch = ROOT/'src/mm_remani/launch/exp0_real.launch'
    cfg = roslaunch.config.load_config_default([str(launch)], 11311, verbose=False)
    params = {k: v.value for k, v in cfg.params.items()}
    (out/'effective_real_params.json').write_text(json.dumps(params, indent=2, ensure_ascii=False)+'\n')
    mm = '/remani_planner_node/mm/'
    dy = '/remani_planner_node/dynamic_point_flow/'
    tr = '/real_base_tracker/'
    op = '/remani_planner_node/optimization/'
    values = {
        'vmax': params[tr+'tracker_max_v'], 'wmax': params[tr+'tracker_max_w'],
        'accel': params[tr+'tracker_max_linear_accel'], 'angular_accel': params[tr+'tracker_max_angular_accel'],
        'track': params[mm+'mobile_base_wheel_base'],
        'wheel_v': params[mm+'mobile_base_wheel_radius']*params[mm+'mobile_base_max_wheel_omega'],
        'wheel_a': params[mm+'mobile_base_wheel_radius']*params[mm+'mobile_base_max_wheel_alpha'],
        'margin': max(params[op+'safe_margin'], params[op+'safe_margin_mani']),
        'max_age': params[dy+'max_age'], 'max_prediction': params[dy+'max_prediction_time'],
        'pos_unc': params[dy+'position_uncertainty'], 'vel_unc': params[dy+'velocity_uncertainty'],
        'acc_unc': params[dy+'acceleration_bound'],
        'goal_pos': params[tr+'goal_position_tolerance'], 'goal_yaw': params[tr+'goal_yaw_tolerance'],
        'goal_speed': params[tr+'goal_speed_tolerance'],
    }
    if any(not math.isfinite(float(v)) or v <= 0 for v in values.values()):
        raise ValueError('nonpositive/nonfinite configuration; review the adapter')
    # Reuse the existing Piper DH/geometry exporter, but reject any disagreement
    # with final launch overrides rather than silently using stale YAML defaults.
    import yaml
    path = ROOT/'src/mm_remani_planner/plan_manage/config'
    model = yaml.safe_load((path/'piper_param.yaml').read_text())['mm']
    init = yaml.safe_load((path/'exp0_param.yaml').read_text())['fsm']['init_state']
    for key in ('mobile_base_length', 'mobile_base_width', 'base_mani_fixed_joint_xyz_ypr', 'manipulator_thickness'):
        if params[mm+key] != model[key]:
            raise ValueError('geometry final override differs: '+key)
    if params['/remani_planner_node/fsm/init_state'] != init or params[mm+'use_fast_armer']:
        raise ValueError('fixed-pose geometry adapter requires the current Piper configuration')
    previous = load_module('legacy_geometry_exporter', ROOT/'experiments/mppi_cbf_validation/run.py')
    geometry = previous.geometry(out)
    geometry['manipulator_thickness'] = params[mm+'manipulator_thickness']
    (out/'geometry.json').write_text(json.dumps(geometry, indent=2)+'\n')
    (out/'limits.json').write_text(json.dumps(values, indent=2)+'\n')
    return params, values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', type=int, default=3)
    parser.add_argument('--samples', type=int, default=192)
    parser.add_argument('--scenarios', default=SCENES)
    parser.add_argument('--profiles', default='workspace,sensitivity')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--render-only', type=Path)
    args = parser.parse_args()
    if args.render_only:
        from visualize import render
        render(args.render_only.resolve())
        return
    if args.seeds < 1 or args.samples < 9:
        parser.error('seeds >= 1 and samples >= 9 required')
    if set(args.scenarios.split(',')) - set(SCENES.split(',')):
        parser.error('unknown scenario')
    if set(args.profiles.split(',')) - {'workspace', 'sensitivity'}:
        parser.error('unknown profile')
    out = (args.out or HERE/'results'/time.strftime('%Y%m%d_%H%M%S')).resolve()
    out.mkdir(parents=True, exist_ok=False)
    params, values = export_configuration(out)
    predict = ROOT/'src/mm_remani_planner/plan_env/src/dynamic_point_buffer.cpp'
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp', 'tf2_ros'], text=True))
    build = ['g++', '-std=c++17', '-O3', '-fPIC', '-shared', '-Wall', '-Wextra', '-Wpedantic',
             '-I/usr/include/eigen3', '-I'+str(predict.parents[1]/'include'),
             str(HERE/'core.cpp'), str(predict), '-o', str(out/'core.so')] + flags
    sources = list(HERE.glob('*.py'))+[HERE/'core.cpp', predict,
        ROOT/'src/mm_remani_planner/plan_env/include/plan_env/dynamic_point_buffer.h',
        ROOT/'src/mm_remani_planner/mm_config/src/mm_config.cpp',
        ROOT/'src/mm_remani/scripts/real_base_tracker.py',
        ROOT/'src/mm_remani/config/real_speed.yaml',
        ROOT/'experiments/mppi_cbf_validation/run.py']
    sources += list((ROOT/'src/mm_remani/launch').glob('*.launch'))
    sources += [ROOT/'src/mm_remani/launch/piper_advanced_param.xml']
    sources += list((ROOT/'src/mm_remani_planner/plan_manage/config').glob('*.yaml'))
    manifest = {
        'started': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'platform': platform.platform(),
        'command': sys.argv, 'build': build, 'seeds': args.seeds, 'samples': args.samples,
        'scenarios': args.scenarios.split(','), 'profiles': args.profiles.split(','),
        'sources_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        'control_dt': .05, 'rollout_dt': .1, 'horizon_steps': 15,
        'production_tracker_rate_hz': params['/real_base_tracker/control_rate'],
        'assumed_plant_tau': .12, 'mismatch_plant_tau': .28,
        'baseline': 'actual RealBaseTracker class; memory-only rospy; experiment reference/guard; no REMANI online replanning',
        'prediction': 'directly linked production DynamicPointBuffer::predictPosition/predictionUncertainty',
        'sensitivity_only': {'position_uncertainty': .02, 'velocity_uncertainty': .03, 'acceleration_bound': .02},
        'scope': 'offline synthetic closed loop, fixed arm XY cover, controller-layer comparison; NOT full REMANI/Gazebo/real-robot validation',
    }
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    subprocess.run(build, check=True)
    core = ct.CDLL(str(out/'core.so'))
    core.self_test.restype = ct.c_int
    if core.self_test() != 0:
        raise RuntimeError('core self-tests failed')
    adapter = TrackerAdapter(params)
    # Verify the fake transport before crossing the C callback boundary.
    inputs = (ct.c_double*10)(0,0,0,0,0,.07,0,0,.15,0)
    outputs = (ct.c_double*3)()
    adapter.call(1, 0, inputs, outputs)
    adapter.call(0, .05, inputs, outputs)
    assert 0 < outputs[0] <= values['accel']*.05+1e-9 and not adapter.errors
    core.run_comparison.argtypes = [ct.c_char_p, DP, ct.c_char_p, ct.c_int, ct.c_int, ct.c_char_p, ct.c_char_p, CALLBACK]
    core.run_comparison.restype = ct.c_int
    array = (ct.c_double*len(values))(*values.values())
    result = core.run_comparison(str(out).encode(), array, str(out/'geometry.csv').encode(),
                                 args.seeds, args.samples, args.scenarios.encode(), args.profiles.encode(), adapter.callback)
    if result or adapter.errors:
        raise RuntimeError(f'benchmark failed: code={result}, tracker errors={adapter.errors[:3]}')
    for relative, digest in manifest['sources_sha256'].items():
        if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest() != digest:
            raise RuntimeError('source changed during the run: '+relative)
    from verify import verify
    verify(out)
    from independent_audit import audit
    audit(out)
    from visualize import render
    render(out)
    print('REPORT:', out/'REPORT.md', flush=True)


if __name__ == '__main__':
    main()
