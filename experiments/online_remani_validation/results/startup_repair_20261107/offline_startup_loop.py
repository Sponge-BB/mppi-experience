#!/usr/bin/python3
"""Offline startup cycle and independent 5 ms plant; no ROS graph or devices."""
import ast
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import resource
import sys
import threading
import time
from unittest.mock import patch

import numpy as np
import yaml

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
SCRIPTS = ROOT/'src/mm_remani/scripts'
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS/'mppi')]
from whole_body_mppi import WholeBodyNode, startup_plan
from active_arm_model import PiperArmModel
from native_reference_adapter import NativeReferenceAdapter
from standby_whole_body_mppi import StandbyWholeBodyMPPI


class Sink:
    def __init__(self):
        self.last = None
        self.count = 0

    def publish(self, message):
        self.last = message
        self.count += 1

    def get_num_connections(self):
        return 1


class ClearSnapshot:
    """Synthetic empty environment; constant 1 m external clearance."""
    header = np.zeros(32)

    def fresh(self, now):
        return True

    def query(self, spheres, times):
        return np.ones(len(spheres)), np.zeros(len(spheres))


def merge(dst, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            merge(dst[key], value)
        else:
            dst[key] = value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quantized', action='store_true', help='Apply the real driver 0.001 degree target rounding')
    quantized = parser.parse_args().quantized
    suffix = '_quantized' if quantized else ''
    cpu_start, wall_start = time.process_time(), time.monotonic()
    source = ROOT/'experiments/online_remani_validation/results/hardware_startup_check_20261107/runtime_feedback.json'
    recorded = json.loads(source.read_text())
    q = np.array(recorded['topics']['/joint_states_single']['position_rad'])
    target = np.deg2rad(recorded['startup_parameters']['/remani_planner_node/fsm/init_state'][2:])
    parameter_files = [ROOT/'src/mm_remani_planner/plan_manage/config'/name for name in
        ('exp0_param.yaml', 'piper_param.yaml', 'remani_planner_param.yaml', 'fixed_collision_spheres_b.yaml')]
    params = {}
    for path in parameter_files:
        merge(params, yaml.safe_load(path.read_text()))
    config_file = ROOT/'src/mm_remani/config/mppi.yaml'
    config = yaml.safe_load(config_file.read_text())
    model = PiperArmModel.from_parameters(params)
    model.lower[5], model.upper[5] = max(model.lower[5], -.35), min(model.upper[5], .35)
    kernel = StandbyWholeBodyMPPI(model, config['base_limits'],
        (model.lower, model.upper, config['arm_velocity'], config['arm_acceleration']), config['postures'],
        horizon=config['horizon'], step=1/config['rate_hz'], random_samples=config['random_samples'],
        arm_position_servo=False, config=config.get('standby', {}))
    plant_file = SCRIPTS/'standby_simulator.py'
    tree = ast.parse(plant_file.read_text())
    definitions = [item for item in tree.body if
        (isinstance(item, ast.ClassDef) and item.name == 'Plant') or
        (isinstance(item, ast.Assign) and any(isinstance(key, ast.Name) and
            key.id in ('Q_START', 'STEP') for key in item.targets))]
    namespace = dict(np=np, math=math)
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(plant_file), 'exec'), namespace)
    plant = namespace['Plant'](model, config['base_limits'], config['arm_velocity'], config['arm_acceleration'])
    plant.state = np.r_[np.zeros(5), q, np.zeros(6)]

    node = WholeBodyNode.__new__(WholeBodyNode)
    node.lock = threading.RLock(); node.execute = True; node.simulation = True
    node.kernel = kernel; node.model = model; node.limits = config['base_limits']
    node.base = plant.state[:5].copy(); node.joints = plant.state[5:].copy()
    node.snapshot = ClearSnapshot(); node.anchor = np.zeros(3); node.arm_anchor = q.copy()
    node.startup_complete = False; node.startup_target = target.copy()
    node.startup_enable_requested = False; node.startup_settled_since = None
    node.execution_revision = 0; node.execution_enabled = node.execution_latched = False
    node.safety_speed_limit = None; node.deadline = config['deadline_s']; node.last_visual = float('inf')
    node.arm_enabled = True; node.seed = 11; node.sent_command = False; node.sent_navigation_goal = None
    node.input_error = 'waiting_inputs'; node.reference = NativeReferenceAdapter(kernel)
    for name in ('base_pub', 'arm_pub', 'enable_pub', 'fault_pub', 'info_pub'):
        setattr(node, name, Sink())
    node.base_stamp = node.joint_stamp = 100.
    initial_gap = float(model.configuration_gaps_batch(q[None])[0])
    preflight, info = startup_plan(kernel, plant.state, target, node.snapshot, 100., preflight=True)
    if preflight is None:
        raise ValueError('initial_startup_preflight_infeasible')

    now, elapsed, last_arm = 100., 0., -1e9
    base_command = np.zeros(2); joint_command = None
    max_base = max_yaw = max_base_command = 0.
    min_gap, min_predicted_gap = initial_gap, info['min_gap']
    cycle_times = []; solve_times = []; outcomes = Counter(); reason = 'simulation_duration_limit'
    rows = []
    with patch('whole_body_mppi.rospy.get_time', side_effect=lambda: now), patch('whole_body_mppi.rospy.loginfo'):
        for index in range(int(120./kernel.dt)+1):
            if time.process_time()-cpu_start >= 55.:
                reason = 'offline_cpu_budget'; break
            elapsed = index*kernel.dt; now = 100.+elapsed
            node.base = plant.state[:5].copy(); node.joints = plant.state[5:].copy()
            node.base_stamp = node.joint_stamp = now
            before_arm = node.arm_pub.count
            began = time.perf_counter(); node.cycle(); cycle_times.append((time.perf_counter()-began)*1000.)
            status = json.loads(node.info_pub.last.data); outcomes[status['state']] += 1
            if 'solve_ms' in status: solve_times.append(status['solve_ms'])
            if 'min_gap' in status: min_predicted_gap = min(min_predicted_gap, status['min_gap'])
            if node.base_pub.last is not None:
                base_command = np.array([node.base_pub.last.linear.x, node.base_pub.last.angular.z])
            if node.arm_pub.count > before_arm:
                joint_command = np.asarray(node.arm_pub.last.position); last_arm = elapsed
                if quantized:
                    factor = 180000./math.pi
                    joint_command = np.array([round(float(value)*factor)/factor for value in joint_command])
            max_base_command = max(max_base_command, float(np.abs(base_command).max()))
            rows.append([elapsed, *plant.state, float(np.max(np.abs(plant.state[5:11]-target))), status['state']])
            if node.startup_complete:
                reason = 'startup_complete'; break
            states = []
            for substep in range(round(kernel.dt/.005)):
                held = joint_command if elapsed+substep*.005-last_arm < .30 else None
                plant.step(base_command, held, .005); states.append(plant.state.copy())
            states = np.array(states)
            gap = model.configuration_gaps_batch(states[:, 5:11])
            min_gap = min(min_gap, float(gap.min()))
            max_base = max(max_base, float(np.linalg.norm(states[:, :2], axis=1).max()))
            max_yaw = max(max_yaw, float(np.abs(states[:, 2]).max()))
            if (min_gap < 0. or np.any(states[:, 5:11] < model.lower) or np.any(states[:, 5:11] > model.upper)):
                reason = 'model_configuration_violation'; break

    def distribution(values):
        return dict(zip(('p50', 'p95', 'max'), map(float, np.percentile(values, [50, 95, 100])))) if values else None
    import csv
    with (OUT/('offline_startup_state'+suffix+'.csv')).open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t', 'x', 'y', 'yaw', 'v', 'w']+['q'+str(i) for i in range(1, 7)]+
                        ['dq'+str(i) for i in range(1, 7)]+['max_target_error_rad', 'status'])
        writer.writerows(rows)
    relevant = [Path(__file__).resolve(), source, config_file, plant_file, *parameter_files,
                SCRIPTS/'whole_body_mppi.py', SCRIPTS/'mppi/active_arm_mppi.py',
                SCRIPTS/'mppi/standby_whole_body_mppi.py', SCRIPTS/'mppi/active_arm_model.py',
                ROOT/'src/mm_remani/config/mppi_shoulder_wrist_proxy.json']
    report = dict(scope='Offline real WholeBodyNode.cycle/startup_plan/checked_first_step; fake ROS clock/publishers; no graph or hardware',
        environment='Synthetic clear static snapshot, constant external gap 1 m; no dynamic obstacles or perception',
        prediction='velocity_legacy, current hardware prediction settings; independent nominal position Plant, not hardware calibration',
        driver_target_quantization_deg=.001 if quantized else None,
        startup_complete=node.startup_complete, reason=reason, simulated_elapsed_s=elapsed,
        initial_q_rad=q.tolist(), target_q_rad=target.tolist(), final_state=plant.state.tolist(),
        final_max_target_error_rad=float(np.max(np.abs(plant.state[5:11]-target))),
        final_max_joint_speed_rad_s=float(np.max(np.abs(plant.state[11:17]))),
        max_base_displacement_m=max_base, max_base_yaw_rad=max_yaw, max_base_command_abs=max_base_command,
        initial_configuration_gap_m=initial_gap, min_configuration_gap_m=min_gap,
        min_predicted_gap_m=min_predicted_gap, external_gap_m=1., configuration_check_step_s=.005,
        cycle_count=len(cycle_times), statuses=dict(outcomes), arm_messages=node.arm_pub.count,
        enable_messages=node.enable_pub.count, configured_deadline_ms=node.deadline*1000.,
        cycle_ms=distribution(cycle_times), solve_ms=distribution(solve_times),
        cpu_s=time.process_time()-cpu_start, wall_s=time.monotonic()-wall_start,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
        source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in relevant},
        limitations=['No physical response calibration, CAN timing or enable/stop validation.',
                     'Fake ROS time advances 100 ms each cycle, not during computation; wall computation deadline remains checked.',
                     'Zero base arises from startup commands; Plant base remains freely integrated.',
                     'Configuration gaps are sampled model checks, not proof of continuous geometry or real contact.'])
    (OUT/('report'+suffix+'.json')).write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({key: report[key] for key in ('startup_complete', 'reason', 'simulated_elapsed_s',
        'final_max_target_error_rad', 'max_base_displacement_m', 'min_configuration_gap_m',
        'cycle_count', 'statuses', 'cycle_ms', 'cpu_s', 'peak_rss_mib')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
