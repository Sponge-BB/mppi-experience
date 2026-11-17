#!/usr/bin/env python3
"""Prescribed arm-only geometry witness on a recorded navigation base path."""
import ast
from collections import deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import sys
import time

for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
SOURCE = OUT/'empty_clockfix_s11'
SCRIPTS = ROOT/'src/mm_remani/scripts'
sys.path.insert(0, str(SCRIPTS/'mppi'))
from active_arm_model import PiperArmModel, DEFAULT_PROXY, DEFAULT_URDF
from active_arm_mppi import WholeBodyMPPI


def definitions():
    path = SCRIPTS/'standby_simulator.py'
    names = {'Plant', 'Q_START', 'STEP', 'MEASUREMENT_DELAY'}
    selected = [node for node in ast.parse(path.read_text()).body if
        (isinstance(node, ast.ClassDef) and node.name in names) or
        (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in node.targets))]
    namespace = {'np': np, 'math': math}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def main():
    began = time.monotonic(); pure = definitions(); step = pure['STEP']
    profile = json.loads((SOURCE/'experiment.json').read_text())['controller_profile']
    model = PiperArmModel.from_parameters(SOURCE/'planner_params.yaml')
    model.lower[5] = max(model.lower[5], -.35); model.upper[5] = min(model.upper[5], .35)
    rows = np.genfromtxt(SOURCE/'state.csv', delimiter=',', names=True)
    ts = np.arange(round(float(rows['t'][-1])/step)+1)*step
    base = np.column_stack([np.interp(ts, rows['t'], np.unwrap(rows[key]) if key == 'yaw' else rows[key])
                            for key in ('x', 'y', 'yaw', 'v', 'w')])
    assert np.all(np.diff(rows['t']) > 0) and ts[-1] <= rows['t'][-1]+1e-9
    kernel = WholeBodyMPPI(model, profile['base_limits'],
        (model.lower, model.upper, profile['arm_velocity'], profile['arm_acceleration']),
        profile['postures'], step=1/profile['rate_hz'], arm_position_servo=True)
    trajectory = np.tile(np.r_[np.zeros(5), pure['Q_START'], np.zeros(6)], (len(ts), 1))
    trajectory[:, :5] = base
    cases = {'hold_q_start': (trajectory.copy(), np.zeros((len(ts), 6)))}
    plant = pure['Plant'](model, profile['base_limits'], kernel.vmax, kernel.amax)
    history = deque([plant.state.copy() for _ in range(5)], maxlen=5)
    start = 2.365; start_tick = round(start/step); period = round(kernel.dt/step)
    target = None; acceleration = np.zeros((len(ts), 6))
    for tick in range(len(ts)-1):
        plant.state[:5] = base[tick]
        trajectory[tick, 5:] = plant.state[5:]
        if tick >= start_tick and (tick-start_tick) % period == 0:
            measured = history[0]
            intent = np.r_[np.zeros(2), np.clip(1.8*(profile['postures'][0]-measured[5:11])
                -.2*measured[11:17], -kernel.vmax, kernel.vmax)]
            target = kernel.position_target(measured, intent)
        acceleration[tick+1] = plant.step(np.zeros(2), target, step)
        # Base pose is imposed from the recorded trajectory, not re-planned.
        plant.state[:5] = base[tick+1]
        history.append(plant.state.copy())
    trajectory[-1, 5:] = plant.state[5:]
    cases['prescribed_retract_2p365s'] = (trajectory, acceleration)
    report = dict(scope='Offline geometric feasibility witness, not an MPPI/ROS/hardware result.',
        base='Recorded empty_clockfix_s11 x/y/yaw/v/w unchanged; 20ms samples linearly interpolated to5ms; first15ms hold first sample.',
        scene=dict(x=.45, y_at_2s=.65, vy=-.08, radius=.13, z_min=1.05, z_max=1.30, starts_s=2.),
        policy='Q_START hold versus prescribed posture_0 at2.365s,10Hz position targets,20ms feedback delay,independent5ms Plant. Start is borrowed from prior witness, not measured detection in this navigation scene.',
        audit='67 physical spheres, finite cylinder, x+/-1.2/y+/-1.4 walls, native self/base-arm/ground configuration model and joint position/rate limits; no sensor/ESDF acceptance or mesh certificate; discrete5ms only.',
        integration_step_s=step, duration_s=float(ts[-1]), start_command_s=start,
        maximum_recorded_base_speed_mps=float(np.max(np.abs(base[:, 3]))),
        recorded_base_path_length_m=float(np.sum(np.linalg.norm(np.diff(base[:, :2], axis=0), axis=1))),
        final_recorded_base_pose=base[-1, :3].tolist(), cases={})
    with (OUT/'scene_audit.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case', 't', 'x', 'y', 'yaw']+['q%d' % j for j in range(1, 7)]+
                        ['cylinder_fixed_gap_m', 'cylinder_arm_gap_m', 'wall_gap_m', 'configuration_gap_m'])
        for name, (states, ddq) in cases.items():
            gaps = np.empty((len(ts), 4))
            for first in range(0, len(ts), 128):
                last = min(first+128, len(ts)); s = states[first:last]
                spheres = model.body_spheres_batch(s[:, 5:11], s[:, :3]); t = ts[first:last]
                radial = np.hypot(spheres[:, :, 0]-.45, spheres[:, :, 1]-(.65-.08*(t[:, None]-2.)))-.13
                vertical = np.abs(spheres[:, :, 2]-1.175)-.125
                cylinder = np.hypot(np.maximum(radial, 0.), np.maximum(vertical, 0.))+np.minimum(np.maximum(radial, vertical), 0.)-spheres[:, :, 3]
                cylinder[t < 2.] = np.inf
                nf = len(model.fixed_spheres)
                gaps[first:last, 0] = cylinder[:, :nf].min(axis=1)
                gaps[first:last, 1] = cylinder[:, nf:].min(axis=1)
                gaps[first:last, 2] = (np.minimum(1.2-np.abs(spheres[:, :, 0]), 1.4-np.abs(spheres[:, :, 1]))-spheres[:, :, 3]).min(axis=1)
                gaps[first:last, 3] = model.configuration_gaps_batch(s[:, 5:11])
            external = gaps[:, :3].min(axis=1)
            hit = np.flatnonzero(external < 0.); minimum = int(np.argmin(external))
            q_margin = float(np.min(np.minimum(states[:, 5:11]-model.lower, model.upper-states[:, 5:11])))
            velocity_margin = float(np.min(kernel.vmax-np.abs(states[:, 11:17])))
            acceleration_margin = float(np.min(kernel.amax-np.abs(ddq)))
            valid = not len(hit) and gaps[:, 3].min() >= -1e-10 and min(q_margin, velocity_margin, acceleration_margin) >= -1e-10
            report['cases'][name] = dict(feasible_sampled_witness=bool(valid),
                first_external_collision_s=None if not len(hit) else float(ts[hit[0]]),
                minimum_external_gap_m=float(external[minimum]), minimum_external_time_s=float(ts[minimum]),
                minimum_cylinder_fixed_gap_m=float(gaps[:, 0].min()), minimum_cylinder_arm_gap_m=float(gaps[:, 1].min()),
                minimum_wall_gap_m=float(gaps[:, 2].min()), minimum_configuration_gap_m=float(gaps[:, 3].min()),
                minimum_joint_position_margin_rad=q_margin, minimum_velocity_margin_rad_s=velocity_margin,
                minimum_acceleration_margin_rad_s2=acceleration_margin,
                max_arm_change_deg=float(np.rad2deg(np.max(np.abs(states[:, 5:11]-pure['Q_START'])))),
                maximum_base_deviation_from_recorded_m=float(np.max(np.linalg.norm(states[:, :2]-base[:, :2], axis=1))))
            for index in range(0, len(ts), 10):
                writer.writerow([name, ts[index], *states[index, :3], *states[index, 5:11], *gaps[index]])
    files = [Path(__file__).resolve(), SOURCE/'state.csv', SOURCE/'experiment.json', SOURCE/'planner_params.yaml',
             SCRIPTS/'standby_simulator.py', SCRIPTS/'mppi/active_arm_model.py', SCRIPTS/'mppi/active_arm_mppi.py', DEFAULT_PROXY, DEFAULT_URDF]
    report['source_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    report['wall_s'] = time.monotonic()-began
    report['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    (OUT/'scene_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: report[key] for key in ('cases', 'wall_s', 'peak_rss_mib')}, indent=2))


if __name__ == '__main__':
    main()
