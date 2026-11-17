#!/usr/bin/env python3
"""Two prescribed arm-only witnesses; no MPPI solve, ROS, map, or hardware."""
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

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
resource.setrlimit(resource.RLIMIT_AS, (300*1024**2, 300*1024**2))
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[4]
SOURCE = OUT.parent/'fast_zero_s11'
SCRIPTS = ROOT/'src/mm_remani/scripts'
sys.path.insert(0, str(SCRIPTS/'mppi'))
from active_arm_model import PiperArmModel, DEFAULT_PROXY, DEFAULT_URDF
from active_arm_mppi import WholeBodyMPPI


def load_plant_definitions():
    source = SCRIPTS/'standby_simulator.py'
    names = {'Plant', 'StandbySimulator', 'actor_at', 'cylinder_gap',
             'Q_START', 'STEP', 'MEASUREMENT_DELAY', 'COMMAND_WATCHDOG'}
    selected = []
    for node in ast.parse(source.read_text()).body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names:
            selected.append(node)
        elif isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in node.targets):
            selected.append(node)
    namespace = {'np': np, 'math': math}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


def triggers():
    found = {}
    for line, text in enumerate((SOURCE/'dynamic_observations.jsonl').open(), 1):
        row = json.loads(text); h = row['header']
        if row['elapsed_s'] is not None and row['points'] and h[4] == 1 and h[19] == 1:
            found['first_dynamic_received'] = dict(elapsed_s=row['elapsed_s'], line=line,
                file='dynamic_observations.jsonl', received_ros_time=row['ros_time'],
                source_ros_time=h[18], snapshot_header=h, observed_points=len(row['points']))
            break
    for line, text in enumerate((SOURCE/'mppi_status.jsonl').open(), 1):
        row = json.loads(text); status = row['status']
        if (row['elapsed_s'] is not None and status.get('state') == 'command_delivered'
                and (status.get('selected_family') or [''])[0] == 'posture_0'):
            found['first_posture_delivery'] = dict(elapsed_s=row['elapsed_s'], line=line,
                file='mppi_status.jsonl', received_ros_time=row['ros_time'],
                selected_family=status['selected_family'], selection=status.get('selection'))
            break
    if len(found) != 2:
        raise RuntimeError('Both recorded trigger events are required; no substitute times allowed')
    return found


def run(name, trigger, definitions, model, profile):
    step = definitions['STEP']; duration = 24.
    plant = definitions['Plant'](model, profile['base_limits'], profile['arm_velocity'], profile['arm_acceleration'])
    kernel = WholeBodyMPPI(model, profile['base_limits'],
        (model.lower, model.upper, profile['arm_velocity'], profile['arm_acceleration']),
        profile['postures'], step=1/profile['rate_hz'], arm_position_servo=True)
    posture = np.asarray(profile['postures'][0])
    # Call the production simulator's existing audit verbatim, without ROS init.
    audit = object.__new__(definitions['StandbySimulator'])
    audit.plant = plant; audit.model = model; audit.first_violation = None
    audit.minimum_external = audit.minimum_configuration = float('inf')
    delay = definitions['MEASUREMENT_DELAY']
    history = deque([plant.state.copy() for _ in range(1+round(delay/step))],
                    maxlen=1+round(delay/step))
    start_tick = int(math.ceil(trigger['elapsed_s']/step-1e-8))
    period_ticks = int(round(1/profile['rate_hz']/step))
    target = None; ddq = np.zeros(6); last_send = -1e9; command_count = 0
    minimum_q_margin = float('inf'); max_dq = np.zeros(6); max_ddq = np.zeros(6)
    max_arm = max_base = max_yaw = 0.; min_external_t = None; csv_rows = 0
    previous_minimum = float('inf'); failure = None; samples = 0
    with (OUT/(name+'.csv')).open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t','x','y','yaw']+['q%d'%j for j in range(1,7)]+
            ['dq%d'%j for j in range(1,7)]+['ddq%d'%j for j in range(1,7)]+
            ['external_gap_m','configuration_gap_m','command_active'])
        for tick in range(round(duration/step)+1):
            elapsed = tick*step
            actor = definitions['actor_at'](elapsed, 'arm_crossing')
            external, configuration, failure = audit.audit(elapsed, actor, ddq)
            samples += 1; z = plant.state
            if external < previous_minimum:
                previous_minimum = external; min_external_t = elapsed
            minimum_q_margin = min(minimum_q_margin, float(np.min(np.minimum(z[5:11]-model.lower,model.upper-z[5:11]))))
            max_dq = np.maximum(max_dq,np.abs(z[11:17])); max_ddq = np.maximum(max_ddq,np.abs(ddq))
            max_arm = max(max_arm,float(np.max(np.abs(z[5:11]-definitions['Q_START']))))
            max_base = max(max_base,float(np.linalg.norm(z[:2]))); max_yaw = max(max_yaw,abs(float(z[2])))
            if tick % 10 == 0 or failure or elapsed >= duration:
                writer.writerow([elapsed,*z[:3],*z[5:17],*ddq,external,configuration,int(target is not None)])
                csv_rows += 1
            if failure or elapsed >= duration:
                break
            if tick >= start_tick and (tick-start_tick) % period_ticks == 0:
                state = history[0]
                control = np.r_[np.zeros(2),np.clip(1.8*(posture-state[5:11])-.2*state[11:17],-kernel.vmax,kernel.vmax)]
                target = kernel.position_target(state,control)
                if np.any(target < model.lower) or np.any(target > model.upper):
                    raise RuntimeError('out-of-bounds prescribed position target')
                last_send = elapsed; command_count += 1
            applied = target if elapsed-last_send < definitions['COMMAND_WATCHDOG'] else None
            ddq = plant.step(np.zeros(2),applied,step)
            history.append(plant.state.copy())
    return dict(trigger=trigger, first_command_elapsed_s=start_tick*step,
        completed=failure is None and elapsed >= duration, reason=failure or 'completed',
        elapsed_s=elapsed, audit_samples=samples, csv_rows=csv_rows,
        min_external_gap_m=audit.minimum_external, min_external_gap_elapsed_s=min_external_t,
        min_configuration_gap_m=audit.minimum_configuration,
        min_joint_position_margin_rad=minimum_q_margin,
        max_abs_joint_velocity_rad_s=max_dq.tolist(), max_abs_joint_acceleration_rad_s2=max_ddq.tolist(),
        max_arm_change_deg=math.degrees(max_arm), max_base_displacement_m=max_base,
        max_base_yaw_rad=max_yaw, commands=command_count, final_state=plant.state.tolist(),
        final_posture_error_deg=np.rad2deg(plant.state[5:11]-posture).tolist(),
        first_violation=audit.first_violation, trajectory_csv=name+'.csv')


if __name__ == '__main__':
    began = time.monotonic(); definitions = load_plant_definitions()
    profile = json.loads((SOURCE/'experiment.json').read_text())['controller_profile']
    model = PiperArmModel.from_parameters(SOURCE/'planner_params.yaml')
    model.lower[5] = max(model.lower[5],-.35); model.upper[5] = min(model.upper[5],.35)
    files = [Path(__file__).resolve(), SCRIPTS/'standby_simulator.py', SCRIPTS/'mppi/active_arm_model.py',
             SCRIPTS/'mppi/active_arm_mppi.py', DEFAULT_PROXY, DEFAULT_URDF,
             SOURCE/'planner_params.yaml', SOURCE/'experiment.json',
             SOURCE/'dynamic_observations.jsonl', SOURCE/'mppi_status.jsonl']
    report = dict(scope='Prescribed arm-only offline feasibility witnesses, not MPPI results: no optimization, ESDF/map acceptance, 95ms deadline claim, ROS, hardware, or mesh certification.',
        initial_state='Q_START and zero base/joint velocities; each run independently holds until its recorded trigger, not a replay of prior MPPI actions.',
        policy='10Hz: u_arm=clip(1.8*(posture_0-q)-0.2*dq,+/-vmax); goal=kernel.position_target(measured_state,u); base target always zero, no physical base lock.',
        feedback='Actual saved plant state delayed 20ms; no state relabeling, additional scheduling jitter or observation-to-action solve delay.',
        integration_step_s=definitions['STEP'], measurement_delay_s=definitions['MEASUREMENT_DELAY'],
        command_watchdog_s=definitions['COMMAND_WATCHDOG'], csv_period_s=.05,
        audit='Unmodified StandbySimulator.audit every5ms: current67 physical spheres, arena walls, analytic finite-height cylinder, joint q/dq/ddq limits, current self/base-arm/ground model.',
        source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        controller_profile=profile, cases={})
    for name, trigger in triggers().items():
        report['cases'][name] = run(name,trigger,definitions,model,profile)
        print(json.dumps({'case':name,**{k:report['cases'][name][k] for k in
            ('completed','reason','elapsed_s','min_external_gap_m','max_arm_change_deg')}}),flush=True)
    report['wall_s']=time.monotonic()-began
    report['peak_rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    (OUT/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
