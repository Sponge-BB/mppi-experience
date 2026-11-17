#!/usr/bin/env python3
"""Read-only evidence audit for the free-base standby ROS comparison.

Writes only the selected audit output, never trial data. A collision is an
experimental outcome, not an evidence failure. Interrupted trials are excluded.
The geometry check independently evaluates solid cylinders and walls against
the frozen native DH sphere model; it does not certify the URDF meshes.
"""
import argparse
import ast
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


HERE = Path(__file__).resolve().parent
DEFAULT_ROOT = HERE/'results/standby_arm_influence_20261105_01'
MODES = ('native_piper', 'current_unmodified', 'current_standby_extension')
NATIVE_COMMIT = 'f138ec82dce5c58be2e7028548ed317efe541c4f'
LIMITATIONS = [
    'Native means the exact Piper-adapted static-planning baseline f138ec8; it is not unchanged official upstream.',
    'C is a newly added, separately authorized standby protection mode; B is the existing current idle behavior.',
    'C estimates velocity from an ideal, world-frame, single-object surface cloud; A/B retain their existing perception pipelines. This is not a same-perception MPPI algorithm ablation.',
    'C does not run REMANI slow-layer optimization, prefix injection or a slow/fast trajectory handoff.',
    'All groups use a numerical free whole-body plant and a disclosed 67-sphere DH approximation, with self/mount exclusions. No hardware or mesh collision certification is claimed.',
    'CSV statistics use recorded samples. Fine-integration collision time and contact state come from first_violation and are audited separately.',
    'This scene and seed are a functional experiment, not a statistically established general success rate.',
]


def read_json(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def records(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def evidence_ready(trial):
    try:
        status = read_json(trial/'run_status.json')
    except (OSError, ValueError):
        return False
    return bool(status.get('result_written') is True
                and status.get('required_process_failed') is False
                and status.get('graph_audit_written') is True
                and not (trial/'interruption_status.json').exists())


def independent_external_gap(spheres, t, scene):
    """Analytic solid capped-cylinder SDF; independent of runtime query code."""
    xyz, r = spheres[:, :3], spheres[:, 3]
    xmin, xmax, ymin, ymax = scene['arena']
    wall = np.minimum.reduce([xyz[:, 0]-xmin-r, xmax-xyz[:, 0]-r,
                              xyz[:, 1]-ymin-r, ymax-xyz[:, 1]-r])
    gaps = [float(wall.min())]
    if t >= scene['appearance_s']:
        for actor in scene['actors']:
            dt = t-scene['appearance_s']
            center = np.array([actor['x']+dt*actor['vx'], actor['y']+dt*actor['vy']])
            radial = np.linalg.norm(xyz[:, :2]-center, axis=1)-actor['r']
            vertical = np.abs(xyz[:, 2]-.5*(actor['z_min']+actor['z_max']))-.5*(actor['z_max']-actor['z_min'])
            sdf = np.sqrt(np.maximum(radial, 0.)**2+np.maximum(vertical, 0.)**2)
            sdf += np.minimum(np.maximum(radial, vertical), 0.)
            gaps.append(float(np.min(sdf-r)))
    return min(gaps)


def frozen_model(trial, config):
    spec = importlib.util.spec_from_file_location('standby_audit_frozen_model', trial/'runtime/active_arm_model.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PiperArmModel.from_parameters(trial/'parameters.yaml', config['urdf_path'])


def csv_array(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('state.csv has no observations')
    keys = list(rows[0])
    return {key: np.asarray([float(row[key]) for row in rows]) for key in keys}


def check_trial(trial):
    errors = []
    checks = {}

    def check(name, good, detail=None):
        checks[name] = dict(passed=bool(good), detail=detail)
        if not good:
            errors.append(name)

    check('wrapper_completed_without_interruption', evidence_ready(trial))
    if not evidence_ready(trial):
        return dict(trial=trial.name, evidence_valid=False, errors=errors, checks=checks)
    config = read_json(trial/'config.json')
    manifest = read_json(trial/'manifest.json')
    prepared = read_json(trial/'prepared.json')
    result = read_json(trial/'result.json')
    graph = read_json(trial/'graph_audit.json')
    scene = config['scene_definition']
    mode = config.get('mode')
    check('known_mode', mode in MODES, mode)
    check('config_manifest_prepared_mode', manifest.get('mode') == mode == prepared.get('mode'))
    for name, key in [('config.json', 'config_sha256'), ('experiment.launch', 'launch_sha256'), ('manifest.json', 'manifest_sha256')]:
        check('frozen_'+name, prepared.get(key) == sha(trial/name))
    for name, key in [('config.json', 'config_sha256'), ('parameters.yaml', 'parameters_sha256')]:
        check('manifest_'+name, manifest.get(key) == sha(trial/name))
    wrong_runtime = [name for name, expected in manifest['runtime_sha256'].items()
                     if not (trial/'runtime'/name).is_file() or sha(trial/'runtime'/name) != expected]
    check('all_frozen_runtime_hashes', not wrong_runtime, dict(count=len(manifest['runtime_sha256']), mismatches=wrong_runtime))
    wrong_binary = [name for name, expected in manifest['binary_sha256'].items()
                    if sha(trial/'ros_pkgs/online_remani_experiment'/name) != expected]
    check('executed_binary_hashes', not wrong_binary, wrong_binary)
    launch = ET.parse(trial/'experiment.launch').getroot()
    nodes = [dict(node.attrib) for node in launch.findall('node')]
    check('manifest_launch_nodes', manifest['nodes'] == nodes)
    check('no_hardware_nodes', not any('piper' in node['pkg'] or node['type'] in ('base_control.py', 'joint_cmd_bridge.py') for node in nodes))
    check('no_navigation_goal', config.get('navigation_goal') is None and config.get('goal') is None
          and scene.get('navigation_goal') is None and manifest.get('no_navigation_goal') is True
          and result.get('goal_messages_received') == 0 and not graph.get('navigation_goal_publishers'))
    expected = [] if mode == 'native_piper' else ['/active_arm_mppi' if mode == 'current_unmodified' else '/standby_whole_body_mppi']
    check('isolated_graph_and_actuator_owner', graph.get('use_sim_time') is True
          and graph.get('master') == '127.0.0.1:11441' and graph.get('whole_body_command_publishers') == expected)
    check('base_not_physically_locked', manifest.get('physical_base_locked') is False
          and result.get('physical_base_locked') is False and scene.get('base_constraint') == 'free')
    precheck = read_json(trial/'geometry_precheck.json')
    check('offline_arm_only_route_exists', precheck.get('passed') is True
          and precheck['arm_only_witness_min_external_gap_m'] > .05
          and precheck['held_arm_min_external_gap_m'] < 0.
          and precheck['fixed_structure_min_external_gap_m'] > .05)
    check('fine_plant_step', 0. < result.get('maximum_integration_step_s', 1.) <= .005+1e-10)
    if mode == 'native_piper':
        native = HERE/'builds/native_piper_idle_20261105_01'
        build = read_json(native/'build_manifest.json')
        check('native_exact_piper_commit', build.get('build_complete') is True
              and build.get('source_git_commit') == NATIVE_COMMIT
              and manifest.get('native_baseline_commit') == NATIVE_COMMIT
              and not build.get('source_patches'))
        wrong = [name for name, expected_sha in build['isolated_source_sha256'].items()
                 if sha(native/'source'/name) != expected_sha]
        check('native_archived_source_hashes', not wrong, dict(count=len(build['isolated_source_sha256']), mismatches=wrong))
        check('native_controller_really_commanded', result.get('native_commands_received', 0) > 0
              and result.get('whole_body_commands_received') == 0)
    model = frozen_model(trial, config)
    check('configured_full_joint_freedom_matches_model', all(np.allclose(config['arm_limits'][key], value, atol=1e-10, rtol=0.)
          for key, value in [('lower', model.lower), ('upper', model.upper),
                             ('velocity', model.velocity_limits), ('acceleration', model.acceleration_limits)]))
    state = csv_array(trial/'state.csv')
    t = state['t']; n = len(t)
    x = np.c_[state['x'], state['y']]; yaw = np.unwrap(state['yaw'])
    q = np.column_stack([state['q'+str(i)] for i in range(1, 7)])
    dq = np.column_stack([state['dq'+str(i)] for i in range(1, 7)])
    check('finite_monotonic_observations', n > 2 and all(np.isfinite(v).all() for v in state.values())
          and np.all(np.diff(t) > 0.) and t[0] >= 0.)
    metrics = dict(max_base_displacement_m=float(np.max(np.linalg.norm(x-x[0], axis=1))),
                   base_path_length_m=float(np.sum(np.linalg.norm(np.diff(x, axis=0), axis=1))),
                   max_base_yaw_change_rad=float(np.max(np.abs(yaw-yaw[0]))),
                   base_cumulative_turn_rad=float(np.sum(np.abs(np.diff(yaw)))),
                   final_base_error_m=float(np.linalg.norm(x[-1]-x[0])),
                   final_base_yaw_error_rad=float(abs(yaw[-1]-yaw[0])),
                   max_arm_change_deg=float(np.rad2deg(np.max(np.abs(q-q[0])))),
                   final_arm_error_deg=float(np.rad2deg(np.max(np.abs(q[-1]-q[0])))))
    deltas = {key: abs(value-result[key]) for key, value in metrics.items()}
    check('independent_csv_metrics_match', max(deltas.values()) < 1e-10, deltas)
    position_gap = np.minimum(q-model.lower, model.upper-q)
    velocity_gap = model.velocity_limits-np.abs(dq)
    accelerations = np.diff(dq, axis=0)/np.diff(t)[:, None]
    accel_gap = model.acceleration_limits-np.abs(accelerations)
    margins = dict(position_rad=float(position_gap.min()), velocity_rad_s=float(velocity_gap.min()),
                   sampled_acceleration_rad_s2=float(accel_gap.min()))
    check('joint_position_velocity_acceleration_limits', min(margins.values()) >= -1e-8, margins)
    spheres = model.body_spheres(q, np.c_[x, state['yaw']])
    external = np.asarray([independent_external_gap(body, ti, scene) for body, ti in zip(spheres, t)])
    config_gaps = model.configuration_gaps_batch(q)
    check('independent_solid_geometry_matches_csv', np.max(np.abs(external-state['clearance'])) < 1e-8
          and np.max(np.abs(config_gaps-state['configuration_gap'])) < 1e-8,
          dict(external_max_error_m=float(np.max(np.abs(external-state['clearance']))),
               configuration_max_error_m=float(np.max(np.abs(config_gaps-state['configuration_gap'])))))
    check('fine_truth_minimum_contains_csv', result['min_external_gap_m'] <= float(external.min())+1e-8
          and result['min_configuration_gap_m'] <= float(config_gaps.min())+1e-8)
    contact = result.get('first_violation')
    contact_audit = None
    if contact:
        z = np.asarray(contact['state']); stamp = contact['t']
        gap = independent_external_gap(model.body_spheres(z[5:11], z[:3]), stamp, scene)
        contact_audit = dict(t=stamp, external_gap_m=gap,
                             last_csv_t=float(t[-1]), time_after_last_csv_s=float(stamp-t[-1]))
        check('first_contact_snapshot_matches_geometry', abs(gap-contact['external_gap_m']) < 1e-8)
    completed = result.get('reason') == 'completed'
    if completed:
        check('completed_reaches_declared_horizon', t[-1] >= config['duration']-.05
              and not result['collision_physical'] and not result['joint_limit_violation'])
    elif result.get('reason') == 'collision':
        check('collision_has_truth_contact', result['collision_physical'] and contact is not None
              and (contact['external_gap_m'] < 0. or contact['configuration_gap_m'] < 0.))
    else:
        check('recognized_terminal_outcome', False, result.get('reason'))
    cycles = records(trial/'cycles.jsonl')
    cycle_summary = {}
    if mode == 'current_standby_extension':
        planned = [row for row in cycles if 'noise_rms_per_coordinate' in row]
        accepted = [row for row in planned if row.get('reason') == 'ok']
        reasons = {}
        selections = {}
        for row in cycles:
            key = row.get('reason', 'unknown'); reasons[key] = reasons.get(key, 0)+1
        for row in accepted:
            key = row.get('selection', 'unknown'); selections[key] = selections.get(key, 0)+1
        noise = np.asarray([row['noise_rms_per_coordinate'] for row in planned])
        check('actual_8d_stochastic_plans', bool(planned) and noise.shape[1:] == (8,)
              and np.all(noise > 0.) and all(row.get('sample_dimensions') == 8
              and row.get('random_samples', 0) > 0 for row in planned))
        moving_candidates = all(any(np.linalg.norm(u) > 0.
            for u in row.get('structured_first_base_commands', [])) for row in planned)
        check('base_motion_candidates_not_removed', bool(planned) and moving_candidates)
        check('actual_accepted_whole_body_commands', bool(accepted) and result.get('whole_body_commands_received', 0) > 0)
        check('accepted_controls_finite_and_8d', bool(accepted) and all(np.asarray(row['command']).shape == (8,)
              and np.isfinite(row['command']).all() for row in accepted))
        means = [row for row in accepted if row.get('selection') == 'exponential_mean']
        check('accepted_mean_rechecked_and_weights_normalized', bool(means)
              and all(row.get('weighted_mean_valid') is True and np.all(np.asarray(row['weights']) >= 0.)
              and abs(sum(row['weights'])-1.) < 1e-8 for row in means))
        check('accepted_freshness_and_deadline', bool(accepted) and all(row['solve_ms'] <= config['deadline_s']*1000.+1.
              and -.05 <= row['ros_stamp']-row['cloud_stamp'] <= .25+1e-8
              and -.05 <= row['ros_stamp']-row['odom_stamp'] <= .25+1e-8
              and -.05 <= row['ros_stamp']-row['joint_stamp'] <= .25+1e-8 for row in accepted))
        forbidden = ('standby_arm_scene', 'active_arm_scene')
        imports = []
        for name in ('standby_mppi_node.py', 'standby_whole_body_mppi.py', 'standby_observed_cloud.py'):
            tree = ast.parse((trial/'runtime'/name).read_text())
            imports.extend(node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom))
        check('standby_controller_no_truth_scene_imports', not any(name in forbidden for name in imports))
        cycle_summary = dict(cycles=len(cycles), planned=len(planned), accepted=len(accepted),
                             reasons=reasons, selections=selections,
                             noise_rms_min_per_coordinate=noise.min(axis=0).tolist() if len(noise) else [],
                             max_accepted_base_command=np.max(np.abs([row['command'][:2] for row in accepted]), axis=0).tolist() if accepted else [],
                             accepted_latency_ms_quantiles=np.percentile([row['solve_ms'] for row in accepted], [50, 95, 100]).tolist() if accepted else [])
    elif mode == 'current_unmodified':
        active = records(trial/'active_arm_cycles.jsonl')
        reasons = {}
        for row in active:
            key = row.get('reason', row.get('status', 'unknown')); reasons[key] = reasons.get(key, 0)+1
        cycle_summary = dict(cycles=len(active), reasons=reasons)
    return dict(trial=trial.name, mode=mode, evidence_valid=not errors, errors=errors, checks=checks,
                outcome=result['reason'], avoidance_completed=completed and not result['collision_physical'],
                csv_sample_count=n, csv_time_span=[float(t[0]), float(t[-1])],
                independent_csv_metrics=metrics, csv_min_external_gap_m=float(external.min()),
                fine_integration_min_external_gap_m=result['min_external_gap_m'],
                first_contact_audit=contact_audit, cycle_summary=cycle_summary,
                offline_witness_min_external_gap_m=precheck['arm_only_witness_min_external_gap_m'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--trials', nargs='+', help='Exact trial directory names; default selects completed wrapper evidence')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    trials = [root/name for name in args.trials] if args.trials else [p for p in sorted(root.iterdir()) if p.is_dir() and evidence_ready(p)]
    if not trials:
        raise SystemExit('No completed, uninterrupted trial evidence is available')
    findings = []
    for trial in trials:
        try:
            findings.append(check_trial(trial))
        except Exception as exc:
            findings.append(dict(trial=trial.name, evidence_valid=False, errors=[type(exc).__name__+': '+str(exc)]))
    modes = [f.get('mode') for f in findings if f['evidence_valid']]
    comparison_complete = all(mode in modes for mode in MODES)
    comparison_checks = {}
    if len(trials) > 1 and all(f['evidence_valid'] for f in findings):
        configs = [read_json(trial/'config.json') for trial in trials]
        scenes = [config['scene_definition'] for config in configs]
        comparison_checks['identical_scene_and_initial_posture'] = all(scene == scenes[0] for scene in scenes)
        comparison_checks['identical_physical_limits'] = all(read_json(trial/'limits.json') == read_json(trials[0]/'limits.json') for trial in trials)
        comparison_checks['identical_joint_limits'] = all(config['arm_limits'] == configs[0]['arm_limits'] for config in configs)
        shared_runtime = ('standby_comparison_simulator.py', 'active_arm_simulator.py', 'active_arm_mppi.py', 'active_arm_model.py', 'standby_arm_scene.py')
        comparison_checks['identical_plant_dynamics_geometry'] = all(
            sha(trial/'runtime'/name) == sha(trials[0]/'runtime'/name)
            for trial in trials for name in shared_runtime)
    payload = dict(schema='standby_comparison_audit_v1', evidence_valid=all(f['evidence_valid'] for f in findings)
                   and all(comparison_checks.values()), comparison_checks=comparison_checks,
                   comparison_complete=comparison_complete, trials=findings, limitations=LIMITATIONS,
                   excluded_trial_directories=[p.name for p in sorted(root.iterdir())
                        if p.is_dir() and (p/'config.json').exists() and not evidence_ready(p)])
    output = args.output or root/'audit.json'
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+'\n')
    for item in findings:
        print(json.dumps({k: item.get(k) for k in ('trial', 'mode', 'evidence_valid', 'outcome', 'independent_csv_metrics', 'errors')}, ensure_ascii=False))
    print('Audit:', output, 'complete_three_way_comparison:', comparison_complete)
    return 0 if payload['evidence_valid'] else 1


if __name__ == '__main__':
    sys.exit(main())
