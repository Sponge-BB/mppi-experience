#!/usr/bin/env python3
"""Candidate B, fixed-arm native REMANI+MPPI numerical SIL only.

Uses the preceding 6 m midpoint-crossing scenario, with its watchdog already
disabled. This changes fixed geometry in both layers, not hardware defaults.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import yaml

HERE = Path(__file__).resolve().parent
BUILD = HERE/'builds/urdf_spheres_b_20261029_01'
MODEL = HERE/'results/urdf_sphere_simplification_20261029_01/candidate_coarse.json'
DEFAULT_OUT = HERE/'results/urdf_spheres_b_online_20261029_01/mid_cross_seed11'
PARAMETER = '/remani_planner_node/mm/fixed_collision_spheres'


def sphere_rows(model):
    if model['profile'] != 'coarse' or model['sphere_count'] != 47:
        raise ValueError('Candidate B must be the 47-sphere coarse model')
    return [[*sphere['center_ground'], sphere['radius']] for sphere in model['spheres']]


def configure_spheres(out, model_path=MODEL):
    out = Path(out)
    model_path = Path(model_path)
    model = json.loads(model_path.read_text())
    rows = sphere_rows(model)
    params_path = out/'parameters.yaml'
    params = yaml.safe_load(params_path.read_text())
    if PARAMETER in params:
        raise ValueError('A sphere model was already configured in this trial')
    params[PARAMETER] = rows
    params_path.write_text(yaml.safe_dump(params, sort_keys=True))
    shutil.copy2(model_path, out/'fixed_structure_spheres.json')
    shutil.copy2(Path(__file__), out/'runtime'/Path(__file__).name)
    manifest_path = out/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['urdf_spheres_b'] = dict(
        enabled=True, fixed_structure_spheres=47, old_base_approximation_spheres=140,
        moving_arm_spheres_unchanged=20, expected_exported_total_spheres=67,
        source_model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
        parameter=PARAMETER, frame=model['frame_contract'],
        variable_radius_consumers=['REMANI model/search', 'REMANI optimizer',
                                   'native3D export', 'MPPI native3D query', 'markers'],
        adjacent_self_collision='custom Piper soft base-arm checks now use the existing hard-check start index 2; legacy unchanged',
        additional_safety_margin=0., original_weights_and_margins_unchanged=True,
        legacy_2d_geometry='retained for proposals/legacy metrics only; native3D remains execution authority',
        geometry_count_reduction_is_not_timing_improvement=True,
        scope='fixed-arm ROS numerical SIL, not Gazebo/hardware or complete whole-robot mesh certification')
    manifest['long_goal_crossing_experiment']['compiled_binary_reused'] = False
    manifest['long_goal_crossing_experiment']['unchanged'].remove('native 3D collision geometry and soft costs')
    manifest['long_goal_crossing_experiment']['notes'] = (
        'Candidate B changes geometry and hence summed costs; previous results are context, not an equivalent-geometry speed comparison')
    manifest['watchdog_off_experiment']['unchanged'] = (
        'Scene/timing/limits retained; this child separately changes fixed-structure geometry in REMANI and MPPI')
    manifest['experiment_sources_sha256'][Path(__file__).name] = hashlib.sha256(
        (out/'runtime'/Path(__file__).name).read_bytes()).hexdigest()
    manifest['urdf_spheres_b']['parameters_sha256'] = hashlib.sha256(params_path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')


def patch_trial(out, settings):
    from run_long_goal_crossing import patch_trial as parent_patch
    from run_watchdog_off import disable_watchdog
    parent_patch(out, settings)
    disable_watchdog(out)
    configure_spheres(out)


def main():
    from build_urdf_spheres_b import check
    from run_long_goal_crossing import register_scene, SCENE
    from run_coupled_objective import run_trial, sha
    from run_native_collision_3d import trial_kwargs
    from run_native_dynamic_fast import clean_environment
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--duration', type=float, default=65.)
    args = parser.parse_args()
    manifest = check(BUILD)
    if manifest['prepared_only']:
        raise ValueError('Build has not completed')
    register_scene()
    args.out.resolve().parent.mkdir(parents=True, exist_ok=True)
    with clean_environment():
        valid = run_trial(args.out.resolve(), BUILD, SCENE, 'engineering', 11,
                          args.duration, 0., 0., True, sha(BUILD/'build_manifest.json'),
                          prepare_hook=patch_trial, **trial_kwargs(manifest))
    print('URDF_SPHERES_B_ONLINE', 'valid_interface='+str(valid), args.out, flush=True)
    if not valid:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
