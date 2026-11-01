#!/usr/bin/env python3
"""One SIL rerun without self cropping: this synthetic cloud has no robot points.

Production perception and every planner/controller parameter remain unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / 'results/sim_crop_bypass_20261029_01/head_on_bypass_seed11'


def bypass_crop(out):
    """Rewire only the freshly generated simulation launch, not hardware files."""
    out = Path(out)
    launch_path = out / 'experiment.launch'
    tree = ET.parse(launch_path)
    launch = tree.getroot()
    crop = next(node for node in launch.findall('node') if node.get('name') == 'crop_box_filter')
    height = next(node for node in launch.findall('node') if node.get('name') == 'passthrough_z')
    remap = next(r for r in height.findall('remap') if r.get('from') == '~output')
    if remap.get('to') != '/livox/lidar_no_ground':
        raise ValueError('Unexpected simulation perception routing')
    launch.remove(crop)
    remap.set('to', '/velodyne_points')
    tree.write(launch_path, encoding='unicode')
    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['nodes'] = [node for node in manifest['nodes'] if node['name'] != 'crop_box_filter']
    manifest['simulation_perception_adjustment'] = dict(
        self_crop_bypassed=True, removed_node='crop_box_filter',
        upstream='passthrough_z', output='/velodyne_points',
        reason='Synthetic raw cloud contains only environment walls and actor cylinders, no self returns.',
        inactive_parameters='Existing /crop_box_filter parameters are retained but have no running consumer.',
        unchanged='height filtering, tracking, 3D geometry, CV, all planner/controller parameters, 200/250ms timing',
        production_changed=False, launch_sha256=hashlib.sha256(launch_path.read_bytes()).hexdigest(),
        scope='simulation-specific perception correction, not a hardware self-filter prescription')
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


def patch_trial(out, settings):
    from diagnose_execution_timing import patch_trial as timing_probe_patch
    timing_probe_patch(out, settings)
    bypass_crop(out)
    out = Path(out)
    shutil.copy2(Path(__file__), out / 'runtime' / Path(__file__).name)
    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['experiment_sources_sha256'][Path(__file__).name] = hashlib.sha256(
        (out / 'runtime' / Path(__file__).name).read_bytes()).hexdigest()
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


def main():
    from run_execution_timing_pair import BUILD
    from run_coupled_objective import check_build, run_trial, sha
    from run_native_collision_3d import trial_kwargs
    from run_native_dynamic_fast import clean_environment
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    manifest, _ = check_build(BUILD)
    args.out.resolve().parent.mkdir(parents=True, exist_ok=True)
    with clean_environment():
        valid = run_trial(args.out.resolve(), BUILD, 'dynamic_head_on_center', 'engineering',
            11, 55., 0., 0., True, sha(BUILD / 'build_manifest.json'),
            prepare_hook=patch_trial, **trial_kwargs(manifest))
    print('SIMULATION_CROP_BYPASS', 'valid_interface=' + str(valid), args.out, flush=True)
    if not valid:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
