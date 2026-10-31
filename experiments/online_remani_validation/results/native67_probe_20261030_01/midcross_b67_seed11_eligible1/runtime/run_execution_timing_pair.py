#!/usr/bin/env python3
"""Two head-on closed loops: current versus matched execution timing only."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

import build_native_dynamic_fast as parent_builder
from build_shared_local_safety import sha
from run_coupled_objective import check_space, run_trial
from run_native_collision_3d import trial_kwargs
from run_native_dynamic_fast import clean_environment

HERE = Path(__file__).resolve().parent
PARENT = HERE / 'builds/native_dynamic_fast_20261028_01'
BUILD = HERE / 'builds/execution_timing_20261028_01'
OUT = HERE / 'results/execution_timing_pair_20261028_01'
CONFIGS = (('current', 50., .075), ('matched', 200., .250))


def patch_api(source):
    changes = (
        ('struct Online {native_realtime::Stats realtime_probe;',
         'struct Online {double execution_lease=.075;native_realtime::Stats realtime_probe;'),
        ('V u=t<.075-1e-8?first:V{};', 'V u=t<h.execution_lease-1e-8?first:V{};'),
        ('if(t>=.075&&std::abs(s.v)<.01', 'if(t>=h.execution_lease&&std::abs(s.v)<.01'),
        ('// execute first action for the 75ms actuator lease, then brake with the same lag/wheel model.',
         '// execute first action for the configured actuator lease, then brake with the same lag/wheel model.'),
    )
    for old, new in changes:
        if source.count(old) != 1:
            raise ValueError('Timing patch anchor changed: ' + old)
        source = source.replace(old, new, 1)
    return source + '''
// Isolated timing experiment: same value as the simulated actuator lease.
extern "C" void online_set_execution_lease(void*p,double seconds){
  if(p)static_cast<Online*>(p)->execution_lease=seconds;
}
'''


def prepare_build(build=BUILD):
    build = Path(build).resolve()
    parent = parent_builder.check(PARENT)
    if build.exists():
        raise FileExistsError('Fresh output required: ' + str(build))
    check_space(build.parent)
    build.mkdir()
    shutil.copytree(PARENT / 'source', build / 'source')
    (build / 'build').mkdir()
    for name in parent_builder.previous.PRODUCTS:
        shutil.copy2(PARENT / 'build' / name, build / 'build' / name)
    source = build / 'source/online_api.cpp'
    source.write_text(patch_api(source.read_text()))
    recipe = copy.deepcopy(parent['native_dynamic_fast_recipe'])
    item = recipe['mppi']
    for key in ('compile_command', 'actual_link_command'):
        item[key] = [arg.replace(str(PARENT), str(build)) for arg in item[key]]
    item['expected_object'] = item['expected_object'].replace(str(PARENT), str(build))
    parent_builder.previous.verify_recipe(recipe)
    subprocess.run(item['compile_command'], check=True)
    subprocess.run(item['actual_link_command'], cwd=item['cwd'], check=True)
    manifest = copy.deepcopy(parent)
    manifest.update(execution_timing_pair_experiment=True,
        execution_timing_parent=str(PARENT), execution_timing_parent_manifest_sha256=sha(PARENT / 'build_manifest.json'),
        execution_timing_runner_sha256=sha(Path(__file__)),
        execution_timing_recipe=recipe, execution_timing_default_lease_s=.075,
        execution_timing_scope='only CABI-configured actuator hold in existing brake validation; planner unchanged',
        mppi_library_sha256=sha(build / 'build/libonline_mppi.so'),
        isolated_source_sha256=parent_builder.previous.previous.native_recipe.source_inventory(build / 'source'))
    (build / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    for name, digest in parent['isolated_source_sha256'].items():
        if name != 'online_api.cpp' and sha(build / 'source' / name) != digest:
            raise ValueError('Unexpected planning source change: ' + name)
    for name, digest in parent_builder.previous.reused_hashes(parent).items():
        if sha(build / 'build' / name) != digest:
            raise ValueError('Unexpected native binary change: ' + name)
    parent_builder.check(PARENT)
    print('BUILT TIMING PAIR', build, flush=True)
    return build


def run_pair(build=BUILD, out=OUT):
    from execution_timing_runtime import patch_trial
    build, out = Path(build).resolve(), Path(out).resolve()
    from run_coupled_objective import check_build
    manifest, _ = check_build(build)
    if not manifest.get('execution_timing_pair_experiment'):
        raise ValueError('Requires timing-pair child build')
    parent_builder.check(PARENT)
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    digest = sha(build / 'build_manifest.json')
    plan = dict(build=str(build), build_manifest_sha256=digest, scene='dynamic_head_on_center',
        seed=11, duration_s=55., configs=[dict(group=g, deadline_ms=d, command_lease_s=l) for g, d, l in CONFIGS],
        same_binary=True, full_online_native_planner=True, native_dynamic_fast=True, demand=True,
        factor='execution timing bundle: deadline and actuator/brake hold',
        unchanged=['160 collision spheres', 'candidate count/order', 'MPPI weights', 'CV horizon',
                   'speed limits', 'REMANI budget', 'latest-state prepublication checks'],
        scope='two fixed-arm numerical ROS closed loops; not Gazebo/hardware or a success-rate study')
    (out / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    completed, runtime_sources = [], None
    try:
        for group, deadline, lease in CONFIGS:
            name = 'dynamic_head_on_center_%s_seed11' % group
            print('START', name, deadline, lease, flush=True)
            with clean_environment():
                valid = run_trial(out / name, build, plan['scene'], 'engineering', 11, 55., 0., 0., True,
                    digest, expected_runtime_sources=runtime_sources,
                    prepare_hook=lambda trial, settings, d=deadline, l=lease: patch_trial(trial, d, l),
                    **trial_kwargs(manifest))
            current = json.loads((out / name / 'manifest.json').read_text())['experiment_sources_sha256']
            runtime_sources = current if runtime_sources is None else runtime_sources
            result = json.loads((out / name / 'result.json').read_text())
            completed.append(dict(trial=name, group=group, valid_interface=valid, termination=result['reason']))
            (out / 'progress.json').write_text(json.dumps(completed, indent=2) + '\n')
            print('DONE', name, result['reason'], 'valid=', valid, flush=True)
            if not valid:
                raise RuntimeError('Trial interface/audit failed; result retained')
        (out / 'complete.json').write_text(json.dumps(dict(complete=True, completed=completed), indent=2) + '\n')
    except Exception as exc:
        (out / 'stopped.json').write_text(json.dumps(dict(completed=completed, reason=str(exc)), indent=2) + '\n')
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, default=OUT)
    parser.add_argument('--prepare', action='store_true', help='compile child only; no ROS')
    args = parser.parse_args()
    if args.prepare:
        prepare_build(args.build)
    else:
        run_pair(args.build, args.out)


if __name__ == '__main__':
    main()
