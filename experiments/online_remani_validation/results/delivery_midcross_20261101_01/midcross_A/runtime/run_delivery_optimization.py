#!/usr/bin/env python3
"""Two fresh B67 delivery OFF/ON pairs, isolated numerical ROS SIL only."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml

from run_coupled_objective import HERE, check_build, check_space, run_trial, sha
from run_native_collision_3d import trial_kwargs
from run_native_dynamic_fast import clean_environment
from run_sphere_scenarios import ACTORS, patch_trial as sphere_patch, register_scenes

BUILD = HERE / 'builds/delivery_optimization_20261030_01'
OUT = HERE / 'results/delivery_optimization_20261030_01'
SCENES = ('static', 'midcross')
GROUPS = ('baseline', 'optimized')
SWITCHES = ('DELIVERY_FAST_QUERY', 'DELIVERY_ACCEPT_CANDIDATE')


def jobs(scenes=SCENES):
    return [dict(scene=scene, group=group, seed=11,
                 directory=scene+'_'+group+'_seed11')
            for index, scene in enumerate(scenes)
            for group in (GROUPS if index % 2 == 0 else tuple(reversed(GROUPS)))]


def set_planner_switches(tree, enabled):
    planners = tree.getroot().findall("node[@name='remani_planner_node']")
    if len(planners) != 1:
        raise ValueError('Exactly one private planner required')
    planner = planners[0]
    for key in SWITCHES:
        existing = planner.findall("env[@name='%s']" % key)
        if len(existing) > 1:
            raise ValueError('Ambiguous experiment switch: '+key)
        value = str(int(enabled))
        if existing:
            existing[0].set('value', value)
        else:
            ET.SubElement(planner, 'env', name=key, value=value)
    # An inherited same-input replay must never pause these online trials.
    replay = planner.findall("env[@name='DELIVERY_REPLAY_KIND']")
    if len(replay) != 1 or replay[0].get('value') != '':
        raise ValueError('Online trial must explicitly disable diagnostic replay')


def patch_trial(out, settings, scene, group):
    if group not in GROUPS:
        raise ValueError('Unknown delivery group')
    out = Path(out)
    sphere_patch(out, settings, scene, 'b47')
    tree = ET.parse(out/'experiment.launch')
    set_planner_switches(tree, group == 'optimized')
    tree.write(out/'experiment.launch', encoding='unicode')
    shutil.copy2(Path(__file__), out/'runtime'/Path(__file__).name)
    path = out/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['delivery_optimization'] = dict(
        enabled=group == 'optimized', group=group,
        switches={key: int(group == 'optimized') for key in SWITCHES},
        same_binary=True, collision_model='B67 native 3D',
        fast_query='same-distance native dynamic-point lookup; no margins or geometry relaxation',
        timeout_candidate='reconstruct last accepted iterate, native validation then existing live acceptance',
        optimization_budget_unchanged=True, initial_budget_s=1.5, refresh_budget_s=.6,
        no_new_projection_gate=True, no_wheel_relaxation=True,
        live_closed_loop=True, diagnostic_pause=False,
        scope='fixed-arm full REMANI+MPPI numerical ROS SIL; not Gazebo or hardware')
    # Inherited notes concern geometry comparisons, not this treatment.
    manifest['sphere_model_comparison']['group'] = 'b47'
    manifest['sphere_model_comparison']['scope'] = 'both treatment arms use identical B67 geometry'
    manifest['timing_only_capture']['common_to_both_collision_models'] = False
    manifest['timing_only_capture']['common_to_both_delivery_groups'] = True
    manifest['experiment_sources_sha256'][Path(__file__).name] = sha(out/'runtime'/Path(__file__).name)
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')


def normalized_launch(path):
    """Pair differences allowed only in the two private-planner switches/paths."""
    root = ET.parse(path/'experiment.launch').getroot()
    for node in root.findall('node'):
        seen = set()
        for item in node.findall('env'):
            key = item.get('name')
            if key in seen:
                raise ValueError('Duplicate launch environment: '+key)
            seen.add(key)
            if node.get('name') == 'remani_planner_node' and key in SWITCHES:
                node.remove(item)
    return ET.tostring(root, encoding='unicode').replace(str(path), '<trial>')


def check_pair_inputs(baseline, optimized):
    paths = [Path(baseline), Path(optimized)]
    for name in ('parameters.yaml', 'limits.json', 'scenario.json', 'geometry.json',
                 'shared_collision_config.json'):
        loader = yaml.safe_load if name.endswith('.yaml') else json.loads
        if loader((paths[0]/name).read_text()) != loader((paths[1]/name).read_text()):
            raise ValueError('Unexpected pair settings difference: '+name)
    if normalized_launch(paths[0]) != normalized_launch(paths[1]):
        raise ValueError('Unexpected pair launch environment difference')
    manifests = [json.loads((p/'manifest.json').read_text()) for p in paths]
    for key in ('experiment_sources_sha256', 'native_binaries_sha256', 'mppi_library_sha256'):
        if manifests[0][key] != manifests[1][key]:
            raise ValueError('Pair runtime/build differs: '+key)
    bodies = [json.loads((p/'native_collision_3d_body.json').read_text())['body'] for p in paths]
    if bodies[0] != bodies[1] or len(bodies[0]) != 67:
        raise ValueError('Actual exported B67 body differs')
    for path, enabled in zip(paths, (False, True)):
        tree = ET.parse(path/'experiment.launch')
        planner = tree.getroot().find("node[@name='remani_planner_node']")
        for key in SWITCHES:
            values = planner.findall("env[@name='%s']" % key)
            if len(values) != 1 or values[0].get('value') != str(int(enabled)):
                raise ValueError('Wrong recorded switch '+key)
    return dict(passed=True, same_binary=True, same_runtime=True, same_parameters=True,
                identical_exported_native_spheres=67,
                differing_planner_environment_only=list(SWITCHES),
                scope='online paths and optimization inputs can diverge; not same-input causal attribution')


def run(build, out, duration=55., resume=False):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest, _ = check_build(build)
    if not manifest.get('delivery_optimization_experiment') or manifest.get('prepared_only'):
        raise ValueError('Completed isolated delivery optimization build required')
    register_scenes()
    digest = sha(build/'build_manifest.json')
    if resume:
        plan = json.loads((out/'plan.json').read_text())
        if plan['build_manifest_sha256'] != digest or Path(plan['build']) != build:
            raise ValueError('Cannot mix builds on resume')
        duration, work = plan['duration_s'], plan['trials']
    else:
        check_space(out.parent)
        out.mkdir(parents=True, exist_ok=False)
        work = jobs()
        plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
            build_manifest_sha256=digest, trials=work, duration_s=duration,
            scenes=list(SCENES), groups=list(GROUPS), paired_order='AB/BA by scene', seed=11,
            goal=[6., 0.], bounds=[-1.5, 7.5, -2., 2.],
            actor_appearance_after_motion_s=2., scene_actors={s: ACTORS[s] for s in SCENES},
            native_spheres=67, wheel_omega=4.10, prediction_upper_bound_s=3.,
            deadline_ms=200., lease_ms=250., initial_budget_s=1.5, refresh_budget_s=.6,
            treatment_switches=list(SWITCHES), same_binary=True,
            logging='light timing capture; actual search subset at 1 Hz, no large input snapshots',
            no_outcome_selected_reruns=True, storage_reserve_mib=400,
            scope='fresh full REMANI+MPPI fixed-arm numerical ROS SIL; NOT Gazebo or hardware',
            limitations='one seed per pair; tests combined changes online, not individual causal effects or success rates')
        (out/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    completed = json.loads((out/'progress.json').read_text()) if (out/'progress.json').exists() else []
    for item in work:
        if any(row['directory'] == item['directory'] for row in completed):
            continue
        trial = out/item['directory']
        if trial.exists():
            raise FileExistsError('Unfinished output retained, no overwrite/repeat: '+str(trial))
        check_space(out)
        print('START', item['directory'], flush=True)
        with clean_environment():
            valid = run_trial(trial, build, 'sphere_timing_'+item['scene'], 'engineering',
                item['seed'], duration, 0., 0., True, digest,
                prepare_hook=lambda path, settings, i=item: patch_trial(path, settings, i['scene'], i['group']),
                **trial_kwargs(manifest))
        result = json.loads((trial/'result.json').read_text()) if (trial/'result.json').exists() else {}
        completed.append(dict(**item, valid_experiment=valid, termination=result.get('reason')))
        (out/'progress.json').write_text(json.dumps(completed, ensure_ascii=False, indent=2)+'\n')
        print('DONE', item['directory'], 'valid='+str(valid), result.get('reason'), flush=True)
        if not valid:
            (out/'stopped.json').write_text(json.dumps(dict(trial=item, reason='invalid setup/interface; retained, no outcome replacement'), indent=2)+'\n')
            raise RuntimeError('Invalid trial retained; inspect before continuing')
        for scene in SCENES:
            pair = [out/(scene+'_'+g+'_seed11') for g in GROUPS]
            if all(any(c['directory'] == p.name for c in completed) for p in pair):
                target = out/(scene+'_pair_check.json')
                if not target.exists():
                    target.write_text(json.dumps(check_pair_inputs(*pair), indent=2)+'\n')
    (out/'complete.json').write_text(json.dumps(dict(complete=True, completed=completed), indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, default=OUT)
    parser.add_argument('--duration', type=float, default=55.)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    if not 15. <= args.duration <= 120.:
        parser.error('duration must be 15..120 seconds')
    run(args.build, args.out, args.duration, args.resume)
