#!/usr/bin/env python3
"""Fresh 140-vs-47 fixed-sphere timing pairs, full native numerical SIL only."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import yaml

HERE = Path(__file__).resolve().parent
BUILD = HERE/'builds/urdf_sphere_timing_20261029_01'
OUT = HERE/'results/sphere_scenarios_20261029_01'
MODEL = HERE/'results/urdf_sphere_simplification_20261029_01/candidate_coarse.json'
# Same scenarios in each pair. The fourth multi actor is an oncoming actor.
ACTORS = {
    'empty': (),
    'static': ((3., 0., 0., 0., .23),),
    'midcross': ((3., -1.6, 0., .18, .23),),
    'headon': ((4.3, 0., -.18, 0., .20),),
    'multi': ((1.55, -1.65, 0., .26, .23), (2.4, 1.4, 0., -.20, .23),
              (3., -1.7, 0., .17, .20), (4.3, .3, -.18, 0., .20)),
}
NAMES = dict(empty='空旷', static='静态障碍', midcross='路径中段横穿',
             headon='中心线迎面', multi='四障碍交会')
CASES = {'sphere_timing_'+key: ('timing_'+key, NAMES[key], actors)
         for key, actors in ACTORS.items()}


def register_scenes():
    import dynamic_scenarios as scenes
    from run_long_goal_crossing import register_scene
    register_scene()
    scenes.CASES.update(CASES)
    scenes.SCENES = tuple(dict.fromkeys((*scenes.SCENES, *CASES)))


def jobs(scenes):
    return [dict(scene=scene, group=group, directory=scene+'_'+group+'_seed11', seed=11)
            for index, scene in enumerate(scenes)
            for group in (('legacy', 'b47') if index % 2 == 0 else ('b47', 'legacy'))]


def configure_comparison(out, scene, group):
    from native_collision_runtime import replace_once
    from run_urdf_spheres_b import sphere_rows
    out = Path(out)
    runtime = out/'runtime'
    # Current hardware launch defaults to B. Explicitly remove ALL inherited
    # rows before selecting either group; this child selects by row presence.
    params_path = out/'parameters.yaml'
    params = yaml.safe_load(params_path.read_text())
    removed = [key for key in params if '/mm/fixed_collision_spheres' in key]
    for key in removed:
        del params[key]
    candidate = json.loads(MODEL.read_text())
    if group == 'b47':
        params['/remani_planner_node/mm/fixed_collision_spheres'] = sphere_rows(candidate)
    elif group != 'legacy':
        raise ValueError('Unknown geometry group')
    params_path.write_text(yaml.safe_dump(params, sort_keys=True))

    # All cases are installed in every runtime so code hashes stay identical.
    path = runtime/'dynamic_scenarios.py'
    extra = ('SPHERE_TIMING_CASES = '+repr(CASES)+'\n'
             'CASES.update(SPHERE_TIMING_CASES)\n'
             'SCENES = tuple(dict.fromkeys((*SCENES, *SPHERE_TIMING_CASES)))\n')
    path.write_text(replace_once(path.read_text(), 'APPEARANCE=2.\n', extra+'APPEARANCE=2.\n'))
    from dynamic_scenarios import scene_definition
    definition = scene_definition('sphere_timing_'+scene)
    definition.update(time_origin='motion', source='fresh paired sphere-model timing scenarios; same prescribed actors within each pair')
    (out/'scenario.json').write_text(json.dumps(definition, ensure_ascii=False, indent=2)+'\n')
    shutil.copy2(Path(__file__), runtime/Path(__file__).name)

    path = out/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['scenario'] = definition
    manifest['sphere_model_comparison'] = dict(
        scene_label=scene, group=group, same_binary_for_both_groups=True,
        fixed_spheres=47 if group == 'b47' else 140,
        expected_total_spheres=67 if group == 'b47' else 160,
        arm_spheres_unchanged=20, inherited_sphere_keys_removed=removed,
        parameter_selection='custom rows only for B; all inherited production B rows removed',
        collision_geometry_and_count_changed=True, margins_and_weights_unchanged=True,
        adjacent_self_policy='custom Piper soft base-arm starts at existing hard-check link2; legacy behavior unchanged',
        query_count_reduction_is_not_whole_solver_speedup=True,
        scope='same compiled solver, online feedback differs; not identical optimization inputs or statistical success rate')
    legacy = manifest['long_goal_crossing_experiment']
    legacy.update(actor_crossing_x_m=None, actor_velocity_m_s=None,
                  constant_velocity_actor_persists=bool(ACTORS[scene]),
                  notes='Common 6m arena/speed/horizon; each scene comes from manifest.scenario. Only collision model differs within a pair.')
    legacy['unchanged'] = [item for item in legacy['unchanged']
                           if item != 'native 3D collision geometry and soft costs']
    manifest['watchdog_off_experiment']['unchanged'] = 'common watchdog-OFF timing capture; scene and selected geometry are declared separately'
    for name in ('dynamic_scenarios.py', Path(__file__).name):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256((runtime/name).read_bytes()).hexdigest()
    # Refresh only this NEW trial's configuration attestations after selection.
    legacy['configuration_sha256'] = {name: hashlib.sha256((out/name).read_bytes()).hexdigest()
        for name in ('limits.json', 'parameters.yaml', 'scenario.json', 'shared_collision_config.json')}
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')


def patch_trial(out, settings, scene, group):
    from run_long_goal_crossing import patch_trial as long_goal_patch
    from run_watchdog_off import disable_watchdog
    from timing_capture_runtime import patch_trial as timing_capture_patch
    long_goal_patch(out, settings)
    disable_watchdog(out)
    configure_comparison(out, scene, group)
    timing_capture_patch(out)


def check_pair_inputs(left, right):
    """Exclude geometry keys only; compare actual parameters, code and limits."""
    def parameters(path):
        return {key: value for key, value in yaml.safe_load((path/'parameters.yaml').read_text()).items()
                if '/mm/fixed_collision_spheres' not in key}
    if parameters(left) != parameters(right):
        raise ValueError('Unexpected non-geometry parameter difference')
    for name in ('limits.json', 'scenario.json', 'geometry.json', 'shared_collision_config.json'):
        if json.loads((left/name).read_text()) != json.loads((right/name).read_text()):
            raise ValueError('Pair settings differ: '+name)
    lm, rm = [json.loads((p/'manifest.json').read_text()) for p in (left, right)]
    if lm['experiment_sources_sha256'] != rm['experiment_sources_sha256']:
        raise ValueError('Pair runtime code differs')
    for key in ('native_binaries_sha256', 'mppi_library_sha256'):
        if lm[key] != rm[key]:
            raise ValueError('Pair binary differs')
    for path, expected in ((left, 160), (right, 67)):
        body = json.loads((path/'native_collision_3d_body.json').read_text())['body']
        if len(body) != expected:
            raise ValueError('Actual exported sphere count differs: '+str(path))
    return dict(passed=True, same_binary=True, same_runtime=True, same_non_geometry_parameters=True,
                actual_exported_spheres=[160, 67], scope='online states and optimization requests may differ after applying controls')


def run(out, scenes, duration, resume=False):
    from run_coupled_objective import run_trial, sha, check_space, check_build
    from run_native_collision_3d import trial_kwargs
    from run_native_dynamic_fast import clean_environment
    out = Path(out).resolve()
    manifest, _ = check_build(BUILD)
    if not manifest.get('urdf_sphere_timing_experiment'):
        raise ValueError('The unchanged, production-rebaselined timing build is required')
    register_scenes()
    if resume:
        plan = json.loads((out/'plan.json').read_text())
        if plan['build_manifest_sha256'] != sha(BUILD/'build_manifest.json'):
            raise ValueError('Cannot mix binaries on resume')
        duration, work = plan['duration_s'], plan['trials']
    else:
        out.mkdir(parents=True, exist_ok=False)
        work = jobs(scenes)
        plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(BUILD),
            build_manifest_sha256=sha(BUILD/'build_manifest.json'), trials=work,
            duration_s=duration, seed=11, goal=[6., 0.], bounds=[-1.5, 7.5, -2., 2.],
            actor_appearance_after_motion_s=2., scene_actors={k: ACTORS[k] for k in scenes},
            wheel_omega=4.10, prediction_upper_bound_s=3., deadline_ms=200., lease_ms=250.,
            same_binary=True, paired_order='AB/BA alternating by scene',
            logging='common lightweight timing capture; no full input-replay maps',
            fixed_sphere_counts=[140, 47], total_sphere_counts=[160, 67],
            scope='fixed-arm complete native REMANI+MPPI numerical SIL; NOT Gazebo/physical robot',
            limitations='one seed per pair; geometry shape and accumulated costs also change, so not a sphere-count-only causal experiment')
        (out/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    completed = json.loads((out/'progress.json').read_text()) if (out/'progress.json').exists() else []
    for item in work:
        if any(row['directory'] == item['directory'] for row in completed):
            continue
        trial = out/item['directory']
        if trial.exists():
            raise FileExistsError('Unfinished output retained; do not overwrite or silently repeat: '+str(trial))
        check_space(out)
        print('START', item['directory'], flush=True)
        with clean_environment():
            valid = run_trial(trial, BUILD, 'sphere_timing_'+item['scene'], 'engineering',
                item['seed'], duration, 0., 0., True, plan['build_manifest_sha256'],
                prepare_hook=lambda path, settings, i=item: patch_trial(path, settings, i['scene'], i['group']),
                **trial_kwargs(manifest))
        result = json.loads((trial/'result.json').read_text()) if (trial/'result.json').exists() else {}
        record = dict(**item, valid_experiment=valid, termination=result.get('reason'))
        completed.append(record)
        (out/'progress.json').write_text(json.dumps(completed, ensure_ascii=False, indent=2)+'\n')
        print('DONE', item['directory'], 'valid='+str(valid), result.get('reason'), flush=True)
        if not valid:
            (out/'stopped.json').write_text(json.dumps(dict(trial=item, reason='invalid interface/setup; retained without replacement'), indent=2)+'\n')
            raise RuntimeError('Invalid trial retained; inspect before starting more trials')
        for scene in scenes:
            a, b = out/(scene+'_legacy_seed11'), out/(scene+'_b47_seed11')
            if all(any(c['directory'] == p.name for c in completed) for p in (a, b)):
                target = out/(scene+'_pair_check.json')
                if not target.exists():
                    target.write_text(json.dumps(check_pair_inputs(a, b), indent=2)+'\n')
    (out/'complete.json').write_text(json.dumps(dict(complete=True, completed=completed), indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=OUT)
    parser.add_argument('--scenes', nargs='+', choices=tuple(ACTORS), default=list(ACTORS))
    parser.add_argument('--duration', type=float, default=55.)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    run(args.out, args.scenes, args.duration, args.resume)
