#!/usr/bin/env python3
"""One full native REMANI+MPPI SIL run: faster wheels, 3 s forecast, 6 m goal.

Only fresh experiment snapshots are adjusted. No hardware configuration or
compiled planner/controller is changed. This is NOT Gazebo or a real-car run.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / 'results/long_goal_crossing_20261029_01/mid_cross_seed11'
SCENE = 'dynamic_mid_cross'
GOAL = [6., 0.]
BOUNDS = [-1.5, 7.5, -2., 2.]
ACTOR = (3., -1.6, 0., .18, .23)
WHEEL_OMEGA = 4.10
PREDICTION_S = 3.


def register_scene():
    # The parent preparer asks for a scene definition before creating runtime
    # copies; registration is process-local, not an edit to the shared module.
    import dynamic_scenarios as scenarios
    scenarios.CASES[SCENE] = ('09_mid_cross', '远目标中段横穿', (ACTOR,))
    if SCENE not in scenarios.SCENES:
        scenarios.SCENES = (*scenarios.SCENES, SCENE)


def configure_trial(out):
    """Apply the declared settings to an already prepared, NEW bypass trial."""
    import yaml
    from native_collision_runtime import replace_once
    out = Path(out)
    runtime = out / 'runtime'
    path = out / 'limits.json'
    limits = json.loads(path.read_text())
    previous_limits = dict(limits)
    params_path = out / 'parameters.yaml'
    params = yaml.safe_load(params_path.read_text())
    omega_key = '/remani_planner_node/mm/mobile_base_max_wheel_omega'
    prediction_key = '/remani_planner_node/dynamic_point_flow/max_prediction_time'
    previous_omega, previous_prediction = params[omega_key], params[prediction_key]
    params[omega_key] = WHEEL_OMEGA
    params[prediction_key] = PREDICTION_S
    limits['wheel_v'] = WHEEL_OMEGA * params['/remani_planner_node/mm/mobile_base_wheel_radius']
    limits['max_prediction'] = PREDICTION_S
    # Preserve the existing limits key order: the MPPI C ABI is positional.
    path.write_text(json.dumps(limits, indent=2) + '\n')
    params_path.write_text(yaml.safe_dump(params, sort_keys=True))
    shared_path = out / 'shared_collision_config.json'
    shared = json.loads(shared_path.read_text())
    shared['max_prediction'] = PREDICTION_S
    shared_path.write_text(json.dumps(shared, indent=2) + '\n')

    # All three wall consumers must agree: synthetic sensor, projected metric,
    # and the independent <=5 ms native-sphere truth monitor.
    path = runtime / 'simulator.py'
    text = path.read_text()
    for old, new in (
        ('xs=np.arange(-1.5,4.51,.06)', 'xs=np.arange(-1.5,7.51,.06)'),
        ('for x in (-1.5,4.5) for y in ys', 'for x in (-1.5,7.5) for y in ys'),
        ('min(p[0]+1.5,4.5-p[0])', 'min(p[0]+1.5,7.5-p[0])'),
        ('goal.pose.position.x=3.;', 'goal.pose.position.x=6.;'),
        ('self.s[:2]-[3.,0.]', 'self.s[:2]-[6.,0.]'),
    ):
        text = replace_once(text, old, new)
    path.write_text(text)
    path = runtime / 'native_collision_runtime.py'
    text = path.read_text()
    for old, new in (
        ('(-1.5-x, x-4.5)', '(-1.5-x, x-7.5)'),
        ('for side in (-1.5, 4.5)', 'for side in (-1.5, 7.5)'),
    ):
        text = replace_once(text, old, new)
    path.write_text(text)
    path = runtime / 'dynamic_scenarios.py'
    text = path.read_text()
    text = replace_once(text, "SCENES=('empty'",
        'CASES["%s"]=("09_mid_cross","远目标中段横穿",(%r,))\n' % (SCENE, ACTOR)
        + "SCENES=('empty'")
    text = replace_once(text, 'centers[:,0]+1.5,4.5-centers[:,0]',
        'centers[:,0]+1.5,7.5-centers[:,0]')
    path.write_text(text)

    from dynamic_scenarios import scene_definition
    definition = scene_definition(SCENE)
    definition.update(time_origin='motion',
        source='This experiment: one cylinder crossing the middle of a 6 m route, constant velocity',
        predicted_center_crossing_after_motion_s=2. + 1.6 / .18)
    (out / 'scenario.json').write_text(json.dumps(definition, ensure_ascii=False, indent=2) + '\n')
    shutil.copy2(Path(__file__), runtime / Path(__file__).name)
    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['scenario'] = definition
    manifest['parameter_overrides'].update({omega_key: WHEEL_OMEGA, prediction_key: PREDICTION_S})
    manifest['long_goal_crossing_experiment'] = dict(
        goal=GOAL, previous_goal=[3., 0.], bounds=BOUNDS,
        previous_bounds=[-1.5, 4.5, -2., 2.],
        wheel_omega_rad_s=WHEEL_OMEGA, previous_wheel_omega_rad_s=previous_omega,
        wheel_v_m_s=limits['wheel_v'], previous_wheel_v_m_s=previous_limits['wheel_v'],
        wheel_acceleration_unchanged=True, prediction_upper_bound_s=PREDICTION_S,
        previous_native_prediction_upper_bound_s=previous_prediction,
        actual_mppi_horizon='Native validity minus observation age and 10 ms reserve, rounded down to 0.1 s steps',
        map_grid='Native 15 x 10 m grid unchanged; goal and whole fixed body remain inside its x upper limit of 7.5 m',
        actor_crossing_x_m=3., actor_velocity_m_s=[0., .18],
        appearance_after_motion_s=2., constant_velocity_actor_persists=True,
        terminal_near_obstacle_cost='MPPI预测轨迹末端近障惩罚: not restored in this settings-only experiment',
        unchanged=['native 3D collision geometry and soft costs', 'candidate strategy',
            'wheel and body acceleration limits', 'body speed and yaw-speed caps',
            '200 ms deadline / 250 ms command lease', 'REMANI solve budgets',
            'latest-state rollout and fallback braking validation'],
        notes='Settings changed together to observe behavior; not a single-factor causal comparison',
        production_changed=False, compiled_binary_reused=True,
        scope='One fixed-arm differential-drive native ROS numerical SIL run, NOT Gazebo/hardware')
    manifest['simulation_perception_adjustment']['unchanged'] = 'Synthetic self-crop bypass retained; this experiment separately changes wheel speed, prediction and arena/goal'
    manifest['execution_timing_pair']['unchanged'] = '200/250 ms timing and latest-state validation retained; speed/horizon separately changed by long_goal_crossing_experiment'
    manifest['legacy_factorial_runtime']['final_goal_unchanged'] = False
    for name in ('simulator.py', 'native_collision_runtime.py', 'dynamic_scenarios.py', Path(__file__).name):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256((runtime / name).read_bytes()).hexdigest()
    manifest['long_goal_crossing_experiment']['configuration_sha256'] = {
        name: hashlib.sha256((out / name).read_bytes()).hexdigest()
        for name in ('limits.json', 'parameters.yaml', 'scenario.json', 'shared_collision_config.json')}
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


def patch_trial(out, settings):
    from run_sim_crop_bypass import patch_trial as bypass_patch
    bypass_patch(out, settings)
    configure_trial(out)


def main():
    register_scene()
    from run_execution_timing_pair import BUILD
    from run_coupled_objective import check_build, run_trial, sha
    from run_native_collision_3d import trial_kwargs
    from run_native_dynamic_fast import clean_environment
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--duration', type=float, default=65.)
    args = parser.parse_args()
    manifest, _ = check_build(BUILD)
    args.out.resolve().parent.mkdir(parents=True, exist_ok=True)
    with clean_environment():
        valid = run_trial(args.out.resolve(), BUILD, SCENE, 'engineering',
            11, args.duration, 0., 0., True, sha(BUILD / 'build_manifest.json'),
            prepare_hook=patch_trial, **trial_kwargs(manifest))
    print('LONG_GOAL_MID_CROSSING', 'valid_interface=' + str(valid), args.out, flush=True)
    if not valid:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
