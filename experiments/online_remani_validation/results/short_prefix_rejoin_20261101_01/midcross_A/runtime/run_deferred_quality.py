#!/usr/bin/env python3
"""One head-on pair: common first plan, periodic versus deferred refresh checks."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_deferred_quality as builder
import run_fast_refresh_feedback as parent
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE/'results/deferred_quality_20261101_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def patch(trial, settings, enabled):
    parent.builder = builder
    parent.patch(trial, settings, True)
    tree = ET.parse(trial/'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node') if n.get('name') == 'remani_planner_node')
    for name, value in dict(COLLAB_PREFIX=1, FEASIBLE_QUALITY_POLISH=1,
                            DEFERRED_QUALITY_CHECK=int(enabled)).items():
        common.set_env(planner, name, value)
    tree.write(trial/'experiment.launch', encoding='unicode')
    settings.update(deferred_quality_check=bool(enabled), prefix_initializer=True,
        legacy_prefix=True, feasible_quality_polish=True, polish_refresh_only=True,
        initial_plan_policy='first native feasible return in both groups',
        comparison_group='B' if enabled else 'A', fast_refresh_feedback=True,
        optimization_budget_s=1.2, deferred_reserve_s=.15,
        changed='refresh accepted-candidate native check scheduling only', production_modified=False)
    save(trial/'coupling_parameters.json', settings)
    path = trial/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['legacy_factorial_runtime']['prefix'] = True
    manifest['deferred_quality'] = dict(enabled=bool(enabled), same_binary=True,
        runtime_switch='DEFERRED_QUALITY_CHECK', initial_plan_common_first_safe=True,
        refresh_only=True, pending_is_not_native_feasible=True,
        budget_s=1.2, deferred_reserve_s=.15, polish_cutoff_s=1.05,
        reserve_is_soft_not_wcet=True, internal_deferred_check_inside_original_budget=True,
        final_native_and_latest_gates_retained=True,
        native_final_and_latest_time_measured_outside_solver_budget=True,
        fallback='first certified native candidate; final and latest gates still apply',
        actual_accepted_and_chosen_recorded_separately=True,
        unchanged=['original objective and weights', 'MPPI and its execution validation',
            'prefix initializer, feedback and time mapping', '67 physical spheres and wheel limits',
            '200ms MPPI deadline and 250ms command lease', 'original retry and delivery delay'],
        search_candidates_recorded=False, diagnostic_pause=False, hardware=False, gazebo=False,
        async_pair_not_identical_input=True, production_modified=False)
    if 'delivery_next_online' in manifest:
        manifest['delivery_next_online'].update(early_exit=True,
            only_group_difference='ancestor configuration held common; this pair varies DEFERRED_QUALITY_CHECK',
            both_final_and_latest_outside_solver_budget=True,
            candidate_check_inside_solver_budget=True)
        for name in ('group_A_final_gate_outside_solver_budget', 'group_B_live_gate_inside_solver_budget'):
            manifest['delivery_next_online'].pop(name, None)
    for source in (Path(__file__), HERE/'report_deferred_quality.py'):
        if source.exists():
            shutil.copy2(source, trial/'runtime'/source.name)
            manifest['experiment_sources_sha256'][source.name] = common.lifecycle.sha(source)
    save(path, manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enabled', type=int, choices=(0, 1), required=True)
    parser.add_argument('--tag', required=True)
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    if not (OUT/'plan.json').exists():
        save(OUT/'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            research_goal='Useful slow reference delivery and smooth adoption during fast MPPI avoidance',
            task='Use existing budget for optimization rather than repeated accepted-candidate checks',
            groups='A current periodic refresh checks; B first-safe plus one deferred candidate check',
            unique_switch='DEFERRED_QUALITY_CHECK 0/1',
            both_q1_first_safe=True, both_qgt1_polish=True,
            seed=11, goal=[6., 0.], duration_s=55., scene='sphere_timing_headon',
            planned_trials=['headon_A', 'headon_B'], physical_spheres=67,
            optimization_budget_s=1.2, deferred_reserve_s=.15,
            mppi_deadline_ms=200., command_lease_ms=250.,
            primary_measurements=['common first reference polynomial identity',
                'first native safe, pending and chosen quality; fallback and rejection reasons',
                'accepted updates and native-check wall time within original budget',
                'whole request to actual new-reference effective command',
                'latest acceptance, phase residual, adoption motion and physical gaps'],
            one_online_pair_is_mechanism_evidence_not_population_statistics=True,
            async_inputs_not_bitwise_equal=True, production_modified=False,
            scope='fixed-arm complete REMANI+MPPI ROS numerical SIL; not Gazebo/hardware',
            github_upload_remains_paused=True))
        source_plan = (HERE/'results/feasible_quality_20261101_01/下一轮实验方案.txt').read_text()
        source_plan = source_plan.replace(
            '此文件是下一轮设计；本轮设计任务没有启动新仿真或修改控制算法。',
            '执行记录见headon_A、headon_B；本轮结果和图示见report。')
        (OUT/'本轮方案.txt').write_text(source_plan)
    manifest = builder.check()
    common.register_scenes()
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT/args.tag, builder.OUT,
            'sphere_timing_headon', 'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT/'build_manifest.json'),
            reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('DEFERRED_QUALITY_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()
