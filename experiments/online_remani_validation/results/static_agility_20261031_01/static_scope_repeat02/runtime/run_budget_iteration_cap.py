#!/usr/bin/env python3
"""Prospective follow-up of the two cap-limited ordinal-1 cases.

Fresh captures, not exact repeats of the previous process/map state. All five
within-capture arms use 1200 iterations; only the iteration computational cap
changes, never wheel/collision limits. Candidates are private and unpublished.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import yaml
import run_budget_scaling as base
import run_backend_feasibility as common
from run_coupled_objective import check_space, sha
from run_dual_timescale import verify_production

JOBS = (dict(seed=11, ordinal=1), dict(seed=12, ordinal=1))
KEY = '/remani_planner_node/optimization/max_iterations'
ITERATIONS = 1200


def prepare(out, build, seed, ordinal):
    runtime = base.prepare(out, build, seed, ordinal)
    path = out/'parameters.yaml'
    original = yaml.safe_load(path.read_text())
    if original.get(KEY) != 300:
        raise ValueError('Follow-up requires the attested original 300-iteration configuration')
    changed = dict(original)
    changed[KEY] = ITERATIONS
    path.write_text(yaml.safe_dump(changed, sort_keys=True))
    actual = yaml.safe_load(path.read_text())
    restored = dict(actual)
    restored[KEY] = 300
    if restored != original or actual[KEY] != ITERATIONS:
        raise ValueError('Iteration-cap override changed other loaded parameters')
    import roslaunch
    resolved = roslaunch.config.load_config_default([str(out/'experiment.launch')], common.PORT, verbose=False)
    if any(resolved.params[k].value != value for k, value in actual.items()):
        raise ValueError('Effective launch parameters differ from frozen YAML')
    coupling = out/'coupling_parameters.json'
    settings = json.loads(coupling.read_text())
    settings.update(iteration_cap_followup=True, optimizer_max_iterations=ITERATIONS,
        original_optimizer_max_iterations=300, physical_constraints_unchanged=True,
        comparison_scope='within each new capture only; not same inputs as earlier cap=300 batch')
    base.write_json(coupling, settings)
    manifest_path = out/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['parameter_overrides'][KEY] = ITERATIONS
    manifest.update(coupling_experiment=settings, parameters_sha256=sha(coupling),
        loaded_parameters_sha256=sha(path), iteration_cap_followup=True)
    base.write_json(manifest_path, manifest)
    return runtime


def run(build, out):
    build, out = build.resolve(), out.resolve()
    base.validate_build(build)
    check_space(out.parent, 2*1024**3)
    out.mkdir(parents=True, exist_ok=False)
    build_hash = sha(build/'build_manifest.json')
    base.write_json(out/'plan.json', dict(created=datetime.now(timezone.utc).isoformat(),
        scope=__doc__, build=str(build), build_manifest_sha256=build_hash,
        diagnostic_jobs=list(JOBS), phases=list(base.PHASES), budgets_s=list(base.BUDGETS),
        treatment_order=list(base.TREATMENTS), source_sha256=sha(Path(__file__)),
        parent_runner_sha256=sha(Path(base.__file__)), optimizer_max_iterations=ITERATIONS,
        followup_reason='prior ordinal-1 extended arms hit LBFGSERR_MAXIMUMITERATION at 300 accepted steps before 3 seconds',
        execution_weight=0., handover_weight=0., constraints_relaxed=False,
        no_outcome_selected_rerun=True, diagnostic_candidates_published=False,
        native_map_buffer_dumped=False, navigation_metrics_valid=False,
        primary_endpoint='normal return and exact final native/projected validation before deadline',
        limitations=['Two prospective cap-mechanism captures, not a general success-rate estimate.',
            'Both captures and all failures retained; previous batch is not overwritten.',
            'All five arms share cap=1200. No same-input cap300-vs1200 causal contrast is claimed.',
            'No robot/Gazebo/online publication, no relaxed physical constraints.']))
    before = verify_production(build)
    saved_env, saved_prepare = common.diagnostic_environment, common.prepare_capture
    completed, runtime = [], None
    common.diagnostic_environment, common.prepare_capture = base.environment, prepare
    try:
        for job in JOBS:
            target = out/('static_budget_seed%d_refresh%d' % (job['seed'], job['ordinal']))
            observed, captured = common.run_capture(target, build, job['seed'], job['ordinal'], build_hash, runtime)
            if runtime is None:
                runtime = observed
            completed.append(dict(trial=target.name, kind='paused', captured=captured, **job))
            base.write_json(out/'progress.json', completed)
    except Exception as error:
        base.write_json(out/'stopped.json', dict(completed=completed,
            reason=type(error).__name__+': '+str(error), retained_incomplete=True))
        raise
    finally:
        common.diagnostic_environment, common.prepare_capture = saved_env, saved_prepare
    if verify_production(build) != before or sha(build/'build_manifest.json') != build_hash:
        raise RuntimeError('Frozen build/production changed')
    base.write_json(out/'complete.json', dict(complete=True, completed=completed,
        optimizer_max_iterations=ITERATIONS, navigation_metrics_valid=False,
        diagnostic_candidates_published=False))
    print('ITERATION_CAP_FOLLOWUP_COMPLETE', out, flush=True)


def report(root, build, out):
    import report_budget_scaling as audit
    root, build, out = root.resolve(), build.resolve(), out.resolve()
    if out.exists():
        raise FileExistsError(out)
    plan = json.loads((root/'plan.json').read_text())
    if plan.get('optimizer_max_iterations') != ITERATIONS:
        raise ValueError('Unexpected iteration-cap protocol')
    audit.JOBS = tuple((j['seed'],j['ordinal']) for j in JOBS)
    result = audit.build_report(root, build)
    result.update(followup_reporter_sha256=sha(Path(__file__)), iteration_cap=ITERATIONS,
        comparison_scope='within fresh cap1200 captures only; not exact cap300 cross-batch comparison',
        limitations=[s.replace('四个预先指定','两个预先指定').replace('四请求','两请求')
            for s in audit.LIMITATIONS]+plan['limitations'])
    for trial in result['trials']:
        parameters = yaml.safe_load((root/trial['trial']/'parameters.yaml').read_text())
        manifest = json.loads((root/trial['trial']/'manifest.json').read_text())
        if parameters.get(KEY) != ITERATIONS or manifest['parameter_overrides'].get(KEY) != ITERATIONS:
            raise ValueError('Loaded iteration cap differs from protocol')
    out.mkdir(parents=True, exist_ok=False)
    base.write_json(out/'summary.json', result)
    lines = ['300次迭代上限补测（新输入、两请求、全部保留）',
        '全部五臂统一1200次上限；轮速、轮加速度、碰撞投影与余量不变。',
        '与前批不是同一输入，不据两批数值差宣称迭代上限的单因素收益。',
        '证据有效：'+str(result['evidence_valid']), '']
    for group in result['aggregate']:
        lines.append('%s：最终按时合格%d/%d，失败%d，未知%d。' % (group['phase'],group['passed'],
            len(JOBS),group['failed'],group['unknown']))
    for trial in result['trials']:
        lines.append('\n'+trial['trial'])
        for phase in trial['phases']:
            failure = phase.get('failure_record') or {}
            counts = phase.get('canonical_counts',{}).get('all',{})
            lines.append('  %s: %s；result=%s；accepted=%s；判定%.3fms；后验双过%s（不计交付）。' % (
                phase['phase'],phase.get('solver_reason'),failure.get('lbfgs_result'),
                phase.get('iterate_audit',{}).get('solve_summary',{}).get('accepted_total'),
                (phase.get('decision_elapsed_s') or 0.)*1000.,counts.get('both_pass')))
    (out/'迭代上限补测结论.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(evidence_valid=result['evidence_valid'],aggregate=result['aggregate']),ensure_ascii=False))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--report-root',type=Path)
    a=p.parse_args()
    if a.report_root:
        report(a.report_root,a.build,a.out)
    else:
        run(a.build,a.out)
