#!/usr/bin/env python3
"""One prespecified paused dynamic input: baseline versus delivery improvements."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import run_native67_probe as lifecycle
from run_coupled_objective import HERE, check_space, sha

BUILD = HERE / 'builds/delivery_optimization_20261030_01'
OUT = HERE / 'results/delivery_same_input_20261030_01'
PHASES = ('delivery_baseline_060', 'delivery_optimized_060')
ORIGINAL_PREPARE = lifecycle.prepare_capture
PREDICATE = lifecycle.PREDICATE


def source_switch_proof(build):
    build = Path(build)
    manifest = json.loads((build/'build_manifest.json').read_text())
    names = ('poly_traj_optimizer.cpp', 'delivery_fast_query.hpp', 'delivery_accepted_candidate.hpp')
    hashes = {name: sha(build/'source'/name) for name in names}
    source = (build/'source/poly_traj_optimizer.cpp').read_text()
    fast = (build/'source/delivery_fast_query.hpp').read_text()
    expected = manifest.get('isolated_source_sha256', {})
    checks = dict(frozen_sources=all(hashes[name] == expected.get(name) for name in names),
        same_budget='const double budgets[]={.60,.60};' in source,
        labels='const char* labels[]={"delivery_baseline_060","delivery_optimized_060"};' in source,
        switch_query='setenv("DELIVERY_FAST_QUERY",i?"1":"0",1);' in source,
        switch_candidate='setenv("DELIVERY_ACCEPT_CANDIDATE",i?"1":"0",1);' in source,
        dynamic_getenv='std::getenv("DELIVERY_FAST_QUERY")' in fast,
        compiled_delivery_child=manifest.get('delivery_optimization_experiment') is True
            and manifest.get('prepared_only') is False)
    return dict(checks=checks, valid=all(checks.values()), source_sha256=hashes,
        note='fast-query switch verified from frozen compiled source; no per-query branch counter claimed')


def prepare_capture(out, build, seed=11, ordinal=1):
    switch_proof = source_switch_proof(build)
    if not switch_proof['valid']:
        raise ValueError('Paired same-input build switch proof failed: '+str(switch_proof['checks']))
    manifest = ORIGINAL_PREPARE(out, build, seed, ordinal)
    tree = ET.parse(out/'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node') if n.get('name') == 'remani_planner_node')
    # Before capture and in all other nodes use baseline settings. The compiled
    # private replay loop owns the two arm-specific switches and restores them.
    for key in ('DELIVERY_FAST_QUERY', 'DELIVERY_ACCEPT_CANDIDATE'):
        existing = planner.findall("env[@name='%s']" % key)
        if existing:
            existing[0].set('value', '0')
        else:
            ET.SubElement(planner, 'env', name=key, value='0')
    tree.write(out/'experiment.launch', encoding='unicode')
    settings = json.loads((out/'coupling_parameters.json').read_text())
    settings.update(diagnostic_phases=list(PHASES), diagnostic_budgets_s=[.6, .6],
        original_stop_policy=False, diagnostic_candidates_published=False,
        treatment_by_phase={PHASES[0]: {'fast_query': False, 'accept_candidate': False},
                            PHASES[1]: {'fast_query': True, 'accept_candidate': True}},
        treatment='equivalent dynamic distance query plus last-accepted timeout candidate native validation',
        budget_unchanged=True, objective_unchanged=True, constraints_unchanged=True,
        initialization_unchanged=True, constraints_relaxed=False,
        candidate_gate='same native IsTrajSafe; no publication from diagnostic; latest FSM gates not exercised',
        scope='one frozen backend input; not a complete online delivery measurement')
    (out/'coupling_parameters.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2)+'\n')
    manifest.update(coupling_experiment=settings,
        parameters_sha256=sha(out/'coupling_parameters.json'),
        delivery_same_input_experiment=True, navigation_metrics_valid=False,
        diagnostic_candidates_published=False)
    for name in ('run_delivery_same_input.py', 'report_delivery_same_input.py'):
        source = HERE/name
        manifest['experiment_sources_sha256'][name] = sha(source)
        shutil.copy2(source, out/'runtime'/name)
    manifest['delivery_same_input'] = dict(budgets_s=[.6, .6], labels=list(PHASES),
        modes=[dict(fast_query=0, accept_candidate=0), dict(fast_query=1, accept_candidate=1)],
        switch_source_proof=switch_proof,
        fixed_order=True, no_outcome_selected_rerun=True,
        resource_whole_wall_includes_posterior_audits=True,
        backend_decision_timer_excludes_posterior_audits=True)
    (out/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    return manifest


def run_capture(out, build, seed=11, ordinal=1):
    previous = lifecycle.prepare_capture
    lifecycle.prepare_capture = prepare_capture
    try:
        return lifecycle.run_capture(out, build, seed, ordinal)
    finally:
        lifecycle.prepare_capture = previous


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, default=OUT)
    args = parser.parse_args()
    build, out = args.build.resolve(), args.out.resolve()
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'),
        trials=[dict(directory='midcross_b67_seed11_eligible1', seed=11, ordinal=1)],
        scene='sphere_timing_midcross', goal=[6., 0.], collision_spheres=67,
        capture_predicate=PREDICATE, diagnostic_phases=list(PHASES), budgets_s=[.6, .6],
        treatment_by_phase={PHASES[0]: dict(fast_query=0, accept_candidate=0),
                            PHASES[1]: dict(fast_query=1, accept_candidate=1)},
        no_outcome_selected_rerun=True, navigation_metrics_valid=False,
        diagnostic_candidates_published=False,
        limitations=['one freshly captured input, not a historical failed request replay',
            'two arms in fixed order; not a multi-seed success-rate estimate',
            'both distance query and timeout recovery change; no independent per-change causal attribution',
            'native candidate acceptance is not live FSM publication or MPPI consumption',
            'posterior diagnostic audits are outside solver/decision timers; whole RESOURCE wall includes them'])
    (out/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    job = plan['trials'][0]
    try:
        status = run_capture(out/job['directory'], build, job['seed'], job['ordinal'])
    except Exception as error:
        (out/'stopped.json').write_text(json.dumps(dict(reason=str(error)), indent=2)+'\n')
        raise
    (out/'complete.json').write_text(json.dumps(dict(complete=True,
        completed=[dict(**job, captured=status['diagnostic_complete'])]), indent=2)+'\n')
    print('DELIVERY_SAME_INPUT_COMPLETE', out, flush=True)


if __name__ == '__main__':
    main()
