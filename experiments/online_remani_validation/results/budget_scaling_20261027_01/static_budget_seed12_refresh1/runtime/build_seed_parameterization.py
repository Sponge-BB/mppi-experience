#!/usr/bin/env python3
"""Build a reversible seed-only repair from an attested diagnostic parent."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE, sha
from build_native_v10 import compile_source
from run_coupled_objective import check_build
from run_dual_timescale import verify_production

BASE = HERE / 'builds/delivery_stability_20261024_02'


def validate(base):
    m, _ = check_build(base)
    if not m.get('delivery_stability_experiment') or m.get('delivery_stability_prepared_only'):
        raise ValueError('Compiled diagnostic parent required')
    for name, digest in m['isolated_source_sha256'].items():
        if sha(base / 'source' / name) != digest:
            raise ValueError('Parent source drift: ' + name)
    for name, digest in m['delivery_stability_archive_sha256'].items():
        if sha(base / 'build' / name) != digest:
            raise ValueError('Parent archive drift: ' + name)
    verify_production(base)
    return m


def prepare(base, out):
    from patch_seed_parameterization import transform_search, project_off
    base, out = base.resolve(), out.resolve()
    m = validate(base)
    original = (base / 'source/kino_astar.cpp').read_text()
    modified = transform_search(original)
    if project_off(modified, base / 'source/kino_astar.cpp') != original:
        raise ValueError('Exact OFF source projection failed')
    out.mkdir(parents=True, exist_ok=False)
    shutil.copytree(base / 'source', out / 'source')
    shutil.copytree(base / 'build', out / 'build')
    shutil.copy2(HERE / 'seed_parameterization.hpp', out / 'source/seed_parameterization.hpp')
    (out / 'source/kino_astar.cpp').write_text(modified)
    m.update(seed_parameterization_experiment=True, prepared_only=True,
        seed_parameterization_parent_initialization_unchanged=m.get('initialization_unchanged'),
        initialization_unchanged=False,
        initialization_change_scope='seed geometry/time parameterization only; optimizer initialization policy otherwise retained',
        seed_parameterization_parent=str(base),
        seed_parameterization_parent_manifest_sha256=sha(base / 'build_manifest.json'),
        seed_parameterization_builder_sha256=sha(Path(__file__)),
        seed_parameterization_patch_sha256=sha(HERE / 'patch_seed_parameterization.py'),
        seed_parameterization_scope='fixed-arm static numerical SIL seed geometry/time repair; not hardware or Gazebo',
        seed_parameterization_declarations=dict(
            default_off=True, switch='SEED_PARAMETERIZATION_FIX', exact_off_source_projection=True,
            unchanged_optimizer=True, unchanged_mppi=True, unchanged_wheel_limits=True,
            unchanged_collision_geometry_and_margin=True, normal_budget_s=.6,
            relative_forward_reverse_metric=True, tight_curvature_metric_lower_bound='Euclidean chord',
            duration_and_length_acceleration='.7 * original max_acc',
            finite_transactional_outputs=True, invalid_seed_no_downstream_sampling=True,
            exact_terminal_sample_instead_of_T_minus_01=True),
        production_modified=False,
        seed_parameterization_parent_archive_sha256=m['delivery_stability_archive_sha256'],
        isolated_source_sha256={str(p.relative_to(out/'source')): sha(p)
            for p in (out/'source').rglob('*') if p.is_file()})
    (out/'build_manifest.json').write_text(json.dumps(m, indent=2)+'\n')
    print('PREPARED', out, flush=True)


def compile_prepared(out):
    out = out.resolve()
    m = json.loads((out/'build_manifest.json').read_text())
    if not m.get('seed_parameterization_experiment') or not m.get('prepared_only'):
        raise ValueError('Fresh prepared seed repair required')
    base = Path(m['seed_parameterization_parent'])
    validate(base)
    if sha(base/'build_manifest.json') != m['seed_parameterization_parent_manifest_sha256']:
        raise ValueError('Parent manifest drift')
    for name, digest in m['isolated_source_sha256'].items():
        if sha(out/'source'/name) != digest:
            raise ValueError('Prepared source drift: ' + name)
    parent = Path(m['parent_build'])
    _, obj = compile_source(parent, out, 'exp_search', out/'source/kino_astar.cpp', 'unused', out/'source')
    subprocess.run(['ar', 'r', str(out/'build/libexp_search.a'), obj], check=True)
    link = shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    # Recompile the node TUs; do not rely on unattested loose parent objects.
    for name in ('remani_planner_node.cpp', 'remani_replan_fsm.cpp', 'planner_manager.cpp'):
        old = next(x for x in link if x.endswith('/'+name+'.o'))
        _, obj = compile_source(parent, out, 'remani_planner_node', out/'source'/name, old, out/'source')
        link = [obj if x == old else x for x in link]
    link = [str(out/'build'/x) if x in ('libexp_search.a', 'libexp_opt.a') else x for x in link]
    link[link.index('-o')+1] = str(out/'build/remani_planner_node')
    subprocess.run(link, cwd=str(parent/'build'), check=True)
    for name in ('libexp_opt.a', 'libonline_mppi.so', 'mm_controller_node', 'dynamic_obstacle_tracker_node'):
        if sha(out/'build'/name) != sha(base/'build'/name):
            raise ValueError('Out-of-scope binary changed: ' + name)
    archives = {n: sha(out/'build'/n) for n in ('libexp_opt.a', 'libexp_search.a')}
    m.update(prepared_only=False, seed_parameterization_archive_sha256=archives,
        state_contract_archive_sha256=archives, delivery_stability_archive_sha256=archives,
        native_binaries_sha256={n: sha(out/'build'/n) for n in
            ('remani_planner_node', 'mm_controller_node', 'dynamic_obstacle_tracker_node')})
    (out/'build_manifest.json').write_text(json.dumps(m, indent=2)+'\n')
    check_build(out)
    verify_production(out)
    print('BUILT', out, flush=True)


def attest_scope(out):
    """Correct inherited scope metadata, without recompiling or changing code."""
    out = out.resolve()
    m, _ = check_build(out)
    if m.get('prepared_only') or not m.get('seed_parameterization_experiment'):
        raise ValueError('Completed seed repair build required')
    before = {n:sha(out/'build'/n) for n in ('remani_planner_node', 'libexp_search.a', 'libexp_opt.a', 'libonline_mppi.so')}
    m.update(seed_parameterization_parent_initialization_unchanged=
        json.loads((Path(m['seed_parameterization_parent'])/'build_manifest.json').read_text()).get('initialization_unchanged'),
        initialization_unchanged=False,
        initialization_change_scope='seed geometry/time parameterization only; optimizer initialization policy otherwise retained',
        seed_parameterization_metadata_attestor_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(m, indent=2)+'\n')
    if before != {n:sha(out/'build'/n) for n in before}:
        raise ValueError('Scope attestation changed a binary')
    print('SCOPE_ATTESTED', out, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, default=BASE)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--compile', action='store_true')
    p.add_argument('--finish-only', action='store_true')
    a = p.parse_args()
    if a.finish_only and not a.compile:
        p.error('--finish-only requires --compile')
    if not a.finish_only:
        prepare(a.base, a.out)
    if a.compile:
        compile_prepared(a.out)
