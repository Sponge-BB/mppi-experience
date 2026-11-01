#!/usr/bin/env python3
"""Prepare/build the isolated same-budget feasibility-stop diagnostic.

Only exp_opt and the three planner translation units are recompiled.  The
search archive, MPPI library, MMController and dynamic tracker are byte-reused.
No production source is modified and no ROS process is started by this builder.
Preparation is the default; compilation requires --compile explicitly.
"""
import argparse
import copy
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_shared_local_safety import NATIVE, checked_hash, sha
from run_coupled_objective import check_build
from run_dual_timescale import verify_production


HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/seed_parameterization_20261025_01'
ARCHIVES = ('libexp_opt.a', 'libexp_search.a')
PATCHED = ('poly_traj_optimizer.cpp', 'domain_lbfgs.hpp')
HEADER = 'feasibility_stop_gate.hpp'
PLANNER_TUS = ('remani_planner_node.cpp', 'remani_replan_fsm.cpp', 'planner_manager.cpp')
REUSED = ('mm_controller_node', 'dynamic_obstacle_tracker_node', 'libonline_mppi.so', 'libexp_search.a')


def source_inventory(directory):
    inventory = {}
    for path in Path(directory).rglob('*'):
        if path.is_symlink():
            raise ValueError('Symlink in immutable source inventory: ' + str(path))
        if path.is_file():
            inventory[path.relative_to(directory).as_posix()] = sha(path)
    return inventory


def validate(base):
    """Fail closed on any frozen source, native, MPPI or archive drift."""
    from patch_feasibility_stop import BASE_OPTIMIZER_SHA256, BASE_LBFGS_SHA256
    base = Path(base).resolve()
    if base != BASE.resolve():
        raise ValueError('Requires the frozen seed_parameterization_20261025_01 parent')
    manifest, _ = check_build(base)
    if (not manifest.get('seed_parameterization_experiment') or manifest.get('prepared_only') or
            manifest.get('delivery_stability_prepared_only')):
        raise ValueError('Compiled seed-parameterization diagnostic parent required')
    sources = manifest.get('isolated_source_sha256', {})
    if not sources or source_inventory(base / 'source') != sources:
        raise ValueError('Parent complete source inventory differs from manifest')
    checked_hash(base / 'source/poly_traj_optimizer.cpp', BASE_OPTIMIZER_SHA256)
    checked_hash(base / 'source/domain_lbfgs.hpp', BASE_LBFGS_SHA256)
    native = manifest.get('native_binaries_sha256', {})
    if set(native) != set(NATIVE):
        raise ValueError('Parent must attest exactly three native binaries')
    for name, digest in native.items():
        checked_hash(base / 'build' / name, digest)
    checked_hash(base / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
    archives = manifest.get('seed_parameterization_archive_sha256', {})
    if set(archives) != set(ARCHIVES):
        raise ValueError('Parent must attest both native archives')
    for name, digest in archives.items():
        checked_hash(base / 'build' / name, digest)
    verify_production(base)
    return manifest


def _flags(parent, target):
    path = parent / 'build/CMakeFiles' / (target + '.dir') / 'flags.make'
    text = path.read_text()
    includes = [x for x in text.splitlines() if x.startswith('CXX_INCLUDES =')]
    flags = [x for x in text.splitlines() if x.startswith('CXX_FLAGS =')]
    if len(includes) != 1 or len(flags) != 1:
        raise ValueError('Ambiguous compiler flags: ' + target)
    words = shlex.split(flags[0].split('=', 1)[1])
    if '-O2' not in words or '-std=gnu++17' not in words or '-ffast-math' in words or any(
            x.startswith('-O') and x != '-O2' for x in words):
        raise ValueError('Compiler profile drift: ' + target)
    return path, shlex.split(includes[0].split('=', 1)[1])


def compile_commands(parent, out):
    """Record inherited recipes and every reused loose object/static archive.

    Three planner TUs are rebuilt instead of trusting their historical .o
    files.  planning_visualization.cpp.o is unmodified, explicitly hashed and
    left in the immutable parent.  The model/environment archives are also
    explicitly hashed; no other loose optimizer/search objects are reused.
    """
    parent, out = Path(parent).resolve(), Path(out).resolve()
    path = parent / 'build/CMakeFiles/remani_planner_node.dir/link.txt'
    link = shlex.split(path.read_text())
    if link.count('-o') != 1 or link.index('-o') + 1 >= len(link):
        raise ValueError('Ambiguous native planner link output')
    inherited_objects = [x for x in link if x.endswith('.o')]
    expected_object_names = set(PLANNER_TUS) | {'planning_visualization.cpp'}
    if len(inherited_objects) != 4 or {Path(x).name[:-2] for x in inherited_objects} != expected_object_names:
        raise ValueError('Unexpected planner object inventory')
    records, commands = {}, []
    for target, sources in (('exp_opt', ('poly_traj_optimizer.cpp',)),
                            ('remani_planner_node', PLANNER_TUS)):
        flag_path, includes = _flags(parent, target)
        records[str(flag_path)] = sha(flag_path)
        for name in sources:
            obj = out / 'build' / (name + '.o')
            command = ['/usr/bin/c++', '-std=gnu++17', '-O2', '-DNDEBUG', '-fPIC',
                       '-I' + str(out / 'source'), *includes,
                       '-c', str(out / 'source' / name), '-o', str(obj)]
            commands.append(dict(target=target, source=name, object=str(obj), command=command))
            if target == 'remani_planner_node':
                matches = [x for x in inherited_objects if Path(x).name == name + '.o']
                if len(matches) != 1:
                    raise ValueError('Ambiguous inherited planner object: ' + name)
                link = [str(obj) if x == matches[0] else x for x in link]
    if link.count('libexp_opt.a') != 1 or link.count('libexp_search.a') != 1:
        raise ValueError('Native link must include exactly one optimizer and one search archive')
    dependencies = {}
    for arg in link:
        if arg.endswith('.a') and arg not in ARCHIVES or (
                arg.endswith('.o') and not Path(arg).is_absolute()):
            dependency = Path(arg)
            if not dependency.is_absolute():
                dependency = parent / 'build' / dependency
            dependency = dependency.resolve()
            if not dependency.is_file() or dependency.is_symlink():
                raise ValueError('Missing/unsafe frozen link dependency: ' + str(dependency))
            dependencies[str(dependency)] = sha(dependency)
    link = [str(out / 'build' / x) if x in ARCHIVES else x for x in link]
    link[link.index('-o') + 1] = str(out / 'build/remani_planner_node')
    return dict(parent=str(parent), flags_sha256=records, link_recipe_path=str(path),
                link_recipe_sha256=sha(path), compile_commands=commands,
                archive_command=['ar', 'r', str(out / 'build/libexp_opt.a'),
                                 str(out / 'build/poly_traj_optimizer.cpp.o')],
                link_command=link, cwd=str(parent / 'build'),
                reused_link_dependencies_sha256=dependencies,
                compiler_profile='inherited helper-equivalent C++17 -O2 -DNDEBUG -fPIC; no fast math')


def validate_recipe(recipe):
    for name, digest in recipe['flags_sha256'].items():
        checked_hash(Path(name), digest)
    checked_hash(Path(recipe['link_recipe_path']), recipe['link_recipe_sha256'])
    for name, digest in recipe['reused_link_dependencies_sha256'].items():
        checked_hash(Path(name), digest)


def verify_reuse(out, manifest):
    for name, digest in manifest['feasibility_stop_reused_binary_sha256'].items():
        checked_hash(out / 'build' / name, digest)


def verify_sources(out, manifest, base):
    from patch_feasibility_stop import project_off
    if source_inventory(out / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Prepared source inventory drift')
    for name, kind in zip(PATCHED, ('optimizer', 'lbfgs')):
        if project_off((out / 'source' / name).read_text(), kind=kind) != (base / 'source' / name).read_text():
            raise ValueError('Exact OFF source projection failed: ' + name)


def prepare(base, out):
    from patch_feasibility_stop import SWITCH, transform_optimizer, transform_lbfgs
    raw_out = Path(out)
    if raw_out.is_symlink():
        raise ValueError('Output may not be symlinked')
    base, out = Path(base).resolve(), raw_out.resolve()
    manifest = validate(base)
    parent = Path(manifest['parent_build']).resolve()
    for frozen in (base, parent):
        if out == frozen or frozen in out.parents or out in frozen.parents:
            raise ValueError('Output must be independent of immutable builds')
    if out.exists():
        raise FileExistsError('Refusing to overwrite existing build: ' + str(out))
    manifest_hash = sha(base / 'build_manifest.json')
    transformed = {}
    for name, transform in zip(PATCHED, (transform_optimizer, transform_lbfgs)):
        original = (base / 'source' / name).read_text()
        transformed[name] = transform(original)
        if transformed[name] == original:
            raise ValueError('Expected declared source transformation: ' + name)
    header = HERE / HEADER
    if not header.is_file() or header.is_symlink() or HEADER in manifest['isolated_source_sha256']:
        raise ValueError('Fresh non-symlinked feasibility-stop header required')
    recipe = compile_commands(parent, out)
    validate_recipe(recipe)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'build').mkdir()
    shutil.copytree(base / 'source', out / 'source')
    for name, modified in transformed.items():
        (out / 'source' / name).write_text(modified)
    shutil.copy2(header, out / 'source' / HEADER)
    # Six required products only: no CMake caches or gigabytes of historical .o.
    for name in (*NATIVE, *ARCHIVES, 'libonline_mppi.so'):
        shutil.copy2(base / 'build' / name, out / 'build' / name)
    sources = source_inventory(out / 'source')
    expected = dict(manifest['isolated_source_sha256'])
    for name in (*PATCHED, HEADER):
        expected[name] = sha(out / 'source' / name)
    if sources != expected:
        raise ValueError('Prepared source changed outside two declared files and one header')
    result = copy.deepcopy(manifest)
    result.update(
        prepared_only=True, feasibility_stop_experiment=True, feasibility_stop_prepared_only=True,
        feasibility_stop_parent=str(base), feasibility_stop_parent_manifest_sha256=manifest_hash,
        feasibility_stop_parent_source_sha256=dict(manifest['isolated_source_sha256']),
        feasibility_stop_parent_native_sha256=dict(manifest['native_binaries_sha256']),
        feasibility_stop_parent_archive_sha256=dict(manifest['seed_parameterization_archive_sha256']),
        feasibility_stop_parent_mppi_sha256=manifest['mppi_library_sha256'],
        feasibility_stop_builder_sha256=sha(Path(__file__)),
        feasibility_stop_patcher_sha256=sha(HERE / 'patch_feasibility_stop.py'),
        feasibility_stop_header_sha256=sha(header), feasibility_stop_recipe=recipe,
        feasibility_stop_reused_binary_sha256={name: sha(base / 'build' / name) for name in REUSED},
        feasibility_stop_exact_off_source_projection=True,
        feasibility_stop_switch=SWITCH, feasibility_stop_default=False,
        feasibility_stop_scope='new same-process paused static fixed-input numerical SIL; stop-eligibility only; NOT historical exact replay, hardware or Gazebo',
        feasibility_stop_matrix=dict(budgets_s=[.6, .6, .6],
            phases=['delivery_nominal_060', 'delivery_stop_on_060', 'delivery_repeat_060'],
            treatment_order=['OFF', 'ON', 'OFF'], diagnostic_candidates_never_published=True,
            same_initial_objective_gradient_required=True, no_budget_refresh=True),
        feasibility_stop_preserved=dict(objective_and_gradient=True, seed=True,
            search_archive=True, wheel_limits=True, hard_acceptance=True,
            collision_geometry_and_margin=True, mppi_library=True, controller_and_tracker_binaries=True),
        feasibility_stop_common_diagnostic_change='three paused fixed-input passes now 0.6 s each: OFF / ON / OFF; no historical 1.5 s long-budget pass',
        feasibility_stop_parent_diagnostic_declarations=copy.deepcopy(manifest.get('diagnostic_declarations')),
        diagnostic_declarations=dict(unchanged_wheel_and_geometry=True, admission_off=True,
            clearance_recovery_off=True, legacy_prefix_off=True, legacy_rolling_off=True,
            prediction_soft_weights=0., normal_budget_s=.6, longer_budget_diagnostic_only=False,
            accepted_iterates_not_cost_callback_trials=True, fixed_input_same_binary_off_on_off=True),
        optimizer_stop_policy_unchanged=False, production_modified=False, production_default=False,
        isolated_source_sha256=sources)
    verify_sources(out, result, base)
    verify_reuse(out, result)
    validate(base)
    checked_hash(base / 'build_manifest.json', manifest_hash)
    (out / 'build_manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print('PREPARED FEASIBILITY STOP', out, flush=True)
    return out


def compile_prepared(out):
    out = Path(out).resolve()
    manifest = json.loads((out / 'build_manifest.json').read_text())
    if (not manifest.get('feasibility_stop_experiment') or not manifest.get('prepared_only') or
            not manifest.get('feasibility_stop_prepared_only')):
        raise ValueError('Fresh prepared feasibility-stop build required')
    base = Path(manifest['feasibility_stop_parent']).resolve()
    parent_manifest = validate(base)
    checked_hash(base / 'build_manifest.json', manifest['feasibility_stop_parent_manifest_sha256'])
    checked_hash(Path(__file__), manifest['feasibility_stop_builder_sha256'])
    checked_hash(HERE / 'patch_feasibility_stop.py', manifest['feasibility_stop_patcher_sha256'])
    checked_hash(HERE / HEADER, manifest['feasibility_stop_header_sha256'])
    verify_sources(out, manifest, base)
    verify_reuse(out, manifest)
    recipe = compile_commands(Path(parent_manifest['parent_build']), out)
    if recipe != manifest['feasibility_stop_recipe']:
        raise ValueError('Compile/link recipe changed after preparation')
    validate_recipe(recipe)
    # Compile the optimizer then replace its one archive member.  Recompile all
    # planner TUs that include the optimizer interface; leave search untouched.
    optimizer, *planner = recipe['compile_commands']
    subprocess.run(optimizer['command'], check=True)
    subprocess.run(recipe['archive_command'], check=True)
    for command in planner:
        subprocess.run(command['command'], check=True)
    subprocess.run(recipe['link_command'], cwd=recipe['cwd'], check=True)
    validate(base)
    checked_hash(base / 'build_manifest.json', manifest['feasibility_stop_parent_manifest_sha256'])
    validate_recipe(recipe)
    verify_sources(out, manifest, base)
    verify_reuse(out, manifest)
    archives = {name: sha(out / 'build' / name) for name in ARCHIVES}
    if archives['libexp_opt.a'] == manifest['feasibility_stop_parent_archive_sha256']['libexp_opt.a']:
        raise ValueError('Optimizer archive unexpectedly unchanged')
    if sha(out / 'build/remani_planner_node') == manifest['feasibility_stop_parent_native_sha256']['remani_planner_node']:
        raise ValueError('Planner binary unexpectedly unchanged')
    manifest.update(prepared_only=False, feasibility_stop_prepared_only=False,
        feasibility_stop_archive_sha256=archives, state_contract_archive_sha256=archives,
        delivery_stability_archive_sha256=archives, seed_parameterization_archive_sha256=archives,
        native_binaries_sha256={name: sha(out / 'build' / name) for name in NATIVE})
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check_build(out)
    verify_production(out)
    print('BUILT FEASIBILITY STOP', out, flush=True)
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=BASE)
    parser.add_argument('--out', type=Path, required=True, help='fresh independent build directory')
    parser.add_argument('--compile', action='store_true', help='explicitly compile/link after preparation')
    parser.add_argument('--finish-only', action='store_true', help='compile an existing prepared build')
    args = parser.parse_args()
    if args.finish_only and not args.compile:
        parser.error('--finish-only requires --compile')
    if not args.finish_only:
        prepare(args.base, args.out)
    if args.compile:
        compile_prepared(args.out)
