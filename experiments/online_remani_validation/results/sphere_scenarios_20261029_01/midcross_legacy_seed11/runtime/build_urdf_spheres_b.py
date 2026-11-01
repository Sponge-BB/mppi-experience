#!/usr/bin/env python3
"""Build candidate B in a separate tree; never build production or run ROS.

MMConfig has new members. Recompile its model, all search members, optimizer,
and all four planner translation units against the child header. The map,
controller and tracker are immutable reused artifacts; MPPI is also rebuilt
without changing its existing variable-radius collision protocol.
"""
import argparse
import copy
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from run_coupled_objective import check_build
from run_dual_timescale import verify_production


HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/execution_timing_20261028_01'
FROZEN = HERE / 'builds/fixed_arm_01'
DEFAULT_OUT = HERE / 'builds/urdf_spheres_b_20261029_01'
ARCHIVES = ('libexp_model.a', 'libexp_search.a', 'libexp_opt.a')
REUSED = ('mm_controller_node', 'dynamic_obstacle_tracker_node')
PLANNER = ('remani_planner_node.cpp', 'remani_replan_fsm.cpp',
           'planner_manager.cpp', 'planning_visualization.cpp')
SEARCH = ('kino_astar.cpp', 'sample_mani_RRT.cpp', 'rrt.cpp')
CAPABILITIES = (
    'production_sources_sha256', 'controller_revision', 'cbf_enabled',
    'experiment_profile', 'classified_cloud', 'local_execution_owner',
    'execution_candidates', 'v12_controller', 'v12_perception',
    'default_prediction_s', 'resource_probe', 'failure_diagnosis',
    'dual_timescale_probe', 'collab_factorial', 'coupling_objective',
    'shared_collision', 'shared_collision_source_sha256',
    'delivery_diagnostics', 'domain_recovery_experiment',
    'state_contract_experiment', 'boundary_admission_experiment',
    'native_collision_3d_experiment', 'native_collision_3d_protocol',
    'native_collision_3d_switch', 'native_collision_3d_default',
    'native_collision_3d_export_switch', 'native_realtime_experiment',
    'native_dynamic_fast_experiment', 'execution_timing_pair_experiment',
    'execution_timing_default_lease_s', 'experiment_parameter_overrides',
    'planner_variants',
)


def _flags(target):
    path = FROZEN / 'build/CMakeFiles' / (target + '.dir') / 'flags.make'
    rows = path.read_text().splitlines()
    includes = [r.split('=', 1)[1] for r in rows if r.startswith('CXX_INCLUDES =')]
    flags = [r.split('=', 1)[1] for r in rows if r.startswith('CXX_FLAGS =')]
    if len(includes) != 1 or len(flags) != 1:
        raise ValueError('Ambiguous frozen flags: ' + target)
    words = shlex.split(flags[0])
    if ('-O2' not in words or '-std=gnu++17' not in words or
            '-ffast-math' in words or any(w.startswith('-O') and w != '-O2' for w in words)):
        raise ValueError('Frozen compiler profile changed: ' + target)
    return path, shlex.split(includes[0])


def _source(out, name):
    """Only unchanged visualization/RRT come directly from the frozen tree."""
    if name == 'planning_visualization.cpp':
        return FROZEN / 'source/mm_remani_planner/plan_manage/src' / name
    if name == 'rrt.cpp':
        return FROZEN / 'source/mm_remani_planner/path_searching/src' / name
    return out / 'source' / name


def recipes(out):
    out = Path(out).resolve()
    commands, flags, source_inputs = [], {}, {}
    groups = (('exp_model', ('mm_config.cpp',)), ('exp_search', SEARCH),
              ('exp_opt', ('poly_traj_optimizer.cpp',)),
              ('remani_planner_node', PLANNER), ('online_mppi', ('online_api.cpp',)))
    for target, names in groups:
        flag_path, includes = _flags(target)
        flags[str(flag_path)] = sha(flag_path)
        for name in names:
            source, obj = _source(out, name), out / 'build' / (name + '.o')
            command = ['/usr/bin/c++', '-std=gnu++17', '-O2', '-DNDEBUG', '-fPIC',
                       '-I' + str(out / 'source/include'), '-I' + str(out / 'source'),
                       *includes, '-c', str(source), '-o', str(obj)]
            commands.append(dict(target=target, source=str(source), object=str(obj), command=command))
            if not str(source).startswith(str(out) + '/'):
                source_inputs[str(source)] = sha(source)
    links, dependencies, link_hashes = {}, {}, {}
    for target in ('remani_planner_node', 'online_mppi'):
        path = FROZEN / 'build/CMakeFiles' / (target + '.dir') / 'link.txt'
        link_hashes[str(path)] = sha(path)
        link = shlex.split(path.read_text())
        objects = [arg for arg in link if arg.endswith('.o')]
        expected = PLANNER if target == 'remani_planner_node' else ('online_api.cpp',)
        if sorted(Path(v).name for v in objects) != sorted(n + '.o' for n in expected):
            raise ValueError('Unexpected frozen loose object inventory: ' + target)
        for index, arg in enumerate(link):
            if arg.endswith('.o'):
                link[index] = str(out / 'build' / Path(arg).name)
            elif arg in ARCHIVES:
                link[index] = str(out / 'build' / arg)
            elif arg.endswith('.a'):
                dependency = Path(arg)
                if not dependency.is_absolute():
                    dependency = FROZEN / 'build' / dependency
                dependency = dependency.resolve()
                dependencies[str(dependency)] = sha(dependency)
                link[index] = str(dependency)
        if link.count('-o') != 1:
            raise ValueError('Ambiguous frozen link output: ' + target)
        link[link.index('-o') + 1] = str(out / 'build' / (
            'remani_planner_node' if target == 'remani_planner_node' else 'libonline_mppi.so'))
        links[target] = link
    archive_commands = [
        ['ar', 'rcs', str(out / 'build/libexp_model.a'), str(out / 'build/mm_config.cpp.o')],
        ['ar', 'rcs', str(out / 'build/libexp_search.a'),
         *[str(out / 'build' / (name + '.o')) for name in SEARCH]],
        ['ar', 'rcs', str(out / 'build/libexp_opt.a'), str(out / 'build/poly_traj_optimizer.cpp.o')],
    ]
    # The fixed headers remain compiler inputs, not an unrecorded CMake copy.
    frozen_headers = {str(p): sha(p) for p in (FROZEN / 'source').rglob('*')
                      if p.is_file() and p.suffix in ('.h', '.hpp')}
    return dict(compile_commands=commands, archive_commands=archive_commands,
                link_commands=links, cwd=str(FROZEN / 'build'),
                inherited_flags_sha256=flags, inherited_link_sha256=link_hashes,
                reused_link_archives_sha256=dependencies,
                unchanged_source_inputs_sha256=source_inputs,
                frozen_header_inputs_sha256=frozen_headers,
                compiler_profile='C++17 -O2 -DNDEBUG -fPIC; no fast-math; child model include first')


def verify_recipe(recipe):
    for key in ('inherited_flags_sha256', 'inherited_link_sha256',
                'reused_link_archives_sha256', 'unchanged_source_inputs_sha256',
                'frozen_header_inputs_sha256'):
        for name, digest in recipe[key].items():
            checked_hash(Path(name), digest)


def _validate_base():
    manifest, _ = check_build(BASE)
    if source_inventory(BASE / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Frozen parent source inventory changed')
    if manifest.get('prepared_only') or not manifest.get('execution_timing_pair_experiment'):
        raise ValueError('Completed execution-timing parent required')
    return manifest


def prepare(out=DEFAULT_OUT):
    out = Path(out).resolve()
    if out in (BASE.resolve(), FROZEN.resolve()) or BASE in out.parents or FROZEN in out.parents:
        raise ValueError('Child must be independent of frozen parents')
    if (out / 'build_manifest.json').exists() or (out / 'build').exists():
        raise FileExistsError('Refusing to overwrite a prepared/built child: ' + str(out))
    if not (out / 'source/mm_config.cpp').is_file() or not (
            out / 'source/include/mm_config/mm_config.hpp').is_file():
        raise ValueError('Prepare the reviewed candidate B source tree first')
    parent = _validate_base()
    recipe = recipes(out)
    verify_recipe(recipe)
    (out / 'build').mkdir()
    for name in REUSED:
        shutil.copy2(BASE / 'build' / name, out / 'build' / name)
    manifest = {key: copy.deepcopy(parent[key]) for key in CAPABILITIES if key in parent}
    manifest.update(prepared_only=True, urdf_spheres_b_experiment=True,
        source_root=str(out / 'source'), parent_build=str(BASE),
        urdf_spheres_b_parent_manifest_sha256=sha(BASE / 'build_manifest.json'),
        urdf_spheres_b_parent_source_sha256=parent['isolated_source_sha256'],
        urdf_spheres_b_recipe=recipe, builder_sha256=sha(Path(__file__)),
        isolated_source_sha256=source_inventory(out / 'source'),
        urdf_spheres_b_reused_binary_sha256={name: sha(out / 'build' / name) for name in REUSED},
        urdf_spheres_b_geometry=dict(fixed_structure_spheres=47, moving_arm_spheres=20,
            variable_fixed_radii=True, old_fixed_structure_spheres=140,
            scope='URDF fixed-structure candidate B; existing moving-arm geometry retained'),
        urdf_spheres_b_rebuilt_targets=['exp_model', 'exp_search', 'exp_opt',
                                      'remani_planner_node', 'online_mppi'],
        urdf_spheres_b_preserved=dict(environment_archive=True, controller_tracker=True,
            wheel_constraints=True, optimizer_budget_and_iteration_cap=True,
            moving_arm_sphere_model=True, native_variable_radius_wire_protocol=True),
        production_modified=False, production_default=False,
        scope='isolated fixed-arm numerical SIL build; not an approved real-car configuration')
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    return out


def check(out=DEFAULT_OUT):
    out = Path(out).resolve()
    manifest = json.loads((out / 'build_manifest.json').read_text())
    if not manifest.get('urdf_spheres_b_experiment'):
        raise ValueError('Not a candidate B build')
    parent = _validate_base()
    checked_hash(BASE / 'build_manifest.json', manifest['urdf_spheres_b_parent_manifest_sha256'])
    if manifest['urdf_spheres_b_parent_source_sha256'] != parent['isolated_source_sha256']:
        raise ValueError('Parent source provenance differs')
    checked_hash(Path(__file__), manifest['builder_sha256'])
    if source_inventory(out / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child complete source inventory changed')
    recipe = recipes(out)
    if recipe != manifest['urdf_spheres_b_recipe']:
        raise ValueError('Child compile/link recipe changed')
    verify_recipe(recipe)
    for name, digest in manifest['urdf_spheres_b_reused_binary_sha256'].items():
        checked_hash(out / 'build' / name, digest)
    verify_production(out)
    if not manifest['prepared_only']:
        for name, digest in manifest['urdf_spheres_b_archive_sha256'].items():
            checked_hash(out / 'build' / name, digest)
        check_build(out)
    return manifest


def compile_prepared(out=DEFAULT_OUT):
    out = Path(out).resolve()
    manifest = check(out)
    if not manifest['prepared_only']:
        raise ValueError('Fresh prepared child required; no in-place rebuild')
    recipe = manifest['urdf_spheres_b_recipe']
    for item in recipe['compile_commands']:
        print('COMPILE', item['target'], Path(item['source']).name, flush=True)
        subprocess.run(item['command'], check=True)
    for command in recipe['archive_commands']:
        subprocess.run(command, check=True)
    for command in recipe['link_commands'].values():
        subprocess.run(command, cwd=recipe['cwd'], check=True)
    # Recheck recorded inputs after compilation, before claiming completion.
    check(out)
    manifest.update(prepared_only=False,
        urdf_spheres_b_archive_sha256={name: sha(out / 'build' / name) for name in ARCHIVES},
        native_binaries_sha256={name: sha(out / 'build' / name) for name in NATIVE},
        mppi_library_sha256=sha(out / 'build/libonline_mppi.so'))
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--compile', action='store_true')
    group.add_argument('--check', action='store_true')
    group.add_argument('--print-recipe', action='store_true')
    args = parser.parse_args()
    if args.print_recipe:
        print(json.dumps(recipes(args.out), indent=2))
    elif args.compile:
        print('BUILT', compile_prepared(args.out))
    elif args.check:
        check(args.out)
        print('CHECKED', args.out)
    else:
        print('PREPARED', prepare(args.out))


if __name__ == '__main__':
    main()
