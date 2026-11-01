#!/usr/bin/env python3
"""Private optimizer-only child adding runtime proof of solver overlay ownership."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from build_urdf_spheres_b import recipes, verify_recipe, ARCHIVES, PLANNER, FROZEN
from build_candidate_audit_consistency import check as check_parent
from build_state_contract import Transformer
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = HERE/'builds/candidate_audit_20261030_02'
DEFAULT = HERE/'builds/candidate_audit_20261030_03'
IMPLEMENTATION = ('build_candidate_solver_pin.py',)


def transform(text):
    t = Transformer(text)
    anchor = '''    delivery_replay::Finally candidate_solver_overlay_release([&](){
      if(candidate_solver_overlay_owned)GridMapParallelTestAccessor::releaseCurrentOverlay(*grid_map_);
    });'''
    t.after(anchor, '''
    delivery_replay::Finally candidate_solver_pin_observation([&](){
      probe.add("solver_overlay_freeze_requested",candidate_solver_freeze_requested?1:0);
      probe.add("solver_overlay_pin_owned",candidate_solver_overlay_owned?1:0);
      probe.add("overlay_evaluate_complete_count",candidate_record.evaluate_complete_count);
      probe.add("overlay_evaluate_changed_count",candidate_record.evaluate_overlay_changed_count);
      probe.add("solver_overlay_entry_key",candidate_record.solver_entry_key);
      probe.add("solver_overlay_return_key",candidate_record.context?candidate_record.context():"null");
    });''')
    return t.finish()


def recipe(out):
    r = recipes(out)
    r['compile_commands'] = [x for x in r['compile_commands'] if x['target'] == 'exp_opt']
    r['archive_commands'] = [x for x in r['archive_commands'] if x[2].endswith('/libexp_opt.a')]
    link = r['link_commands']['remani_planner_node']
    old = str(FROZEN/'build/libexp_env.a')
    if link.count(old) != 1:
        raise ValueError('Expected exactly one environment archive')
    link[link.index(old)] = str(out/'build/libexp_env.a')
    link.append('-lz')
    r['link_commands'] = {'remani_planner_node': link}
    return r


def prepare(out=DEFAULT):
    out = Path(out).resolve()
    if out.exists():
        raise FileExistsError(out)
    parent = check_parent(BASE)
    out.mkdir(parents=True)
    shutil.copytree(BASE/'source', out/'source')
    (out/'build').mkdir()
    source = out/'source/poly_traj_optimizer.cpp'
    source.write_text(transform(source.read_text()))
    reuse = ['mm_controller_node', 'dynamic_obstacle_tracker_node', 'libonline_mppi.so',
             'libexp_model.a', 'libexp_search.a', 'libexp_env.a']
    reuse += [name+'.o' for name in PLANNER]
    for name in reuse:
        shutil.copy2(BASE/'build'/name, out/'build'/name)
    r = recipe(out)
    verify_recipe(r)
    manifest = copy.deepcopy(parent)
    manifest.update(
        prepared_only=True, parent_build=str(BASE), source_root=str(out/'source'),
        candidate_solver_pin_experiment=True,
        candidate_solver_pin_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
        candidate_solver_pin_implementation_sha256={name: sha(HERE/name) for name in IMPLEMENTATION},
        candidate_solver_pin_reused_artifacts_sha256={name: sha(out/'build'/name) for name in reuse},
        candidate_solver_pin_recipe=r,
        candidate_solver_pin_source_changes=['poly_traj_optimizer.cpp'],
        candidate_solver_pin_runtime_observation_fields=[
            'solver_overlay_freeze_requested', 'solver_overlay_pin_owned',
            'overlay_evaluate_complete_count', 'overlay_evaluate_changed_count',
            'solver_overlay_entry_key', 'solver_overlay_return_key'],
        scope='same solver pin switch and unchanged acceptance; coarse resource end-record proves requested/owned pin and overlay versions before release',
        isolated_source_sha256=source_inventory(out/'source'))
    (out/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    check(out)
    return out


def check(out=DEFAULT):
    out = Path(out).resolve()
    manifest = json.loads((out/'build_manifest.json').read_text())
    check_parent(BASE)
    checked_hash(BASE/'build_manifest.json', manifest['candidate_solver_pin_parent_manifest_sha256'])
    for name, digest in manifest['candidate_solver_pin_implementation_sha256'].items():
        checked_hash(HERE/name, digest)
    if source_inventory(out/'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child source changed')
    if recipe(out) != manifest['candidate_solver_pin_recipe']:
        raise ValueError('Recipe changed')
    verify_recipe(manifest['candidate_solver_pin_recipe'])
    for name, digest in manifest['candidate_solver_pin_reused_artifacts_sha256'].items():
        checked_hash(out/'build'/name, digest)
        checked_hash(BASE/'build'/name, digest)
    if not manifest['prepared_only']:
        for name, digest in manifest['candidate_solver_pin_archives_sha256'].items():
            checked_hash(out/'build'/name, digest)
        check_build(out)
    return manifest


def compile_prepared(out=DEFAULT):
    out = Path(out).resolve()
    manifest = check(out)
    if not manifest['prepared_only']:
        raise ValueError('Already compiled')
    r = manifest['candidate_solver_pin_recipe']
    for item in r['compile_commands']:
        print('COMPILE', item['target'], Path(item['source']).name, flush=True)
        subprocess.run(item['command'], check=True)
    for command in r['archive_commands']:
        subprocess.run(command, check=True)
    for command in r['link_commands'].values():
        subprocess.run(command, cwd=r['cwd'], check=True)
    check(out)
    manifest.update(
        prepared_only=False,
        native_binaries_sha256={name: sha(out/'build'/name) for name in NATIVE},
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        urdf_spheres_b_archive_sha256={name: sha(out/'build'/name) for name in ARCHIVES},
        candidate_solver_pin_archives_sha256={name: sha(out/'build'/name)
                                              for name in (*ARCHIVES, 'libexp_env.a')})
    (out/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    check(out)
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=DEFAULT)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--compile', action='store_true')
    group.add_argument('--check', action='store_true')
    args = parser.parse_args()
    result = compile_prepared(args.out) if args.compile else check(args.out) if args.check else prepare(args.out)
    print('CHECKED' if args.check else result)
