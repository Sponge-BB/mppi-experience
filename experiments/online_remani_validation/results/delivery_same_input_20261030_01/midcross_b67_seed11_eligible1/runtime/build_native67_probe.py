#!/usr/bin/env python3
"""Isolated native67 diagnostic; rebuild optimizer and its four planner TUs."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

from build_urdf_spheres_b import recipes, verify_recipe, ARCHIVES
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from run_coupled_objective import check_build
from patch_native67_probe import transform_optimizer, transform_replay, transform_lbfgs

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/urdf_sphere_timing_20261029_01'
DEFAULT=HERE/'builds/native67_probe_20261030_01'


def prepare(out=DEFAULT):
    out=Path(out).resolve()
    if out.exists():raise FileExistsError(out)
    parent,_=check_build(BASE)
    if source_inventory(BASE/'source')!=parent['isolated_source_sha256']:
        raise ValueError('Parent source inventory differs')
    if not parent.get('urdf_sphere_timing_experiment'):raise ValueError('Wrong parent')
    out.mkdir(parents=True)
    shutil.copytree(BASE/'source',out/'source');(out/'build').mkdir()
    edits={'poly_traj_optimizer.cpp':transform_optimizer,'delivery_replay_probe.hpp':transform_replay,
           'domain_lbfgs.hpp':transform_lbfgs}
    for name, transform in edits.items():
        p=out/'source'/name;p.write_text(transform(p.read_text()))
    shutil.copy2(HERE/'native67_probe.hpp',out/'source/native67_probe.hpp')
    recipe=recipes(out)
    recipe['compile_commands']=[r for r in recipe['compile_commands'] if r['target'] in ('exp_opt','remani_planner_node')]
    recipe['archive_commands']=[c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    verify_recipe(recipe)
    reuse=('mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so','libexp_model.a','libexp_search.a')
    for name in reuse:shutil.copy2(BASE/'build'/name,out/'build'/name)
    m=copy.deepcopy(parent)
    m.update(prepared_only=True,source_root=str(out/'source'),parent_build=str(BASE),
      binary_unchanged_from_parent=False,compiled_in_this_preparation=True,
      scope='Paused same-process native67 optimizer diagnostic; not live delivery or hardware experiment',
      native67_probe_experiment=True,native67_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
      native67_parent_sources_sha256=parent['isolated_source_sha256'],
      native67_recipe=recipe,native67_reused_artifacts_sha256={name:sha(out/'build'/name) for name in reuse},
      native67_implementation_sha256={name:sha(HERE/name) for name in ('patch_native67_probe.py','native67_probe.hpp','build_native67_probe.py')},
      isolated_source_sha256=source_inventory(out/'source'),
      native67_scope={'budgets_s':[.6,3.0],'stop_rule':'unchanged native','final_gate':'unchanged native IsTrajSafe; no additional projection/deadline/cache gate',
      'capture':'first eligible ordinary refresh with nonempty valid native dynamic snapshot; no selection by outcome',
      'post_solver_audit':'sampled initial,last accepted,returned x at .01s; budget excluded; never published',
      'shape':'47 fixed plus unchanged 20 moving arm spheres','production_changed':False})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    check(out);return out


def check(out=DEFAULT):
    out=Path(out).resolve();m=json.loads((out/'build_manifest.json').read_text())
    if not m.get('native67_probe_experiment'):raise ValueError('Wrong diagnostic build')
    parent,_=check_build(BASE)
    checked_hash(BASE/'build_manifest.json',m['native67_parent_manifest_sha256'])
    if source_inventory(BASE/'source')!=m['native67_parent_sources_sha256']:raise ValueError('Parent changed')
    if source_inventory(out/'source')!=m['isolated_source_sha256']:raise ValueError('Child changed')
    for name,digest in m['native67_reused_artifacts_sha256'].items():checked_hash(out/'build'/name,digest)
    for name,digest in m['native67_implementation_sha256'].items():checked_hash(HERE/name,digest)
    verify_recipe(m['native67_recipe'])
    if not m['prepared_only']:check_build(out)
    return m


def compile_prepared(out=DEFAULT):
    out=Path(out).resolve();m=check(out)
    if not m['prepared_only']:raise ValueError('Already built')
    r=m['native67_recipe']
    for item in r['compile_commands']:
        print('COMPILE',item['target'],Path(item['source']).name,flush=True)
        subprocess.run(item['command'],check=True)
    for c in r['archive_commands']:subprocess.run(c,check=True)
    for c in r['link_commands'].values():subprocess.run(c,cwd=r['cwd'],check=True)
    check(out)
    m.update(prepared_only=False,native_binaries_sha256={n:sha(out/'build'/n) for n in NATIVE},
      mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
      urdf_spheres_b_archive_sha256={n:sha(out/'build'/n) for n in ARCHIVES})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=DEFAULT)
    g=p.add_mutually_exclusive_group();g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    a=p.parse_args()
    print(compile_prepared(a.out) if a.compile else check(a.out) if a.check else prepare(a.out))
