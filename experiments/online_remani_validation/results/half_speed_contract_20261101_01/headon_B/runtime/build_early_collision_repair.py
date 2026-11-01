#!/usr/bin/env python3
"""Isolated child: earlier yield to the existing one-shot native collision repair."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_window_continuity as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/window_continuity_20261101_01'
OUT=HERE/'builds/early_collision_repair_20261101_01'
FILES=('early_collision_repair.hpp','early_collision_repair_transform.py','test_early_collision_repair.cpp')
REBUILT=('poly_traj_optimizer.cpp.o','libexp_opt.a','remani_planner_node')


def prepare():
    manifest=parent.check();OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source',OUT/'source');(OUT/'build').mkdir()
    for name in FILES:shutil.copy2(HERE/name,OUT/'source'/name)
    from early_collision_repair_transform import transform_optimizer,transform_gate,transform_window
    for name,transform in (('poly_traj_optimizer.cpp',transform_optimizer),
                           ('early_delivery_gate.hpp',transform_gate),('window_continuity.hpp',transform_window)):
        path=OUT/'source'/name;path.write_text(transform(path.read_text()))
    reused={}
    for path in (BASE/'build').iterdir():
        if path.name not in REBUILT:os.link(path,OUT/'build'/path.name);reused[path.name]=sha(path)
    recipe=recipes(OUT)
    recipe['compile_commands']=[c for c in recipe['compile_commands'] if Path(c['source']).name=='poly_traj_optimizer.cpp']
    recipe['archive_commands']=[c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    link=recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a');link.append('-lz')
    _,includes=_flags('exp_opt')
    recipe['policy_test_command']=['/usr/bin/c++','-std=gnu++17','-O2',
        '-I'+str(OUT/'source/include'),'-I'+str(OUT/'source'),*includes,
        str(OUT/'source/test_early_collision_repair.cpp'),'-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
        '-lrostime','-lcpp_common','-lrosconsole','-o',str(OUT/'build/test_early_collision_repair')]
    verify_recipe(recipe);manifest=copy.deepcopy(manifest)
    manifest.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        early_collision_repair_experiment=True,early_repair_parent_sha256=sha(BASE/'build_manifest.json'),
        early_repair_reused_sha256=reused,early_repair_recipe=recipe,
        early_repair_implementation_sha256={n:sha(HERE/n) for n in ('build_early_collision_repair.py',*FILES)},
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False,
        early_repair_scope='same binary late versus early triggering of existing failed-piece environment weight10 repair; same original deadline; accepted physical x reconstructed; environment types0/1 only; native/latest checks retained; no pause')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return check()


def check():
    parent.check();manifest=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',manifest['early_repair_parent_sha256'])
    for name,digest in manifest['early_repair_implementation_sha256'].items():checked_hash(HERE/name,digest)
    for name,digest in manifest['early_repair_reused_sha256'].items():checked_hash(OUT/'build'/name,digest)
    if source_inventory(OUT/'source')!=manifest['isolated_source_sha256']:raise ValueError('Child sources changed')
    verify_recipe(manifest['early_repair_recipe'])
    if not manifest['prepared_only']:
        for name,digest in manifest['early_repair_compiled_sha256'].items():checked_hash(OUT/'build'/name,digest)
        checked_hash(OUT/'early_repair_policy_check.json',manifest['early_repair_policy_check_sha256']);check_build(OUT)
    return manifest


def compile_child():
    manifest=check()
    if not manifest['prepared_only']:raise RuntimeError('Completed child is immutable')
    r=manifest['early_repair_recipe']
    for command in r['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=r['cwd'],check=True)
    for command in r['archive_commands']:subprocess.run(command,cwd=r['cwd'],check=True)
    subprocess.run(r['link_commands']['remani_planner_node'],cwd=r['cwd'],check=True)
    subprocess.run(r['policy_test_command'],cwd=r['cwd'],check=True)
    test=subprocess.run([str(OUT/'build/test_early_collision_repair')],capture_output=True,text=True,check=True)
    record=json.loads(test.stdout)
    if not record['pass']:raise ValueError('Early repair policy check failed')
    (OUT/'early_repair_policy_check.json').write_text(json.dumps(record,indent=2)+'\n')
    manifest.update(prepared_only=False,native_binaries_sha256={n:sha(OUT/'build'/n) for n in NATIVE},
        early_repair_compiled_sha256={n:sha(OUT/'build'/n) for n in (*REBUILT,'test_early_collision_repair')},
        early_repair_policy_check_sha256=sha(OUT/'early_repair_policy_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return check()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=('prepare','compile','check'))
    args=p.parse_args();{'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('EARLY_COLLISION_REPAIR_BUILD',args.action,OUT,'OK',flush=True)
