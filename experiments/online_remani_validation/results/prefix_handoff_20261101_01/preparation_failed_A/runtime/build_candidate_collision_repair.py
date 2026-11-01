#!/usr/bin/env python3
"""Compile one isolated same-budget collision-repair A/B planner binary."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_coverage_alignment as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/coverage_alignment_20261031_01'
OUT=HERE/'builds/candidate_collision_repair_20261031_01'
HEADER='candidate_collision_repair.hpp'
TRANSFORM='candidate_collision_repair_transform.py'
TEST='test_candidate_collision_repair.cpp'
REBUILT=('poly_traj_optimizer.cpp.o','libexp_opt.a','remani_planner_node')


def parent_check():
    parent.OUT=BASE
    return parent.check()


def prepare():
    manifest=parent_check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source',OUT/'source')
    (OUT/'build').mkdir()
    for name in (HEADER,TEST):shutil.copy2(HERE/name,OUT/'source'/name)
    from candidate_collision_repair_transform import transform
    path=OUT/'source/poly_traj_optimizer.cpp'
    path.write_text(transform(path.read_text()))
    reused={}
    for path in (BASE/'build').iterdir():
        if path.name not in REBUILT:
            os.link(path,OUT/'build'/path.name);reused[path.name]=sha(path)
    recipe=recipes(OUT)
    recipe['compile_commands']=[c for c in recipe['compile_commands'] if Path(c['source']).name=='poly_traj_optimizer.cpp']
    recipe['archive_commands']=[c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    link=recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a')
    link.append('-lz')
    _,includes=_flags('exp_opt')
    recipe['repair_test_command']=['/usr/bin/c++','-std=gnu++17','-O2',
        '-I'+str(OUT/'source/include'),'-I'+str(OUT/'source'),*includes,
        str(OUT/'source'/TEST),'-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
        '-lrostime','-lcpp_common','-lrosconsole','-o',str(OUT/'build/test_candidate_collision_repair')]
    verify_recipe(recipe)
    manifest=copy.deepcopy(manifest)
    manifest.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        candidate_collision_repair_experiment=True,candidate_collision_repair_parent_sha256=sha(BASE/'build_manifest.json'),
        candidate_collision_repair_reused_sha256=reused,candidate_collision_repair_recipe=recipe,
        candidate_collision_repair_implementation_sha256={name:sha(HERE/name) for name in ('build_candidate_collision_repair.py',HEADER,TRANSFORM,TEST)},
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False,
        candidate_collision_repair_scope='same binary A/B; factor10 one original failing environmental-collision piece; original remaining optimizer budget and native full/half/static/self/wheel/latest gates retained; actual x reconstruction and fresh accepted/early scopes')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return check()


def check():
    parent_check();manifest=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',manifest['candidate_collision_repair_parent_sha256'])
    for name,digest in manifest['candidate_collision_repair_implementation_sha256'].items():checked_hash(HERE/name,digest)
    for name,digest in manifest['candidate_collision_repair_reused_sha256'].items():checked_hash(OUT/'build'/name,digest)
    if source_inventory(OUT/'source')!=manifest['isolated_source_sha256']:raise ValueError('Child source inventory changed')
    verify_recipe(manifest['candidate_collision_repair_recipe'])
    if not manifest['prepared_only']:
        for name,digest in manifest['candidate_collision_repair_compiled_sha256'].items():checked_hash(OUT/'build'/name,digest)
        checked_hash(OUT/'repair_gradient_check.json',manifest['candidate_collision_repair_gradient_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest=check()
    if not manifest['prepared_only']:raise RuntimeError('Completed build is immutable')
    recipe=manifest['candidate_collision_repair_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=recipe['cwd'],check=True)
    for command in recipe['archive_commands']:subprocess.run(command,cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'],cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['repair_test_command'],cwd=recipe['cwd'],check=True)
    result=subprocess.run([str(OUT/'build/test_candidate_collision_repair')],check=True,capture_output=True,text=True)
    record=json.loads(result.stdout)
    if not record['pass']:raise ValueError('Collision weight gradient check failed')
    (OUT/'repair_gradient_check.json').write_text(json.dumps(record,indent=2)+'\n')
    manifest.update(prepared_only=False,native_binaries_sha256={name:sha(OUT/'build'/name) for name in NATIVE},
        candidate_collision_repair_compiled_sha256={name:sha(OUT/'build'/name) for name in (*REBUILT,'test_candidate_collision_repair')},
        candidate_collision_repair_gradient_check_sha256=sha(OUT/'repair_gradient_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return check()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('prepare','compile','check'))
    args=parser.parse_args();{'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('CANDIDATE_COLLISION_REPAIR_BUILD',args.action,OUT,'OK',flush=True)
