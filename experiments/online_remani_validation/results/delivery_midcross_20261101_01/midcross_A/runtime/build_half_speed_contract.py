#!/usr/bin/env python3
"""Isolated slow-reference admission ablation; actual control checker reused."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_early_collision_repair as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/early_collision_repair_20261101_01'
OUT=HERE/'builds/half_speed_contract_20261101_01'
FILES=('slow_reference_contract.hpp','half_speed_contract_transform.py','test_half_speed_contract.cpp')
REBUILT=('poly_traj_optimizer.cpp.o','remani_replan_fsm.cpp.o','libexp_opt.a','remani_planner_node')


def prepare():
    manifest=parent.check();OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source',OUT/'source');(OUT/'build').mkdir()
    for name in FILES:shutil.copy2(HERE/name,OUT/'source'/name)
    from half_speed_contract_transform import transform_collision,transform_fsm
    for name,transform in (('coverage_collision.hpp',transform_collision),('remani_replan_fsm.cpp',transform_fsm)):
        path=OUT/'source'/name;path.write_text(transform(path.read_text()))
    reused={}
    for path in (BASE/'build').iterdir():
        if path.name not in REBUILT:os.link(path,OUT/'build'/path.name);reused[path.name]=sha(path)
    recipe=recipes(OUT)
    recipe['compile_commands']=[c for c in recipe['compile_commands'] if Path(c['source']).name in ('poly_traj_optimizer.cpp','remani_replan_fsm.cpp')]
    recipe['archive_commands']=[c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    link=recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a');link.append('-lz')
    _,includes=_flags('exp_opt')
    recipe['contract_test_command']=['/usr/bin/c++','-std=gnu++17','-O2',
        '-I'+str(OUT/'source/include'),'-I'+str(OUT/'source'),*includes,
        str(OUT/'source/test_half_speed_contract.cpp'),'-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
        '-lrostime','-lcpp_common','-lrosconsole','-o',str(OUT/'build/test_half_speed_contract')]
    verify_recipe(recipe);manifest=copy.deepcopy(manifest)
    manifest.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        half_speed_contract_experiment=True,half_contract_parent_sha256=sha(BASE/'build_manifest.json'),
        half_contract_reused_sha256=reused,half_contract_recipe=recipe,
        half_contract_implementation_sha256={n:sha(HERE/n) for n in ('build_half_speed_contract.py',*FILES)},
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False,
        half_contract_scope='same binary both versus nominal-only slow hard reference gates; soft full+half objective and actual MPPI safety library unchanged; early/late repair disabled both groups; no pause')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return check()


def check():
    parent.check();manifest=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',manifest['half_contract_parent_sha256'])
    for name,digest in manifest['half_contract_implementation_sha256'].items():checked_hash(HERE/name,digest)
    for name,digest in manifest['half_contract_reused_sha256'].items():checked_hash(OUT/'build'/name,digest)
    if source_inventory(OUT/'source')!=manifest['isolated_source_sha256']:raise ValueError('Child sources changed')
    verify_recipe(manifest['half_contract_recipe'])
    if not manifest['prepared_only']:
        for name,digest in manifest['half_contract_compiled_sha256'].items():checked_hash(OUT/'build'/name,digest)
        checked_hash(OUT/'half_contract_check.json',manifest['half_contract_check_sha256']);check_build(OUT)
    return manifest


def compile_child():
    manifest=check()
    if not manifest['prepared_only']:raise RuntimeError('Completed child is immutable')
    r=manifest['half_contract_recipe']
    for command in r['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=r['cwd'],check=True)
    for command in r['archive_commands']:subprocess.run(command,cwd=r['cwd'],check=True)
    subprocess.run(r['link_commands']['remani_planner_node'],cwd=r['cwd'],check=True)
    subprocess.run(r['contract_test_command'],cwd=r['cwd'],check=True)
    test=subprocess.run([str(OUT/'build/test_half_speed_contract')],capture_output=True,text=True,check=True)
    record=json.loads(test.stdout)
    if not record['pass']:raise ValueError('Slow reference contract check failed')
    (OUT/'half_contract_check.json').write_text(json.dumps(record,indent=2)+'\n')
    manifest.update(prepared_only=False,native_binaries_sha256={n:sha(OUT/'build'/n) for n in NATIVE},
        half_contract_compiled_sha256={n:sha(OUT/'build'/n) for n in (*REBUILT,'test_half_speed_contract')},
        half_contract_check_sha256=sha(OUT/'half_contract_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return check()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=('prepare','compile','check'))
    args=p.parse_args();{'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('HALF_SPEED_CONTRACT_BUILD',args.action,OUT,'OK',flush=True)
