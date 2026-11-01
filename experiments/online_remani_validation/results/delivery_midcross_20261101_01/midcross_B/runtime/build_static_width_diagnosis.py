#!/usr/bin/env python3
"""Isolated read-only search/MINCO/final-curve trace child, no production edits."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_static_agility as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, recipes, verify_recipe
from native_collision_runtime import replace_once
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/static_agility_20261031_03'
OUT=HERE/'builds/static_width_diagnosis_20261031_01'
UNITS=('planner_manager.cpp','poly_traj_optimizer.cpp')
REBUILT=tuple(n+'.o' for n in UNITS)+('libexp_opt.a','remani_planner_node')


def transform_manager(text):
    text='#include "static_width_trace.hpp"\n'+text
    text=replace_once(text,'    Eigen::VectorXd init_len(7);',
        '    static_width_trace::curves("minco_before_optimizer",initMJO_container,\n'
        '      singul_container,true,request_probe.id);\n\n'
        '    Eigen::VectorXd init_len(7);')
    return replace_once(text,'    if(state_contract_trace::enabled()){\n'
        '      state_contract_trace::queue("state_contract_solver_return",',
        '    static_width_trace::curves("optimizer_return",\n'
        '      *ploy_traj_opt_->getMinSnapOptContainerPtr(),singul_container,flag_success,request_probe.id);\n'
        '    if(state_contract_trace::enabled()){\n'
        '      state_contract_trace::queue("state_contract_solver_return",')


def transform_optimizer(text):
    text='#include "static_width_trace.hpp"\n'+text
    return replace_once(text,
        '    if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_native_astar_result",',
        '    static_width_trace::search(simple_path_container,yaw_list_container,t_list_container,\n'
        '      singul_container,status,iniState,finState);\n'
        '    if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_native_astar_result",')


def parent_check():
    parent.OUT=BASE
    return parent.check()


def prepare():
    m=parent_check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source',OUT/'source')
    (OUT/'build').mkdir()
    for name,transform in ((UNITS[0],transform_manager),(UNITS[1],transform_optimizer)):
        path=OUT/'source'/name;path.write_text(transform(path.read_text()))
    shutil.copy2(HERE/'static_width_trace.hpp',OUT/'source/static_width_trace.hpp')
    reused={}
    for path in (BASE/'build').iterdir():
        if path.name not in REBUILT:
            os.link(path,OUT/'build'/path.name);reused[path.name]=sha(path)
    recipe=recipes(OUT)
    recipe['compile_commands']=[c for c in recipe['compile_commands'] if Path(c['source']).name in UNITS]
    recipe['archive_commands']=[c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    link=recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a')
    link.append('-lz');verify_recipe(recipe)
    m=copy.deepcopy(m)
    m.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        static_width_diagnosis=True,static_width_parent_sha256=sha(BASE/'build_manifest.json'),
        static_width_reused_sha256=reused,static_width_recipe=recipe,
        static_width_implementation_sha256={name:sha(HERE/name) for name in
            ('build_static_width_diagnosis.py','static_width_trace.hpp')},
        static_width_switch='STATIC_WIDTH_TRACE_PATH; requires STATE_CONTRACT_CAPTURE=1',
        static_width_scope='Read-only native raw search, pre-optimizer MINCO, optimizer return; deferred request-end output',
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False)
    (OUT/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    check();return OUT


def check():
    parent_check()
    m=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',m['static_width_parent_sha256'])
    for name,digest in m['static_width_implementation_sha256'].items():checked_hash(HERE/name,digest)
    for name,digest in m['static_width_reused_sha256'].items():checked_hash(OUT/'build'/name,digest)
    if source_inventory(OUT/'source')!=m['isolated_source_sha256']:raise ValueError('Child sources changed')
    verify_recipe(m['static_width_recipe'])
    if not m['prepared_only']:
        for name,digest in m['static_width_compiled_sha256'].items():checked_hash(OUT/'build'/name,digest)
        check_build(OUT)
    return m


def compile_child():
    m=check()
    if not m['prepared_only']:raise RuntimeError('Completed build is immutable')
    recipe=m['static_width_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=recipe['cwd'],check=True)
    for command in recipe['archive_commands']:
        subprocess.run(command,cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'],cwd=recipe['cwd'],check=True)
    m.update(prepared_only=False,native_binaries_sha256={name:sha(OUT/'build'/name) for name in NATIVE},
        static_width_compiled_sha256={name:sha(OUT/'build'/name) for name in REBUILT})
    (OUT/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    check();return OUT


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('prepare','compile','check'))
    parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args();OUT=args.out.resolve()
    result={'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('STATIC_WIDTH_DIAGNOSIS',args.action,OUT,'OK',flush=True)
