#!/usr/bin/env python3
"""Isolated Kino A* parameter probe and minimal selectable frontend correction."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_static_width_diagnosis as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, recipes, verify_recipe
from native_collision_runtime import replace_once
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/static_width_diagnosis_20261031_01'
OUT=HERE/'builds/frontend_rejoin_20261031_01'
REBUILT=('kino_astar.cpp.o','libexp_search.a','remani_planner_node')

def transform(text):
    text='#include "frontend_rejoin_probe.hpp"\n'+text
    text=replace_once(text,'  nh.param("search/allow_reverse", allow_reverse_, true);',
        '  frontend_rejoin::apply(frontend_rejoin::mode(),traj_steer_penalty_,\n'
        '    traj_steer_change_penalty_,inv_yaw_resolution_);\n'
        '  nh.param("search/allow_reverse", allow_reverse_, true);')
    return replace_once(text,'  reset();\n  simple_path_container.clear();',
        (HERE/'frontend_rejoin_probe.inc').read_text()+'\n  reset();\n  simple_path_container.clear();')

def parent_check():
    parent.OUT=BASE
    return parent.check()

def prepare():
    manifest=parent_check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source',OUT/'source');(OUT/'build').mkdir()
    path=OUT/'source/kino_astar.cpp';path.write_text(transform(path.read_text()))
    shutil.copy2(HERE/'frontend_rejoin_probe.hpp',OUT/'source/frontend_rejoin_probe.hpp')
    reused={}
    for path in (BASE/'build').iterdir():
        if path.name not in REBUILT:
            os.link(path,OUT/'build'/path.name);reused[path.name]=sha(path)
    # Parent retains the other search objects only inside its immutable archive.
    subprocess.run(['ar','x',str(BASE/'build/libexp_search.a'),
                    'sample_mani_RRT.cpp.o','rrt.cpp.o'],cwd=str(OUT/'build'),check=True)
    for name in ('sample_mani_RRT.cpp.o','rrt.cpp.o'):reused[name]=sha(OUT/'build'/name)
    recipe=recipes(OUT)
    recipe['compile_commands']=[c for c in recipe['compile_commands'] if Path(c['source']).name=='kino_astar.cpp']
    recipe['archive_commands']=[c for c in recipe['archive_commands'] if c[2].endswith('/libexp_search.a')]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    link=recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a')
    link.append('-lz');verify_recipe(recipe)
    manifest=copy.deepcopy(manifest)
    manifest.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        frontend_rejoin=True,frontend_rejoin_parent_sha256=sha(BASE/'build_manifest.json'),
        frontend_rejoin_reused_sha256=reused,frontend_rejoin_recipe=recipe,
        frontend_rejoin_implementation_sha256={name:sha(HERE/name) for name in
            ('build_frontend_rejoin.py','frontend_rejoin_probe.hpp','frontend_rejoin_probe.inc')},
        frontend_rejoin_modes=dict(baseline='unchanged',penalty='steering penalties x0.25',
            yaw='yaw resolution 0.3 rad',both='both changes'),
        frontend_rejoin_probe='one paused input, four native searches plus baseline repeat, no candidate publication',
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False)
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check();return OUT

def check():
    parent_check();manifest=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',manifest['frontend_rejoin_parent_sha256'])
    for name,digest in manifest['frontend_rejoin_implementation_sha256'].items():checked_hash(HERE/name,digest)
    for name,digest in manifest['frontend_rejoin_reused_sha256'].items():checked_hash(OUT/'build'/name,digest)
    if source_inventory(OUT/'source')!=manifest['isolated_source_sha256']:raise ValueError('Child sources changed')
    verify_recipe(manifest['frontend_rejoin_recipe'])
    if not manifest['prepared_only']:
        for name,digest in manifest['frontend_rejoin_compiled_sha256'].items():checked_hash(OUT/'build'/name,digest)
        check_build(OUT)
    return manifest

def compile_child():
    manifest=check()
    if not manifest['prepared_only']:raise RuntimeError('Completed build is immutable')
    recipe=manifest['frontend_rejoin_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=recipe['cwd'],check=True)
    for command in recipe['archive_commands']:subprocess.run(command,cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'],cwd=recipe['cwd'],check=True)
    manifest.update(prepared_only=False,native_binaries_sha256={name:sha(OUT/'build'/name) for name in NATIVE},
        frontend_rejoin_compiled_sha256={name:sha(OUT/'build'/name) for name in REBUILT})
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check();return OUT

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('prepare','compile','check'))
    parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args();OUT=args.out.resolve()
    {'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('FRONTEND_REJOIN_BUILD',args.action,OUT,'OK',flush=True)
