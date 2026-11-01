#!/usr/bin/env python3
"""Isolated child: continuous dynamic soft integral within the finite CV window."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_candidate_exit_audit as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from native_collision_runtime import replace_once
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/candidate_exit_audit_20261101_01'
OUT=HERE/'builds/window_continuity_20261101_01'
FILES=('window_continuity.hpp','window_continuity_transform.py','test_window_continuity.cpp')
REBUILT=('poly_traj_optimizer.cpp.o','libexp_opt.a','remani_planner_node')


def prepare():
    manifest=parent.check();OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source',OUT/'source');(OUT/'build').mkdir()
    for name in FILES:shutil.copy2(HERE/name,OUT/'source'/name)
    from window_continuity_transform import transform_optimizer
    p=OUT/'source/poly_traj_optimizer.cpp';p.write_text(transform_optimizer(p.read_text()))
    from rootcause_map_loader import METHOD
    p=OUT/'source/candidate_audit.hpp';text=p.read_text()
    text=replace_once(text,'#include <Eigen/Dense>\n',
        '#include <Eigen/Dense>\n#include <boost/property_tree/ptree.hpp>\n#include <boost/property_tree/json_parser.hpp>\n')
    p.write_text(replace_once(text,'class GridMapParallelTestAccessor {\n public:\n',
        'class GridMapParallelTestAccessor {\n public:\n'+METHOD))
    reused={}
    for p in (BASE/'build').iterdir():
        if p.name not in REBUILT:os.link(p,OUT/'build'/p.name);reused[p.name]=sha(p)
    recipe=recipes(OUT)
    recipe['compile_commands']=[c for c in recipe['compile_commands'] if Path(c['source']).name=='poly_traj_optimizer.cpp']
    recipe['archive_commands']=[c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    link=recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a');link.append('-lz')
    _,includes=_flags('exp_opt')
    recipe['window_test_command']=['/usr/bin/c++','-std=gnu++17','-O2',
        '-I'+str(OUT/'source/include'),'-I'+str(OUT/'source'),*includes,
        str(OUT/'source/test_window_continuity.cpp'),'-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
        '-lrostime','-lcpp_common','-lrosconsole','-o',str(OUT/'build/test_window_continuity')]
    verify_recipe(recipe);manifest=copy.deepcopy(manifest)
    manifest.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        window_continuity_experiment=True,window_parent_sha256=sha(BASE/'build_manifest.json'),
        window_reused_sha256=reused,window_recipe=recipe,
        window_implementation_sha256={n:sha(HERE/n) for n in ('build_window_continuity.py',*FILES)},
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False,
        window_scope='same binary old/fixed objective; fixed CV intervals clipped per polynomial piece; dynamic environmental increment only; complete moving-node and integration-weight derivatives; native hard checks retained; no pause online')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return check()


def check():
    parent.check();manifest=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',manifest['window_parent_sha256'])
    for n,d in manifest['window_implementation_sha256'].items():checked_hash(HERE/n,d)
    for n,d in manifest['window_reused_sha256'].items():checked_hash(OUT/'build'/n,d)
    if source_inventory(OUT/'source')!=manifest['isolated_source_sha256']:raise ValueError('Child sources changed')
    verify_recipe(manifest['window_recipe'])
    if not manifest['prepared_only']:
        for n,d in manifest['window_compiled_sha256'].items():checked_hash(OUT/'build'/n,d)
        checked_hash(OUT/'window_gradient_check.json',manifest['window_gradient_check_sha256']);check_build(OUT)
    return manifest


def compile_child():
    manifest=check()
    if not manifest['prepared_only']:raise RuntimeError('Completed child is immutable')
    r=manifest['window_recipe']
    for command in r['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=r['cwd'],check=True)
    for command in r['archive_commands']:subprocess.run(command,cwd=r['cwd'],check=True)
    subprocess.run(r['link_commands']['remani_planner_node'],cwd=r['cwd'],check=True)
    subprocess.run(r['window_test_command'],cwd=r['cwd'],check=True)
    test=subprocess.run([str(OUT/'build/test_window_continuity')],capture_output=True,text=True,check=True)
    record=json.loads(test.stdout)
    if not record['pass']:raise ValueError('Window derivative check failed')
    (OUT/'window_gradient_check.json').write_text(json.dumps(record,indent=2)+'\n')
    manifest.update(prepared_only=False,native_binaries_sha256={n:sha(OUT/'build'/n) for n in NATIVE},
        window_compiled_sha256={n:sha(OUT/'build'/n) for n in (*REBUILT,'test_window_continuity')},
        window_gradient_check_sha256=sha(OUT/'window_gradient_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return check()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=('prepare','compile','check'))
    args=p.parse_args();{'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('WINDOW_CONTINUITY_BUILD',args.action,OUT,'OK',flush=True)
