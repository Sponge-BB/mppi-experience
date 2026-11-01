#!/usr/bin/env python3
"""Full frozen-CV coverage plus coordinated activation in one isolated A/B binary."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_time_indexed_collision as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from native_collision_runtime import replace_once
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/time_indexed_collision_20261031_01'
OUT = HERE / 'builds/coverage_alignment_20261031_01'
HEADERS = ('coverage_collision.hpp', 'coverage_alignment.hpp')
TRANSFORM = 'coverage_alignment_transform.py'
TEST_SOURCE = 'test_coverage_gradient.cpp'
REBUILT = ('poly_traj_optimizer.cpp.o', 'remani_replan_fsm.cpp.o', 'planner_manager.cpp.o',
           'libexp_opt.a', 'remani_planner_node')


def optimizer(text):
    return replace_once(text, '#include "time_indexed_collision.hpp"',
                        '#include "coverage_collision.hpp"')


def fsm(text):
    # Root supplies the coordinated boundary/activation transformation before
    # preparation. It is hash-pinned with the rest of this experiment.
    from coverage_alignment_transform import transform_fsm
    return replace_once(transform_fsm(text), '#include "time_indexed_collision.hpp"',
                        '#include "coverage_collision.hpp"')


def manager(text):
    from coverage_alignment_transform import transform_manager
    return transform_manager(text)


def parent_check():
    parent.OUT = BASE
    return parent.check()


def prepare():
    manifest = parent_check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE / 'source', OUT / 'source')
    (OUT / 'build').mkdir()
    for name in (*HEADERS, TEST_SOURCE):
        shutil.copy2(HERE / name, OUT / 'source' / name)
    for name, transform in (('poly_traj_optimizer.cpp', optimizer), ('remani_replan_fsm.cpp', fsm),
                            ('planner_manager.cpp', manager)):
        path = OUT / 'source' / name
        path.write_text(transform(path.read_text()))
    capture = OUT / 'source/temporal_collision_capture.hpp'
    capture.write_text(replace_once(capture.read_text(),
        'original validator restarts local t=0 at validation_now+max(0,candidate_start-validation_now); scales resume and max(0.1,resume/2), hold interval included; past candidate epoch does not advance local phase',
        'branch-dependent; candidate_phase_alignment event is authoritative; COVERAGE_ALIGNMENT=1 uses measured projected phase, actual pose independently checked; legacy retains original restart-zero mapping'))
    reused = {}
    for path in (BASE / 'build').iterdir():
        if path.name not in REBUILT:
            os.link(path, OUT / 'build' / path.name)
            reused[path.name] = sha(path)
    recipe = recipes(OUT)
    recipe['compile_commands'] = [c for c in recipe['compile_commands']
        if Path(c['source']).name in ('poly_traj_optimizer.cpp', 'remani_replan_fsm.cpp', 'planner_manager.cpp')]
    _, includes = _flags('exp_opt')
    recipe['gradient_test_command'] = ['/usr/bin/c++','-std=gnu++17','-O2',
        '-I'+str(OUT/'source/include'),'-I'+str(OUT/'source'),*includes,
        str(OUT/'source'/TEST_SOURCE),'-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
        '-lrostime','-lcpp_common','-lrosconsole','-o',str(OUT/'build/test_coverage_gradient')]
    recipe['archive_commands'] = [c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands'] = {'remani_planner_node': recipe['link_commands']['remani_planner_node']}
    link = recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))] = str(OUT/'build/libexp_env.a')
    link.append('-lz')
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT/'source'),parent_build=str(BASE),
        coverage_alignment_experiment=True,coverage_alignment_parent_sha256=sha(BASE/'build_manifest.json'),
        coverage_alignment_reused_sha256=reused,coverage_alignment_recipe=recipe,
        coverage_alignment_implementation_sha256={name:sha(HERE/name) for name in ('build_coverage_alignment.py',TRANSFORM,*HEADERS,TEST_SOURCE)},
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False,
        coverage_alignment_scope='same binary A/B; exact frozen CV source; A legacy guide allowance/cap and horizon; B native continuous uncertainty and full valid CV window; nominal+half; history skipped/future unknown outside source; coordinated boundary/activation; native static/self/wheel retained')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return check()


def check():
    parent_check()
    manifest=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',manifest['coverage_alignment_parent_sha256'])
    for name,digest in manifest['coverage_alignment_implementation_sha256'].items():checked_hash(HERE/name,digest)
    for name,digest in manifest['coverage_alignment_reused_sha256'].items():checked_hash(OUT/'build'/name,digest)
    if source_inventory(OUT/'source')!=manifest['isolated_source_sha256']:raise ValueError('Child sources changed')
    verify_recipe(manifest['coverage_alignment_recipe'])
    if not manifest['prepared_only']:
        for name,digest in manifest['coverage_alignment_compiled_sha256'].items():checked_hash(OUT/'build'/name,digest)
        checked_hash(OUT/'coverage_gradient_check.json',manifest['coverage_alignment_gradient_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest=check()
    if not manifest['prepared_only']:raise RuntimeError('Completed child is immutable')
    recipe=manifest['coverage_alignment_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=recipe['cwd'],check=True)
    for command in recipe['archive_commands']:subprocess.run(command,cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'],cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['gradient_test_command'],cwd=recipe['cwd'],check=True)
    test=subprocess.run([str(OUT/'build/test_coverage_gradient')],check=True,text=True,capture_output=True)
    gradient_check=json.loads(test.stdout)
    if not gradient_check['pass']:raise ValueError('Dynamic time gradient finite difference failed')
    (OUT/'coverage_gradient_check.json').write_text(json.dumps(gradient_check,indent=2)+'\n')
    manifest.update(prepared_only=False,native_binaries_sha256={name:sha(OUT/'build'/name) for name in NATIVE},
        coverage_alignment_compiled_sha256={name:sha(OUT/'build'/name) for name in (*REBUILT,'test_coverage_gradient')},
        coverage_alignment_gradient_check_sha256=sha(OUT/'coverage_gradient_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return check()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('prepare','compile','check'))
    args=parser.parse_args();{'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('COVERAGE_ALIGNMENT_BUILD',args.action,OUT,'OK',flush=True)
