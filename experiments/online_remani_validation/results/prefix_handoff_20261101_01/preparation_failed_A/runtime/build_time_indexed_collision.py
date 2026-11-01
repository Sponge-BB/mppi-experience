#!/usr/bin/env python3
"""One isolated binary, runtime A/B switch, native final latest gate retained."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_handoff_alignment as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, _flags, recipes, verify_recipe
from native_collision_runtime import replace_once
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/handoff_alignment_20261031_01'
OUT = HERE / 'builds/time_indexed_collision_20261031_01'
HEADERS = ('time_indexed_collision.hpp', 'temporal_collision_source.hpp', 'temporal_collision_capture.hpp')
TEST_SOURCE = 'test_time_indexed_gradient.cpp'
REBUILT = ('poly_traj_optimizer.cpp.o', 'remani_replan_fsm.cpp.o', 'grid_map.cpp.o',
           'libexp_opt.a', 'libexp_env.a', 'remani_planner_node')
MAP_SOURCE = BASE / 'source/grid_map.cpp'


def optimizer(text):
    text = replace_once(text, '#include "handoff_alignment_probe.hpp"',
        '#include "handoff_alignment_probe.hpp"\n#include "time_indexed_collision.hpp"\n#include "temporal_collision_capture.hpp"')
    anchor = '    const auto* native67_context=boundary_admission::currentRequest();'
    text = replace_once(text, anchor,
        '    time_indexed_collision::freeze();\n'
        '    temporal_capture::planning(*grid_map_,*mm_config_,iniState_container,finState_container,\n'
        '      singul_container,input_hash,time_indexed_collision::requestedEpoch(),\n'
        '      time_indexed_collision::enabled()?"time_indexed":"swept",manipulator_thickness_);\n' + anchor)
    anchor = '    Eigen::VectorXd pos = traj.getPos(t);\n    double yaw = traj.getCarAngle(t);\n    return mm_config_->checkPlanningCollision('
    text = replace_once(text, anchor,
        '    if(time_indexed_collision::active())\n'
        '      return time_indexed_collision::hardCollision(*grid_map_,*mm_config_,traj,t,coll_type,manipulator_thickness_);\n' + anchor)
    text = replace_once(text,
        '        handoff_alignment::planningReject(traj_data,t,coll_type,mm_config_,grid_map_);',
        '        if(!time_indexed_collision::active())\n'
        '          handoff_alignment::planningReject(traj_data,t,coll_type,mm_config_,grid_map_);')
    anchor = '    // Ini/Fin Gear Pos\n'
    text = replace_once(text, anchor,
        '    time_indexed_collision::Durations temporal_durations(T_container,gradT_container);\n\n' + anchor)
    anchor = '        // collision\n        gradp.setZero();'
    text = replace_once(text, anchor,
        '        // The native sample moves both in space and in absolute CV time.\n'
        '        time_indexed_collision::Sample temporal_sample(time_indexed_collision::elapsedAt(trajid,i,s1));\n' + anchor)
    anchor = '        if(mani_hold_enabled_){'
    text = replace_once(text, anchor,
        '        time_indexed_collision::accumulateTimeGradient(trajid,i,alpha,omg*step);\n\n' + anchor)
    text = replace_once(text,
        '      grid_map_->evaluatePlanningEDTWithGrad(car_pts[i], dist, dist_grad);}',
        '      time_indexed_collision::evaluate(*grid_map_,car_pts[i],dist,dist_grad);}')
    text = replace_once(text,
        '        grid_map_->evaluatePlanningEDTWithGrad(\n            pt_on_link, dist, dist_grad);}',
        '        time_indexed_collision::evaluate(*grid_map_,pt_on_link,dist,dist_grad);}')
    anchor = '        costp += wei_obs_ * dist_err_3;'
    text = replace_once(text, anchor, anchor + '\n'
        '        time_indexed_collision::penaltyTimeDerivative(-wei_obs_*3.0*dist_err_2);')
    anchor = '          costp_mani += wei_mani_obs_ * factor * pow(dist_err, 3);'
    text = replace_once(text, anchor, anchor + '\n'
        '          time_indexed_collision::penaltyTimeDerivative(-wei_mani_obs_*factor*3.0*dist_err*dist_err);')
    anchor = '    wheel_cause::nativeGate(good_traj,diagnosis.reason,diagnosis.rejection_value,diagnosis.rejection_time);'
    text = replace_once(text, anchor, anchor + '\n'
        '    temporal_capture::finalCandidate(singul_traj_data,good_traj,diagnosis.reason);')
    return text


def fsm(text):
    text = replace_once(text, '#include "boundary_admission.hpp"',
        '#include "boundary_admission.hpp"\n#include "time_indexed_collision.hpp"\n#include "temporal_collision_capture.hpp"')
    anchor = '    bool plan_success = planner_manager_->reboundReplan('
    text = replace_once(text, anchor,
        '    time_indexed_collision::Request temporal_request(*planner_manager_->grid_map_,desired_start_time);\n' + anchor)
    # Capture before insufficient-horizon rejection, using the exact original
    # step expression without changing validator control flow or timing rules.
    anchor = '    const ros::Time now = ros::Time::now();\n    auto reject = [&](const char* branch, double local_t, double wall_ahead,'
    text = replace_once(text, anchor,
        '    const ros::Time now = ros::Time::now();\n'
        '    temporal_capture::acceptance(traj,snapshot,now,*planner_manager_->grid_map_,\n'
        '      dynamic_point_flow_active_resume_scale_,std::max(0.05,global_plan_dynamic_check_time_step_));\n'
        '    auto reject = [&](const char* branch, double local_t, double wall_ahead,')
    return text


def environment(text):
    text = '#include "temporal_collision_source.hpp"\n' + text
    anchor = '      dynamic_point_overlay_ready_ = snapshot.valid && !swept_cloud->empty();'
    return replace_once(text, anchor, anchor + '\n'
        '      temporal_capture_source::published(this,snapshot,now);')


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
    for name, transform in (('poly_traj_optimizer.cpp', optimizer), ('remani_replan_fsm.cpp', fsm)):
        path = OUT / 'source' / name
        path.write_text(transform(path.read_text()))
    (OUT / 'source/grid_map.cpp').write_text(environment(MAP_SOURCE.read_text()))
    path = OUT / 'source/include/plan_env/grid_map.h'
    path.write_text(replace_once(path.read_text(), '  friend class HandoffAlignmentMapAccessor;',
        '  friend class HandoffAlignmentMapAccessor;\n'
        '  friend class TimeIndexedCollisionMapAccessor;\n'
        '  friend class TemporalCollisionCaptureMapAccessor;'))
    reused = {}
    for path in (BASE / 'build').iterdir():
        if path.name not in REBUILT:
            os.link(path, OUT / 'build' / path.name)
            reused[path.name] = sha(path)
    recipe = recipes(OUT)
    recipe['compile_commands'] = [c for c in recipe['compile_commands']
        if Path(c['source']).name in ('poly_traj_optimizer.cpp', 'remani_replan_fsm.cpp')]
    flag_path, includes = _flags('exp_env')
    source, obj = OUT / 'source/grid_map.cpp', OUT / 'build/grid_map.cpp.o'
    recipe['compile_commands'].append(dict(target='exp_env', source=str(source), object=str(obj),
        command=['/usr/bin/c++', '-std=gnu++17', '-O2', '-DNDEBUG', '-fPIC',
                 '-I'+str(OUT/'source/include'), '-I'+str(OUT/'source'), *includes,
                 '-c', str(source), '-o', str(obj)]))
    recipe['inherited_flags_sha256'][str(flag_path)] = sha(flag_path)
    recipe['gradient_test_command'] = ['/usr/bin/c++','-std=gnu++17','-O2',
        '-I'+str(OUT/'source/include'),'-I'+str(OUT/'source'),*includes,
        str(OUT/'source'/TEST_SOURCE),'-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
        '-lrostime','-lcpp_common','-lrosconsole','-o',str(OUT/'build/test_time_indexed_gradient')]
    recipe['archive_commands'] = [c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    # Copy the archive before replacing a single object: inherited hard links
    # are never mutated. Unchanged environment members remain byte-identical.
    shutil.copy2(BASE/'build/libexp_env.a',OUT/'build/libexp_env.a')
    recipe['archive_commands'].append(['ar','rcs',str(OUT/'build/libexp_env.a'),str(obj)])
    recipe['link_commands'] = {'remani_planner_node': recipe['link_commands']['remani_planner_node']}
    link = recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))] = str(OUT/'build/libexp_env.a')
    link.append('-lz')
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT/'source'),parent_build=str(BASE),
        time_indexed_collision_experiment=True,time_indexed_collision_parent_sha256=sha(BASE/'build_manifest.json'),
        time_indexed_collision_reused_sha256=reused,time_indexed_collision_recipe=recipe,
        time_indexed_collision_implementation_sha256={name:sha(HERE/name) for name in ('build_time_indexed_collision.py',*HEADERS,TEST_SOURCE)},
        time_indexed_collision_map_source=dict(path=str(MAP_SOURCE),sha256=sha(MAP_SOURCE)),
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False,
        time_indexed_collision_scope='same binary A/B; exact frozen CV source and original guide inflation; nominal+half; original swept beyond source horizon; native static/self/wheel and latest acceptance retained')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return check()


def check():
    parent_check()
    manifest=json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json',manifest['time_indexed_collision_parent_sha256'])
    for name,digest in manifest['time_indexed_collision_implementation_sha256'].items():checked_hash(HERE/name,digest)
    for name,digest in manifest['time_indexed_collision_reused_sha256'].items():checked_hash(OUT/'build'/name,digest)
    origin=manifest['time_indexed_collision_map_source'];checked_hash(Path(origin['path']),origin['sha256'])
    if source_inventory(OUT/'source')!=manifest['isolated_source_sha256']:raise ValueError('Child sources changed')
    verify_recipe(manifest['time_indexed_collision_recipe'])
    if not manifest['prepared_only']:
        for name,digest in manifest['time_indexed_collision_compiled_sha256'].items():checked_hash(OUT/'build'/name,digest)
        checked_hash(OUT/'temporal_gradient_check.json',manifest['time_indexed_collision_gradient_check_sha256'])
        check_build(OUT)
    return manifest


def compile_child():
    manifest=check()
    if not manifest['prepared_only']:raise RuntimeError('Completed child is immutable')
    recipe=manifest['time_indexed_collision_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE',Path(command['source']).name,flush=True)
        subprocess.run(command['command'],cwd=recipe['cwd'],check=True)
    for command in recipe['archive_commands']:subprocess.run(command,cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'],cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['gradient_test_command'],cwd=recipe['cwd'],check=True)
    test=subprocess.run([str(OUT/'build/test_time_indexed_gradient')],check=True,text=True,capture_output=True)
    gradient_check=json.loads(test.stdout)
    if not gradient_check['pass']:raise ValueError('Dynamic time gradient finite difference failed')
    (OUT/'temporal_gradient_check.json').write_text(json.dumps(gradient_check,indent=2)+'\n')
    manifest.update(prepared_only=False,native_binaries_sha256={name:sha(OUT/'build'/name) for name in NATIVE},
        time_indexed_collision_compiled_sha256={name:sha(OUT/'build'/name) for name in (*REBUILT,'test_time_indexed_gradient')},
        time_indexed_collision_gradient_check_sha256=sha(OUT/'temporal_gradient_check.json'))
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return check()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('prepare','compile','check'))
    args=parser.parse_args();{'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('TIME_INDEXED_COLLISION_BUILD',args.action,OUT,'OK',flush=True)
