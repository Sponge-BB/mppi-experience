#!/usr/bin/env python3
"""Create a small independent active-arm child of the last completed build."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from build_urdf_spheres_b import FROZEN, recipes, verify_recipe
from build_feasibility_stop import source_inventory
from active_arm_coupling_transform import (transform_collab, transform_bridge,
    transform_optimizer_unfreeze, transform_joint_feedback, transform_live_collision_export)

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/short_prefix_rejoin_20261101_01'
OUT=HERE/'builds/active_arm_20261104_01'
UNITS=('mm_config.cpp','poly_traj_optimizer.cpp','remani_planner_node.cpp',
       'remani_replan_fsm.cpp','planner_manager.cpp','kino_astar.cpp')
ARCHIVES=('libexp_model.a','libexp_opt.a','libexp_search.a')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare():
    from active_arm_model_transform import transform_mm_config
    if OUT.exists():raise FileExistsError(str(OUT))
    parent=json.loads((BASE/'build_manifest.json').read_text())
    if parent.get('prepared_only'):raise ValueError('parent not complete')
    OUT.mkdir();shutil.copytree(BASE/'source',OUT/'source');(OUT/'build').mkdir()
    for name in ('active_arm_coupling.hpp','active_arm_coupling_transform.py','active_arm_model_transform.py'):
        shutil.copy2(HERE/name,OUT/'source'/name)
    transforms={'collab_factorial.hpp':transform_collab,'coupling_bridge.hpp':transform_bridge,
        'poly_traj_optimizer.cpp':transform_optimizer_unfreeze,
        'remani_replan_fsm.cpp':transform_joint_feedback,
        'native_collision_export.hpp':transform_live_collision_export,
        'mm_config.cpp':transform_mm_config}
    for name,fn in transforms.items():
        p=OUT/'source'/name;p.write_text(fn(p.read_text()))
    p=OUT/'source/poly_traj_optimizer.cpp';text=p.read_text()
    anchor='    // Manipulator\n    vel = traj_data.getVel(t).tail(manipulator_dof_);'
    if text.count(anchor)!=1:raise ValueError('joint hard-check anchor changed')
    text=text.replace(anchor,'''    // Position bounds are a hard sampled acceptance check, in addition to
    // the native soft penalty. This does not claim continuous-time bounds.
    const Eigen::VectorXd arm_position=traj_data.getPos(t).tail(manipulator_dof_);
    if((arm_position.array()<min_joint_pos_.array()-1e-5).any() ||
       (arm_position.array()>max_joint_pos_.array()+1e-5).any()){
      failure_probe::reject("joint_position",arm_position.norm(),t);return true;
    }
    // Manipulator
    vel = traj_data.getVel(t).tail(manipulator_dof_);''')
    p.write_text(text)
    changed={n+'.o' for n in UNITS}|set(ARCHIVES)|{'remani_planner_node'}
    reused={}
    for f in (BASE/'build').iterdir():
        if f.is_file() and f.name not in changed:
            os.link(f,OUT/'build'/f.name);reused[f.name]=sha(f)
    recipe=recipes(OUT)
    recipe['compile_commands']=[r for r in recipe['compile_commands'] if Path(r['source']).name in UNITS]
    recipe['link_commands']={'remani_planner_node':recipe['link_commands']['remani_planner_node']}
    link=recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a')
    link.append('-lz');verify_recipe(recipe)
    manifest=copy.deepcopy(parent)
    manifest.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        active_arm_experiment=True,active_arm_parent_sha256=sha(BASE/'build_manifest.json'),
        active_arm_reused_sha256=reused,active_arm_recipe=recipe,
        isolated_source_sha256=source_inventory(OUT/'source'),production_modified=False,
        active_arm_scope='native-DH numerical whole-body velocity-control SIL; moving arm geometry; no hardware',
        active_arm_changes=['conditional arm optimizer coordinates','named q/dq feedback; acceleration boundary approximated zero',
            'measured-q ESDF export and extended crop','explicit nonadjacent self collision sphere pairs',
            'joint position sampled hard gate','full-state prefix and joint handoff gradients'])
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def check():
    m=json.loads((OUT/'build_manifest.json').read_text())
    assert sha(BASE/'build_manifest.json')==m['active_arm_parent_sha256']
    assert source_inventory(OUT/'source')==m['isolated_source_sha256']
    for name,h in m['active_arm_reused_sha256'].items():assert sha(OUT/'build'/name)==h,name
    verify_recipe(m['active_arm_recipe'])
    if not m['prepared_only']:
        for name,h in m['active_arm_compiled_sha256'].items():assert sha(OUT/'build'/name)==h,name
    return m


def compile_child():
    m=check()
    if not m['prepared_only']:raise ValueError('completed child is immutable')
    recipe=m['active_arm_recipe']
    for row in recipe['compile_commands']:
        print('COMPILE',Path(row['source']).name,flush=True)
        subprocess.run(row['command'],cwd=recipe['cwd'],check=True)
    for command in recipe['archive_commands']:subprocess.run(command,cwd=recipe['cwd'],check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'],cwd=recipe['cwd'],check=True)
    check()
    names=(*[n+'.o' for n in UNITS],*ARCHIVES,'remani_planner_node')
    m.update(prepared_only=False,active_arm_compiled_sha256={n:sha(OUT/'build'/n) for n in names},
        native_binaries_sha256={n:sha(OUT/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')})
    (OUT/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    return check()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('prepare','compile','check'))
    a=p.parse_args();{'prepare':prepare,'compile':compile_child,'check':check}[a.action]()
    print('ACTIVE_ARM_BUILD',a.action,'OK',flush=True)
