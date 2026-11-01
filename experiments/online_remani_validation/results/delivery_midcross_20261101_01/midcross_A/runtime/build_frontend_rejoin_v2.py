#!/usr/bin/env python3
"""Native base-A* only same-input probe; full online wrapper is unchanged."""
import argparse
import json
from pathlib import Path
import shutil

import build_frontend_rejoin as prior
from native_collision_runtime import replace_once

HERE=Path(__file__).resolve().parent
OUT=HERE/'builds/frontend_rejoin_20261031_02'
original_transform=prior.transform
original_check=prior.check

def transform(text):
    text=original_transform(text)
    text=replace_once(text,'#include "frontend_rejoin_probe.hpp"',
                      '#include "frontend_rejoin_probe_v2.hpp"')
    original='''      const int status=KinoAstarSearchAndGetSimplePath(start_pos,start_vel,start_yaw,start_singul,start_gripper,
        end_pos,end_vel,end_yaw,end_gripper,init_ctrl,continous_failures_count,
        simple_path_container,yaw_list_container,singul_container,t_list_container);'''
    replacement='''      reset();
      simple_path_container.clear();yaw_list_container.clear();singul_container.clear();t_list_container.clear();
      Eigen::VectorXd probe_start(4+manipulator_dof_),probe_end(4+manipulator_dof_);
      probe_start.head(2)=start_pos.head(2);probe_start(2)=start_yaw;
      probe_start(3)=start_vel.head(2).norm()*start_singul;
      probe_start.tail(manipulator_dof_)=start_pos.tail(manipulator_dof_);
      probe_end.head(2)=end_pos.head(2);probe_end(2)=end_yaw;probe_end(3)=0.;
      probe_end.tail(manipulator_dof_)=end_pos.tail(manipulator_dof_);
      const int status=search(probe_start,probe_end,init_ctrl);
      sdf_map_->resetAstarBuffer();
      if(has_path_){
        getKinoNode();
        simple_path_container.resize(1);yaw_list_container.resize(1);
        for(const auto& pose:sample_trajs_){
          Eigen::VectorXd point(2);point=pose.head(2);
          simple_path_container[0].push_back(point);yaw_list_container[0].push_back(pose(2));
        }
      }'''
    text=replace_once(text,original,replacement)
    text=text.replace('ros::Time::now().toSec() - start_time','frontend_rejoin::probeNow() - start_time')
    text=replace_once(text,'  double start_time = ros::Time::now().toSec();',
        '  double start_time = frontend_rejoin::probeNow();')
    text=text.replace('\\"kind\\":\\"capture\\",','\\"kind\\":\\"capture\\",\\"scope\\":\\"base_astar_only\\",')
    text=text.replace('\\"kind\\":\\"variant\\",','\\"kind\\":\\"variant\\",\\"scope\\":\\"base_astar_only\\",')
    text=text.replace('\\"wrapper_wall_ms\\":','\\"base_astar_and_parameterization_wall_ms\\":')
    return text

def configure():
    prior.OUT=OUT;prior.transform=transform

def prepare():
    configure();prior.prepare()
    shutil.copy2(HERE/'frontend_rejoin_probe_v2.hpp',OUT/'source/frontend_rejoin_probe_v2.hpp')
    path=OUT/'build_manifest.json';manifest=json.loads(path.read_text())
    manifest.update(frontend_rejoin_probe_scope='base_astar_only; no arm sampler or candidate publication; native search budget uses wall time only while paused',
        frontend_rejoin_v2_implementation_sha256={name:prior.sha(HERE/name) for name in
            ('build_frontend_rejoin_v2.py','frontend_rejoin_probe_v2.hpp')},
        isolated_source_sha256=prior.source_inventory(OUT/'source'))
    path.write_text(json.dumps(manifest,indent=2)+'\n');return check()

def check():
    configure();manifest=original_check()
    for name,digest in manifest.get('frontend_rejoin_v2_implementation_sha256',{}).items():
        prior.checked_hash(HERE/name,digest)
    return manifest

def compile_child():
    configure();check();prior.compile_child();check()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('prepare','compile','check'))
    parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args();OUT=args.out.resolve()
    {'prepare':prepare,'compile':compile_child,'check':check}[args.action]()
    print('FRONTEND_REJOIN_V2',args.action,OUT,'OK',flush=True)
