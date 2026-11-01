#!/usr/bin/env python3
"""Isolated A/B mailbox build: four FSM-layout consumers, same MPPI/model."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

TOOLS=Path(__file__).resolve().parent
HERE=TOOLS.parents[2]
sys.path.insert(0,str(HERE))
import build_delivery_next as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import recipes, verify_recipe, FROZEN
from native_collision_runtime import replace_once
from run_coupled_objective import check_build

BASE=parent.DEFAULT
OUT=HERE/'builds/state_handoff_20261031_01'
UNITS=('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp','planning_visualization.cpp')
REBUILT=tuple(n+'.o' for n in UNITS)+('remani_planner_node',)


def transform_header(text):
    text=replace_once(text,'#include <std_msgs/String.h>\n',
        '#include <std_msgs/String.h>\n#include "latest_odom.hpp"\n')
    return replace_once(text,'  private:\n',r'''  private:
    bool independent_odom_=false;
    ros::CallbackQueue odom_receive_queue_;
    std::unique_ptr<ros::AsyncSpinner> odom_receive_spinner_;
    state_handoff::Inbox odom_inbox_;
    std::uint64_t consumed_odom_sequence_=0;
    nav_msgs::OdometryConstPtr applied_raw_odom_;
    void receiveOdomOnly(const nav_msgs::OdometryConstPtr &msg);
    void applyLatestOdom();
    void recordHandoffState(const std::string &stage);
''')


METHODS=r'''
  void REMANIReplanFSM::receiveOdomOnly(const nav_msgs::OdometryConstPtr &msg) {
    // No tracing, FSM reads/writes, model access or planner calls on this thread.
    odom_inbox_.receive(msg,ros::Time::now().toSec(),dual_probe::mono());
  }

  void REMANIReplanFSM::applyLatestOdom() {
    if(!independent_odom_)return;
    const auto sample=odom_inbox_.snapshot();
    if(!sample.message || sample.sequence==consumed_odom_sequence_)return;
    consumed_odom_sequence_=sample.sequence;
    mmCarOdomCallback(sample.message); // Original processing, main ROS thread only.
  }

  void REMANIReplanFSM::recordHandoffState(const std::string &stage) {
    const auto raw=applied_raw_odom_;
    if(!raw)return;
    const auto received=odom_inbox_.snapshot();
    const double capture_sim=ros::Time::now().toSec();
    const double capture_mono=dual_probe::mono();
    dual_probe::emit("handoff_state",dual_probe::Fields().str("stage",stage)
      .add("independent_odom",independent_odom_?"true":"false")
      .add("capture_sim",capture_sim).add("capture_mono",capture_mono)
      .add("applied_stamp",raw->header.stamp.toSec())
      .add("applied_age_s",capture_sim-raw->header.stamp.toSec())
      .add("applied_x",raw->pose.pose.position.x).add("applied_y",raw->pose.pose.position.y)
      .add("applied_yaw",tf::getYaw(raw->pose.pose.orientation))
      .add("applied_body_vx",raw->twist.twist.linear.x)
      .add("applied_body_vy",raw->twist.twist.linear.y)
      .add("applied_w",raw->twist.twist.angular.z)
      .add("processed_vx",mm_state_vel_(0)).add("processed_vy",mm_state_vel_(1))
      .add("processed_gear",mm_car_singul_).add("consumed_sequence",consumed_odom_sequence_)
      .add("latest_received_sequence",received.sequence)
      .add("latest_received_stamp",received.message?received.message->header.stamp.toSec():0.)
      .add("receiver_receipt_ros",received.receipt_ros).add("receiver_receipt_mono",received.receipt_mono)
      .add("selected_boundary_x",start_pos_(0)).add("selected_boundary_y",start_pos_(1))
      .add("selected_boundary_yaw",start_yaw_)
      .str("semantics","raw applied odometry; fixed solver boundary separately preserved; no new acceptance threshold"));
  }
'''


def transform_fsm(text):
    text=replace_once(text,'  REMANIReplanFSM::~REMANIReplanFSM(){}',
        '  REMANIReplanFSM::~REMANIReplanFSM(){\n'
        '    odom_sub_.shutdown();\n'
        '    if(odom_receive_spinner_)odom_receive_spinner_->stop();\n'
        '  }\n'+METHODS)
    text=replace_once(text,'    odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::mmCarOdomCallback, this);',
        '    const char* receiver_switch=std::getenv("INDEPENDENT_ODOM_RECEIVER");\n'
        '    independent_odom_=receiver_switch && std::string(receiver_switch)=="1";\n'
        '    if(independent_odom_){\n'
        '      ros::NodeHandle odom_node(nh);odom_node.setCallbackQueue(&odom_receive_queue_);\n'
        '      odom_sub_=odom_node.subscribe("odom_world",1,&REMANIReplanFSM::receiveOdomOnly,this);\n'
        '      odom_receive_spinner_.reset(new ros::AsyncSpinner(1,&odom_receive_queue_));\n'
        '    }else{\n'
        '      odom_sub_=nh.subscribe("odom_world",1,&REMANIReplanFSM::mmCarOdomCallback,this);\n'
        '    }')
    text=replace_once(text,
        '    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);',
        '    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);\n'
        '    if(odom_receive_spinner_)odom_receive_spinner_->start();')
    text=replace_once(text,'    exec_timer_.stop(); // To avoid blockage',
        '    applyLatestOdom();\n    exec_timer_.stop(); // To avoid blockage')
    text=replace_once(text,'  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){',
        '  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){\n    applyLatestOdom();')
    text=replace_once(text,'  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){',
        '  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){\n    applyLatestOdom();')
    text=replace_once(text,'    have_odom_ = true;\n  }',
        '    have_odom_ = true;\n    applied_raw_odom_=msg;\n  }')
    text=replace_once(text,'    bool plan_success = planner_manager_->reboundReplan(',
        '    recordHandoffState("before_solve");\n'
        '    bool plan_success = planner_manager_->reboundReplan(')
    text=replace_once(text,'    have_new_target_ = false;\n\n    dual_probe::emit("manager_result",',
        '    have_new_target_ = false;\n'
        '    recordHandoffState("after_solve_before_refresh");\n'
        '    applyLatestOdom();\n'
        '    recordHandoffState("after_solve_refreshed");\n\n'
        '    dual_probe::emit("manager_result",')
    text=replace_once(text,'      dual_probe::Scope dual_validation("live_acceptance");',
        '      dual_probe::Scope dual_validation("live_acceptance");\n'
        '      recordHandoffState("acceptance_entry");')
    # The existing live gate may take appreciable time. Refresh only after its
    # calculations, before publication; no ongoing optimization input changes.
    text=replace_once(text,'  void REMANIReplanFSM::sendPolyTrajROSMsg(){',
        '  void REMANIReplanFSM::sendPolyTrajROSMsg(){\n'
        '    applyLatestOdom();\n    recordHandoffState("publication_entry");')
    return text


def prepare():
    m=parent.check(BASE)
    if OUT.exists():raise FileExistsError(OUT)
    OUT.mkdir();shutil.copytree(BASE/'source',OUT/'source');(OUT/'build').mkdir()
    hp=OUT/'source/plan_manage/remani_replan_fsm.h';hp.write_text(transform_header(hp.read_text()))
    cp=OUT/'source/remani_replan_fsm.cpp';cp.write_text(transform_fsm(cp.read_text()))
    shutil.copy2(TOOLS/'latest_odom.hpp',OUT/'source/latest_odom.hpp')
    reused={}
    for p in sorted((BASE/'build').iterdir()):
        if p.name not in REBUILT:
            os.link(p,OUT/'build'/p.name);reused[p.name]=sha(p)
    r=recipes(OUT)
    r['compile_commands']=[c for c in r['compile_commands'] if Path(c['source']).name in UNITS]
    r['link_commands']={'remani_planner_node':r['link_commands']['remani_planner_node']}
    link=r['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(OUT/'build/libexp_env.a')
    link.append('-lz');r['archive_commands']=[]
    verify_recipe(r)
    m=copy.deepcopy(m)
    m.update(prepared_only=True,source_root=str(OUT/'source'),parent_build=str(BASE),
        state_handoff_experiment=True,state_handoff_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
        state_handoff_recipe=r,state_handoff_reused_sha256=reused,
        state_handoff_implementation_sha256={n:sha(TOOLS/n) for n in ('build_state_handoff.py','latest_odom.hpp')},
        isolated_source_sha256=source_inventory(OUT/'source'),
        production_modified=False,production_default=False,
        scope='fixed-arm numerical ROS SIL; mailbox plus main-thread refresh only')
    (OUT/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    return OUT


def check():
    m=json.loads((OUT/'build_manifest.json').read_text());parent.check(BASE)
    checked_hash(BASE/'build_manifest.json',m['state_handoff_parent_manifest_sha256'])
    if source_inventory(OUT/'source')!=m['isolated_source_sha256']:raise ValueError('Child source changed')
    for n,d in m['state_handoff_implementation_sha256'].items():checked_hash(TOOLS/n,d)
    for n,d in m['state_handoff_reused_sha256'].items():
        checked_hash(OUT/'build'/n,d)
        if not (OUT/'build'/n).samefile(BASE/'build'/n):raise ValueError('Immutable parent reuse changed')
    verify_recipe(m['state_handoff_recipe'])
    if not m['prepared_only']:
        for n,d in m['state_handoff_compiled_sha256'].items():checked_hash(OUT/'build'/n,d)
        check_build(OUT)
    return m


def compile_prepared():
    m=check()
    if not m['prepared_only']:raise ValueError('Already built')
    r=m['state_handoff_recipe']
    for c in r['compile_commands']:
        print('COMPILE',Path(c['source']).name,flush=True);subprocess.run(c['command'],check=True)
    subprocess.run(r['link_commands']['remani_planner_node'],cwd=r['cwd'],check=True)
    m.update(prepared_only=False,native_binaries_sha256={n:sha(OUT/'build'/n) for n in NATIVE},
        state_handoff_compiled_sha256={n:sha(OUT/'build'/n) for n in REBUILT})
    (OUT/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check();return OUT


if __name__=='__main__':
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group()
    g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    a=p.parse_args();print(check() if a.check else compile_prepared() if a.compile else prepare())
