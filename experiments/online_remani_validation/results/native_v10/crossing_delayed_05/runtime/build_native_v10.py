#!/usr/bin/env python3
"""Build isolated cooperative native reference + validated MPPI execution."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE,sha,replace


def compile_source(parent,out,target,source,relative_object,include=None):
    cm=parent/'build/CMakeFiles'/(target+'.dir')
    flags=(cm/'flags.make').read_text()
    inc=next(s.split('=',1)[1].strip() for s in flags.splitlines() if s.startswith('CXX_INCLUDES ='))
    obj=out/'build'/(source.name+'.o')
    cmd=['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC']
    if include:cmd+=['-I'+str(include)]
    subprocess.run(cmd+shlex.split(inc)+['-c',str(source),'-o',str(obj)],check=True)
    return relative_object,str(obj)


def build(base,out,planner_build=None):
    base=base.resolve();out=out.resolve();m=json.loads((base/'build_manifest.json').read_text())
    assert sha(base/'build/libonline_mppi.so')==m['mppi_library_sha256']
    parent=Path(m['parent_build']);snapshot=parent/'source/mm_remani_planner/plan_manage'
    out.mkdir(parents=True,exist_ok=False);(out/'source/plan_manage').mkdir(parents=True);(out/'build').mkdir()
    for n in ('mm_controller_node','dynamic_obstacle_tracker_node'):
        shutil.copy2(base/'build'/n,out/'build'/n)
    shutil.copy2(base/'source/online_kernel.hpp',out/'source/online_kernel.hpp')
    api=(base/'source/online_api.cpp').read_text()
    api=replace(api,'struct Online {','struct Online {V actual{};')
    api=replace(api,'static_cast<Online*>(ptr)->m.feedback({v,w});',
        '{auto&h=*static_cast<Online*>(ptr);h.actual={v,w};h.m.feedback({v,w});}')
    (out/'source/online_api.cpp').write_text(api+'\n'+(HERE/'execution_api.inc').read_text())
    header=(snapshot/'include/plan_manage/remani_replan_fsm.h').read_text()
    header=replace(header,'  private:', '''  private:
    bool local_execution_owner_=false;
    std::string execution_goal_key_,last_execution_state_;
    ros::Publisher execution_state_pub_,execution_reference_pub_;
    ros::Subscriber local_execution_sub_;
    nav_msgs::Path local_execution_path_;
    ros::Time last_cooperative_check_,last_cooperative_replan_;
    void publishExecutionState();
    void localExecutionCallback(const nav_msgs::Path::ConstPtr&msg);
    void checkCooperativeExecution();''')
    (out/'source/plan_manage/remani_replan_fsm.h').write_text(header)
    cpp=(snapshot/'src/remani_replan_fsm.cpp').read_text()
    cpp=replace(cpp,'#include <cmath>','#include <cmath>\n#include <std_msgs/String.h>\n#include <ros/serialization.h>')
    # Header requires the String declaration before its use, without changing
    # the class layout seen by other unmodified native sources.
    header='#include <std_msgs/String.h>\n'+header
    (out/'source/plan_manage/remani_replan_fsm.h').write_text(header)
    cpp=replace(cpp,'    exec_state_ = FSM_EXEC_STATE::INIT;',
        '    nh.param("fsm/local_execution_owner",local_execution_owner_,false);\n    exec_state_ = FSM_EXEC_STATE::INIT;')
    cpp=replace(cpp,
        '    dynamic_point_flow_state_pub_ = nh.advertise', '''    if(local_execution_owner_){
      execution_state_pub_=nh.advertise<std_msgs::String>("/planning/execution_state",1,true);
      execution_reference_pub_=nh.advertise<std_msgs::String>("/planning/execution_reference",10);
      local_execution_sub_=nh.subscribe("/planning/local_execution_rollout",1,&REMANIReplanFSM::localExecutionCallback,this);
      publishExecutionState();
    }
    dynamic_point_flow_state_pub_ = nh.advertise''')
    cpp=replace(cpp,'    exec_timer_.start();\n  }','    publishExecutionState();\n    exec_timer_.start();\n  }')
    cpp=replace(cpp,'    if (!msg->data) {','    if(local_execution_owner_){tracking_fault_active_=msg->data;return;}\n    if (!msg->data) {')
    cpp=replace(cpp,'  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){',
        '  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){\n    if(local_execution_owner_){checkCooperativeExecution();return;}')
    cpp=replace(cpp,'    cout << "Triggered! traget type: "',
        '    if(local_execution_owner_){execution_goal_key_=executionHex(*msg);publishExecutionState();}\n    cout << "Triggered! traget type: "')
    cpp=replace(cpp,'    exec_state_ = new_state;','    exec_state_ = new_state;\n    publishExecutionState();')
    cpp=replace(cpp,'    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){',
        '    std::vector<std::string> execution_segments;\n    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){')
    cpp=replace(cpp,'      poly_traj_pub_.publish(msg);',
        '      poly_traj_pub_.publish(msg);\n      if(local_execution_owner_)execution_segments.push_back(executionHex(msg));')
    cpp=replace(cpp,'\n  }\n\n  bool REMANIReplanFSM::isRearDistantGoal()', '''
    if(local_execution_owner_){
      std_msgs::String transaction;transaction.data=execution_goal_key_+"\\n"+std::to_string(execution_segments.size());
      for(const auto&s:execution_segments)transaction.data+="\\n"+s;
      execution_reference_pub_.publish(transaction);
    }
  }

  bool REMANIReplanFSM::isRearDistantGoal()''')
    cpp=replace(cpp,'    poly_traj_pub_.publish(msg);\n    have_published_trajectory_',
        '    poly_traj_pub_.publish(msg);\n    if(local_execution_owner_){std_msgs::String transaction;transaction.data=execution_goal_key_+"\\n1\\n"+executionHex(msg);execution_reference_pub_.publish(transaction);}\n    have_published_trajectory_')
    cpp=replace(cpp,'namespace remani_planner\n{','namespace remani_planner\n{\n'+(HERE/'native_execution.inc').read_text())
    (out/'source/remani_replan_fsm.cpp').write_text(cpp)
    shutil.copy2(snapshot/'src/remani_planner_node.cpp',out/'source/remani_planner_node.cpp')
    targets=[('online_mppi',['online_api.cpp']),('remani_planner_node',['remani_planner_node.cpp','remani_replan_fsm.cpp'])]
    for target,sources in targets:
        if target=='remani_planner_node' and planner_build is not None:
            planner_build=planner_build.resolve()
            for name in ['remani_planner_node.cpp','remani_replan_fsm.cpp','plan_manage/remani_replan_fsm.h']:
                assert sha(out/'source'/name)==sha(planner_build/'source'/name),'cannot reuse changed planner source'
            shutil.copy2(planner_build/'build/remani_planner_node',out/'build/remani_planner_node');continue
        cm=parent/'build/CMakeFiles'/(target+'.dir');link=shlex.split((cm/'link.txt').read_text())
        for name in sources:
            matching=[s for s in link if s.endswith('/'+name+'.o') or s==name+'.o'];assert len(matching)==1,(name,matching)
            old,obj=compile_source(parent,out,target,out/'source'/name,matching[0],out/'source')
            link=[obj if s==old else s for s in link]
        dest=out/'build'/('libonline_mppi.so' if target=='online_mppi' else target)
        link[link.index('-o')+1]=str(dest);subprocess.run(link,cwd=str(parent/'build'),check=True)
    shutil.copy2(out/'build/remani_planner_node',out/'build/remani_planner_node_no_wait')
    m.update(controller_revision='native_mppi_v10_execution',local_execution_owner=True,
        execution_parent=str(base),mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        planner_variants=['current'],
        controller_changes=['atomic goal-keyed complete native PolynomialTraj reference transactions',
            'native guide collision requests refresh without cancelling independently validated local motion',
            'publication-time exact controls and full fixed-arm footprint rollout + braking fallback validation'],
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print(out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=HERE/'builds/native_v9_03');p.add_argument('--out',type=Path,required=True)
    p.add_argument('--planner-build',type=Path,help='reuse native binary only after exact generated source hashes match')
    a=p.parse_args();build(a.base,a.out,a.planner_build)
