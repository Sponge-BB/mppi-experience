#!/usr/bin/env python3
"""Diagnostic-only wall budgets when the numerical world/ROS clock is paused."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE,sha,replace
from build_native_v10 import compile_source

HEADER='''#pragma once
#include <cstdlib>
#include <ros/ros.h>
namespace collab_budget {
inline ros::Time now(){
  const char* flag=std::getenv("COLLAB_PAIR");
  if(flag&&std::atoi(flag))return ros::Time(ros::WallTime::now().toSec());
  return ros::Time::now();
}
}
'''


def build(base,out):
    base=base.resolve();out=out.resolve();m=json.loads((base/'build_manifest.json').read_text())
    out.mkdir(parents=True,exist_ok=False);shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so','libexp_opt.a','libexp_search.a'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    parent=Path(m['parent_build'])
    (out/'source/collab_search_clock.hpp').write_text(HEADER)
    paths=parent/'source/mm_remani_planner/path_searching/src'
    for name in ('kino_astar.cpp','sample_mani_RRT.cpp','rrt.cpp'):
        text=(base/'source'/name).read_text() if (base/'source'/name).exists() else (paths/name).read_text()
        text='#include "collab_search_clock.hpp"\n'+text
        if name=='kino_astar.cpp':
            text=replace(text,'double start_time = ros::Time::now().toSec();','double start_time = collab_budget::now().toSec();')
            text=text.replace('ros::Time::now().toSec() - start_time','collab_budget::now().toSec() - start_time')
        elif name=='sample_mani_RRT.cpp':
            text=replace(text,'auto t_start = ros::Time::now();','auto t_start = collab_budget::now();')
            text=text.replace('ros::Time::now() - t_start','collab_budget::now() - t_start')
        else:
            text=replace(text,'ros::Time time_1 = ros::Time::now();','ros::Time time_1 = collab_budget::now();')
            text=text.replace('ros::Time::now() - time_1','collab_budget::now() - time_1')
        (out/'source'/name).write_text(text)
        _,obj=compile_source(parent,out,'exp_search',out/'source'/name,'unused',out/'source')
        subprocess.run(['ar','r',str(out/'build/libexp_search.a'),obj],check=True)
    header=(out/'source/collab_factorial.hpp').read_text()
    anchor='  done=true;failure_probe::PauseOwned paused;'
    header=replace(header,anchor,'''  if(envInt("COLLAB_PAIR_REQUIRE_ELIGIBLE")){
    std::vector<std::array<double,8>> suggested;std::string why;
    if(!prefixRows(start,yaw,sign,suggested,why))return;
  }
'''+anchor)
    (out/'source/collab_factorial.hpp').write_text(header)
    cm=parent/'build/CMakeFiles/remani_planner_node.dir';link=shlex.split((cm/'link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(x for x in link if x.endswith('/'+name+'.o'))
        if name=='remani_replan_fsm.cpp':_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        else:
            obj=str(base/'build'/(name+'.o'))
            if not Path(obj).exists():_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if x==old else x for x in link]
    link=[str(out/'build/libexp_opt.a') if x=='libexp_opt.a' else str(out/'build/libexp_search.a') if x=='libexp_search.a' else x for x in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node');subprocess.run(link,cwd=str(parent/'build'),check=True)
    m.update(collab_diagnostic_clock=True,collab_diagnostic_parent=str(base),
        diagnostic_scope='search budgets use wall time only when COLLAB_PAIR=1; ROS state/prediction time remains frozen',
        native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=HERE/'builds/collab_factorial_20261022_03')
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();build(a.base,a.out)
