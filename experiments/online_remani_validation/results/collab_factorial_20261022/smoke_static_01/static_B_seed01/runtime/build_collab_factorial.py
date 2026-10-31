#!/usr/bin/env python3
"""Build target x prefix ablations without changing production sources."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE, sha, replace
from build_native_v10 import compile_source


def transform_fsm(text):
    text = replace(text, '#include "dual_timescale_probe.hpp"',
                   '#include "dual_timescale_probe.hpp"\n#include "collab_factorial.hpp"')
    text = replace(text, '    nh.param("fsm/local_execution_owner",local_execution_owner_,false);',
                   '    collab::configure(nh);\n    nh.param("fsm/local_execution_owner",local_execution_owner_,false);')
    text = replace(text, '    bool reach_horizon;\n    // Warm deformation',
                   '''    collab::refresh()=local_execution_owner_&&have_local_traj_;
    collab::goalToken()=dual_probe::hash(execution_goal_key_)&0xffffffffULL;
    collab::pairedOnce(*planner_manager_,start_pos_,start_vel_,start_acc_,start_jer_,start_yaw_,start_singul_,
      gripper_state_,end_pt_,end_yaw_,waypoint_gripper_close_[wpt_id_]);
    bool reach_horizon;
    // Warm deformation''')
    return text


def transform_manager(text):
    text = replace(text, '#include "dual_timescale_probe.hpp"',
                   '#include "dual_timescale_probe.hpp"\n#include "collab_factorial.hpp"')
    anchor = '''    reach_horizon = true;
    double t;'''
    text = replace(text, anchor, '''    if(collab::refresh()&&collab::settings().rolling)
      return collab::rollingTarget(*this,start_pt,start_yaw,global_end_pt,global_end_yaw,
        local_target_pos,local_target_vel,local_target_acc,reach_horizon);
'''+anchor)
    start = text.index('      dual_probe::emit("stage_begin",dual_probe::Fields().str("stage","search"));')
    end = text.index('      // std::cout << "computeInit 4', start)
    text = text[:start]+'''      int status=KinoAstar::NO_PATH;
      bool prefix_used=false;
      if(collab::refresh()&&collab::settings().prefix)
        prefix_used=collab::prefixSearch(*this,headState,tailState,start_yaw,start_singul,start_gripper,
          local_target_yaw,local_target_gripper,continous_failures_count,
          simple_path_container,yaw_list_container,initMJO_container,singul_container);
      if(prefix_used)status=KinoAstar::REACH_END;
      else status=collab::search(*this,headState,tailState,start_yaw,start_singul,start_gripper,
          local_target_yaw,local_target_gripper,continous_failures_count,
          simple_path_container,yaw_list_container,initMJO_container,singul_container);
'''+text[end:]
    # Every request, including target rejection, records its map versions and
    # goal. Do not infer a spatial violation solely from a ROS warning string.
    return text


def build(base, out):
    base=base.resolve();out=out.resolve()
    m=json.loads((base/'build_manifest.json').read_text())
    if not m.get('dual_timescale_probe'):raise RuntimeError('requires frozen coarse V12 base')
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so','libexp_opt.a'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    shutil.copy2(HERE/'collab_factorial.hpp',out/'source/collab_factorial.hpp')
    header=HERE.parents[1]/'src/mm_remani_planner/plan_manage/include/plan_manage/planner_manager.h'
    copied=replace(header.read_text(),'    /* main planning interface */',
        '''    int experimentFailureCount() const {return continous_failures_count_;}
    void experimentSetFailureCount(int value) {continous_failures_count_=value;}
    /* main planning interface */''')
    (out/'source/plan_manage/planner_manager.h').write_text(copied)
    (out/'source/remani_replan_fsm.cpp').write_text(transform_fsm((base/'source/remani_replan_fsm.cpp').read_text()))
    (out/'source/planner_manager.cpp').write_text(transform_manager((base/'source/planner_manager.cpp').read_text()))
    parent=Path(m['parent_build']);cm=parent/'build/CMakeFiles/remani_planner_node.dir'
    link=shlex.split((cm/'link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(x for x in link if x.endswith('/'+name+'.o'))
        if name=='remani_planner_node.cpp':
            obj=str(base/'build'/name)+'.o'
            if not Path(obj).exists():_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        else:_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if x==old else x for x in link]
    search=out/'build/libexp_search.a'
    shutil.copy2(HERE/'builds/resource_probe_v12_01/build/libexp_search.a',search)
    source=(base/'source/sample_mani_RRT.cpp').read_text()
    source='#include <cstdlib>\n'+source
    for name,offset in [('goal_gen_',11),('state_gen_',23),('node_gen_',37)]:
        source=source.replace(name+'.seed(std::random_device{}());',
            name+'.seed(std::getenv("COLLAB_SEED")?std::stoul(std::getenv("COLLAB_SEED"))+'+str(offset)+':std::random_device{}());')
    source=replace(source,
        '  bool SampleMani::search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state){',
        '''  bool SampleMani::search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state){
    if(const char* seed=std::getenv("COLLAB_SEED")){
      const unsigned value=std::stoul(seed);goal_gen_.seed(value+11);state_gen_.seed(value+23);node_gen_.seed(value+37);
    }''')
    (out/'source/sample_mani_RRT.cpp').write_text(source)
    _,search_object=compile_source(parent,out,'exp_search',out/'source/sample_mani_RRT.cpp','unused',out/'source')
    subprocess.run(['ar','r',str(search),search_object],check=True)
    link=[str(out/'build/libexp_opt.a') if x=='libexp_opt.a' else str(search) if x=='libexp_search.a' else x for x in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    m.update(collab_factorial=True,collab_parent=str(base),production_modified=False,
             collab_changes=['optional bounded rolling target; original final task preserved',
               'optional checked MPPI geometry prefix plus native continuation; same-gear MinSnap refit',
               'original wheel, acceleration, shape, safety, retry and budget retained'],
             native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
             isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
             collab_builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/dual_timeline_v12_01')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.base,a.out)
