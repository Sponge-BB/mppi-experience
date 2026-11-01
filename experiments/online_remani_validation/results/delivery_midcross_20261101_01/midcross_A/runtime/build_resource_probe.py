#!/usr/bin/env python3
"""Add accounting to an immutable V12 copy; preserve its solver and controller."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE, sha, replace
from build_native_v10 import compile_source


def build(base, out):
    base=base.resolve();out=out.resolve()
    manifest=json.loads((base/'build_manifest.json').read_text())
    parent=Path(manifest['parent_build'])
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    shutil.copy2(HERE/'resource_probe.hpp',out/'source/resource_probe.hpp')
    opt_path=parent/'source/mm_remani_planner/traj_opt/src/poly_traj_optimizer.cpp'
    text=opt_path.read_text()
    text=replace(text,'#include "optimizer/poly_traj_optimizer.hpp"',
                 '#include "optimizer/poly_traj_optimizer.hpp"\n#include "resource_probe.hpp"')
    text=replace(text,'    traj_num_ = initInnerPts_container.size();', '''    resource_probe::Scope probe("optimizer");
    unsigned long long input_hash=1469598103934665603ULL;
    resource_probe::hashMatrices(input_hash,iniState_container);
    resource_probe::hashMatrices(input_hash,finState_container);
    resource_probe::hashMatrices(input_hash,initInnerPts_container);
    resource_probe::hashMatrices(input_hash,initT_container);
    resource_probe::hashBytes(input_hash,singul_container.data(),singul_container.size()*sizeof(int));
    probe.add("input_hash",input_hash);probe.add("budget_s",max_wall_time_);
    probe.add("segments",initT_container.size());
    double requested_duration=0.;int requested_pieces=0;
    for(const auto&t:initT_container){requested_duration+=t.sum();requested_pieces+=t.size();}
    probe.add("pieces",requested_pieces);probe.add("requested_duration_s",requested_duration);
    traj_num_ = initInnerPts_container.size();''')
    text=replace(text,'    // printf("\\033[32miter = %d, time = %5.3f(ms), \\n\\033[0m", iter_num_, (ros::Time::now() - t1).toSec() * 1000.0);',
                 '''    probe.add("iterations",iter_num_);probe.add("lbfgs_result",result);
    probe.add("force_stop",int(force_stop_type_));probe.add("variables",variable_num_);
    probe.add("lbfgs_wall_ms",(ros::WallTime::now()-optimization_start_wall_time_).toSec()*1000.);
    // Paired diagnostic only: recursively rebuild exactly the same optimizer
    // request while this serialized planner callback keeps its map unchanged.
    static bool paired_done=false;
    const char* paired_pid_file=std::getenv("RESOURCE_PAIRED_PID_FILE");
    if(paired_pid_file&&!paired_done&&resource_probe::label()=="normal"&&max_wall_time_<=.61){
      paired_done=true;std::ifstream pf(paired_pid_file);int pid=0;pf>>pid;
      if(pid<=1||kill(pid,0)!=0)throw std::runtime_error("invalid owned MPPI probe PID");
      const double saved_budget=max_wall_time_;
      for(double budget:{.60,1.50})for(int pass=0;pass<4;++pass){
        const bool loaded=(pass==1||pass==2);
        if(kill(pid,loaded?SIGCONT:SIGSTOP)!=0)throw std::runtime_error("paired load signal failed");
        usleep(300000);max_wall_time_=budget;
        resource_probe::label()=loaded?"paired_mppi_active":"paired_mppi_paused";
        std::vector<Eigen::MatrixXd> cp,wp;std::vector<Eigen::VectorXd> times,ee;
        OptimizeTrajectory_lbfgs(iniState_container,finState_container,initInnerPts_container,
          initT_container,singul_container,cp,wp,times,ee);
      }
      kill(pid,SIGCONT);max_wall_time_=saved_budget;resource_probe::label()="normal";
      probe.add("paired_diagnostic_only",1);return false;
    }''')
    text=replace(text,'    return good_traj;','    probe.success=good_traj;\n    return good_traj;')
    (out/'source/poly_traj_optimizer.cpp').write_text(text)
    manager=(parent/'source/mm_remani_planner/plan_manage/src/planner_manager.cpp').read_text()
    manager=replace(manager,'#include <plan_manage/planner_manager.h>',
                    '#include <plan_manage/planner_manager.h>\n#include "resource_probe.hpp"')
    manager=replace(manager,'    static int count = 0;', '''    resource_probe::Scope request_probe("replan");
    request_probe.add("start_x",start_pt(0));request_probe.add("start_y",start_pt(1));
    request_probe.add("start_yaw",start_yaw);request_probe.add("start_speed",start_vel.head(2).norm());
    request_probe.add("target_x",local_target_pt(0));request_probe.add("target_y",local_target_pt(1));
    request_probe.add("target_yaw",local_target_yaw);request_probe.add("poly_init",int(flag_polyInit));
    request_probe.add("budget_s",ploy_traj_opt_->getMaxWallTime());
    static int count = 0;''')
    manager=replace(manager,'    t_init = ros::Time::now() - t_start;',
                    '    request_probe.add("frontend_wall_ms",(resource_probe::clockSeconds(CLOCK_MONOTONIC)-request_probe.wall)*1000.);\n    t_init = ros::Time::now() - t_start;')
    # Every exit is accounted for; success is set only on the actual final exit.
    start=manager.index('  bool MMPlannerManager::reboundReplan(')
    end=manager.find('\n  bool MMPlannerManager::',start+10)
    if end<0:raise RuntimeError('cannot delimit replan method')
    method=manager[start:end]
    if method.count('return true;')!=1:raise RuntimeError('ambiguous success exit')
    manager=manager[:start]+method.replace('return true;','request_probe.success=true;return true;')+manager[end:]
    (out/'source/planner_manager.cpp').write_text(manager)
    _,opt_object=compile_source(parent,out,'exp_opt',out/'source/poly_traj_optimizer.cpp','unused',out/'source')
    archive=out/'build/libexp_opt.a';shutil.copy2(parent/'build/libexp_opt.a',archive)
    subprocess.run(['ar','r',str(archive),opt_object],check=True)
    # Reuse the already reviewed V11/V12 native search boundary protections.
    _,sample_object=compile_source(parent,out,'exp_search',out/'source/sample_mani_RRT.cpp','unused',out/'source')
    search_archive=out/'build/libexp_search.a';shutil.copy2(parent/'build/libexp_search.a',search_archive)
    subprocess.run(['ar','r',str(search_archive),sample_object],check=True)
    cm=parent/'build/CMakeFiles/remani_planner_node.dir'
    link=shlex.split((cm/'link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(s for s in link if s.endswith('/'+name+'.o'))
        _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if s==old else s for s in link]
    link=[str(archive) if s=='libexp_opt.a' else str(search_archive) if s=='libexp_search.a' else s for s in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    manifest.update(resource_probe=True,resource_probe_parent=str(base),
        resource_changes=['per-request wall/thread CPU/process CPU/runqueue accounting',
                          'optimizer input fingerprint and problem dimensions',
                          'optional separate fixed-request paused/active MPPI probe'],
        native_binaries_sha256={n:sha(out/'build'/n) for n in
          ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')})
    manifest['isolated_source_sha256']={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()}
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/native_v12_07')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.base,a.out)
