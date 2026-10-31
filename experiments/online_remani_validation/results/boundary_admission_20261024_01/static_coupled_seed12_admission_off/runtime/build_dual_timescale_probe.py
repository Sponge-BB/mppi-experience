#!/usr/bin/env python3
"""Build coarse FSM/search timeline hooks in an isolated frozen V12 copy."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE, sha, replace
from build_native_v10 import compile_source


def instrument_fsm(text):
    text = replace(text, '#include <cmath>', '#include <cmath>\n#include "dual_timescale_probe.hpp"')
    text = replace(text, '  if(blocked&&(last_cooperative_replan_.isZero()||', '''  dual_probe::emit("cooperative_check",dual_probe::Fields()
    .add("blocked",blocked?"true":"false").add("guide_time",current).add("guide_id",info.traj_id)
    .add("raw_seq",planner_manager_->grid_map_->getDynamicObservationSeq())
    .add("esdf_seq",planner_manager_->grid_map_->getESDFObservationSeq())
    .add("trigger_cooldown_remaining",std::max(0.,.75-(now-last_cooperative_replan_).toSec())));
  if(blocked&&(last_cooperative_replan_.isZero()||''')
    text = replace(text, '    last_cooperative_replan_=now;flag_relan_astar_=true;', '''    dual_probe::emit("refresh_trigger",dual_probe::Fields().add("guide_id",info.traj_id).add("guide_time",current));
    last_cooperative_replan_=now;flag_relan_astar_=true;''')
    text = replace(text, '        if(ros::Time::now()<next_cooperative_attempt_)goto force_return;', '''        const bool dual_cooldown=ros::Time::now()<next_cooperative_attempt_;
        dual_probe::condition("cooperative_retry",dual_cooldown,dual_probe::Fields()
          .add("until_sim",next_cooperative_attempt_.toSec()));
        if(dual_cooldown)goto force_return;''')
    text = replace(text, '        next_cooperative_attempt_=ros::Time::now()+ros::Duration(.75);', '''        next_cooperative_attempt_=ros::Time::now()+ros::Duration(.75);
        dual_probe::emit("refresh_attempt_result",dual_probe::Fields().add("success",refreshed?"true":"false")
          .add("next_attempt_sim",next_cooperative_attempt_.toSec()));''')
    text = replace(text, '    if (required_esdf_seq == 0)', '''    dual_probe::condition(std::string("esdf:")+planning_context,
      required_esdf_seq>esdf_observation_seq,dual_probe::Fields().str("context",planning_context)
        .add("required_seq",required_esdf_seq).add("esdf_seq",esdf_observation_seq)
        .add("raw_seq",map->getDynamicObservationSeq()));
    if (required_esdf_seq == 0)''')
    text = replace(text, '    exec_state_ = new_state;', '''    dual_probe::emit("fsm_transition",dual_probe::Fields().add("from",pre_s).add("to",int(new_state)).str("cause",pos_call));
    exec_state_ = new_state;''')
    text = replace(text, '      execution_reference_pub_.publish(transaction);', '''      execution_reference_pub_.publish(transaction);
      dual_probe::emit("reference_publish",dual_probe::Fields().str("transport","complete_goal_keyed_transaction")
        .str("transaction_fnv64",std::to_string(dual_probe::hash(transaction.data)))
        .str("goal_fnv64",std::to_string(dual_probe::hash(execution_goal_key_)))
        .add("segments",execution_segments.size()).add("guide_id",data->traj_id).add("guide_start",data->start_time));''')
    anchor = '''                                           bool dynamic_repair){'''
    text = replace(text, anchor, anchor+'''
    dual_probe::Request dual_request(dual_probe::Fields().add("fsm",int(exec_state_))
      .add("local_execution_owner",local_execution_owner_?"true":"false").add("poly_init",flag_use_poly_init?"true":"false")
      .add("start_x",start_pos_(0)).add("start_y",start_pos_(1)).add("start_yaw",start_yaw_)
      .add("raw_seq",planner_manager_->grid_map_->getDynamicObservationSeq())
      .add("esdf_seq",planner_manager_->grid_map_->getESDFObservationSeq()));''')
    start = text.index('  bool REMANIReplanFSM::callReboundReplan(')
    end = text.index('  bool REMANIReplanFSM::callEmergencyStop(', start)
    method = text[start:end]
    method = replace(method, '    if (plan_success){', '''    dual_probe::emit("manager_result",dual_probe::Fields().add("success",plan_success?"true":"false"));
    if (plan_success){
      dual_probe::Scope dual_validation("live_acceptance");''')
    # Preserve every existing rejection and mark it without changing its logic.
    reasons = [
        ('          return finish_failed_planning_attempt();\n        }\n      }\n      if (global_plan_', 'late_moving_handoff'),
        ('        ROS_WARN("[FSM] Rejecting replan candidate: static', 'static_or_swept_collision'),
        ('            ROS_ERROR("[FSM] Rejecting replan candidate: latest point-flow', 'prediction_unavailable'),
        ('          ROS_WARN("[FSM] Rejecting replan candidate: latest time-aligned', 'latest_prediction_collision'),
        ('        ROS_WARN("[FSM] Rejecting replan candidate: live obstacle', 'live_occupancy_collision'),
    ]
    for anchor, reason in reasons:
        if anchor.startswith('          return'):
            method = replace(method, anchor, '          dual_probe::emit("acceptance_reject",dual_probe::Fields().str("reason","'+reason+'"));\n'+anchor)
        else:
            method = replace(method, anchor, '        dual_probe::emit("acceptance_reject",dual_probe::Fields().str("reason","'+reason+'"));\n'+anchor)
    method = replace(method, '      init_time_list_.push_back(init_time);', '''      dual_validation.success=true;dual_validation.finish();
      dual_probe::emit("acceptance_pass");
      init_time_list_.push_back(init_time);''')
    method = replace(method, '    return plan_success;', '    dual_request.success=plan_success;\n    return plan_success;')
    return text[:start]+method+text[end:]


def instrument_manager(text):
    text = replace(text, '#include "resource_probe.hpp"', '#include "resource_probe.hpp"\n#include "dual_timescale_probe.hpp"')
    text = replace(text, '    static bool flag_first_call = true;', '''    dual_probe::Scope dual_initialization("initialization",dual_probe::Fields().add("poly_init",flag_polyInit?"true":"false"));
    static bool flag_first_call = true;''')
    text = replace(text, '      int status = ploy_traj_opt_->astarWithMinTraj(', '''      dual_probe::emit("stage_begin",dual_probe::Fields().str("stage","search"));
      const double dual_search_start=dual_probe::mono();
      int status = ploy_traj_opt_->astarWithMinTraj(''')
    text = replace(text, '      // std::cout << "computeInit 4\\n";', '''      dual_probe::emit("stage_end",dual_probe::Fields().str("stage","search")
        .add("wall_ms",(dual_probe::mono()-dual_search_start)*1000).add("status",status)
        .add("success",(status!=KinoAstar::NO_PATH&&status!=KinoAstar::START_COLLISION&&status!=KinoAstar::GOAL_COLLISION)?"true":"false"));
      // std::cout << "computeInit 4\\n";''')
    start = text.index('  bool MMPlannerManager::computeInitReferenceState(')
    end = text.index('\n  bool MMPlannerManager::', start+10)
    method = text[start:end]
    method = replace(method, '    return true;', '    dual_initialization.success=true;\n    return true;')
    text = text[:start]+method+text[end:]
    text = replace(text, '    resource_probe::Scope request_probe("replan");', '''    dual_probe::Scope dual_manager("manager_replan",dual_probe::Fields()
      .add("start_x",start_pt(0)).add("start_y",start_pt(1)).add("start_yaw",start_yaw)
      .add("target_x",local_target_pt(0)).add("target_y",local_target_pt(1)));
    resource_probe::Scope request_probe("replan");
    dual_probe::emit("resource_link",dual_probe::Fields().add("resource_replan_id",request_probe.id));''')
    text = replace(text, '    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(', '''    dual_probe::emit("stage_begin",dual_probe::Fields().str("stage","optimizer"));
    const double dual_opt_start=dual_probe::mono();
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(''')
    text = replace(text, '    t_opt = ros::Time::now() - t_start;', '''    dual_probe::emit("stage_end",dual_probe::Fields().str("stage","optimizer")
      .add("wall_ms",(dual_probe::mono()-dual_opt_start)*1000).add("success",flag_success?"true":"false"));
    t_opt = ros::Time::now() - t_start;''')
    text = replace(text, '    request_probe.success=true;return true;', '    dual_manager.success=true;request_probe.success=true;return true;')
    return text


def build(base, out, prepare_only=False, finish_only=False):
    base=base.resolve();out=out.resolve()
    manifest=json.loads((base/'build_manifest.json').read_text())
    if not manifest.get('failure_diagnosis'):raise RuntimeError('requires audited failure diagnosis base')
    parent=Path(manifest['parent_build'])
    if not finish_only:
        out.mkdir(parents=True,exist_ok=False);shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
        for name in ('mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so','libexp_opt.a'):
            shutil.copy2(base/'build'/name,out/'build'/name)
    shutil.copy2(HERE/'dual_timescale_probe.hpp',out/'source/dual_timescale_probe.hpp')
    (out/'source/remani_replan_fsm.cpp').write_text(instrument_fsm((base/'source/remani_replan_fsm.cpp').read_text()))
    (out/'source/planner_manager.cpp').write_text(instrument_manager((base/'source/planner_manager.cpp').read_text()))
    if prepare_only:print('PREPARED',out,flush=True);return
    cm=parent/'build/CMakeFiles/remani_planner_node.dir';link=shlex.split((cm/'link.txt').read_text())
    reused=[]
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(s for s in link if s.endswith('/'+name+'.o'))
        if name=='remani_planner_node.cpp':
            # Find the prior build that actually owns the unchanged object.
            if sha(out/'source'/name)!=sha(base/'source'/name):raise RuntimeError('node source changed')
            candidates=[base/'build'/(name+'.o'),HERE/'builds/failure_diagnosis_v12_02/build'/(name+'.o')]
            obj=next((p for p in candidates if p.exists() and sha(p.parent.parent/'source'/name)==sha(out/'source'/name)),None)
            if obj is not None and sha(obj.parent.parent/'source/plan_manage/remani_replan_fsm.h')!=sha(out/'source/plan_manage/remani_replan_fsm.h'):
                raise RuntimeError('cannot reuse node object with changed FSM class header')
            if obj is None:_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
            else:reused.append({'object':str(obj),'sha256':sha(obj),'source_sha256':sha(out/'source'/name)})
        else:_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[str(obj) if s==old else s for s in link]
    search=HERE/'builds/resource_probe_v12_01/build/libexp_search.a'
    if not search.exists():raise RuntimeError('missing reviewed search archive')
    link=[str(out/'build/libexp_opt.a') if s=='libexp_opt.a' else str(search) if s=='libexp_search.a' else s for s in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node');subprocess.run(link,cwd=str(parent/'build'),check=True)
    manifest.update(dual_timescale_probe=True,dual_timescale_parent=str(base),
        production_modified=False,dual_probe_scope='coarse FSM/wait/search/optimization/validation/publication events; no algorithm changes',
        required_env={'FAILURE_PROFILE_DISABLED':'1'},forbidden_env=['FAILURE_DIAG_KIND','RESOURCE_PAIRED_PID_FILE'],
        observer_note='reused optimizer still computes first-gradient hash and 5ms candidate wheel statistics; no leaf timers when required env is set',
        dual_reused_objects=reused,dual_reused_archives={str(p):sha(p) for p in [base/'build/libexp_opt.a',search]},
        native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        dual_builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/failure_diagnosis_v12_04')
    p.add_argument('--out',type=Path,default=HERE/'builds/dual_timeline_v12_01')
    p.add_argument('--prepare-only',action='store_true');p.add_argument('--finish-only',action='store_true')
    a=p.parse_args();build(a.base,a.out,a.prepare_only,a.finish_only)
