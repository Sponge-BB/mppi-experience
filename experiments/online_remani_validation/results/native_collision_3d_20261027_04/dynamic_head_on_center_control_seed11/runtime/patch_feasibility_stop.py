#!/usr/bin/env python3
"""Exact reversible stop-only diagnostic patch; never writes source files."""
import hashlib

BASE_OPTIMIZER_SHA256='3c46d7156d92656d1e339899e375c07b988778c63b9b0ceb780319f055918206'
BASE_LBFGS_SHA256='61f12f271651044b7a05a2f1fd34f0e66362535ce1b09ea02a73d4d25efec0e5'
SWITCH='FEASIBILITY_STOP_GATE'
INCLUDE='#include "feasibility_stop_gate.hpp"\n'

SESSION=r'''
    // Only the predeclared paused diagnostic middle pass may veto cost-stop.
    const char* feasibility_switch=std::getenv("FEASIBILITY_STOP_GATE");
    const bool feasibility_enabled=feasibility_switch && std::string(feasibility_switch)=="1" &&
        delivery_replay::running() && delivery_replay::phase()=="delivery_stop_on_060";
    feasibility_stop_gate::Session feasibility_session(feasibility_enabled,
      [this](const Eigen::VectorXd&)->feasibility_stop_gate::Check {
        feasibility_stop_gate::Check check;
        const double began=ros::WallTime::now().toSec();
        const auto expired=[this](){
          return (ros::WallTime::now()-optimization_start_wall_time_).toSec()>=max_wall_time_;
        };
        const auto timeout=[&](){
          check.budget_expired=true;check.reason="original_deadline_expired";
          force_stop_type_=STOP_FOR_TIMEOUT;
          check.total_wall_s=ros::WallTime::now().toSec()-began;
        };
        if(expired()){timeout();return check;}
        const auto frame=coupling_geometry::currentSnapshot();
        const double frame_now=coupling_geometry::currentTime();
        if(!coupling_geometry::enabled() || !coupling_geometry::activeFrame().active ||
           !frame || !shared_collision::fresh(*frame,frame_now) || !frame->obstacles.empty())
          throw std::runtime_error("feasibility gate requires a fresh frozen static diagnostic frame");
        if(SnapOpt_container_.empty() || SnapOpt_container_.size()!=singul_container_.size())
          throw std::runtime_error("feasibility gate polynomial/gear shape mismatch");
        SingulTrajData candidate;candidate.clearSingulTraj();double elapsed=0.;
        for(size_t b=0;b<SnapOpt_container_.size();++b){
          const auto& times=SnapOpt_container_[b].get_T1();
          if((singul_container_[b]!=1 && singul_container_[b]!=-1) || times.size()<1 ||
             !times.allFinite() || (times.array()<=0.).any() ||
             !std::isfinite(times.sum()) || !SnapOpt_container_[b].get_b().allFinite())
            throw std::runtime_error("feasibility gate malformed polynomial cache");
          candidate.addSingulTraj(SnapOpt_container_[b].getTraj(singul_container_[b]),elapsed);
          elapsed=candidate.singul_traj.back().end_time;
        }
        {
          // Preserve the actual solver verdict; native checking can overwrite it.
          delivery_replay::RestoreFailure restore;
          const double start=ros::WallTime::now().toSec();
          check.native_pass=IsTrajSafe(candidate);check.native_known=true;
          check.native_wall_s=ros::WallTime::now().toSec()-start;
        }
        if(expired()){timeout();return check;}
        if(!check.native_pass){
          check.reason="native_gate_reject";
          check.total_wall_s=ros::WallTime::now().toSec()-began;return check;
        }
        const double projection_start=ros::WallTime::now().toSec();
        check.projected_known=true;check.projected_pass=true;
        // Exactly the static part of coupling_bridge::projectedCandidate:
        // same part-wide .02s grid, gear-derived yaw, full body and gap gate.
        for(const auto& part:candidate.singul_traj){
          const int steps=std::max(1,int(std::ceil(part.duration/.02)));
          for(int k=0;k<=steps;++k){const double q=part.duration*k/steps;
            if(k%16==0 && expired()){
              check.projected_known=false;check.projected_pass=false;
              check.projected_wall_s=ros::WallTime::now().toSec()-projection_start;
              timeout();return check;
            }
            auto p=part.traj.getPos(q);auto v=part.traj.getVel(q);
            const double yaw=std::atan2(part.singul*v(1),part.singul*v(0));Eigen::Vector3d pose(p(0),p(1),yaw);
            auto s=coupling_geometry::staticProjectedPose(*frame,pose);
            if(!s.valid)throw std::runtime_error("feasibility gate invalid static projection");
            if(s.gap<-1e-8){check.projected_pass=false;break;}
          }
          if(!check.projected_pass)break;
        }
        check.projected_wall_s=ros::WallTime::now().toSec()-projection_start;
        if(expired()){timeout();return check;}
        check.reason=check.allowed()?"both_gates_pass":
            (!check.native_pass?"native_gate_reject":"projected_gate_reject");
        check.total_wall_s=ros::WallTime::now().toSec()-began;return check;
      },
      [this](const feasibility_stop_gate::Statistics& stats){
        if(!delivery_replay::running())return;
        delivery_replay::emit("\"kind\":\"feasibility_stop_summary\",\"enabled\":"+
          std::string(stats.enabled?"true":"false")+
          ",\"checks\":"+std::to_string(stats.checks)+",\"blocked_count\":"+std::to_string(stats.blocked)+
          ",\"allowed_count\":"+std::to_string(stats.allowed)+
          ",\"native_pass_count\":"+std::to_string(stats.native_pass)+
          ",\"projected_pass_count\":"+std::to_string(stats.projected_pass)+
          ",\"budget_expired_count\":"+std::to_string(stats.budget_expired)+
          ",\"native_wall_s\":"+delivery_replay::num(stats.native_wall_s)+
          ",\"projected_wall_s\":"+delivery_replay::num(stats.projected_wall_s)+
          ",\"gate_wall_s\":"+delivery_replay::num(stats.total_wall_s)+
          ",\"last_reason\":"+delivery_replay::quote(stats.last.reason)+
          ",\"fatal\":"+std::string(stats.fatal?"true":"false")+
          ",\"fatal_reason\":"+delivery_replay::quote(stats.fatal_reason)+
          ",\"feasibility_decision_elapsed_s\":"+delivery_replay::num(
            (ros::WallTime::now()-optimization_start_wall_time_).toSec())+
          ",\"original_budget_s\":"+delivery_replay::num(max_wall_time_)+
          ",\"effective_variant\":"+delivery_replay::quote(
            stats.enabled?"cost_stop_veto":"original_cost_stop")+
          ",\"scope\":\"paused diagnostic stop veto, unchanged objective and original deadline\"");
      });
'''

FINAL_DEADLINE=r'''
    // Common diagnostic timing guard, OFF/ON/OFF alike. The original budget
    // covers final native validation and output construction, not just LBFGS.
    if(delivery_replay::running() &&
       (ros::WallTime::now()-optimization_start_wall_time_).toSec()>=max_wall_time_){
      diagnosis.reason="original_deadline_exhausted_at_return";
      diagnosis.complete=false;probe.success=false;return false;
    }
'''

OPTIMIZER_REPLACEMENTS=[
 ('const double budgets[]={.60,.60,1.50};','const double budgets[]={.60,.60,.60};'),
 ('const char* labels[]={"delivery_nominal_060","delivery_repeat_060","delivery_long_150"};',
  'const char* labels[]={"delivery_nominal_060","delivery_stop_on_060","delivery_repeat_060"};'),
 ('    delivery_trial.begin();',SESSION+'    delivery_trial.begin();'),
 ('    failure_probe::firstCost(costall,g);',
  '    feasibility_stop_gate::recordEvaluated(x);\n    failure_probe::firstCost(costall,g);'),
 ('    diagnosis.complete=good_traj;',FINAL_DEADLINE+'    diagnosis.complete=good_traj;'),
]
LBFGS_REPLACEMENTS=[
 ('''                        if (rate < param.delta)
                        {
                            ret = LBFGS_STOP;
                            break;
                        }''',
  '''                        if (rate < param.delta)
                        {
                            if (feasibility_stop_gate::allowCostStop(x))
                            {
                                ret = LBFGS_STOP;
                                break;
                            }
                            if (feasibility_stop_gate::abortRequested())
                            {
                                ret = LBFGS_CANCELED;
                                break;
                            }
                        }'''),
]

def _digest(text):
    return hashlib.sha256(text.encode()).hexdigest()

def _replace(text,old,new):
    if text.count(old)!=1:
        raise ValueError('Expected exactly one patch anchor: '+repr(old[:110]))
    return text.replace(old,new,1)

def _transform(text,kind):
    expected=BASE_OPTIMIZER_SHA256 if kind=='optimizer' else BASE_LBFGS_SHA256
    if _digest(text)!=expected:
        raise ValueError('Unattested '+kind+' source; refusing partial/repeated patch')
    for old,new in OPTIMIZER_REPLACEMENTS if kind=='optimizer' else LBFGS_REPLACEMENTS:
        text=_replace(text,old,new)
    if kind=='optimizer':
        text=INCLUDE+text
    else:
        marker='#define DOMAIN_STEP_LBFGS_HPP\n'
        text=_replace(text,marker,marker+'\n'+INCLUDE)
    return text

def transform_optimizer(text):
    return _transform(text,'optimizer')

def transform_lbfgs(text):
    return _transform(text,'lbfgs')

def project_off(text,kind='optimizer'):
    if kind not in ('optimizer','lbfgs'):
        raise ValueError('Unknown source kind')
    if kind=='optimizer':
        if not text.startswith(INCLUDE):raise ValueError('Missing unique optimizer include')
        text=text[len(INCLUDE):]
    else:
        marker='#define DOMAIN_STEP_LBFGS_HPP\n'
        text=_replace(text,marker+'\n'+INCLUDE,marker)
    replacements=OPTIMIZER_REPLACEMENTS if kind=='optimizer' else LBFGS_REPLACEMENTS
    for old,new in reversed(replacements):
        text=_replace(text,new,old)
    expected=BASE_OPTIMIZER_SHA256 if kind=='optimizer' else BASE_LBFGS_SHA256
    if _digest(text)!=expected:
        raise ValueError('OFF source projection differs from exact baseline')
    return text
