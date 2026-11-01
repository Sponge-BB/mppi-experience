#!/usr/bin/env python3
"""Exact reversible diagnostic budget/coordinate patch, no filesystem writes."""
import hashlib

BASE_OPTIMIZER_SHA256='79262a1c2aea24964b34baf5a08dca1d328722b263b0c447841249fc1211c2eb'
BASE_REPLAY_SHA256='469a738668739f856c71d2f0ed96643d8e328f90dcc8089d9a557cae71737efd'
PHASES=('budget_original_060','budget_reference_060','budget_extended_300',
        'budget_scaled_300','budget_repeat_060')
BUDGETS=(.6,.6,3.,3.,.6)
SWITCH='FEASIBILITY_STOP_GATE'
INCLUDE='#include "budget_scaling.hpp"\n'

ORIGINAL_CALL='''    result = domain_lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        lbfgs_params);'''

SCALED_CALL=r'''    const bool scaled_coordinates=delivery_replay::running() &&
        budget_scaling::scaledPhase(delivery_replay::phase());
    if(scaled_coordinates){
      const double setup_start=ros::WallTime::now().toSec();
      const auto scaling_plan=budget_scaling::makePlan(initT_container,traj_dim_,mobile_base_dof_);
      if(scaling_plan.diagonal.size()!=x.size())throw std::runtime_error("native scaling layout mismatch");
      budget_scaling::Context context(this,x,scaling_plan.diagonal,
        PolyTrajOptimizer::costFunctionCallback,PolyTrajOptimizer::earlyExitCallback,
        [](void* raw)->bool{
          return static_cast<PolyTrajOptimizer*>(raw)->force_stop_type_!=DONT_STOP;
        });
      Eigen::VectorXd z=Eigen::VectorXd::Zero(x.size());
      const double setup_wall=ros::WallTime::now().toSec()-setup_start;
      delivery_replay::emit("\"kind\":\"coordinate_scaling_start\",\"enabled\":true"
        ",\"model\":\"x=x0+D*z; z0=0; fixed initial-time MinSnap T^-7 spatial scaling\""
        ",\"canonical_logging\":\"native x/f/g, original cached gradient\""
        ",\"step_semantics\":\"z-space line-search scalar, not physical displacement\""
        ",\"setup_wall_s\":"+delivery_replay::num(setup_wall)+
        ",\"setup_counted_in_budget\":true,\"minimum_initial_time_s\":"+delivery_replay::num(scaling_plan.minimum_time)+
        ",\"maximum_initial_time_s\":"+delivery_replay::num(scaling_plan.maximum_time)+
        ",\"minimum_diagonal\":"+delivery_replay::num(scaling_plan.diagonal.minCoeff())+
        ",\"maximum_diagonal\":"+delivery_replay::num(scaling_plan.diagonal.maxCoeff())+
        ",\"numerical_floor\":"+delivery_replay::num(scaling_plan.numerical_floor)+
        ",\"floor_count\":"+std::to_string(scaling_plan.floor_count)+
        ",\"scaled_count\":"+std::to_string(scaling_plan.scaled_count)+
        ",\"diagonal\":"+delivery_diag::vector(scaling_plan.diagonal));
      result=domain_lbfgs::lbfgs_optimize(z,final_cost,budget_scaling::Context::evaluate,
        NULL,budget_scaling::Context::progress,&context,lbfgs_params);
      x=context.canonical(z);
      delivery_replay::emit("\"kind\":\"coordinate_scaling_end\",\"enabled\":true"
        ",\"evaluations\":"+std::to_string(context.evaluations)+
        ",\"progress_calls\":"+std::to_string(context.progress_calls)+
        ",\"stopped_progress\":"+std::to_string(context.stopped_progress)+
        ",\"maximum_canonical_step_norm\":"+delivery_replay::num(context.maximum_canonical_step)+
        ",\"negative_solver_result_never_delivered\":true");
    }else{
''' + ORIGINAL_CALL + r'''
    }'''

FINAL_SCOPE=r'''
    budget_scaling::FinalState budget_final;
    budget_scaling::CanonicalSession canonical_session(delivery_replay::running());
    budget_scaling::OnExit budget_final_report(delivery_replay::running(),[&](){
      const double emission_elapsed=(ros::WallTime::now()-optimization_start_wall_time_).toSec();
      const double elapsed=budget_final.decision_elapsed_s>=0.?budget_final.decision_elapsed_s:emission_elapsed;
      const bool in_budget=std::isfinite(elapsed) && elapsed<max_wall_time_;
      const bool identity_known=canonical_session.evaluated;
      const bool identity_match=identity_known && budget_scaling::identical(x,canonical_session.last_x);
      const bool accepted_match=canonical_session.accepted && budget_scaling::identical(x,canonical_session.last_accepted_x);
      const auto* d=failure_probe::active();
      delivery_replay::emit("\"kind\":\"budget_final_validation\",\"validation_schema\":\"budget_final_validation_v1\""
        ",\"valid\":"+std::string(std::isfinite(elapsed)&&max_wall_time_>0.?"true":"false")+
        ",\"solver_result_known\":"+(budget_final.solver_result_known?"true":"false")+
        ",\"solver_result\":"+std::to_string(budget_final.solver_result)+
        ",\"force_stop\":"+std::to_string(int(force_stop_type_))+
        ",\"native_known\":"+(budget_final.native_known?"true":"false")+
        ",\"native_pass\":"+(budget_final.native_pass?"true":"false")+
        ",\"projected_known\":"+(budget_final.projected_known?"true":"false")+
        ",\"projected_pass\":"+(budget_final.projected_pass?"true":"false")+
        ",\"elapsed_s\":"+delivery_replay::num(elapsed)+",\"budget_s\":"+delivery_replay::num(max_wall_time_)+
        ",\"marker_emission_elapsed_s\":"+delivery_replay::num(emission_elapsed)+
        ",\"in_budget\":"+(in_budget?"true":"false")+
        ",\"final_x_hash\":"+delivery_replay::quote(std::to_string(feasibility_stop_gate::vectorHash(x)))+
        ",\"last_canonical_x_hash\":"+delivery_replay::quote(identity_known?std::to_string(feasibility_stop_gate::vectorHash(canonical_session.last_x)):"unknown")+
        ",\"last_accepted_x_hash\":"+delivery_replay::quote(canonical_session.accepted?std::to_string(feasibility_stop_gate::vectorHash(canonical_session.last_accepted_x)):"unknown")+
        ",\"cache_identity_known\":"+(identity_known?"true":"false")+
        ",\"cache_identity_match\":"+(identity_match?"true":"false")+
        ",\"accepted_identity_match\":"+(accepted_match?"true":"false")+
        ",\"output_constructed\":"+(budget_final.output_constructed?"true":"false")+
        ",\"output_hash\":"+delivery_replay::quote(std::to_string(budget_final.output_hash))+
        ",\"output_hash_semantics\":\"fnv1a64 native output controlpoints and real times including matrix shapes\""
        ",\"allowed\":"+(budget_final.allowed?"true":"false")+
        ",\"success\":"+(budget_final.allowed?"true":"false")+
        ",\"reason\":"+delivery_replay::quote(d?d->reason:"missing")+
        ",\"scope\":\"all phases same final native/projected/cache/output/deadline gate, no publication\"");
    });
'''

FINAL_PROJECTION=r'''    good_traj = IsTrajSafe(singul_traj_data);
    if(delivery_replay::running()){
      budget_final.native_known=true;budget_final.native_pass=good_traj;
      if(good_traj){
        const auto frame=coupling_geometry::currentSnapshot();
        const double epoch=coupling_geometry::currentTime();
        if(!frame || !coupling_geometry::activeFrame().active ||
           !shared_collision::fresh(*frame,epoch) || !frame->obstacles.empty())
          throw std::runtime_error("final projection requires fresh frozen static frame");
        budget_final.projected_known=true;budget_final.projected_pass=true;
        for(const auto& part:singul_traj_data.singul_traj){
          const int steps=std::max(1,int(std::ceil(part.duration/.02)));
          for(int k=0;k<=steps;++k){const double q=part.duration*k/steps;
            if(k%16==0 && (ros::WallTime::now()-optimization_start_wall_time_).toSec()>=max_wall_time_){
              budget_final.projected_known=false;budget_final.projected_pass=false;
              force_stop_type_=STOP_FOR_TIMEOUT;diagnosis.reason="final_projection_budget_timeout";
              good_traj=false;break;
            }
            auto p=part.traj.getPos(q);auto v=part.traj.getVel(q);
            const double yaw=std::atan2(part.singul*v(1),part.singul*v(0));Eigen::Vector3d pose(p(0),p(1),yaw);
            const auto sample=coupling_geometry::staticProjectedPose(*frame,pose);
            if(!sample.valid)throw std::runtime_error("invalid final static projection sample");
            if(sample.gap<-1e-8){
              budget_final.projected_pass=false;good_traj=false;
              diagnosis.reason="final_projected_collision";break;
            }
          }
          if(!budget_final.projected_pass)break;
        }
      }
    }'''

FINAL_SUCCESS=r'''    if(delivery_replay::running()){
      const bool cache_match=canonical_session.evaluated &&
        budget_scaling::identical(x,canonical_session.last_x) &&
        canonical_session.accepted && budget_scaling::identical(x,canonical_session.last_accepted_x);
      if(good_traj && !cache_match){good_traj=false;diagnosis.reason="final_native_cache_mismatch";}
      if(good_traj && (!budget_final.native_known || !budget_final.native_pass ||
          !budget_final.projected_known || !budget_final.projected_pass ||
          !budget_final.output_constructed || force_stop_type_!=DONT_STOP)){
        good_traj=false;diagnosis.reason="final_full_validation_incomplete";
      }
      budget_final.decision_elapsed_s=(ros::WallTime::now()-optimization_start_wall_time_).toSec();
      if(budget_final.decision_elapsed_s>=max_wall_time_){
        good_traj=false;diagnosis.reason="final_qualification_budget_timeout";
      }
      budget_final.allowed=good_traj;
    }
    diagnosis.complete=good_traj;'''

OPTIMIZER_REPLACEMENTS=[
    ('const double budgets[]={.60,.60,.60};','const double budgets[]={.60,.60,3.00,3.00,.60};'),
    ('const char* labels[]={"delivery_nominal_060","delivery_stop_on_060","delivery_repeat_060"};',
     'const char* labels[]={"'+ '\",\"'.join(PHASES) +'"};'),
    ('for(size_t i=0;i<3;++i){','for(size_t i=0;i<5;++i){'),
    ('// Only the predeclared paused diagnostic middle pass may veto cost-stop.',
     '// Only the four predeclared paused diagnostic veto phases may veto cost-stop.'),
    ('delivery_replay::running() && delivery_replay::phase()=="delivery_stop_on_060";',
     'delivery_replay::running() && budget_scaling::vetoPhase(delivery_replay::phase());'),
    ('    optimization_start_wall_time_ = ros::WallTime::now();',
     '    optimization_start_wall_time_ = ros::WallTime::now();\n'+FINAL_SCOPE),
    (ORIGINAL_CALL,SCALED_CALL),
    ('    } catch(const domain_step::RejectedTrial& error) {',
     '    budget_final.solver_result_known=true;budget_final.solver_result=result;\n'
     '    } catch(const domain_step::RejectedTrial& error) {'),
    ('    good_traj = IsTrajSafe(singul_traj_data);',FINAL_PROJECTION),
    ('    // Common diagnostic timing guard, OFF/ON/OFF alike.',
     '    if(delivery_replay::running()){\n'
     '      budget_final.output_constructed=true;budget_final.output_hash=1469598103934665603ULL;\n'
     '      resource_probe::hashMatrices(budget_final.output_hash,optCps_container);\n'
     '      resource_probe::hashMatrices(budget_final.output_hash,optT_container);\n'
     '    }\n'
     '    // Common diagnostic timing guard, OFF/ON/OFF alike.'),
    ('    diagnosis.complete=good_traj;',FINAL_SUCCESS),
    ('    feasibility_stop_gate::recordEvaluated(x);',
     '    budget_scaling::recordCanonical(x);\n    feasibility_stop_gate::recordEvaluated(x);'),
    ('    delivery_replay::progress(x,g,fx,step,k,ls);',
     '    if(opt->force_stop_type_==DONT_STOP)budget_scaling::recordAccepted(x);\n'
     '    delivery_replay::progress(x,g,fx,step,k,ls);'),
    ('// Common diagnostic timing guard, OFF/ON/OFF alike.',
     '// Common diagnostic timing guard, all five phases alike.'),
]
REPLAY_REPLACEMENTS=[('const auto&s=signatures();if(s.size()!=3)return false;',
                      'const auto&s=signatures();if(s.size()!=5)return false;')]

def _digest(text):
    return hashlib.sha256(text.encode()).hexdigest()

def _replace(text,old,new):
    if text.count(old)!=1:
        raise ValueError('Expected exactly one patch anchor: '+repr(old[:110]))
    return text.replace(old,new,1)

def _transform(text,kind):
    expected=BASE_OPTIMIZER_SHA256 if kind=='optimizer' else BASE_REPLAY_SHA256
    if _digest(text)!=expected:
        raise ValueError('Unattested '+kind+' source; refusing partial/repeated patch')
    replacements=OPTIMIZER_REPLACEMENTS if kind=='optimizer' else REPLAY_REPLACEMENTS
    for old,new in replacements:text=_replace(text,old,new)
    if kind=='optimizer':text=INCLUDE+text
    return text

def transform_optimizer(text):
    return _transform(text,'optimizer')

def transform_replay(text):
    return _transform(text,'replay')

def project_off(text,kind='optimizer'):
    if kind not in ('optimizer','replay'):raise ValueError('Unknown source kind')
    if kind=='optimizer':
        if not text.startswith(INCLUDE):raise ValueError('Missing unique optimizer include')
        text=text[len(INCLUDE):]
    replacements=OPTIMIZER_REPLACEMENTS if kind=='optimizer' else REPLAY_REPLACEMENTS
    for old,new in reversed(replacements):text=_replace(text,new,old)
    expected=BASE_OPTIMIZER_SHA256 if kind=='optimizer' else BASE_REPLAY_SHA256
    if _digest(text)!=expected:raise ValueError('OFF source projection differs from exact parent')
    return text
