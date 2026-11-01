#!/usr/bin/env python3
"""Runtime-gated timeout candidate recovery for a private native67 child only."""
from build_state_contract import Transformer


def transform_optimizer(text):
    t = Transformer(text)
    t.before('#include "native67_probe.hpp"', '#include "delivery_accepted_candidate.hpp"\n')
    t.before('    traj_num_ = initInnerPts_container.size();', '''    delivery_candidate::Scope accepted_candidate;
    probe.add("candidate_mode",accepted_candidate.enabled?1:0);
    bool salvage_attempted=false;

''')
    t.before('    if(force_stop_type_ == STOP_FOR_TIMEOUT){', '''    probe.add("candidate_available",accepted_candidate.usable()?1:0);
    probe.add("candidate_accepted_steps",accepted_candidate.accepted_steps);
    probe.add("candidate_source_initial",accepted_candidate.initial?1:0);
''')
    old_timeout = '''    if(force_stop_type_ == STOP_FOR_TIMEOUT){
      diagnosis.reason="budget_timeout";diagnose("timeout",nullptr);
      ROS_WARN("[Optimizer] Stop after %.2f s wall-time budget.",
               max_wall_time_);
      return false;
    }'''
    new_timeout = '''    if(force_stop_type_ == STOP_FOR_TIMEOUT){
      diagnosis.reason="budget_timeout";diagnose("timeout",nullptr);
      ROS_WARN("[Optimizer] Stop after %.2f s wall-time budget.",
               max_wall_time_);
      if(!accepted_candidate.usable())return false;
      // No blind acceptance: reconstruct the last actual iterate, then execute
      // exactly the ordinary native gate below. FSM latest-data gates remain.
      salvage_attempted=true;probe.add("salvage_attempt",1);
      const auto rebuild_start=ros::WallTime::now();
      std::vector<delivery_candidate::Part> parts;
      bool rebuilt=delivery_candidate::decode(accepted_candidate.x,piece_num_container_,traj_dim_,
        iniState_container_,finState_container_,non_singul_v_,parts);
      if(rebuilt){
        for(int i=0;i<traj_num_;++i){
          SnapOpt_container_[i].generate(parts[i].points,parts[i].times,parts[i].initial,parts[i].final);
          // Callback caches may be a rejected line-search trial: regenerate
          // both polynomial coefficients AND their displayed/output samples.
          cps_container_[i].points=SnapOpt_container_[i].getInitConstrainPoints(cps_num_prePiece_);
          if(!SnapOpt_container_[i].get_b().allFinite()||!cps_container_[i].points.allFinite()){
            rebuilt=false;break;
          }
        }
      }
      probe.add("salvage_rebuilt",rebuilt?1:0);
      probe.add("salvage_rebuild_ms",(ros::WallTime::now()-rebuild_start).toSec()*1000.);
      if(!rebuilt){diagnosis.reason="timeout_candidate_rebuild_failed";return false;}
    }'''
    t.change(old_timeout, new_timeout)
    t.change('    if(result<0){', '    if(result<0&&!salvage_attempted){')
    t.change('''    good_traj = IsTrajSafe(singul_traj_data);
    // native67:''', '''    const auto native_validation_start=ros::WallTime::now();
    good_traj = IsTrajSafe(singul_traj_data);
    probe.add("native_validation_ms",(ros::WallTime::now()-native_validation_start).toSec()*1000.);
    if(salvage_attempted){
      probe.add("salvage_native_result",good_traj?1:0);
      probe.add("salvage_validation_ms",(ros::WallTime::now()-native_validation_start).toSec()*1000.);
    }
    // native67:''')
    t.change('    if(result == domain_lbfgs::LBFGSERR_INVALID_FUNCVAL){',
             '    if(result == domain_lbfgs::LBFGSERR_INVALID_FUNCVAL&&!salvage_attempted){')
    t.after('    native67_probe::Callback native67_callback;', '\n    delivery_candidate::beginEvaluation();')
    t.change('''\n        opt->force_stop_type_=STOP_FOR_ERROR;g.setZero();return 1.0e100;''',
             '''\n        delivery_candidate::fail();
        opt->force_stop_type_=STOP_FOR_ERROR;g.setZero();return 1.0e100;''')
    t.change('''\n      opt->force_stop_type_=STOP_FOR_ERROR;g.setZero();return 1.0e100;''',
             '''\n      delivery_candidate::fail();
      opt->force_stop_type_=STOP_FOR_ERROR;g.setZero();return 1.0e100;''')
    t.after('    if(non_finite){', '\n      delivery_candidate::fail();')
    t.before('    // native67: no diagnostic stop-veto cache.',
             '    delivery_candidate::completed(x,g,costall);\n')
    t.before('    // native67: progress observation only.', '''    if(opt->force_stop_type_==DONT_STOP)delivery_candidate::accepted(x,fx);
''')
    return t.finish()
