#!/usr/bin/env python3
"""Invertible diagnostic-only patch of the frozen admission optimizer.

No compiler/ROS invocation. Does not relax a wheel limit or publish candidates.
"""
from build_state_contract import Transformer


ENTRY = r'''
    if(delivery_replay::requested(max_wall_time_)){
      delivery_replay::used()=true;
      bool sil=false,sim=false;
      ros::param::param("/online_validation/enabled",sil,false);
      ros::param::param("/use_sim_time",sim,false);
      if(!sil||!sim||!coupling_geometry::enabled()||!coupling_geometry::activeFrame().active)
        throw std::runtime_error("delivery replay requires explicit isolated SIL outer frozen frame");
      failure_probe::PauseOwned delivery_paused;
      const auto frozen_frame=coupling_geometry::activeFrame();
      const std::string frozen_prediction=delivery_replay::predictionText();
      const ros::Time frozen_rosnow=ros::Time::now();const double frozen_sim=frozen_rosnow.toSec();
      const auto raw=grid_map_->getDynamicObservationSeq(),esdf=grid_map_->getESDFObservationSeq();
      const ros::Time native_query_time=frozen_rosnow;
      const auto native_dynamic=grid_map_->getDynamicPointSnapshot(native_query_time);
      const std::string native_dynamic_text=delivery_replay::nativeDynamicText(native_dynamic);
      const std::string native_dynamic_hash=delivery_diag::hash(native_dynamic_text);
      const bool native_overlay_enabled=grid_map_->dynamicPointPlanningOverlayEnabled();
      const auto* request_context=boundary_admission::currentRequest();
      // Native begin/end is NOT nested/ref-counted. Only this proven ordinary
      // static request can own a new overlay freeze without releasing a caller's.
      if(!request_context||request_context->dynamic_repair)
        throw std::runtime_error("cannot prove native overlay freeze is not already caller-owned");
      bool native_overlay_owned=false;
      if(native_overlay_enabled&&!native_dynamic.points.empty()){
        native_overlay_owned=grid_map_->beginDynamicPointPlanningSnapshot(native_dynamic,native_query_time);
        if(!native_overlay_owned)throw std::runtime_error("cannot freeze nonempty native dynamic overlay");
      }
      delivery_replay::Finally native_overlay_release([&](){
        if(native_overlay_owned)grid_map_->endDynamicPointPlanningSnapshot();
      });
      bool native_dynamic_stable=true;
      auto observe_native_dynamic=[&](const char* position){
        const auto snapshot=grid_map_->getDynamicPointSnapshot(native_query_time);
        const std::string text=delivery_replay::nativeDynamicText(snapshot),hash=delivery_diag::hash(text);
        const bool overlay=grid_map_->dynamicPointPlanningOverlayEnabled();
        native_dynamic_stable=native_dynamic_stable&&text==native_dynamic_text&&overlay==native_overlay_enabled;
        delivery_replay::emit("\"kind\":\"native_dynamic_observation\",\"position\":"+delivery_replay::quote(position)+
          ",\"query_time_nsec\":"+std::to_string(native_query_time.toNSec())+
          ",\"content_hash\":"+delivery_replay::quote(hash)+",\"count\":"+std::to_string(snapshot.points.size())+
          ",\"overlay_enabled\":"+(overlay?"true":"false")+",\"snapshot\":"+text+
          ",\"matches_entry\":"+(text==native_dynamic_text&&overlay==native_overlay_enabled?"true":"false"));
      };
      const double saved_budget=max_wall_time_;const std::string saved_label=resource_probe::label();
      const double saved_wheel=max_wheel_omega_;
      const auto saved_ini=iniState_container,saved_fin=finState_container,saved_pts=initInnerPts_container;
      const auto saved_times=initT_container;const auto saved_signs=singul_container;
      delivery_replay::phase()="capture";
      std::ostringstream captured;captured<<std::setprecision(17)<<"\"kind\":\"capture\",\"input_hash\":"
        <<delivery_replay::quote(std::to_string(input_hash))<<",\"sim\":"<<frozen_sim
        <<",\"raw_seq\":"<<raw<<",\"esdf_seq\":"<<esdf<<",\"frame_now\":"<<frozen_frame.now
        <<",\"frame\":"<<delivery_diag::frameJson({frozen_frame.snapshot,frozen_frame.now})
        <<",\"prediction\":"<<frozen_prediction<<",\"initial_states\":";
      failure_probe::matrices(captured,saved_ini);captured<<",\"final_states\":";failure_probe::matrices(captured,saved_fin);
      captured<<",\"inner_points\":";failure_probe::matrices(captured,saved_pts);
      captured<<",\"initial_times\":";failure_probe::matrices(captured,saved_times);
      captured<<",\"directions\":[";for(size_t i=0;i<saved_signs.size();++i){if(i)captured<<',';captured<<saved_signs[i];}
      captured<<"],\"native_map_buffer_dumped\":false,\"cross_process_replay_claimed\":false"
        <<",\"native_dynamic_snapshot\":"<<native_dynamic_text
        <<",\"native_dynamic_content_hash\":"<<delivery_replay::quote(native_dynamic_hash)
        <<",\"native_dynamic_count\":"<<native_dynamic.points.size()
        <<",\"native_dynamic_overlay_enabled\":"<<(native_overlay_enabled?"true":"false")
        <<",\"native_dynamic_overlay_owned_freeze\":"<<(native_overlay_owned?"true":"false")
        <<",\"native_dynamic_KDtree_dumped\":false"
        <<",\"native_dynamic_scope\":\"full observed snapshot comparison; empty does not attest internal KD-tree contents\""
        <<",\"refresh_ordinal\":"<<delivery_replay::refreshOrdinal()
        <<",\"exec_weight\":"<<coupling_bridge::settings().execution_weight
        <<",\"hand_weight\":"<<coupling_bridge::settings().delivery_weight
        <<",\"all_passes_paused_before_nominal\":true,\"constraints_relaxed\":false";
      delivery_replay::emit(captured.str());
      observe_native_dynamic("capture");
      delivery_replay::running()=true;delivery_replay::signatures().clear();delivery_replay::auditFailed()=false;
      const double budgets[]={.60,.60,1.50};
      const char* labels[]={"delivery_nominal_060","delivery_repeat_060","delivery_long_150"};
      try{
        for(size_t i=0;i<3;++i){
          max_wall_time_=budgets[i];resource_probe::label()=labels[i];delivery_replay::phase()=labels[i];
          observe_native_dynamic("before_pass");
          std::vector<Eigen::MatrixXd> cp,wp;std::vector<Eigen::VectorXd> times,ee;
          const bool ok=OptimizeTrajectory_lbfgs(saved_ini,saved_fin,saved_pts,saved_times,saved_signs,cp,wp,times,ee);
          observe_native_dynamic("after_pass");
          delivery_replay::emit(std::string("\"kind\":\"recursive_return\",\"success\":")+(ok?"true":"false"));
        }
      }catch(...){max_wall_time_=saved_budget;resource_probe::label()=saved_label;delivery_replay::running()=false;throw;}
      max_wall_time_=saved_budget;resource_probe::label()=saved_label;delivery_replay::running()=false;
      observe_native_dynamic("final");
      const bool stable=std::abs(ros::Time::now().toSec()-frozen_sim)<1e-9&&
        raw==grid_map_->getDynamicObservationSeq()&&esdf==grid_map_->getESDFObservationSeq()&&
        coupling_geometry::activeFrame().snapshot==frozen_frame.snapshot&&
        coupling_geometry::activeFrame().now==frozen_frame.now&&
        delivery_replay::predictionText()==frozen_prediction&&max_wheel_omega_==saved_wheel&&native_dynamic_stable;
      const bool equal=delivery_replay::sameFirst();delivery_replay::phase()="complete";
      delivery_replay::emit(std::string("\"kind\":\"complete\",\"frozen_stable\":")+(stable?"true":"false")+
        ",\"native_dynamic_observed_stable\":"+(native_dynamic_stable?"true":"false")+
        ",\"same_first_fg\":"+(equal?"true":"false")+",\"audits_complete\":"+(delivery_replay::auditFailed()?"false":"true")+
        ",\"diagnostic_candidates_published\":false");
      const char* done=std::getenv("DELIVERY_REPLAY_DONE_PATH");
      if(done){std::ofstream f(done);f<<"{\"schema\":\"delivery_replay_complete_v1\",\"complete\":true,\"frozen_stable\":"
        <<(stable?"true":"false")<<",\"same_first_fg\":"<<(equal?"true":"false")
        <<",\"native_dynamic_observed_stable\":"<<(native_dynamic_stable?"true":"false")
        <<",\"audits_complete\":"<<(delivery_replay::auditFailed()?"false":"true")
        <<",\"diagnostic_candidates_published\":false}\n";}
      return false; // Private captured candidates NEVER escape to the manager.
    }
'''


RECONSTRUCT = r'''
    if(delivery_trial.enabled){
      std::ostringstream head,tail;head<<std::setprecision(17);tail<<std::setprecision(17);
      failure_probe::matrices(head,iniState_container_);failure_probe::matrices(tail,finState_container_);
      delivery_trial.effective_initial=head.str();delivery_trial.effective_final=tail.str();
    }
    delivery_trial.reconstruct=[&](const delivery_replay::Accepted& a)->std::string{
      int at=0;std::vector<Eigen::MatrixXd> points;std::vector<Eigen::VectorXd> times;
      for(int n=0;n<traj_num_;++n){
        points.emplace_back(Eigen::Map<const Eigen::MatrixXd>(a.x.data()+at,traj_dim_,piece_num_container_[n]-1));
        at+=traj_dim_*(piece_num_container_[n]-1);
      }
      for(int n=0;n<traj_num_;++n){
        Eigen::Map<const Eigen::VectorXd> vt(a.x.data()+at,piece_num_container_[n]);
        Eigen::VectorXd rt(piece_num_container_[n]);VirtualT2RealT(vt,rt);times.push_back(rt);at+=piece_num_container_[n];
      }
      std::vector<Eigen::VectorXd> gears;
      for(int n=0;n<traj_num_-1;++n){gears.emplace_back(Eigen::Map<const Eigen::VectorXd>(a.x.data()+at,traj_dim_));at+=traj_dim_;}
      Eigen::Map<const Eigen::VectorXd> angles(a.x.data()+at,traj_num_-1);
      SingulTrajData rebuilt;rebuilt.clearSingulTraj();double elapsed=0.;
      std::ostringstream s;s<<std::setprecision(17)<<"\"reconstructed\":true,\"canonical_parts\":[";
      for(int n=0;n<traj_num_;++n){
        Eigen::MatrixXd ini=iniState_container_[n],fin=finState_container_[n];
        if(n>0){ini.col(0)=gears[n-1];ini.col(1).head(2)=Eigen::Vector2d(-non_singul_v_*cos(angles[n-1]),-non_singul_v_*sin(angles[n-1]));}
        if(n<traj_num_-1){fin.col(0)=gears[n];fin.col(1).head(2)=Eigen::Vector2d(non_singul_v_*cos(angles[n]),non_singul_v_*sin(angles[n]));}
        SnapOpt_container_[n].generate(points[n],times[n],ini,fin);
        rebuilt.addSingulTraj(SnapOpt_container_[n].getTraj(singul_container_[n]),elapsed);
        elapsed=rebuilt.singul_traj.back().end_time;
        if(n)s<<',';s<<"{\"gear\":"<<singul_container_[n]<<",\"times\":"<<delivery_diag::vector(times[n])
          <<",\"b_rows_ascending_power\":"<<delivery_diag::matrix(SnapOpt_container_[n].get_b())<<'}';
      }
      auto* diagnosis_now=failure_probe::active();
      delivery_replay::RestoreFailure restore_native_verdict;
      const auto saved_reason=diagnosis_now->reason;const double saved_value=diagnosis_now->rejection_value,saved_time=diagnosis_now->rejection_time;
      diagnosis_now->reason="ok";diagnosis_now->rejection_value=0.;diagnosis_now->rejection_time=0.;
      const double audit_start=failure_probe::wall();const bool native_ok=IsTrajSafe(rebuilt);
      s<<"],\"native_IsTrajSafe_pass\":"<<(native_ok?"true":"false")
        <<",\"native_reason\":"<<delivery_replay::quote(diagnosis_now->reason)
        <<",\"native_rejection_value\":"<<delivery_replay::num(diagnosis_now->rejection_value)
        <<",\"native_rejection_time\":"<<delivery_replay::num(diagnosis_now->rejection_time)
        <<",\"native_audit_wall_s\":"<<delivery_replay::num(failure_probe::wall()-audit_start)
        <<",\"native_gate_skips_first_0_05_s\":true,\"future_FSM_live_acceptance_certified\":false";
      diagnosis_now->reason=saved_reason;diagnosis_now->rejection_value=saved_value;diagnosis_now->rejection_time=saved_time;
      return s.str();
    };
'''


def transform_optimizer(text):
    t = Transformer(text)
    t.before('#include "state_contract_trace.hpp"', '#include "delivery_replay_probe.hpp"\n')
    t.before('    failure_probe::Frame diagnosis(max_wall_time_,max_wheel_omega_,input_hash,', ENTRY)
    old_start = text.index('    static bool diagnosis_done=false;')
    old_end = text.index('    probe.add("segments",initT_container.size());', old_start)
    t.change(text[old_start:old_end], '''    // Superseded historical post-outcome/wheel-relaxation harness disabled.
    auto diagnose=[](const char*,const SingulTrajData*){return false;};

''')
    recheck_start = text.index('    if(resource_probe::label().find("budget_")==0&&max_wall_time_==3.0){')
    recheck_end = text.index('    if(!good_traj&&(diagnosis.reason=="left_wheel_speed"', recheck_start)
    t.change(text[recheck_start:recheck_end], '    // Legacy relaxed-wheel diagnostic disabled in this build.\n')
    t.after('    state_contract_backend.link(coupled_optimizer.id);', '''
    delivery_replay::Trial delivery_trial(input_hash,max_wall_time_,max_wheel_omega_,
      grid_map_->getDynamicObservationSeq(),grid_map_->getESDFObservationSeq());
''')
    t.before('    domain_lbfgs::lbfgs_parameter_t lbfgs_params;', RECONSTRUCT)
    t.after('    optimization_start_wall_time_ = ros::WallTime::now();', '\n    delivery_trial.begin();\n')
    t.after('    failure_probe::firstCost(costall,g);', '\n    delivery_replay::first(costall,x,g);\n')
    t.after('''    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
''', '''    delivery_replay::progress(x,g,fx,step,k,ls);
''')
    return t.finish()
