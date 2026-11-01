"""One predetermined accepted-iterate exit audit and optional narrow recovery."""
from native_collision_runtime import replace_once


def transform_optimizer(text):
    text=replace_once(text,'#include "candidate_collision_repair.hpp"',
        '#include "candidate_collision_repair.hpp"\n#include "candidate_exit_audit.hpp"')
    anchor='    if(native67_eligible && delivery_replay::requested(max_wall_time_)){'
    text=replace_once(text,anchor,
        '    const bool exit_audit_eligible=native67_eligible && time_indexed_collision::active() &&\n'
        '      coupling_bridge::referenceRevision()>0;\n'
        '    const bool exit_audit_selected=candidate_exit_audit::selected(exit_audit_eligible);\n'
        '    bool exit_audit_completed=false;\n'
        '    delivery_replay::Finally exit_audit_incomplete([&](){\n'
        '      if(exit_audit_selected&&!exit_audit_completed)dual_probe::emit("candidate_exit_audit_incomplete",\n'
        '        dual_probe::Fields().str("reason","selected request returned or threw before complete audit"));\n'
        '    });\n'+anchor)
    text=replace_once(text,'    candidate_collision_repair::Scope collision_repair;',
        '    candidate_exit_audit::Scope exit_audit_record(exit_audit_selected);\n'
        '    candidate_collision_repair::Scope collision_repair;')
    text=replace_once(text,'    Eigen::VectorXd collision_repair_scale;\n    try {',
        '    Eigen::VectorXd collision_repair_scale;\n'+AUDIT_LAMBDA+'\n    try {')
    text=replace_once(text,'      return false; // Initial inadmissibility cannot masquerade as convergence.',
        '      run_exit_audit(domain_lbfgs::LBFGSERR_INVALID_FUNCVAL,"initial_static_map_domain");\n'
        '      return false; // Initial inadmissibility cannot masquerade as convergence.')
    anchor='    if(force_stop_type_ == STOP_FOR_TIMEOUT){\n      diagnosis.reason="budget_timeout";'
    text=replace_once(text,anchor,
        '    run_exit_audit(result,"solver_return");\n'
        '    bool exit_recovered=false;\n'
        '    if(candidate_exit_audit::canRecover(exit_audit_eligible,result,int(force_stop_type_),accepted_candidate.usable())){\n'
        '      if(rebuild_exit_main(accepted_candidate.x)){\n'
        '        x=accepted_candidate.x;salvage_attempted=true;exit_recovered=true;\n'
        '        dual_probe::emit("candidate_exit_recovery_attempt",dual_probe::Fields()\n'
        '          .add("original_lbfgs_result",result).str("accepted_x_hash",std::to_string(delivery_candidate::hash(x)))\n'
        '          .str("scope","actual accepted iterate; ordinary full native and latest gates still decide publication"));\n'
        '      }\n'
        '    }\n'+anchor)
    anchor='    temporal_capture::finalCandidate(singul_traj_data,good_traj,diagnosis.reason);'
    text=replace_once(text,anchor,
        '    if(exit_recovered)dual_probe::emit("candidate_exit_recovery_native_result",dual_probe::Fields()\n'
        '      .add("original_lbfgs_result",result).add("native_pass",good_traj?"true":"false")\n'
        '      .str("native_reason",diagnosis.reason).add("rejection_time",diagnosis.rejection_time));\n'+anchor)
    anchor='    candidate_collision_repair::completed(x,g,costall,total_smoo_cost,total_obs_feas_costs,total_time_cost,extra_geometry,extra_prediction);'
    text=replace_once(text,anchor,anchor+'\n    candidate_exit_audit::completed(x,g,costall);')
    anchor='    if(opt->force_stop_type_==DONT_STOP){delivery_candidate::accepted(x,fx);candidate_collision_repair::accepted(x);}'
    text=replace_once(text,anchor,
        '    if(opt->force_stop_type_==DONT_STOP){delivery_candidate::accepted(x,fx);candidate_collision_repair::accepted(x);candidate_exit_audit::accepted(x,fx);}')
    return text


def transform_repair_header(text):
    text=replace_once(text,'  int target_trajectory=-1,target_piece=-1;',
        '  double environment_factor=factor;\n  int target_trajectory=-1,target_piece=-1;')
    return replace_once(text,'s->target_piece==piece?factor:1.;','s->target_piece==piece?s->environment_factor:1.;')


def transform_map_accessor(text):
    return replace_once(text,'  static std::string writeSnapshot(GridMap& map,const std::string& prefix){',
        '  static unsigned long esdfCrc32(GridMap& map){\n'
        '    const auto& buffer=map.md_.distance_buffer_all_;\n'
        '    const auto* p=reinterpret_cast<const Bytef*>(buffer.data());\n'
        '    std::size_t bytes=buffer.size()*sizeof(double);uLong checksum=crc32(0L,Z_NULL,0);\n'
        '    while(bytes){const uInt n=static_cast<uInt>(std::min<std::size_t>(bytes,1U<<20));\n'
        '      checksum=crc32(checksum,p,n);p+=n;bytes-=n;}return checksum;\n'
        '  }\n\n  static std::string writeSnapshot(GridMap& map,const std::string& prefix){')


AUDIT_LAMBDA=r'''    auto rebuild_exit_main=[&](const Eigen::VectorXd& point){
      std::vector<delivery_candidate::Part> parts;
      if(!delivery_candidate::decode(point,piece_num_container_,traj_dim_,iniState_container_,
           finState_container_,non_singul_v_,parts))return false;
      for(int i=0;i<traj_num_;++i){
        SnapOpt_container_[i].generate(parts[i].points,parts[i].times,parts[i].initial,parts[i].final);
        cps_container_[i].points=SnapOpt_container_[i].getInitConstrainPoints(cps_num_prePiece_);
        if(!SnapOpt_container_[i].get_b().allFinite()||!cps_container_[i].points.allFinite())return false;
      }return true;
    };
    auto run_exit_audit=[&](int original_result,const std::string& exit_stage){
      if(!exit_audit_selected)return;
      bool sil=false,sim=false;ros::param::param("/online_validation/enabled",sil,false);ros::param::param("/use_sim_time",sim,false);
      if(!sil||!sim)throw std::runtime_error("candidate exit audit requires isolated SIL");
      const double original_elapsed=(ros::WallTime::now()-optimization_start_wall_time_).toSec();
      const double original_remaining=std::max(0.,max_wall_time_-original_elapsed);
      const int original_force=int(force_stop_type_);const auto original_x=x;
      const bool actual_available=accepted_candidate.usable();
      const Eigen::VectorXd actual_x=actual_available?accepted_candidate.x:Eigen::VectorXd();
      const auto actual_hash=actual_available?delivery_candidate::hash(actual_x):0ULL;
      const auto snapshot_record=exit_audit_record.accepted;
      const auto last_evaluation=exit_audit_record.evaluated;
      const auto original_diagnostics=candidate_collision_repair::diagnostics(collision_repair,actual_x);
      const auto original_raw=grid_map_->getDynamicObservationSeq(),original_esdf=grid_map_->getESDFObservationSeq();
      const auto* frozen_context=time_indexed_collision::current();
      const auto frozen_cv=frozen_context->source.snapshot;
      const auto frozen_frame=coupling_geometry::activeFrame();
      const std::string frozen_prediction=delivery_replay::predictionText();
      unsigned long long boundary_hash=1469598103934665603ULL;
      resource_probe::hashMatrices(boundary_hash,iniState_container_);resource_probe::hashMatrices(boundary_hash,finState_container_);
      dual_probe::emit("candidate_exit_audit_begin",dual_probe::Fields()
        .add("original_lbfgs_result",original_result).add("original_force_stop",original_force).str("exit_stage",exit_stage)
        .add("accepted_available",actual_available?"true":"false").add("accepted_steps",accepted_candidate.accepted_steps)
        .add("accepted_initial",accepted_candidate.initial?"true":"false")
        .str("accepted_x_hash",std::to_string(actual_hash)).str("solver_return_x_hash",std::to_string(delivery_candidate::hash(original_x)))
        .str("last_evaluation_x_hash",last_evaluation.valid?std::to_string(last_evaluation.x_hash):"")
        .add("original_elapsed_s",original_elapsed).add("original_remaining_budget_s",original_remaining)
        .add("shared_budget_s",max_wall_time_).str("scope","one selected request; pause/audit excluded from online timing"));
      const double diagnostic_begin=dual_probe::mono();
      {
        failure_probe::PauseOwned paused;
        const bool pin_owned=GridMapParallelTestAccessor::freezeCurrentOverlay(*grid_map_);
        delivery_replay::Finally release_pin([&](){if(pin_owned)GridMapParallelTestAccessor::releaseCurrentOverlay(*grid_map_);});
        delivery_replay::RestoreFailure restore_verdict;
        const double frozen_now=ros::Time::now().toSec();
        const auto esdf_hash=GridMapParallelTestAccessor::esdfCrc32(*grid_map_);
        const auto overlay_key=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
        candidate_exit_audit::Verdict verdict;SingulTrajData audited;
        std::unique_ptr<early_delivery::Checker> checker;
        if(actual_available){
          const auto began=ros::WallTime::now();
          checker.reset(new early_delivery::Checker(piece_num_container_,singul_container_,iniState_container_,finState_container_,
            non_singul_v_,mobile_base_wheel_base_,mobile_base_wheel_radius_,max_wheel_omega_,max_wheel_alpha_));
          audited=checker->trajectory(actual_x);verdict.available=true;
          verdict.reconstruct_ms=1000.*(ros::WallTime::now()-began).toSec();
          diagnosis.reason="ok";diagnosis.rejection_value=0.;diagnosis.rejection_time=0.;
          const auto gate_start=ros::WallTime::now();
          {handoff_alignment::FinalGate final_gate;verdict.native_pass=IsTrajSafe(audited);}
          verdict.native_ms=1000.*(ros::WallTime::now()-gate_start).toSec();
          verdict.reason=diagnosis.reason;verdict.value=diagnosis.rejection_value;verdict.time=diagnosis.rejection_time;
          candidate_collision_repair::candidate("exit_accepted_candidate",audited,verdict.native_pass,verdict.reason);
        }else verdict.reason="no_usable_actual_accepted_iterate";
        dual_probe::emit("candidate_exit_audit_result",dual_probe::Fields()
          .add("original_lbfgs_result",original_result).add("original_force_stop",original_force)
          .add("accepted_available",actual_available?"true":"false").str("accepted_x_hash",std::to_string(actual_hash))
          .add("snapshot_matches_actual_accepted",snapshot_record.valid&&snapshot_record.x_hash==actual_hash?"true":"false")
          .add("native_pass",verdict.native_pass?"true":"false").str("native_reason",verdict.reason)
          .add("rejection_value",verdict.value).add("rejection_time",verdict.time)
          .add("reconstruction_ms",verdict.reconstruct_ms).add("native_gate_ms",verdict.native_ms)
          .add("original_remaining_budget_s",original_remaining).str("esdf_crc32",std::to_string(esdf_hash))
          .add("frozen_cv_sequence",frozen_cv.sequence).add("frozen_cv_stamp",frozen_cv.stamp.toSec())
          .add("frozen_requested_epoch",frozen_context->requested_epoch)
          .str("effective_boundary_hash",std::to_string(boundary_hash))
          .str("original_objective_diagnostics",original_diagnostics.text));
        const bool continuation_needed=actual_available&&!verdict.native_pass&&verdict.reason=="planning_collision"&&
          verdict.value>=0.&&verdict.value<=1.&&original_remaining>0.;
        std::string continuation_rows="[";
        if(continuation_needed){
          // Each branch starts from the identical accepted x and the original
          // remaining allowance. They are offline diagnostics, never published.
          decltype(SnapOpt_container_) original_snap;original_snap.swap(SnapOpt_container_);
          const auto original_cps=cps_container_;const auto saved_force=force_stop_type_;
          const auto saved_start=optimization_start_wall_time_;const auto saved_budget=max_wall_time_;const int saved_iter=iter_num_;
          const auto saved_domain=domain_step::counters();
          delivery_replay::Finally restore_solver([&](){
            SnapOpt_container_.swap(original_snap);cps_container_=original_cps;force_stop_type_=saved_force;
            optimization_start_wall_time_=saved_start;max_wall_time_=saved_budget;iter_num_=saved_iter;domain_step::counters()=saved_domain;
          });
          std::vector<delivery_candidate::Part> original_parts;
          if(!delivery_candidate::decode(actual_x,piece_num_container_,traj_dim_,iniState_container_,finState_container_,non_singul_v_,original_parts))
            throw std::runtime_error("cannot decode exit continuation start");
          std::vector<Eigen::VectorXd> times;for(const auto& p:original_parts)times.push_back(p.times);
          int ordinal=0;
          for(double factor:{1.,10.}){
            candidate_exit_audit::Scope branch_record(true);
            candidate_collision_repair::Scope branch_objective;branch_objective.environment_factor=factor;
            branch_objective.target(times,verdict.time);branch_objective.repairing=true;
            delivery_candidate::Scope branch_accepted;
            SnapOpt_container_.clear();SnapOpt_container_.resize(piece_num_container_.size());
            cps_container_=original_cps;
            for(std::size_t n=0;n<piece_num_container_.size();++n)SnapOpt_container_[n].reset(piece_num_container_[n]);
            Eigen::VectorXd branch_x=actual_x;rebuild_exit_main(branch_x);
            force_stop_type_=DONT_STOP;iter_num_=0;max_wall_time_=original_remaining;
            optimization_start_wall_time_=ros::WallTime::now();domain_step::reset();
            early_delivery::Session branch_early(early_delivery_enabled,checker.get(),
              [this](){return (ros::WallTime::now()-optimization_start_wall_time_).toSec();},max_wall_time_,
              [this](const SingulTrajData& candidate){
                delivery_replay::RestoreFailure restore;early_delivery::NativeVerdict v;
                v.pass=IsTrajSafe(candidate);const auto* f=failure_probe::active();v.reason=v.pass?"ok":(f?f->reason:"missing");return v;
              });
            int branch_result=domain_lbfgs::LBFGSERR_UNKNOWNERROR;double branch_cost=0.;bool domain_rejected=false;
            try{
              if(collision_repair_scale.size()==branch_x.size()){
                cheap_online_callbacks::Context callback{this,PolyTrajOptimizer::costFunctionCallback,
                  PolyTrajOptimizer::earlyExitCallback,branch_x,collision_repair_scale,{}};
                Eigen::VectorXd z=Eigen::VectorXd::Zero(branch_x.size());
                branch_result=domain_lbfgs::lbfgs_optimize(z,branch_cost,cheap_online_callbacks::Context::evaluate,NULL,
                  cheap_online_callbacks::Context::progress,&callback,lbfgs_params);branch_x=callback.physical(z);
              }else branch_result=domain_lbfgs::lbfgs_optimize(branch_x,branch_cost,PolyTrajOptimizer::costFunctionCallback,NULL,
                PolyTrajOptimizer::earlyExitCallback,this,lbfgs_params);
            }catch(const domain_step::RejectedTrial&){domain_rejected=true;}
            const double solve_wall=(ros::WallTime::now()-optimization_start_wall_time_).toSec();
            const int branch_force=int(force_stop_type_);const int branch_iterations=iter_num_;
            const bool usable=branch_accepted.usable();
            if(branch_early.selected)branch_x=branch_early.selected_x;else if(usable)branch_x=branch_accepted.x;
            candidate_exit_audit::Verdict after;SingulTrajData branch_traj;
            if(usable&&!domain_rejected){
              const auto began=ros::WallTime::now();branch_traj=checker->trajectory(branch_x);after.available=true;
              after.reconstruct_ms=1000.*(ros::WallTime::now()-began).toSec();
              diagnosis.reason="ok";diagnosis.rejection_time=0.;diagnosis.rejection_value=0.;
              const auto gate_started=ros::WallTime::now();
              {handoff_alignment::FinalGate final_gate;after.native_pass=IsTrajSafe(branch_traj);}
              after.native_ms=1000.*(ros::WallTime::now()-gate_started).toSec();
              after.reason=diagnosis.reason;after.time=diagnosis.rejection_time;after.value=diagnosis.rejection_value;
              candidate_collision_repair::candidate(factor==1.?"exit_continuation_weight1":"exit_continuation_weight10",branch_traj,after.native_pass,after.reason);
            }else after.reason=domain_rejected?"initial_static_map_domain":"no_usable_accepted_iterate";
            auto fields=candidate_collision_repair::diagnostics(branch_objective,branch_x);
            fields.add("factor",factor).add("remaining_allowance_s",original_remaining).add("solver_wall_s",solve_wall)
              .add("lbfgs_result",branch_result).add("force_stop",branch_force).add("iterations",branch_iterations)
              .add("candidate_available",after.available?"true":"false").add("native_pass",after.native_pass?"true":"false")
              .str("native_reason",after.reason).add("rejection_time",after.time).add("rejection_value",after.value)
              .str("initial_x_hash",std::to_string(actual_hash)).str("final_x_hash",std::to_string(delivery_candidate::hash(branch_x)))
              .add("target_trajectory",branch_objective.target_trajectory).add("target_piece",branch_objective.target_piece)
              .add("initial_evaluation_available",branch_record.initial.valid?"true":"false")
              .add("initial_cost",branch_record.initial.cost)
              .str("initial_gradient_hash",branch_record.initial.valid?std::to_string(branch_record.initial.gradient_hash):"")
              .add("initial_x_matches",branch_record.initial.valid&&branch_record.initial.x_hash==actual_hash?"true":"false")
              .add("weight1_original_cost_matches",factor==1.&&branch_record.initial.valid&&snapshot_record.valid&&branch_record.initial.cost==snapshot_record.cost?"true":"false")
              .add("weight1_original_gradient_matches",factor==1.&&branch_record.initial.valid&&snapshot_record.valid&&branch_record.initial.gradient_hash==snapshot_record.gradient_hash?"true":"false")
              .add("published","false").str("timing_scope","paused diagnostic, same original remaining allowance; not online performance");
            dual_probe::emit("candidate_exit_continuation",fields);
            if(ordinal++)continuation_rows+=',';
            continuation_rows+="{"+fields.text.substr(1)+",\"verdict\":"+candidate_exit_audit::verdictJson(after)+
              ",\"final_x\":"+delivery_diag::vector(branch_x)+"}";
          }
        }
        continuation_rows+=']';
        const double map_dump_begin=dual_probe::mono();
        const std::string map_metadata=GridMapParallelTestAccessor::writeSnapshot(*grid_map_,candidate_exit_audit::directory()+"/map");
        const double map_dump_ms=1000.*(dual_probe::mono()-map_dump_begin);
        const auto final_esdf_hash=GridMapParallelTestAccessor::esdfCrc32(*grid_map_);
        const bool stable=original_raw==grid_map_->getDynamicObservationSeq()&&original_esdf==grid_map_->getESDFObservationSeq()&&
          esdf_hash==final_esdf_hash&&overlay_key==GridMapParallelTestAccessor::readOverlayKey(*grid_map_)&&
          ros::Time::now().toSec()==frozen_now&&time_indexed_collision::current()==frozen_context&&
          frozen_context->source.snapshot.sequence==frozen_cv.sequence&&frozen_context->source.snapshot.stamp==frozen_cv.stamp&&
          frozen_frame.snapshot==coupling_geometry::activeFrame().snapshot&&frozen_frame.now==coupling_geometry::activeFrame().now&&
          frozen_prediction==delivery_replay::predictionText();
        std::ostringstream capture;capture<<std::setprecision(17)<<"{\"schema\":\"candidate_exit_audit_v1\",\"request_id\":"<<dual_probe::request()
          <<",\"original_lbfgs_result\":"<<original_result<<",\"original_force_stop\":"<<original_force
          <<",\"exit_stage\":"<<delivery_diag::quote(exit_stage)<<",\"original_elapsed_s\":"<<original_elapsed
          <<",\"original_remaining_budget_s\":"<<original_remaining<<",\"shared_budget_s\":"<<max_wall_time_
          <<",\"accepted_available\":"<<(actual_available?"true":"false")<<",\"accepted_x_hash\":"<<delivery_diag::quote(std::to_string(actual_hash))
          <<",\"solver_return_x_hash\":"<<delivery_diag::quote(std::to_string(delivery_candidate::hash(original_x)))
          <<",\"accepted_evaluation\":"<<candidate_exit_audit::evaluationJson(snapshot_record)
          <<",\"last_evaluation\":"<<candidate_exit_audit::evaluationJson(last_evaluation)
          <<",\"verdict\":"<<candidate_exit_audit::verdictJson(verdict)<<",\"effective_initial_states\":"<<state_contract_trace::matricesJson(iniState_container_)
          <<",\"effective_final_states\":"<<state_contract_trace::matricesJson(finState_container_)
          <<",\"source_snapshot\":"<<delivery_replay::nativeDynamicText(frozen_cv)
          <<",\"time_indexed_parameters\":{\"max_prediction\":"<<frozen_context->source.parameters.max_prediction
          <<",\"uncertainty0\":"<<frozen_context->source.parameters.uncertainty0
          <<",\"uncertainty_v\":"<<frozen_context->source.parameters.uncertainty_v
          <<",\"uncertainty_a\":"<<frozen_context->source.parameters.uncertainty_a
          <<",\"influence\":"<<frozen_context->source.parameters.influence
          <<",\"edge_radius\":"<<frozen_context->edge_radius<<",\"cap\":"<<frozen_context->source.parameters.cap
          <<",\"full_coverage\":"<<(frozen_context->full_coverage?"true":"false")<<"}"
          <<",\"requested_epoch\":"<<frozen_context->requested_epoch<<",\"cv_begin_epoch\":"<<frozen_context->begin_epoch
          <<",\"cv_end_epoch\":"<<frozen_context->end_epoch<<",\"frozen_prediction\":"<<frozen_prediction
          <<",\"esdf_crc32\":"<<delivery_diag::quote(std::to_string(esdf_hash))<<",\"esdf_crc32_after\":"<<delivery_diag::quote(std::to_string(final_esdf_hash))
          <<",\"raw_seq\":"<<original_raw<<",\"esdf_seq\":"<<original_esdf<<",\"stable\":"<<(stable?"true":"false")
          <<",\"continuation_triggered\":"<<(continuation_needed?"true":"false")<<",\"continuations\":"<<continuation_rows
          <<",\"map\":"<<map_metadata<<",\"map_dump_ms\":"<<map_dump_ms
          <<",\"esdf_buffer_dumped\":true,\"cross_process_replay_available\":false,\"diagnostic_candidates_published\":false"
          <<",\"scope\":\"same live native objects while paused; entire run timing after audit is diagnostic-contaminated\"}";
        candidate_exit_audit::append("capture.jsonl",capture.str());
        dual_probe::emit("candidate_exit_audit_frozen_end",dual_probe::Fields().add("stable",stable?"true":"false")
          .add("continuation_triggered",continuation_needed?"true":"false").str("esdf_crc32",std::to_string(esdf_hash))
          .str("esdf_crc32_after",std::to_string(final_esdf_hash)).add("native_pass",verdict.native_pass?"true":"false")
          .str("native_reason",verdict.reason).add("requested_epoch",frozen_context->requested_epoch));
        if(!stable)throw std::runtime_error("candidate exit frozen context changed; audit unusable");
      }
      const double diagnostic_end=dual_probe::mono();
      dual_probe::emit("candidate_exit_audit_end",dual_probe::Fields()
        .add("diagnostic_begin_mono",diagnostic_begin).add("diagnostic_end_mono",diagnostic_end)
        .add("diagnostic_wall_ms",1000.*(diagnostic_end-diagnostic_begin))
        .add("original_elapsed_s",original_elapsed).add("original_remaining_budget_s",original_remaining)
        .str("scope","includes process pause, checks, two conditional continuations and capture; exclude from online performance"));
      probe.add("exit_audit_diagnostic_ms",1000.*(diagnostic_end-diagnostic_begin));
      diagnosis.add("exit_audit_diagnostic_ms",1000.*(diagnostic_end-diagnostic_begin));
      exit_audit_completed=true;
    };'''
