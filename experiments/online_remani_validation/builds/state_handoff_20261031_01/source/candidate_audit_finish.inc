    // Included inside OptimizeTrajectory_lbfgs, after its unchanged native gate.
    // The selected failed trajectory cannot escape to the publishing manager.
    static bool candidate_audit_used=false;
    const char* audit_target_env=std::getenv("CANDIDATE_AUDIT_TARGET");
    const std::string audit_target=audit_target_env?audit_target_env:"";
    const bool audit_is_wheel=diagnosis.reason=="left_wheel_speed" ||
      diagnosis.reason=="right_wheel_speed" || diagnosis.reason=="left_wheel_acceleration" ||
      diagnosis.reason=="right_wheel_acceleration";
    const bool audit_selected=candidate_record.enabled && !wheel_cause::running() && !candidate_audit_used && !good_traj &&
      ((audit_target=="collision" && diagnosis.reason=="planning_collision") ||
       (audit_target=="wheel" && audit_is_wheel));
    if(audit_selected){
      candidate_audit_used=true;
      const std::string actual_reason=diagnosis.reason;
      const double actual_rejection_time=diagnosis.rejection_time;
      const double actual_solver_wall=candidate_solver_wall;
      const double actual_sim=ros::Time::now().toSec();
      const auto actual_raw=grid_map_->getDynamicObservationSeq();
      const auto actual_esdf=grid_map_->getESDFObservationSeq();
      const char* dir_env=std::getenv("CANDIDATE_AUDIT_DIR");
      const char* done_env=std::getenv("CANDIDATE_AUDIT_DONE_PATH");
      if(!dir_env || !done_env)throw std::runtime_error("candidate audit output missing");
      const std::string audit_dir=dir_env;
      if(mkdir(audit_dir.c_str(),0775)!=0 && errno!=EEXIST)
        throw std::runtime_error("cannot create candidate audit output");
      failure_probe::PauseOwned audit_pause;
      const std::string audit_overlay_before_pin=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
      const bool audit_overlay_owned=GridMapParallelTestAccessor::freezeCurrentOverlay(*grid_map_);
      delivery_replay::Finally audit_overlay_release([&](){
        if(audit_overlay_owned)GridMapParallelTestAccessor::releaseCurrentOverlay(*grid_map_);
      });
      const std::string audit_overlay_key=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);

      delivery_replay::RestoreFailure audit_restore_verdict;
      const double audit_sim=ros::Time::now().toSec();
      const auto audit_raw=grid_map_->getDynamicObservationSeq();
      const auto audit_esdf=grid_map_->getESDFObservationSeq();
      const auto audit_ini=iniState_container_,audit_fin=finState_container_;

      // MinSnapOpt/BandedSystem owns raw storage: preserve vector ownership,
      // not a shallow element copy. Create independent post-solver audit caches.
      decltype(SnapOpt_container_) audit_snap_saved;
      const auto audit_cps_saved=cps_container_;
      delivery_replay::Finally audit_restore_caches([&](){
        SnapOpt_container_.swap(audit_snap_saved);cps_container_=audit_cps_saved;
      });
      // Register restoration before transferring the original owning buffer.
      audit_snap_saved.swap(SnapOpt_container_);
      SnapOpt_container_.resize(piece_num_container_.size());
      for(std::size_t n=0;n<piece_num_container_.size();++n)
        SnapOpt_container_[n].reset(piece_num_container_[n]);

      const std::string map_metadata=GridMapParallelTestAccessor::writeSnapshot(*grid_map_,audit_dir+"/map");
      candidate_audit::Params parameters;
      parameters.wheel_base=mobile_base_wheel_base_;parameters.wheel_radius=mobile_base_wheel_radius_;
      parameters.wheel_limit=max_wheel_omega_;parameters.alpha_limit=max_wheel_alpha_;
      parameters.arm_radius=manipulator_thickness_;parameters.ground=ground_safe_dis_;
      parameters.base_safe_margin=safe_margin_;parameters.arm_safe_margin=safe_margin_mani_;
      parameters.joint_vel_limit=max_joint_vel_;parameters.joint_acc_limit=max_joint_acc_;
      std::ofstream audit_out(audit_dir+"/case.json");
      if(!audit_out)throw std::runtime_error("cannot create candidate case");
      audit_out<<std::setprecision(17)<<"{\"schema\":\"candidate_audit_v1\",\"target\":"
        <<candidate_audit::quote(audit_target)<<",\"request_id\":"<<dual_probe::request()
        <<",\"input_hash\":"<<candidate_audit::quote(std::to_string(input_hash))
        <<",\"reason\":"<<candidate_audit::quote(actual_reason)<<",\"rejection_time\":"<<actual_rejection_time
        <<",\"budget_s\":"<<max_wall_time_<<",\"solver_wall_ms\":"<<actual_solver_wall*1000.
        <<",\"raw_seq\":"<<actual_raw<<",\"esdf_seq\":"<<actual_esdf<<",\"sim\":"<<actual_sim
        <<",\"sim_after_pause\":"<<audit_sim<<",\"map_metadata\":\"map.json\",\"map\":"<<map_metadata
        <<",\"dynamic_map_history\":{\"solver_entry\":"<<candidate_record.solver_entry_key
        <<",\"solver_exit\":"<<candidate_record.solver_exit_key
        <<",\"native_gate_begin\":"<<candidate_native_gate_begin
        <<",\"native_gate_end\":"<<candidate_native_gate_end
        <<",\"before_audit_pin\":"<<audit_overlay_before_pin<<",\"audit_begin\":"<<audit_overlay_key
        <<",\"solver_freeze_requested\":"<<(candidate_solver_freeze_requested?"true":"false")
        <<",\"solver_pin_owned\":"<<(candidate_solver_overlay_owned?"true":"false")
        <<",\"audit_pin_owned\":"<<(audit_overlay_owned?"true":"false")
        <<",\"evaluate_complete_count\":"<<candidate_record.evaluate_complete_count
        <<",\"evaluate_overlay_changed_count\":"<<candidate_record.evaluate_overlay_changed_count
        <<",\"solver_entry_exit_key_equal\":"<<(candidate_record.solver_entry_key==candidate_record.solver_exit_key?"true":"false")
        <<",\"scope\":\"each recorded candidate is rechecked against the one current post-pause pinned overlay; this is not its original-time map, and initial collision here does not prove initial-time collision; ordinary online objectives may change with the independent dynamic worker\"}"

        <<",\"params\":{\"wheel_base\":"<<parameters.wheel_base<<",\"wheel_radius\":"<<parameters.wheel_radius
        <<",\"wheel_limit\":"<<parameters.wheel_limit<<",\"alpha_limit\":"<<parameters.alpha_limit
        <<",\"feas_tol\":"<<parameters.feas_tol<<",\"arm_radius\":"<<parameters.arm_radius
        <<",\"ground\":"<<parameters.ground<<",\"base_safe_margin\":"<<parameters.base_safe_margin
        <<",\"arm_safe_margin\":"<<parameters.arm_safe_margin<<",\"constrain_points_per_piece\":"<<cps_num_prePiece_
        <<",\"dense_resolution\":"<<dense_sample_resolution_<<",\"map_resolution\":"<<map_resolution_
        <<",\"joint_vel_limit\":"<<max_joint_vel_<<",\"joint_acc_limit\":"<<max_joint_acc_
        <<"},\"initial_states\":";
      failure_probe::matrices(audit_out,iniState_container);
      audit_out<<",\"effective_initial_states\":";failure_probe::matrices(audit_out,audit_ini);
      audit_out<<",\"final_states\":";failure_probe::matrices(audit_out,finState_container);
      audit_out<<",\"effective_final_states\":";failure_probe::matrices(audit_out,audit_fin);
      audit_out<<",\"inner_points\":";failure_probe::matrices(audit_out,initInnerPts_container);
      audit_out<<",\"initial_times\":";failure_probe::matrices(audit_out,initT_container);
      audit_out<<",\"directions\":[";
      for(std::size_t n=0;n<singul_container.size();++n){if(n)audit_out<<',';audit_out<<singul_container[n];}
      audit_out<<"],\"stages\":[";
      bool final_matches_saved=false;
      for(std::size_t index=0;index<candidate_record.steps.size();++index){
        if(index!=0 && index+1!=candidate_record.steps.size())continue; // bounded initial/final audit only.
        const auto& step=candidate_record.steps[index];
        std::vector<delivery_candidate::Part> parts;
        if(!delivery_candidate::decode(step.x,piece_num_container_,traj_dim_,audit_ini,audit_fin,non_singul_v_,parts))
          throw std::runtime_error("cannot decode captured candidate");
        SingulTrajData rebuilt;rebuilt.clearSingulTraj();double prefix=0.;
        std::vector<double> collision_grid,feasibility_grid;
        std::ostringstream part_json;part_json<<std::setprecision(17)<<'[';
        for(int n=0;n<traj_num_;++n){
          SnapOpt_container_[n].generate(parts[n].points,parts[n].times,parts[n].initial,parts[n].final);
          rebuilt.addSingulTraj(SnapOpt_container_[n].getTraj(singul_container[n]),prefix);
          if(n)part_json<<',';
          part_json<<"{\"gear\":"<<singul_container[n]<<",\"times\":"<<delivery_diag::vector(parts[n].times)
            <<",\"b_rows_ascending_power\":"<<delivery_diag::matrix(SnapOpt_container_[n].get_b())<<'}';
          for(int piece=0;piece<parts[n].times.size();++piece){
            const double duration=parts[n].times[piece],dt=duration/cps_num_prePiece_;
            const auto coefficients=SnapOpt_container_[n].get_b().block(piece*8,0,8,traj_dim_).eval();
            double local=0.;
            for(int j=0;j<=cps_num_prePiece_;++j,local+=dt){
              collision_grid.push_back(prefix+local);feasibility_grid.push_back(prefix+local);

              const double s1=local,s2=s1*s1,s3=s2*s1,s4=s2*s2,s5=s4*s1,s6=s3*s3;
              Eigen::Matrix<double,8,1> beta;
              beta<<0.,1.,2.*s1,3.*s2,4.*s3,5.*s4,6.*s5,7.*s6;
              const Eigen::VectorXd velocity=coefficients.transpose()*beta;
              double factor=0.,df=0.;
              if(smoothedLog(.1*.1-velocity.head(2).squaredNorm(),.001,factor,df)){
                double dense_local=local;
                const int quantity=(j==0 || j==cps_num_prePiece_)?dense_sample_resolution_/2:dense_sample_resolution_;
                if(j!=0)dense_local-=dt*.5;
                for(int l=0;l<=quantity;++l,dense_local+=dt/dense_sample_resolution_)
                  feasibility_grid.push_back(prefix+dense_local);
              }
            }
            prefix+=duration;
          }
        }
        part_json<<']';
        const bool detail=index==0 || index==candidate_record.steps.size()/2 || index+1==candidate_record.steps.size();
        diagnosis.reason="ok";diagnosis.rejection_value=0.;diagnosis.rejection_time=0.;
        const bool native_ok=IsTrajSafe(rebuilt);
        const std::string native_reason=diagnosis.reason;
        const double native_value=diagnosis.rejection_value,native_time=diagnosis.rejection_time;
        const std::string audit=candidate_audit::audit(rebuilt,*mm_config_,*grid_map_,parameters,
          collision_grid,detail,feasibility_grid);
        if(index)audit_out<<',';
        audit_out<<"{\"label\":"<<candidate_audit::quote(step.label)<<",\"index\":"<<index
          <<",\"iteration\":"<<step.iteration<<",\"line_search\":"<<step.line_search<<",\"cost\":"<<step.cost
          <<",\"elapsed_s\":"<<step.solver_elapsed_s<<",\"x\":"<<delivery_diag::vector(step.x)
          <<",\"cost_components\":"<<native67_probe::costJson(step.costs)
          <<",\"cost_components_recorded\":"<<(step.cost_seen?"true":"false")
          <<",\"evaluation_begin_key\":"<<step.evaluation_begin_key
          <<",\"evaluation_end_key\":"<<step.evaluation_end_key
          <<",\"parts\":"<<part_json.str()<<",\"native_IsTrajSafe_pass\":"<<(native_ok?"true":"false")
          <<",\"native_reason\":"<<candidate_audit::quote(native_reason)<<",\"native_rejection_value\":"<<native_value
          <<",\"native_reject_time\":"<<native_time<<",\"audit\":"<<audit<<'}';
        if(index+1==candidate_record.steps.size()){
          final_matches_saved=accepted_candidate.usable() && step.x.size()==accepted_candidate.x.size() &&
            (step.x.array()==accepted_candidate.x.array()).all();
          final_matches_saved=final_matches_saved && native_ok==good_traj && native_reason==actual_reason;
        }
      }

      const std::string audit_overlay_end=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
      const bool audit_overlay_stable=audit_overlay_key==audit_overlay_end;
      const bool frozen_stable=audit_raw==grid_map_->getDynamicObservationSeq() &&
        audit_esdf==grid_map_->getESDFObservationSeq() && ros::Time::now().toSec()==audit_sim;
      const bool solver_map_stable=candidate_entry_raw==actual_raw && candidate_entry_esdf==actual_esdf &&
        actual_raw==audit_raw && actual_esdf==audit_esdf;
      audit_out<<"],\"frozen_checks\":{\"sim_frozen\":"<<(ros::Time::now().toSec()==audit_sim?"true":"false")
        <<",\"raw_esdf_stable\":"<<(frozen_stable?"true":"false")
        <<",\"solver_map_generations_stable\":"<<(solver_map_stable?"true":"false")
        <<",\"overlay_key_stable\":"<<(audit_overlay_stable?"true":"false")
        <<",\"audit_overlay_begin\":"<<audit_overlay_key<<",\"audit_overlay_end\":"<<audit_overlay_end
        <<",\"final_matches_saved_and_actual_verdict\":"<<(final_matches_saved?"true":"false")
        <<"},\"optimizer_sample_scope\":\"collision grid is K-point per-piece base quadrature; feasibility grid also includes the original low-speed dense rule; exact gear-boundary ownership follows native trajectory container\""
        <<",\"diagnostic_candidates_published\":false,\"navigation_metrics_valid\":false}\n";
      audit_out.close();if(!audit_out)throw std::runtime_error("candidate case write failed");
#include "endpoint_time_replay.inc"
      std::ofstream done(done_env);
      done<<"{\"complete\":true,\"target\":"<<candidate_audit::quote(audit_target)
        <<",\"request_id\":"<<dual_probe::request()<<",\"input_hash\":"<<candidate_audit::quote(std::to_string(input_hash))
        <<",\"reason\":"<<candidate_audit::quote(actual_reason)<<",\"candidate_file\":\"candidate_audit/case.json\""
        <<",\"frozen_stable\":"<<(frozen_stable&&solver_map_stable&&audit_overlay_stable?"true":"false")
        <<",\"audits_complete\":"<<(final_matches_saved?"true":"false")
        <<",\"diagnostic_candidates_published\":false}\n";
      done.close();if(!done)throw std::runtime_error("candidate completion write failed");
    }
