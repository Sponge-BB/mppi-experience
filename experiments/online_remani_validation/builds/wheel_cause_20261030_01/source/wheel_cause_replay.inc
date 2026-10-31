      // Still inside the original PauseOwned scope, before its pin is released.
      if(wheel_cause::requested() && audit_target=="wheel"){
        bool wheel_sil=false,wheel_sim=false;
        ros::param::param("/online_validation/enabled",wheel_sil,false);
        ros::param::param("/use_sim_time",wheel_sim,false);
        if(!wheel_sil||!wheel_sim||!candidate_solver_freeze_requested||!candidate_solver_overlay_owned||
           candidate_record.solver_entry_key!=candidate_record.solver_exit_key||
           candidate_record.solver_entry_key!=candidate_native_gate_begin||
           candidate_native_gate_begin!=candidate_native_gate_end||!audit_overlay_stable||
           !solver_map_stable||!final_matches_saved||delivery_replay::running())
          throw std::runtime_error("wheel-cause replay requires a proven original solver/native fixed map and isolated SIL");
        if(!mm_config_->hasCustomBaseSpheres())throw std::runtime_error("wheel-cause replay requires unchanged custom spheres");
        std::vector<Eigen::Vector3d> wheel_base_points;mm_config_->getCarPts(Eigen::Vector3d::Zero(),wheel_base_points);
        if(wheel_base_points.size()!=47)throw std::runtime_error("wheel-cause replay requires unchanged 47 spheres");
        const std::string wheel_log=audit_dir+"/wheel_cause_replay.jsonl";
        const auto wheel_saved_ini=iniState_container,wheel_saved_fin=finState_container;
        const auto wheel_saved_points=initInnerPts_container;
        const auto wheel_saved_times=initT_container;const auto wheel_saved_signs=singul_container;
        const auto wheel_saved_snap=SnapOpt_container_;const auto wheel_saved_cps=cps_container_;
        const auto wheel_saved_piece_counts=piece_num_container_;const auto wheel_saved_internal_signs=singul_container_;
        const auto wheel_saved_internal_ini=iniState_container_,wheel_saved_internal_fin=finState_container_;
        const auto wheel_saved_gear_vel=gearVel_container_;
        const auto wheel_saved_hold_reference=mani_hold_reference_;
        const bool wheel_saved_hold_enabled=mani_hold_enabled_;
        const int wheel_saved_traj_num=traj_num_,wheel_saved_variable_num=variable_num_,wheel_saved_iter=iter_num_;
        const auto wheel_saved_force_stop=force_stop_type_;const auto wheel_saved_start=optimization_start_wall_time_;
        const double wheel_saved_budget=max_wall_time_,wheel_saved_limit=max_wheel_omega_,wheel_saved_alpha=max_wheel_alpha_;
        const std::string wheel_saved_label=resource_probe::label();
        const auto wheel_saved_domain=domain_step::counters();
        const auto wheel_saved_last=wheel_cause::lastResult();const bool wheel_saved_running=wheel_cause::running();
        delivery_replay::Finally wheel_restore([&](){
          SnapOpt_container_=wheel_saved_snap;cps_container_=wheel_saved_cps;
          piece_num_container_=wheel_saved_piece_counts;singul_container_=wheel_saved_internal_signs;
          iniState_container_=wheel_saved_internal_ini;finState_container_=wheel_saved_internal_fin;
          gearVel_container_=wheel_saved_gear_vel;mani_hold_reference_=wheel_saved_hold_reference;
          mani_hold_enabled_=wheel_saved_hold_enabled;traj_num_=wheel_saved_traj_num;
          variable_num_=wheel_saved_variable_num;iter_num_=wheel_saved_iter;
          force_stop_type_=wheel_saved_force_stop;optimization_start_wall_time_=wheel_saved_start;
          max_wall_time_=wheel_saved_budget;resource_probe::label()=wheel_saved_label;
          domain_step::counters()=wheel_saved_domain;wheel_cause::lastResult()=wheel_saved_last;
          wheel_cause::running()=wheel_saved_running;
        });
        const auto wheel_frame=coupling_geometry::activeFrame();
        const std::string wheel_prediction=delivery_replay::predictionText();
        const auto wheel_overlay=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
        const auto wheel_raw=grid_map_->getDynamicObservationSeq(),wheel_esdf=grid_map_->getESDFObservationSeq();
        const double wheel_now=ros::Time::now().toSec();
        const double wheel_budgets[]={.6,1.2,.6},wheel_scales[]={1.,1.,1.5};
        const char* wheel_labels[]={"original_060","original_120","time150_060"};
        std::vector<wheel_cause::Result> wheel_results;
        bool wheel_all_captures=true;
        for(int wheel_run=0;wheel_run<3;++wheel_run){
          // Each normal entry builds a fresh x, MinSnap cache and L-BFGS memory.
          // Clear displayed/control-point caches too; no prior trial can leak in.
          SnapOpt_container_.clear();piece_num_container_.clear();
          clear_resize_Cps_container(wheel_saved_times.size());
          auto wheel_times=wheel_saved_times;
          for(auto& times:wheel_times)times*=wheel_scales[wheel_run];
          max_wall_time_=wheel_budgets[wheel_run];resource_probe::label()=std::string("wheel_cause_")+wheel_labels[wheel_run];
          wheel_cause::lastResult()=wheel_cause::Result{};wheel_cause::running()=true;
          std::vector<Eigen::MatrixXd> wheel_cp,wheel_wp;
          std::vector<Eigen::VectorXd> wheel_output_times,wheel_ee;
          const bool wheel_ok=OptimizeTrajectory_lbfgs(wheel_saved_ini,wheel_saved_fin,wheel_saved_points,
            wheel_times,wheel_saved_signs,wheel_cp,wheel_wp,wheel_output_times,wheel_ee);
          wheel_cause::running()=false;
          const auto wheel_result=wheel_cause::lastResult();wheel_results.push_back(wheel_result);
          wheel_all_captures=wheel_all_captures&&wheel_result.error.empty()&&!wheel_result.body.empty();
          std::ostringstream wheel_row;wheel_row<<std::setprecision(17)<<"\"kind\":\"run\",\"label\":"
            <<candidate_audit::quote(wheel_labels[wheel_run])<<",\"budget_s\":"<<wheel_budgets[wheel_run]
            <<",\"time_scale\":"<<wheel_scales[wheel_run]<<",\"return_success\":"<<(wheel_ok?"true":"false")
            <<",\"requested_initial_states\":";failure_probe::matrices(wheel_row,wheel_saved_ini);
          wheel_row<<",\"requested_final_states\":";failure_probe::matrices(wheel_row,wheel_saved_fin);
          wheel_row<<",\"requested_initial_times\":";failure_probe::matrices(wheel_row,wheel_times);
          wheel_row<<",\"capture_error\":"<<candidate_audit::quote(wheel_result.error);
          if(!wheel_result.body.empty())wheel_row<<','<<wheel_result.body;
          wheel_cause::emit(wheel_log,wheel_row.str());
        }
        const bool wheel_environment_stable=wheel_now==ros::Time::now().toSec()&&
          wheel_raw==grid_map_->getDynamicObservationSeq()&&wheel_esdf==grid_map_->getESDFObservationSeq()&&
          wheel_overlay==GridMapParallelTestAccessor::readOverlayKey(*grid_map_)&&
          wheel_frame.snapshot==coupling_geometry::activeFrame().snapshot&&wheel_frame.now==coupling_geometry::activeFrame().now&&
          wheel_prediction==delivery_replay::predictionText()&&wheel_saved_limit==max_wheel_omega_&&wheel_saved_alpha==max_wheel_alpha_;
        const bool wheel_same_first=wheel_results.size()==3&&wheel_cause::sameFirst(wheel_results[0],wheel_results[1]);
        const bool wheel_retime_only=wheel_results.size()==3&&wheel_cause::retimeCoordinates(wheel_results[0],wheel_results[2],1.5);
        std::ostringstream wheel_complete;wheel_complete<<"\"kind\":\"complete\",\"group_count\":3"
          <<",\"all_captures_complete\":"<<(wheel_all_captures?"true":"false")
          <<",\"environment_stable\":"<<(wheel_environment_stable?"true":"false")
          <<",\"original_budgets_same_first_x_f_g\":"<<(wheel_same_first?"true":"false")
          <<",\"retime_only_initial_time_coordinates\":"<<(wheel_retime_only?"true":"false")
          <<",\"original_solver_pin_owned\":"<<(candidate_solver_overlay_owned?"true":"false")
          <<",\"original_solver_entry_exit_equal\":"<<(candidate_record.solver_entry_key==candidate_record.solver_exit_key?"true":"false")
          <<",\"constraints_relaxed\":false,\"diagnostic_candidates_published\":false,\"navigation_metrics_valid\":false"
          <<",\"treatment_scope\":\"T times 1.5 is one fixed diagnostic perturbation; p/v/a/j boundaries and gear signs remain unchanged; regenerated minimum-snap curves are not simple playback slowing; wall-clock budgets may accept different numbers of steps\"";
        wheel_cause::emit(wheel_log,wheel_complete.str());
        if(!wheel_all_captures||!wheel_environment_stable||!wheel_same_first||!wheel_retime_only)
          throw std::runtime_error("wheel-cause replay invariant failed; reject causal interpretation");
      }
