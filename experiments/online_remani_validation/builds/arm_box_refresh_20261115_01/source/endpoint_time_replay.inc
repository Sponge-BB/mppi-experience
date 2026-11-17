      // Same owned PauseOwned and solver/native map pin as the captured failure.
      // Replay never invokes astarWithMinTraj; its separate child-only gate is off here.
      if(endpoint_time::requested() && audit_target=="wheel"){
        bool endpoint_sil=false,endpoint_sim=false;
        ros::param::param("/online_validation/enabled",endpoint_sil,false);
        ros::param::param("/use_sim_time",endpoint_sim,false);
        if(!endpoint_sil||!endpoint_sim||!candidate_solver_freeze_requested||!candidate_solver_overlay_owned||
           candidate_record.solver_entry_key!=candidate_record.solver_exit_key||
           candidate_record.solver_entry_key!=candidate_native_gate_begin||
           candidate_native_gate_begin!=candidate_native_gate_end||!audit_overlay_stable||
           !solver_map_stable||!final_matches_saved||delivery_replay::running()||wheel_cause::running()||endpoint_time::fixRequested())
          throw std::runtime_error("endpoint-time replay requires proven fixed-map isolated SIL capture");
        if(!mm_config_->hasCustomBaseSpheres())throw std::runtime_error("endpoint-time requires unchanged custom spheres");
        std::vector<Eigen::Vector3d> endpoint_base_points;mm_config_->getCarPts(Eigen::Vector3d::Zero(),endpoint_base_points);
        if(endpoint_base_points.size()!=47)throw std::runtime_error("endpoint-time requires unchanged 47 spheres");
        const std::string endpoint_log=audit_dir+"/endpoint_time_replay.jsonl";
        const auto endpoint_ini=iniState_container,endpoint_fin=finState_container,endpoint_points=initInnerPts_container;
        const auto endpoint_times=initT_container;const auto endpoint_signs=singul_container;
        decltype(SnapOpt_container_) endpoint_saved_snap;
        const auto endpoint_saved_cps=cps_container_;
        const auto endpoint_saved_pieces=piece_num_container_;
        const auto endpoint_saved_signs=singul_container_;
        const auto endpoint_saved_ini=iniState_container_,endpoint_saved_fin=finState_container_;
        const auto endpoint_saved_gear_vel=gearVel_container_;
        const auto endpoint_saved_hold_reference=mani_hold_reference_;
        const bool endpoint_saved_hold_enabled=mani_hold_enabled_;
        const int endpoint_saved_traj_num=traj_num_,endpoint_saved_variables=variable_num_,endpoint_saved_iter=iter_num_;
        const auto endpoint_saved_force_stop=force_stop_type_;
        const auto endpoint_saved_start=optimization_start_wall_time_;
        const double endpoint_saved_budget=max_wall_time_,endpoint_wheel_limit=max_wheel_omega_,endpoint_alpha_limit=max_wheel_alpha_;
        const std::string endpoint_saved_label=resource_probe::label();
        const auto endpoint_saved_domain=domain_step::counters();
        const auto endpoint_saved_result=wheel_cause::lastResult();
        const bool endpoint_saved_running=wheel_cause::running();
        const char* endpoint_fix_env=std::getenv("ENDPOINT_TIME_FIX");
        const bool endpoint_had_fix=endpoint_fix_env!=nullptr;
        const std::string endpoint_saved_fix=endpoint_fix_env?endpoint_fix_env:"";
        delivery_replay::Finally endpoint_restore([&](){
          SnapOpt_container_.swap(endpoint_saved_snap);cps_container_=endpoint_saved_cps;
          piece_num_container_=endpoint_saved_pieces;singul_container_=endpoint_saved_signs;
          iniState_container_=endpoint_saved_ini;finState_container_=endpoint_saved_fin;
          gearVel_container_=endpoint_saved_gear_vel;mani_hold_reference_=endpoint_saved_hold_reference;
          mani_hold_enabled_=endpoint_saved_hold_enabled;traj_num_=endpoint_saved_traj_num;
          variable_num_=endpoint_saved_variables;iter_num_=endpoint_saved_iter;
          force_stop_type_=endpoint_saved_force_stop;optimization_start_wall_time_=endpoint_saved_start;
          max_wall_time_=endpoint_saved_budget;resource_probe::label()=endpoint_saved_label;
          domain_step::counters()=endpoint_saved_domain;wheel_cause::lastResult()=endpoint_saved_result;
          wheel_cause::running()=endpoint_saved_running;
          if(endpoint_had_fix)setenv("ENDPOINT_TIME_FIX",endpoint_saved_fix.c_str(),1);else unsetenv("ENDPOINT_TIME_FIX");
        });
        endpoint_saved_snap.swap(SnapOpt_container_); // buffer ownership only; MinSnapOpt must never be copied.
        const auto endpoint_frame=coupling_geometry::activeFrame();
        const std::string endpoint_prediction=delivery_replay::predictionText();
        const auto endpoint_overlay=GridMapParallelTestAccessor::readOverlayKey(*grid_map_);
        const auto endpoint_raw=grid_map_->getDynamicObservationSeq(),endpoint_esdf=grid_map_->getESDFObservationSeq();
        const double endpoint_now=ros::Time::now().toSec();
        if(endpoint_saved_cps.size()!=endpoint_times.size())throw std::runtime_error("endpoint-time control-point container mismatch");
        for(std::size_t n=0;n<endpoint_saved_cps.size();++n){
          const int columns=endpoint_times[n].size()*cps_num_prePiece_+1;
          if(endpoint_saved_cps[n].points.rows()!=traj_dim_||endpoint_saved_cps[n].points.cols()!=columns||
             endpoint_saved_cps[n].cp_size!=columns)throw std::runtime_error("endpoint-time control-point matrix mismatch");
        }
        const char* endpoint_labels[]={"original_060","endpoint150_060"};
        std::vector<wheel_cause::Result> endpoint_results;bool endpoint_captures_complete=true;
        for(int endpoint_run=0;endpoint_run<2;++endpoint_run){
          SnapOpt_container_.clear();piece_num_container_.clear();
          cps_container_=endpoint_saved_cps;for(auto& cp:cps_container_)cp.points.setZero();
          max_wall_time_=.6;resource_probe::label()=std::string("endpoint_time_")+endpoint_labels[endpoint_run];
          if(setenv("ENDPOINT_TIME_FIX",endpoint_run?"1":"0",1)!=0)throw std::runtime_error("cannot set diagnostic endpoint-time switch");
          wheel_cause::lastResult()=wheel_cause::Result{};wheel_cause::running()=true;
          const bool endpoint_fix=endpoint_time::fixRequested();
          const auto times=endpoint_time::initialTimes(endpoint_times,endpoint_fix);
          const auto factors=endpoint_time::factors(endpoint_times,endpoint_fix);
          std::vector<Eigen::MatrixXd> cp,wp;std::vector<Eigen::VectorXd> output_times,ee;
          const bool ok=OptimizeTrajectory_lbfgs(endpoint_ini,endpoint_fin,endpoint_points,times,endpoint_signs,cp,wp,output_times,ee);
          wheel_cause::running()=false;
          const auto result=wheel_cause::lastResult();endpoint_results.push_back(result);
          endpoint_captures_complete=endpoint_captures_complete&&result.error.empty()&&!result.body.empty();
          std::ostringstream row;row<<std::setprecision(17)<<"\"kind\":\"run\",\"label\":"<<candidate_audit::quote(endpoint_labels[endpoint_run])
            <<",\"budget_s\":0.6,\"endpoint_time_fix_enabled\":"<<(endpoint_fix?"true":"false")
            <<",\"endpoint_time_scale\":"<<(endpoint_fix?1.5:1.)<<",\"return_success\":"<<(ok?"true":"false")
            <<",\"requested_initial_states\":";failure_probe::matrices(row,endpoint_ini);
          row<<",\"requested_final_states\":";failure_probe::matrices(row,endpoint_fin);
          row<<",\"requested_inner_points\":";failure_probe::matrices(row,endpoint_points);
          row<<",\"requested_initial_times\":";failure_probe::matrices(row,times);
          row<<",\"per_piece_time_factors\":";failure_probe::matrices(row,factors);
          row<<",\"capture_error\":"<<candidate_audit::quote(result.error);
          if(!result.body.empty())row<<','<<result.body;
          endpoint_time::emit(endpoint_log,row.str());
        }
        const bool environment_stable=endpoint_now==ros::Time::now().toSec()&&
          endpoint_raw==grid_map_->getDynamicObservationSeq()&&endpoint_esdf==grid_map_->getESDFObservationSeq()&&
          endpoint_overlay==GridMapParallelTestAccessor::readOverlayKey(*grid_map_)&&
          endpoint_frame.snapshot==coupling_geometry::activeFrame().snapshot&&endpoint_frame.now==coupling_geometry::activeFrame().now&&
          endpoint_prediction==delivery_replay::predictionText()&&endpoint_wheel_limit==max_wheel_omega_&&endpoint_alpha_limit==max_wheel_alpha_;
        const bool endpoint_only=endpoint_results.size()==2&&endpoint_time::endpointCoordinates(endpoint_results[0],endpoint_results[1]);
        const bool original_matches_capture=endpoint_results.size()==2&&!candidate_record.steps.empty()&&
          endpoint_results[0].first_x.size()==candidate_record.steps.front().x.size()&&
          (endpoint_results[0].first_x.array()==candidate_record.steps.front().x.array()).all()&&
          endpoint_results[0].first_cost==candidate_record.steps.front().cost&&
          endpoint_results[0].first_gradient_hash!=0&&endpoint_results[0].first_gradient_hash==diagnosis.first_gradient_hash;
        std::ostringstream complete;complete<<"\"kind\":\"complete\",\"group_count\":2"
          <<",\"all_captures_complete\":"<<(endpoint_captures_complete?"true":"false")
          <<",\"environment_stable\":"<<(environment_stable?"true":"false")
          <<",\"capture_original_same_first_x_f_g\":"<<(original_matches_capture?"true":"false")
          <<",\"retime_only_initial_endpoint_time_coordinates\":"<<(endpoint_only?"true":"false")
          <<",\"original_solver_pin_owned\":"<<(candidate_solver_overlay_owned?"true":"false")
          <<",\"original_solver_entry_exit_equal\":"<<(candidate_record.solver_entry_key==candidate_record.solver_exit_key?"true":"false")
          <<",\"constraints_relaxed\":false,\"diagnostic_candidates_published\":false,\"navigation_metrics_valid\":false"
          <<",\"production_frontend_modified\":false,\"child_frontend_fix_available\":true"
          <<",\"captured_online_frontend_fix_enabled\":false,\"historical_case_imported\":false"
          <<",\"treatment_scope\":\"fresh same-process paused input; only first/last real duration of each gear segment times 1.5 exactly once; singleton piece not double-multiplied; same PVAJ/gear/inner points/limits; child frontend switch off throughout online capture; replay does not invoke frontend; not simple curve playback slowing\"";
        endpoint_time::emit(endpoint_log,complete.str());
        if(!endpoint_captures_complete||!environment_stable||!original_matches_capture||!endpoint_only)
          throw std::runtime_error("endpoint-time evidence invariant failed; reject causal interpretation");
      }
