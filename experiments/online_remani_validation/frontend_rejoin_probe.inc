  if(frontend_rejoin::requested(start_pos(0))){
    bool sil=false,sim=false;
    ros::param::param("/online_validation/enabled",sil,false);
    ros::param::param("/use_sim_time",sim,false);
    if(!sil||!sim||frontend_rejoin::mode()!="baseline")
      throw std::runtime_error("frontend probe requires isolated SIL baseline mode");
    frontend_rejoin::used()=true;
    failure_probe::PauseOwned paused;
    const auto frozen_now=ros::Time::now();
    const auto raw=sdf_map_->getDynamicObservationSeq(),esdf=sdf_map_->getESDFObservationSeq();
    const auto snapshot=sdf_map_->getDynamicPointSnapshot(frozen_now);
    const auto snapshot_text=delivery_replay::nativeDynamicText(snapshot);
    const auto snapshot_hash=delivery_diag::hash(snapshot_text);
    const bool overlay=sdf_map_->dynamicPointPlanningOverlayEnabled();
    bool owns_overlay=false;
    if(overlay&&snapshot.valid&&!snapshot.points.empty()){
      owns_overlay=sdf_map_->beginDynamicPointPlanningSnapshot(snapshot,frozen_now);
      if(!owns_overlay)throw std::runtime_error("frontend probe cannot freeze native overlay");
    }
    delivery_replay::Finally release_overlay([&](){if(owns_overlay)sdf_map_->endDynamicPointPlanningSnapshot();});
    const double saved_steer=traj_steer_penalty_,saved_change=traj_steer_change_penalty_,saved_yaw=inv_yaw_resolution_;
    frontend_rejoin::running()=true;
    delivery_replay::Finally restore([&](){
      frontend_rejoin::running()=false;traj_steer_penalty_=saved_steer;
      traj_steer_change_penalty_=saved_change;inv_yaw_resolution_=saved_yaw;
    });
    const std::string input_text="{\"start_pos\":"+delivery_diag::vector(start_pos)+
      ",\"start_vel\":"+delivery_diag::vector(start_vel)+",\"start_yaw\":"+delivery_diag::number(start_yaw)+
      ",\"start_gear\":"+std::to_string(start_singul)+",\"start_gripper\":"+(start_gripper?"true":"false")+
      ",\"end_pos\":"+delivery_diag::vector(end_pos)+",\"end_vel\":"+delivery_diag::vector(end_vel)+
      ",\"end_yaw\":"+delivery_diag::number(end_yaw)+",\"end_gripper\":"+(end_gripper?"true":"false")+
      ",\"initial_control\":"+delivery_diag::vector(init_ctrl)+",\"failure_count\":"+std::to_string(continous_failures_count)+"}";
    const auto input_hash=delivery_diag::hash(input_text);
    frontend_rejoin::emit("\"kind\":\"capture\",\"input_hash\":"+delivery_diag::quote(input_hash)+
      ",\"input\":"+input_text+",\"sim\":"+delivery_diag::number(frozen_now.toSec())+
      ",\"raw_seq\":"+std::to_string(raw)+",\"esdf_seq\":"+std::to_string(esdf)+
      ",\"native_dynamic\":"+snapshot_text+",\"native_dynamic_hash\":"+delivery_diag::quote(snapshot_hash)+
      ",\"overlay_owned\":"+(owns_overlay?"true":"false")+
      ",\"map_owner\":\"blocked single ros::spin callback; native overlay pinned; map buffers not dumped\""
      ",\"selection\":\"first ordinary native search after reference exists and start_x > 1.8\""
      ",\"navigation_metrics_valid\":false");
    bool stable=true,repeat_equal=false;
    std::string baseline_output;
    const char* variants[]={"baseline","penalty","yaw","both","baseline_repeat"};
    for(const char* variant:variants){
      const std::string selected=std::string(variant)=="baseline_repeat"?"baseline":variant;
      traj_steer_penalty_=saved_steer;traj_steer_change_penalty_=saved_change;inv_yaw_resolution_=saved_yaw;
      frontend_rejoin::apply(selected,traj_steer_penalty_,traj_steer_change_penalty_,inv_yaw_resolution_);
      const double begin=failure_probe::wall();
      const int status=KinoAstarSearchAndGetSimplePath(start_pos,start_vel,start_yaw,start_singul,start_gripper,
        end_pos,end_vel,end_yaw,end_gripper,init_ctrl,continous_failures_count,
        simple_path_container,yaw_list_container,singul_container,t_list_container);
      const double elapsed=failure_probe::wall()-begin;
      const auto output=frontend_rejoin::pathText(simple_path_container,yaw_list_container,t_list_container,singul_container);
      if(selected=="baseline"&&baseline_output.empty())baseline_output=output;
      else if(std::string(variant)=="baseline_repeat")repeat_equal=output==baseline_output;
      const auto after=sdf_map_->getDynamicPointSnapshot(frozen_now);
      const bool same_map=raw==sdf_map_->getDynamicObservationSeq()&&esdf==sdf_map_->getESDFObservationSeq()&&
        ros::Time::now()==frozen_now&&delivery_replay::nativeDynamicText(after)==snapshot_text;
      stable=stable&&same_map;
      std::string nodes="[";
      for(size_t i=0;i<path_nodes_.size();++i){
        if(i)nodes+=',';const auto* node=path_nodes_[i];
        nodes+="{\"state\":"+delivery_diag::vector(node->state)+",\"input\":"+delivery_diag::vector(node->input)+
          ",\"g\":"+delivery_diag::number(node->g_score)+",\"f\":"+delivery_diag::number(node->f_score)+"}";
      }
      nodes+=']';
      frontend_rejoin::emit("\"kind\":\"variant\",\"variant\":"+delivery_diag::quote(variant)+
        ",\"input_hash\":"+delivery_diag::quote(input_hash)+",\"native_dynamic_hash\":"+delivery_diag::quote(snapshot_hash)+
        ",\"raw_seq\":"+std::to_string(raw)+",\"esdf_seq\":"+std::to_string(esdf)+
        ",\"frozen_stable\":"+(same_map?"true":"false")+
        ",\"steer_penalty\":"+delivery_diag::number(traj_steer_penalty_)+
        ",\"steer_change_penalty\":"+delivery_diag::number(traj_steer_change_penalty_)+
        ",\"yaw_resolution\":"+delivery_diag::number(1./inv_yaw_resolution_)+
        ",\"lambda_heu\":"+delivery_diag::number(lambda_heu_)+
        ",\"max_arc\":"+delivery_diag::number(max_arc_)+",\"oneshot_range\":"+delivery_diag::number(oneshot_range_)+
        ",\"status\":"+std::to_string(status)+",\"wrapper_wall_ms\":"+delivery_diag::number(elapsed*1000.)+
        ",\"expanded_nodes\":"+std::to_string(use_node_num_)+",\"iterations\":"+std::to_string(iter_num_)+
        ",\"raw_nodes\":"+nodes+",\"output\":"+output);
    }
    const std::string complete="{\"schema\":\"frontend_rejoin_complete_v1\",\"complete\":true,\"frozen_stable\":"+
      std::string(stable?"true":"false")+",\"baseline_repeat_equal\":"+(repeat_equal?"true":"false")+
      ",\"diagnostic_candidates_published\":false,\"navigation_metrics_valid\":false}";
    frontend_rejoin::emit("\"kind\":\"complete\",\"result\":"+complete);
    const char* done=std::getenv("FRONTEND_REJOIN_DONE_PATH");
    if(!done||!*done)throw std::runtime_error("missing FRONTEND_REJOIN_DONE_PATH");
    std::ofstream finished(done);finished<<complete<<'\n';
    if(!finished)throw std::runtime_error("cannot write frontend probe completion");
    reset();simple_path_container.clear();yaw_list_container.clear();singul_container.clear();t_list_container.clear();
    return KinoAstar::NO_PATH;
  }
