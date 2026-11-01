"""Event-only details for the existing latest-prediction acceptance gate."""
from build_state_contract import Transformer


def transform_fsm(text):
    begin = text.index('  bool REMANIReplanFSM::validatePredictedDynamicTrajectory(')
    end = text.index('  void REMANIReplanFSM::publishDynamicPointFlowStatus(', begin)
    block = text[begin:end]
    t = Transformer(block)
    t.after('    const ros::Time now = ros::Time::now();\n', '''    auto reject = [&](const char* branch, double local_t, double wall_ahead,
                       double execution_scale, bool query_valid, int kind) {
      const auto pos=traj.getPos(local_t);
      const auto current=ros::Time::now();
      dual_probe::Fields detail;
      detail.str("branch",branch).add("snapshot_sequence",snapshot.sequence)
        .add("snapshot_stamp",snapshot.stamp.toSec()).add("snapshot_receive",snapshot.receive_time.toSec())
        .add("snapshot_points",snapshot.points.size()).add("snapshot_valid",snapshot.valid?"true":"false")
        .add("candidate_start",traj.start_time).add("candidate_duration",traj.duration)
        .add("candidate_age",current.toSec()-traj.start_time).add("local_t",local_t)
        .add("wall_ahead",wall_ahead).add("query_stamp",now.toSec()+wall_ahead)
        .add("execution_scale",execution_scale).add("query_valid",query_valid?"true":"false")
        .add("collision_type",kind).add("candidate_x",pos(0)).add("candidate_y",pos(1))
        .add("candidate_yaw",traj.getCarAngle(local_t))
        .add("measured_x",mm_state_pos_(0)).add("measured_y",mm_state_pos_(1))
        .add("measured_yaw",mm_car_yaw_);
      std::ostringstream points;points<<std::setprecision(17)<<'[';
      for(std::size_t i=0;i<snapshot.points.size();++i){
        if(i)points<<',';const auto&p=snapshot.points[i];
        points<<'['<<p.position.x()<<','<<p.position.y()<<','<<p.position.z()<<','
          <<p.velocity.x()<<','<<p.velocity.y()<<','<<p.velocity.z()<<','<<p.track_id<<']';
      }
      points<<']';detail.add("snapshot_points_xyz_velocity_track",points.str());
      dual_probe::emit("latest_prediction_reject_detail",detail);
      return false;
    };
''')
    t.change('''    if (valid_wall_horizon <= start_delay + 1e-3)
      return false;''', '''    if (valid_wall_horizon <= start_delay + 1e-3)
      return reject("insufficient_horizon",0.,start_delay,0.,false,-1);''')
    t.change('''                  snapshot, coll_type, query_valid))
        return false;
      if (!query_valid)
        return false;''', '''                  snapshot, coll_type, query_valid))
        return reject("hold_collision",0.,hold_wall,0.,query_valid,coll_type);
      if (!query_valid)
        return reject("hold_query_invalid",0.,hold_wall,0.,false,coll_type);''')
    t.change('''                    snapshot, coll_type, query_valid))
          return false;
        if (!query_valid)
          return false;''', '''                    snapshot, coll_type, query_valid))
          return reject("trajectory_collision",t,wall_ahead,execution_scale,query_valid,coll_type);
        if (!query_valid)
          return reject("trajectory_query_invalid",t,wall_ahead,execution_scale,false,coll_type);''')
    return text[:begin]+t.finish()+text[end:]
