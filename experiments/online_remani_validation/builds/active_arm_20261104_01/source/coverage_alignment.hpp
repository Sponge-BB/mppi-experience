#pragma once
// Isolated SIL: measured-state epoch and geometric-reference acceptance phase.
// A projection is a reference association, not a promised MPPI trajectory.
#include "dual_timescale_probe.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace coverage_alignment {
inline bool enabled(){
  const char* value=std::getenv("COVERAGE_ALIGNMENT");
  return value&&std::string(value)=="1";
}
inline bool& localOwner(){static thread_local bool value=false;return value;}
inline bool active(){return enabled()&&localOwner();}
class Request {
  bool previous_;
 public:
  explicit Request(bool local):previous_(localOwner()){localOwner()=local;}
  ~Request(){localOwner()=previous_;}
};

struct Phase {
  double local_t=0.,position_residual=0.,yaw_residual=0.;
  double searched_until=0.;
};
template<class Trajectory,class Position>
Phase project(const Trajectory& trajectory,const Position& measured,double yaw,double step){
  Phase result;
  result.searched_until=std::min(2.,std::max(0.,trajectory.duration));
  const Eigen::Vector2d actual=measured.template head<2>();
  double best=std::numeric_limits<double>::infinity();
  double begin=0.;
  Eigen::Vector2d first=trajectory.getPos(0.).template head<2>();
  while(begin<result.searched_until-1e-9){
    const double end=std::min(result.searched_until,begin+step);
    const Eigen::Vector2d last=trajectory.getPos(end).template head<2>();
    const Eigen::Vector2d edge=last-first;
    const double length2=edge.squaredNorm();
    if(length2>1e-12){
      const double fraction=std::max(0.,std::min(1.,(actual-first).dot(edge)/length2));
      const double t=begin+fraction*(end-begin);
      const double distance=(first+fraction*edge-actual).norm();
      // Same initial previous=0 association as progress.py: geometric distance
      // dominates, and an exact spatial tie selects the earlier local phase.
      const double cost=distance+1e-7*t;
      if(cost<best){best=cost;result.local_t=t;}
    }
    begin=end;first=last;
  }
  const Eigen::Vector2d matched=trajectory.getPos(result.local_t).template head<2>();
  result.position_residual=(matched-actual).norm();
  const double angle=trajectory.getCarAngle(result.local_t)-yaw;
  result.yaw_residual=std::atan2(std::sin(angle),std::cos(angle));
  return result;
}

template<class Trajectory,class Position>
void recordPhase(const Trajectory& trajectory,const Position& measured,double measured_yaw,
                 double measurement_epoch,double now,const Phase& phase){
  dual_probe::emit("candidate_phase_alignment",dual_probe::Fields()
    .add("new_phase_mapping",active()?"true":"false")
    .add("measurement_epoch",measurement_epoch)
    .add("latest_measurement_epoch",measurement_epoch)
    .add("candidate_epoch",trajectory.start_time)
    .add("validation_now",now)
    .add("phase_local_t",active()?phase.local_t:0.)
    .add("diagnostic_projected_phase_local_t",phase.local_t)
    .add("position_residual_m",phase.position_residual)
    .add("yaw_residual_rad",phase.yaw_residual)
    .add("measurement_x",measured(0)).add("measurement_y",measured(1))
    .add("measurement_yaw",measured_yaw)
    .add("projection_window_end_s",phase.searched_until)
    .add("nominal_epoch_offset_s",now-(trajectory.start_time+phase.local_t))
    .add("half_speed_epoch_offset_s",now-(trajectory.start_time+2.*phase.local_t))
    .str("phase_method",active()?"measured_pose_segment_projection_initial_2s":"original_restart_zero")
    .str("time_mapping",active()?"query=validation_now+(local_t-phase_local_t)/execution_scale; measured pose checked separately; no future MPPI promise":"original validator restarts local t=0")
    .str("residual_semantics","read-only association residuals; no added residual rejection threshold"));
}
} // namespace coverage_alignment
