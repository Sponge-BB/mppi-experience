#pragma once
// Isolated SIL observation only. No overlay update/freeze, trajectory mutation,
// extra gate, or full-map dump. The child GridMap header must declare
// `friend class HandoffAlignmentMapAccessor;` (no object-layout change).
#include "dual_timescale_probe.hpp"
#include "plan_env/grid_map.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <sstream>
#include <vector>

namespace handoff_alignment {
inline double missing(){return std::numeric_limits<double>::quiet_NaN();}
inline std::string number(double x){
  if(!std::isfinite(x))return "null";
  std::ostringstream s;s<<std::setprecision(17)<<x;return s.str();
}
inline std::string xyz(const Eigen::Vector3d& p){
  return "["+number(p.x())+","+number(p.y())+","+number(p.z())+"]";
}
struct Overlay {
  bool enabled=false,ready=false,frozen=false,found=false;
  std::uint64_t sequence=0,frozen_sequence=0;
  double refresh=0.,horizon=0.,step=0.,extra=0.,influence=0.;
  double distance=missing(),geometric=missing(),radius=missing();
  int point_index=-1;std::size_t point_count=0;
  Eigen::Vector3d point=Eigen::Vector3d::Constant(missing());
};
}

// The only privileged reads are under the same mutex as the native query.
class HandoffAlignmentMapAccessor {
 public:
  static handoff_alignment::Overlay query(GridMap& map,const Eigen::Vector3d& p,double maximum){
    using handoff_alignment::Overlay;
    Overlay out;std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    out.enabled=map.mp_.dynamic_point_overlay_enable_;
    out.ready=map.dynamic_point_overlay_ready_;out.frozen=map.dynamic_point_overlay_frozen_;
    out.sequence=map.dynamic_point_overlay_sequence_;
    out.frozen_sequence=map.dynamic_point_overlay_frozen_sequence_;
    out.refresh=map.dynamic_point_overlay_refresh_time_.toSec();
    out.horizon=map.mp_.dynamic_point_overlay_horizon_;
    out.step=map.mp_.dynamic_point_overlay_time_step_;
    out.extra=map.mp_.dynamic_point_overlay_extra_inflation_;
    out.influence=map.mp_.dynamic_point_overlay_influence_distance_;
    out.point_count=map.dynamic_point_overlay_cloud_?map.dynamic_point_overlay_cloud_->size():0;
    if(!out.enabled||!out.ready||!out.point_count||maximum<=0.||!p.allFinite()||
        map.dynamic_point_overlay_radii_.size()!=out.point_count)return out;
    const double influence=std::min(maximum,out.influence);
    const double limit=influence+map.dynamic_point_overlay_inflation_;
    for(int a=0;a<3;++a)if(p(a)<map.dynamic_point_overlay_min_(a)-limit||
        p(a)>map.dynamic_point_overlay_max_(a)+limit)return out;
    pcl::PointXYZ query;query.x=p.x();query.y=p.y();query.z=p.z();
    std::vector<int> indices;std::vector<float> squared;
    if(map.dynamic_point_overlay_kdtree_.radiusSearch(query,limit,indices,squared)<=0)return out;
    double best=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<indices.size();++i){
      const int k=indices[i];if(k<0||std::size_t(k)>=out.point_count)continue;
      const double geometric=std::sqrt(std::max(0.,double(squared[i])));
      const double radius=map.dynamic_point_overlay_radii_[k];
      if(geometric-radius<best){best=geometric-radius;out.point_index=k;
        out.geometric=geometric;out.radius=radius;
        const auto& q=map.dynamic_point_overlay_cloud_->points[k];out.point={q.x,q.y,q.z};}
    }
    if(best<=influence){out.found=true;out.distance=best;}
    return out;
  }
};

namespace handoff_alignment {
inline bool enabled(){const char* p=std::getenv("HANDOFF_ALIGNMENT_PROBE");return p&&std::string(p)=="1";}
inline bool& finalGate(){static thread_local bool value=false;return value;}
struct FinalGate {
  bool previous;
  FinalGate():previous(finalGate()){finalGate()=true;}
  ~FinalGate(){finalGate()=previous;}
};
struct Limit {unsigned long long request=0;bool first=false,final=false;};
inline Limit& limiter(){static thread_local Limit value;return value;}

inline std::string overlayJson(const Overlay& q){
  std::ostringstream s;s<<"{\"enabled\":"<<(q.enabled?"true":"false")
    <<",\"ready\":"<<(q.ready?"true":"false")<<",\"frozen\":"<<(q.frozen?"true":"false")
    <<",\"sequence\":"<<q.sequence<<",\"frozen_sequence\":"<<q.frozen_sequence
    <<",\"refresh_time\":"<<number(q.refresh)<<",\"configured_horizon_s\":"<<number(q.horizon)
    <<",\"configured_step_s\":"<<number(q.step)<<",\"extra_inflation_m\":"<<number(q.extra)
    <<",\"point_count\":"<<q.point_count<<",\"found\":"<<(q.found?"true":"false")
    <<",\"nearest_index\":"<<q.point_index<<",\"nearest_xyz\":"<<xyz(q.point)
    <<",\"nearest_radius_m\":"<<number(q.radius)<<",\"geometric_distance_m\":"<<number(q.geometric)
    <<",\"distance_after_point_radius_m\":"<<number(q.distance)<<'}';return s.str();
}

struct Forecast {
  bool valid=false;double time=0.,uncertainty=0.;
  plan_env::PredictedPositionVector points;
};
inline Forecast forecast(GridMap& map,const plan_env::DynamicPointSnapshot& snapshot,double time){
  Forecast q;q.time=time;ros::Time epoch;epoch.fromSec(time);
  q.valid=map.getPredictedDynamicObstaclePositions(snapshot,epoch,q.points,q.uncertainty);return q;
}
inline std::string cvJson(const Forecast& q,const Eigen::Vector3d& center,double hard){
  int index=-1;double distance=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<q.points.size();++i){const double d=(q.points[i]-center).norm();
    if(d<distance){index=int(i);distance=d;}}
  const double gap=q.valid&&index>=0?distance-q.uncertainty-hard:missing();
  std::ostringstream s;s<<"{\"query_time\":"<<number(q.time)<<",\"valid\":"<<(q.valid?"true":"false")
    <<",\"points\":"<<q.points.size()<<",\"uncertainty_m\":"<<number(q.uncertainty)
    <<",\"nearest_index\":"<<index<<",\"nearest_xyz\":"
    <<(index>=0?xyz(q.points[index]):"null")<<",\"geometric_distance_m\":"<<number(distance)
    <<",\"hard_gap_m\":"<<number(gap)<<",\"collision\":"
    <<(q.valid?(index>=0&&gap<0.?"true":"false"):"null")<<'}';return s.str();
}

template<class Trajectory,class Model>
std::string poseJson(const Trajectory& trajectory,double t,Model& model,GridMap& map,
                     const Forecast& now,const Forecast& unit,const Forecast& half){
  const auto p=trajectory.getPos(t);const double yaw=trajectory.getCarAngle(t);
  std::vector<Eigen::Vector3d> spheres;model.getCarPts(Eigen::Vector3d(p(0),p(1),yaw),spheres);
  int first=-1,closest=-1;double best=std::numeric_limits<double>::infinity();
  double chosen_esdf=missing(),chosen_plan=missing(),chosen_radius=missing();
  Eigen::Vector3d center=Eigen::Vector3d::Constant(missing());
  for(std::size_t i=0;i<spheres.size();++i){
    const double radius=model.getBaseSphereRadius(i),hard=radius+model.getMapResolution();
    const double plan=map.getPlanningDistance(spheres[i],true),gap=plan-hard;
    if(gap<0.&&first<0)first=int(i);
    if(gap<best){best=gap;closest=int(i);}
  }
  const int selected=first>=0?first:closest;Overlay overlay;
  if(selected>=0){center=spheres[selected];chosen_radius=model.getBaseSphereRadius(selected);
    chosen_esdf=map.getPreciseDistance(center);chosen_plan=map.getPlanningDistance(center,true);
    overlay=HandoffAlignmentMapAccessor::query(map,center,std::max(0.,chosen_esdf));}
  const double hard=chosen_radius+model.getMapResolution();
  const double reconstructed=overlay.found?std::min(chosen_esdf,overlay.distance):chosen_esdf;
  std::ostringstream s;s<<"{\"trajectory_t\":"<<number(t)<<",\"pose_xy_yaw\":["
    <<number(p(0))<<','<<number(p(1))<<','<<number(yaw)<<"],\"base_spheres\":"<<spheres.size()
    <<",\"base_minimum_hard_gap_m\":"<<number(best)<<",\"first_colliding_index\":"<<first
    <<",\"minimum_gap_index\":"<<closest<<",\"reported_sphere_index\":"<<selected
    <<",\"sphere_center\":"<<xyz(center)<<",\"physical_radius_m\":"<<number(chosen_radius)
    <<",\"map_resolution_addition_m\":"<<number(model.getMapResolution())
    <<",\"hard_radius_m\":"<<number(hard)<<",\"precise_esdf_distance_m\":"<<number(chosen_esdf)
    <<",\"native_planning_distance_m\":"<<number(chosen_plan)
    <<",\"reconstructed_distance_difference_m\":"<<number(reconstructed-chosen_plan)
    <<",\"esdf_alone_collision\":"<<(chosen_esdf<hard?"true":"false")
    <<",\"swept_alone_collision\":"<<(overlay.found&&overlay.distance<hard?"true":"false")
    <<",\"swept\":"<<overlayJson(overlay)<<",\"cv_at_capture_now\":"<<cvJson(now,center,hard)
    <<",\"cv_at_now_plus_local_t\":"<<cvJson(unit,center,hard)
    <<",\"cv_at_now_plus_twice_local_t\":"<<cvJson(half,center,hard)<<'}';return s.str();
}

template<class Trajectory,class ModelPtr,class MapPtr>
void planningReject(const Trajectory& trajectory,double t,int collision_type,
                    const ModelPtr& model,const MapPtr& map) noexcept {
  if(!enabled()||!model||!map)return;
  auto& limit=limiter();const auto id=dual_probe::request();
  if(limit.request!=id){limit=Limit{};limit.request=id;}
  const bool final=finalGate();if(final?limit.final:limit.first)return;
  if(final)limit.final=true;else limit.first=true;
  const double began=dual_probe::mono(),capture=ros::Time::now().toSec();
  try{
    ros::Time capture_epoch;capture_epoch.fromSec(capture);
    const auto snapshot=map->getDynamicPointSnapshot(capture_epoch);
    const auto present=forecast(*map,snapshot,capture);
    const auto future=forecast(*map,snapshot,capture+t);
    const auto slower=forecast(*map,snapshot,capture+2.*t);
    const std::string start=poseJson(trajectory,0.,*model,*map,present,present,present);
    const std::string failure=poseJson(trajectory,t,*model,*map,present,future,slower);
    dual_probe::emit("handoff_native_planning_reject",dual_probe::Fields()
      .str("stage",final?"final_native_gate":"first_native_gate_failure")
      .add("collision_type",collision_type).add("failure_t",t).add("capture_time",capture)
      .add("candidate_start_metadata",trajectory.start_time).add("candidate_duration_s",trajectory.duration)
      .add("cv_snapshot_sequence",snapshot.sequence).add("cv_snapshot_stamp",snapshot.stamp.toSec())
      .add("cv_snapshot_valid",snapshot.valid?"true":"false").add("cv_snapshot_points",snapshot.points.size())
      .add("start",start).add("failure",failure).add("diagnostic_wall_ms",(dual_probe::mono()-began)*1000.)
      .str("semantics","read-only base-sphere leaf explanation; CV now/local-t/half-speed are diagnostic alternatives, not changed safety gates; swept points have no retained per-point prediction timestamp; snapshot equality must be checked before attributing a temporal difference"));
  }catch(const std::exception& error){
    dual_probe::emit("handoff_native_planning_probe_error",dual_probe::Fields().str("reason",error.what()));
  }catch(...){
    dual_probe::emit("handoff_native_planning_probe_error",dual_probe::Fields().str("reason","unknown diagnostic exception"));
  }
}
} // namespace handoff_alignment
