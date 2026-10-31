#pragma once
// Read-only, opt-in post-rejection observation. This does not re-run target
// selection or authorize a target, and is NOT a frozen native-map replay.
#include "dual_timescale_probe.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace target_entry_diagnostics {
inline bool enabled(){const char*p=std::getenv("COUPLING_DIAGNOSTIC_DIR");return p&&*p;}
inline std::string number(double value){
  if(!std::isfinite(value))return "null";
  std::ostringstream s;s<<std::setprecision(17)<<value;return s.str();
}
template<class Config,class Map>
void rejected(Config& config,Map& map,const Eigen::Vector3d& goal,
              double horizon,double progress,bool project,int collision,
              unsigned long long raw_before,unsigned long long esdf_before) noexcept {
  if(!enabled())return;
  try{
    const auto stamp=ros::Time::now();const auto dyn_before=map.getDynamicPointSnapshot(stamp);
    const auto raw_probe=map.getDynamicObservationSeq(),esdf_probe=map.getESDFObservationSeq();
    std::vector<Eigen::Vector3d> points;config.getCarPts(goal,points);
    const double threshold=config.getBaseCheckRadius()+config.getMapResolution();
    double stat_min=std::numeric_limits<double>::infinity(),dyn_min=stat_min,combined_min=stat_min;
    int stat_bad=0,dyn_bad=0,overlay_used=0;std::ostringstream rows;rows<<'[';
    for(size_t i=0;i<points.size();++i){
      const auto&q=points[i];const double stat=map.getPreciseDistance(q);double dyn=stat;
      const bool has=map.getDynamicPointPlanningDistance(q,std::max(stat,0.),dyn,nullptr);
      const double combined=has?std::min(stat,dyn):stat;
      stat_min=std::min(stat_min,stat);combined_min=std::min(combined_min,combined);
      if(has){dyn_min=std::min(dyn_min,dyn);++overlay_used;}
      stat_bad+=stat<threshold;dyn_bad+=has&&dyn<threshold;
      if(i)rows<<',';
      rows<<'['<<number(q.x())<<','<<number(q.y())<<','<<number(q.z())<<','
          <<number(stat)<<','<<(has?number(dyn):"null")<<','<<number(combined)<<']';
    }rows<<']';
    const auto dyn_after=map.getDynamicPointSnapshot(ros::Time::now());
    const auto raw_after=map.getDynamicObservationSeq(),esdf_after=map.getESDFObservationSeq();
    const bool stable=raw_before==raw_probe&&raw_probe==raw_after&&
      esdf_before==esdf_probe&&esdf_probe==esdf_after&&dyn_before.sequence==dyn_after.sequence;
    std::string source=stat_bad?(dyn_bad?"both_on_probe":"esdf_on_probe"):
      (dyn_bad?"overlay_on_probe":"no_base_violation_on_probe");
    dual_probe::emit("target_rejection_probe",dual_probe::Fields()
      .str("scope","post-rejection read-only probe; not immutable native map replay")
      .add("goal_x",goal.x()).add("goal_y",goal.y()).add("goal_yaw",goal.z())
      .add("collision_type",collision).add("planning_horizon",horizon)
      .add("global_progress_s",progress).add("project_from_current",project?"true":"false")
      .add("threshold",threshold).str("observed_source",source)
      .add("static_min",number(stat_min)).add("overlay_min",number(dyn_min))
      .add("combined_min",number(combined_min)).add("static_violation_points",stat_bad)
      .add("overlay_violation_points",dyn_bad).add("overlay_query_hits",overlay_used)
      .add("raw_before",raw_before).add("raw_after",raw_after)
      .add("esdf_before",esdf_before).add("esdf_after",esdf_after)
      .add("dynamic_probe_before",dyn_before.sequence).add("dynamic_probe_after",dyn_after.sequence)
      .add("generation_ids_stable",stable?"true":"false")
      .str("point_columns","x,y,z,esdf_distance,overlay_distance_or_null,combined_distance")
      .add("point_distances",rows.str()));
  }catch(const std::exception&e){dual_probe::emit("target_probe_error",dual_probe::Fields().str("reason",e.what()));}
  catch(...){dual_probe::emit("target_probe_error",dual_probe::Fields().str("reason","unknown diagnostic exception"));}
}
}
