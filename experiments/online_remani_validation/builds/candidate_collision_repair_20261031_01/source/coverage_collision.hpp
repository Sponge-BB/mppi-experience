#pragma once
// Isolated SIL experiment: the same binary compares the previous 1.2 s
// time-indexed branch with a complete frozen CV validity window. Geometry,
// static/self checks and full/half execution scales stay fixed. Full coverage
// uses the native continuous CV uncertainty; legacy retains its swept-guide
// extra allowance and cap, needed for reproducing the preceding experiment.
#include "dual_timescale_probe.hpp"
#include "temporal_collision_source.hpp"
#include "plan_env/grid_map.h"
#include <Eigen/Dense>
#include <array>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <vector>

namespace time_indexed_collision {
inline bool enabled(){const char* p=std::getenv("TIME_INDEXED_COLLISION");return p&&std::string(p)=="1";}
inline bool fullCoverage(){const char* p=std::getenv("COVERAGE_ALIGNMENT");return p&&std::string(p)=="1";}
struct Parameters {
  double horizon=0.,step=0.,extra=0.,cap=0.,influence=0.,max_prediction=0.;
  double uncertainty0=0.,uncertainty_v=0.,uncertainty_a=0.;
};
struct Source {
  plan_env::DynamicPointSnapshot snapshot;
  Parameters parameters;double refresh_epoch=0.;bool exact=false,overlay_enabled=false;
};
}

// No map layout changes. Registry and overlay are read under the same overlay
// mutex/order as publication, so the captured source cannot be one generation
// newer than the KD-tree used by the A branch or the B fallback.
class TimeIndexedCollisionMapAccessor {
 public:
  static time_indexed_collision::Source source(GridMap& map){
    using namespace time_indexed_collision;
    Source out;std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    out.overlay_enabled=map.mp_.dynamic_point_overlay_enable_;
    auto source=temporal_capture_source::publishedSource(&map);
    if(source && source->snapshot.sequence==map.dynamic_point_overlay_sequence_ &&
       std::abs(source->refresh_epoch.toSec()-map.dynamic_point_overlay_refresh_time_.toSec())<1e-9){
      out.snapshot=source->snapshot;out.refresh_epoch=source->refresh_epoch.toSec();out.exact=true;
    }
    auto& p=out.parameters;
    p.horizon=map.mp_.dynamic_point_overlay_horizon_;
    p.step=map.mp_.dynamic_point_overlay_time_step_;
    p.extra=map.mp_.dynamic_point_overlay_extra_inflation_;
    p.cap=map.mp_.dynamic_point_overlay_max_inflation_;
    p.influence=map.mp_.dynamic_point_overlay_influence_distance_;
    p.max_prediction=map.dynamic_point_buffer_.maxPredictionTime();
    p.uncertainty0=map.dynamic_point_buffer_.configuredPredictionUncertainty(0.);
    const double u1=map.dynamic_point_buffer_.configuredPredictionUncertainty(1.);
    const double u2=map.dynamic_point_buffer_.configuredPredictionUncertainty(2.);
    p.uncertainty_a=u2-2.*u1+p.uncertainty0;
    p.uncertainty_v=u1-p.uncertainty0-.5*p.uncertainty_a;
    return out;
  }
};

namespace time_indexed_collision {
struct Context {
  GridMap* map=nullptr;Source source;
  double requested_epoch=0.,entry_epoch=0.,begin_epoch=0.,end_epoch=0.,edge_radius=0.;
  bool frozen=false,active=false,full_coverage=false;
  unsigned long long indexed_queries=0,fallback_queries=0,hard_checks=0,hard_failures=0;
  unsigned long long valid_branches=0,before_source_branches=0,unknown_future_branches=0;
  std::vector<Eigen::VectorXd>* gradients=nullptr;
  const std::vector<Eigen::VectorXd>* durations=nullptr;
};
inline Context*& current(){static thread_local Context* p=nullptr;return p;}
class Request {
  Context value_;Context* previous_;
 public:
  Request(GridMap& map,double epoch):previous_(current()){
    value_.map=&map;value_.requested_epoch=epoch;current()=&value_;
  }
  ~Request(){
    if(value_.frozen)dual_probe::emit("time_indexed_collision_summary",dual_probe::Fields()
      .add("enabled",enabled()?"true":"false").add("active",value_.active?"true":"false")
      .add("requested_epoch",value_.requested_epoch).add("optimizer_entry_epoch",value_.entry_epoch)
      .add("source_sequence",value_.source.snapshot.sequence)
      .add("indexed_queries",value_.indexed_queries).add("swept_fallback_queries",value_.fallback_queries)
      .add("hard_checks",value_.hard_checks).add("hard_failures",value_.hard_failures)
      .add("full_coverage",value_.full_coverage?"true":"false")
      .add("valid_branches",value_.valid_branches)
      .add("before_source_branches",value_.before_source_branches)
      .add("history_skipped_branches",value_.full_coverage?value_.before_source_branches:0)
      .add("unknown_future_branches",value_.unknown_future_branches)
      .str("scope","planning objective, native gate and same-request prepublication planning gate; latest CV acceptance unchanged"));
    current()=previous_;
  }
};
inline void freeze(){
  auto* c=current();if(!c||c->frozen)return;c->frozen=true;
  c->full_coverage=fullCoverage();
  c->entry_epoch=ros::Time::now().toSec();c->source=TimeIndexedCollisionMapAccessor::source(*c->map);
  const auto& s=c->source;const auto& p=s.parameters;
  const double age=std::max(0.,s.refresh_epoch-s.snapshot.stamp.toSec());
  const double horizon=c->full_coverage?std::max(0.,p.max_prediction-age):
    std::min(p.horizon,std::max(0.,p.max_prediction-age));
  c->begin_epoch=c->full_coverage?s.snapshot.stamp.toSec():s.refresh_epoch;
  c->end_epoch=c->full_coverage?s.snapshot.stamp.toSec()+p.max_prediction:s.refresh_epoch+horizon;
  double vmax=0.;for(const auto& q:s.snapshot.points)vmax=std::max(vmax,q.velocity.norm());
  c->edge_radius=p.extra+.5*vmax*std::min(horizon,p.step);
  c->active=enabled()&&s.exact&&s.overlay_enabled&&s.snapshot.valid&&!s.snapshot.points.empty();
  dual_probe::emit("time_indexed_collision_scope",dual_probe::Fields()
    .add("enabled",enabled()?"true":"false").add("active",c->active?"true":"false")
    .add("source_exact",s.exact?"true":"false").add("source_sequence",s.snapshot.sequence)
    .add("source_stamp",s.snapshot.stamp.toSec()).add("source_refresh_epoch",s.refresh_epoch)
    .add("requested_epoch",c->requested_epoch).add("optimizer_entry_epoch",c->entry_epoch)
    .add("indexed_begin_epoch",c->begin_epoch).add("indexed_end_epoch",c->end_epoch)
    .add("edge_allowance_m",c->edge_radius).add("guide_radius_cap_m",p.cap)
    .add("full_coverage",c->full_coverage?"true":"false")
    .add("maximum_prediction_s",p.max_prediction).add("guide_horizon_s",p.horizon)
    .str("execution_scales","1,0.5")
    .str("outside_horizon",c->full_coverage?"static only; historical samples skipped, future unknown":"original swept query")
    .str("dynamic_radius",c->full_coverage?"native continuous CV uncertainty, no guide allowance or cap":"legacy swept guide allowance and cap")
    .str("epoch_semantics","explicit FSM requested epoch; initial/guarded eventual rebasing remains checked by original latest gate"));
}
inline double requestedEpoch(){return current()?current()->requested_epoch:0.;}
inline bool active(){return current()&&current()->active;}

struct Branch {
  bool valid=false;double multiplier=1.,dt=0.,radius=0.,radius_derivative=0.;
  plan_env::PredictedPositionVector points;
};
struct SampleState {
  std::array<Branch,2> branches;
  double temporal_cost_gradient=0.,last_distance_gradient=0.;
};
inline SampleState*& sample(){static thread_local SampleState* p=nullptr;return p;}

// Fixed point positions/velocities and source epoch; only the candidate's
// optimized cumulative duration changes the prediction time within a solve.
inline Branch branch(const Context& c,double elapsed,double multiplier){
  Branch out;out.multiplier=multiplier;
  const double epoch=c.requested_epoch+multiplier*elapsed;
  const double raw_dt=epoch-c.source.snapshot.stamp.toSec();
  if(c.full_coverage){
    // Match DynamicPointBuffer::predictPosition's exact validity tolerance
    // and clamp, including a sample that lands at the finite-window edge.
    if(raw_dt < -1e-6||raw_dt > c.source.parameters.max_prediction+1e-6)return out;
    out.dt=std::max(0.,std::min(c.source.parameters.max_prediction,raw_dt));
    if(out.dt!=raw_dt)out.multiplier=0.;
  }else{
    if(epoch<c.begin_epoch-1e-9||epoch>c.end_epoch+1e-9)return out;
    out.dt=raw_dt;if(out.dt<0.||out.dt>c.source.parameters.max_prediction)return out;
  }
  const auto& p=c.source.parameters;
  const double unc=p.uncertainty0+p.uncertainty_v*out.dt+.5*p.uncertainty_a*out.dt*out.dt;
  const double raw=unc+c.edge_radius;
  out.radius=c.full_coverage?unc:std::min(raw,p.cap);
  out.radius_derivative=(c.full_coverage||raw<p.cap)?
    (p.uncertainty_v+p.uncertainty_a*out.dt)*out.multiplier:0.;
  out.points.reserve(c.source.snapshot.points.size());
  for(const auto& q:c.source.snapshot.points)out.points.push_back(q.position+out.dt*q.velocity);
  out.valid=true;return out;
}
class Sample {
  SampleState value_;SampleState* previous_;
 public:
  explicit Sample(double elapsed):previous_(sample()){
    if(active()){value_.branches[0]=branch(*current(),elapsed,1.);
      value_.branches[1]=branch(*current(),elapsed,2.);
      for(const auto& b:value_.branches){
        if(b.valid)++current()->valid_branches;
        else if(current()->requested_epoch+b.multiplier*elapsed<current()->begin_epoch)
          ++current()->before_source_branches;
        else ++current()->unknown_future_branches;
      }}
    sample()=&value_;
  }
  ~Sample(){sample()=previous_;}
};

inline bool indexedDistance(Context& c,SampleState& s,const Eigen::Vector3d& pos,
                            double& distance,Eigen::Vector3d& gradient){
  s.last_distance_gradient=0.;
  bool fallback=false;
  for(const auto& b:s.branches){
    if(!b.valid){if(!c.full_coverage)fallback=true;continue;}
    ++c.indexed_queries;
    double squared=std::numeric_limits<double>::infinity();std::size_t index=0;
    for(std::size_t i=0;i<b.points.size();++i){const double d=(pos-b.points[i]).squaredNorm();
      if(d<squared){squared=d;index=i;}}
    const double geometric=std::sqrt(squared),d=geometric-b.radius;
    if(d<distance&&d<=c.source.parameters.influence){
      const Eigen::Vector3d unit=geometric>1e-10?Eigen::Vector3d((pos-b.points[index])/geometric):Eigen::Vector3d::UnitX();
      distance=d;gradient=unit;
      s.last_distance_gradient=-unit.dot(c.source.snapshot.points[index].velocity)*b.multiplier-b.radius_derivative;
    }
  }
  return fallback;
}
inline void evaluate(GridMap& map,const Eigen::Vector3d& pos,double& distance,Eigen::Vector3d& gradient){
  auto* c=current();auto* s=sample();
  if(!active()||!s){map.evaluatePlanningEDTWithGrad(pos,distance,gradient);return;}
  map.evaluateEDTWithGrad(pos,distance,gradient);
  const bool fallback=indexedDistance(*c,*s,pos,distance,gradient);
  if(fallback){
    ++c->fallback_queries;double d=distance;Eigen::Vector3d grad=Eigen::Vector3d::Zero();
    if(map.getDynamicPointPlanningDistance(pos,std::max(0.,distance),d,&grad)&&d<distance){
      distance=d;gradient=grad.squaredNorm()>1e-12?grad:Eigen::Vector3d::UnitX();s->last_distance_gradient=0.;}
  }
}
inline void penaltyTimeDerivative(double derivative_wrt_distance){
  if(active()&&sample())sample()->temporal_cost_gradient+=derivative_wrt_distance*sample()->last_distance_gradient;
}
struct Durations {
  Context* c;std::vector<Eigen::VectorXd>* old_gradients;const std::vector<Eigen::VectorXd>* old_durations;
  Durations(const std::vector<Eigen::VectorXd>& times,std::vector<Eigen::VectorXd>& gradients):c(current()),old_gradients(c?c->gradients:nullptr),old_durations(c?c->durations:nullptr){
    if(c){c->gradients=&gradients;c->durations=&times;}}
  ~Durations(){if(c){c->gradients=old_gradients;c->durations=old_durations;}}
};
inline double elapsedAt(int trajectory,int piece,double local){
  if(!active()||!current()->durations)return 0.;double elapsed=local;
  const auto& times=*current()->durations;
  for(int n=0;n<trajectory;++n)elapsed+=times[n].sum();
  if(piece)elapsed+=times[trajectory].head(piece).sum();return elapsed;
}
inline void accumulateTimeGradient(int trajectory,int piece,double alpha,double integration_weight){
  auto* c=current();auto* s=sample();if(!active()||!s||!c->gradients)return;
  const double derivative=s->temporal_cost_gradient*integration_weight;
  auto& gradients=*c->gradients;
  for(int n=0;n<trajectory;++n)gradients[n].array()+=derivative;
  if(piece)gradients[trajectory].head(piece).array()+=derivative;
  gradients[trajectory](piece)+=alpha*derivative;
}

inline double hardDistance(GridMap& map,const Eigen::Vector3d& point){
  auto* c=current();auto* s=sample();double distance=map.getPreciseDistance(point);bool fallback=false;
  for(const auto& b:s->branches){
    if(!b.valid){if(!c->full_coverage)fallback=true;continue;}++c->indexed_queries;
    for(const auto& q:b.points)distance=std::min(distance,(point-q).norm()-b.radius);
  }
  if(fallback){++c->fallback_queries;double d=distance;
    if(map.getDynamicPointPlanningDistance(point,std::max(0.,distance),d,nullptr))distance=std::min(distance,d);}
  return distance;
}
template<class Model,class Trajectory>
bool hardCollision(GridMap& map,Model& model,const Trajectory& traj,double t,int& kind,double arm_radius){
  const auto p=traj.getPos(t);const double yaw=traj.getCarAngle(t);
  const Eigen::Vector3d pose(p(0),p(1),yaw);const Eigen::VectorXd arm=p.tail(model.getManiDof());
  // Exact native static/ground and both self-collision predicates.
  if(model.checkcollision(pose,arm,false,kind))return true;
  Sample time(t);auto* c=current();++c->hard_checks;
  std::vector<Eigen::Vector3d> points;model.getCarPts(pose,points);
  for(std::size_t i=0;i<points.size();++i)if(hardDistance(map,points[i])<model.getBaseSphereRadius(i)+model.getMapResolution()){
    kind=0;++c->hard_failures;return true;}
  std::vector<Eigen::Matrix4d> joints,unused;model.getJointTrans(arm,joints,unused);
  Eigen::Matrix4d base=Eigen::Matrix4d::Identity();
  base(0,0)=std::cos(yaw);base(0,1)=-std::sin(yaw);base(1,0)=std::sin(yaw);base(1,1)=std::cos(yaw);base(0,3)=p(0);base(1,3)=p(1);
  Eigen::Matrix4d transform=base*model.getTq0();const auto links=model.getLinkPoint();
  for(int i=0;i<model.getManiDof();++i){transform*=joints[i];
    for(int j=0;j<links[i].cols();++j){const Eigen::Vector3d point=(transform*links[i].col(j)).template head<3>();
      if(hardDistance(map,point)<arm_radius){kind=1;++c->hard_failures;return true;}}}
  kind=-1;return false;
}
} // namespace time_indexed_collision
