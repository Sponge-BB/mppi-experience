#pragma once
// Additive fixed-arm SIL geometry qualification. Original native 3D/static
// and long-horizon dynamic costs/constraints MUST remain in place. This is
// not shared_collision_ros.hpp: enabling it never enables legacy replacement.
#include "coupling_collision.hpp"
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <memory>
#include <stdexcept>

namespace coupling_geometry {
using Snapshot = shared_collision::Snapshot;
using Evaluation = coupling_collision::Evaluation;
inline bool enabled() {
  const char* value=std::getenv("COUPLING_ENGINEERING");
  return value && value[0]=='1' && value[1]=='\0';
}
inline ros::CallbackQueue& inputQueue() { static ros::CallbackQueue q; return q; }
inline ros::Subscriber& subscriber() { static ros::Subscriber s; return s; }
inline std::unique_ptr<ros::AsyncSpinner>& inputSpinner() {
  static std::unique_ptr<ros::AsyncSpinner> spinner; return spinner;
}
inline Eigen::VectorXd& fixedArm() { static Eigen::VectorXd q; return q; }
inline void receive(const std_msgs::Float64MultiArray::ConstPtr& message) {
  if (!shared_collision::bind(message->data.data(),message->data.size())) {
    std::shared_ptr<const Snapshot> empty;
    std::atomic_store(&shared_collision::store(),empty);
  }
}
inline void initialize(ros::NodeHandle& nh) {
  if (!enabled() || inputSpinner()) return;
  bool sil=false,sim=false;
  ros::param::param("/online_validation/enabled",sil,false);
  ros::param::param("/use_sim_time",sim,false);
  if (!sil || !sim) throw std::runtime_error("coupling projection requires isolated SIL");
  std::vector<double> lo,hi;
  nh.getParam("mm/manipulator_min_pos",lo);nh.getParam("mm/manipulator_max_pos",hi);
  if (lo.empty() || lo!=hi ||
      !std::all_of(lo.begin(),lo.end(),[](double q){return std::isfinite(q);}))
    throw std::runtime_error("coupling projection requires exactly locked arm joints");
  fixedArm()=Eigen::Map<const Eigen::VectorXd>(lo.data(),lo.size());
  // Establish destruction order before starting callbacks. Spinner is
  // destroyed/joined before subscriber, queue, arm and the shared storage.
  (void)shared_collision::store();(void)inputQueue();(void)subscriber();
  ros::NodeHandle input_nh(nh);input_nh.setCallbackQueue(&inputQueue());
  subscriber()=input_nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/unified_collision",1,&receive);
  inputSpinner().reset(new ros::AsyncSpinner(1,&inputQueue()));
  inputSpinner()->start();
}
inline bool armMatches(const Eigen::VectorXd& q) {
  return fixedArm().size()>0 && q.size()==fixedArm().size() && q.allFinite() &&
         (q-fixedArm()).cwiseAbs().maxCoeff()<=1e-5;
}
struct FrozenFrame {
  std::shared_ptr<const Snapshot> snapshot;
  double now=0.;
  bool active=false;
};
inline FrozenFrame& activeFrame() { static thread_local FrozenFrame f; return f; }
// Freeze once at the plan boundary, including searches and objective calls.
// Nested guards retain the same immutable frame and acquisition time. A
// separate publication-time hard check must take a NEW scope/latest frame.
class Scope {
  FrozenFrame previous_;
  bool owner_;
public:
  explicit Scope(bool latest=false):previous_(activeFrame()),owner_(latest || !previous_.active) {
    if (owner_) {
      auto& frame=activeFrame();frame.snapshot=shared_collision::snapshot();
      frame.now=ros::Time::now().toSec();frame.active=true;
    }
  }
  Scope(const Scope&)=delete;
  Scope& operator=(const Scope&)=delete;
  ~Scope() { if (owner_) activeFrame()=previous_; }
  bool valid() const {
    const auto& f=activeFrame();
    return f.active && f.snapshot && shared_collision::fresh(*f.snapshot,f.now);
  }
  std::shared_ptr<const Snapshot> snapshot() const { return activeFrame().snapshot; }
  double now() const { return activeFrame().now; }
};
inline std::shared_ptr<const Snapshot> currentSnapshot() {
  return activeFrame().active ? activeFrame().snapshot : shared_collision::snapshot();
}
inline double currentTime() {
  return activeFrame().active ? activeFrame().now : ros::Time::now().toSec();
}
inline bool currentFresh(const std::shared_ptr<const Snapshot>& frame) {
  return frame && shared_collision::fresh(*frame,currentTime());
}

// Hard helpers do NOT taper physical clearance. Use the trajectory's actual
// future sample time, not now for every point. The explicit-snapshot overloads
// are mathematical operations; callers own the frame freshness check.
inline Evaluation staticProjectedPose(const Snapshot& frame,const Eigen::Vector3d& q) {
  coupling_collision::Options options;options.include_dynamic=false;
  return coupling_collision::evaluate(frame,q.x(),q.y(),q.z(),0.,options);
}
inline Evaluation staticProjectedPose(const Eigen::Vector3d& q) {
  const auto frame=currentSnapshot();
  return currentFresh(frame) ? staticProjectedPose(*frame,q) : coupling_collision::invalid();
}
inline Evaluation shortDynamicPose(const Snapshot& frame,const Eigen::Vector3d& q,
                                   double absolute_query_time) {
  const double forecast=absolute_query_time-frame.stamp;
  if (!std::isfinite(forecast) || forecast<0. ||
      !std::isfinite(frame.max_prediction) || forecast>frame.max_prediction)
    return coupling_collision::invalid();  // UNKNOWN, never safe/free/frozen-end.
  coupling_collision::Options options;options.include_static=false;
  options.dynamic_soft_taper_seconds=0.;
  return coupling_collision::evaluate(frame,q.x(),q.y(),q.z(),forecast,options);
}
inline Evaluation shortDynamicPose(const Eigen::Vector3d& q,double absolute_query_time) {
  const auto frame=currentSnapshot();
  return currentFresh(frame) ? shortDynamicPose(*frame,q,absolute_query_time)
                             : coupling_collision::invalid();
}

struct ObjectiveSample {
  double cost=0.,gt=0.;
  Eigen::Vector2d gp=Eigen::Vector2d::Zero(),gv=Eigen::Vector2d::Zero();
  bool valid=false,dynamic_known=false;
  // valid covers only this additive scalar objective. dynamic_known indicates
  // forecast coverage, NOT collision freedom. Neither certifies native
  // long-horizon dynamic feasibility.
};
inline ObjectiveSample projectionCost(const Snapshot& frame,const Eigen::Vector2d& p,
                                     const Eigen::Vector2d& v,int gear,
                                     double absolute_query_time) {
  ObjectiveSample out;
  if (!p.allFinite() || !v.allFinite() || (gear!=-1 && gear!=1) ||
      !std::isfinite(absolute_query_time)) return out;
  // Exact native heading convention; dYaw/dv is gear-independent. At zero
  // speed the original native atan2 convention is not differentiable. This
  // objective uses its bounded native derivative; hard endpoint checks must
  // use their explicit pose yaw, not reconstruct it from zero velocity.
  const double yaw=std::atan2(gear*v.y(),gear*v.x());
  const auto stat=staticProjectedPose(frame,Eigen::Vector3d(p.x(),p.y(),yaw));
  if (!stat.valid) return out;
  double gx=stat.gx,gy=stat.gy,gyaw=stat.gyaw;
  out.cost=stat.cost;out.gt=stat.gt;
  const double forecast=absolute_query_time-frame.stamp;
  out.dynamic_known=std::isfinite(forecast) && forecast>=0. &&
    std::isfinite(frame.max_prediction) && frame.max_prediction>0. &&
    forecast<=frame.max_prediction;
  if (forecast<0. || !std::isfinite(forecast)) return ObjectiveSample{};
  if (out.dynamic_known) {
    coupling_collision::Options options;options.include_static=false;
    options.dynamic_soft_taper_seconds=std::min(.25,frame.max_prediction);
    const auto dyn=coupling_collision::evaluate(frame,p.x(),p.y(),yaw,forecast,options);
    if (!dyn.valid) return ObjectiveSample{};
    out.cost+=dyn.cost;gx+=dyn.gx;gy+=dyn.gy;gyaw+=dyn.gyaw;out.gt+=dyn.gt;
  }
  // Outside the prediction horizon ONLY THE ADDITIONAL DYNAMIC OBJECTIVE is
  // absent. Original native conservative costs and constraints stay active.
  // C2 taper makes this additive objective/gradient continuous at the cutoff;
  // shortDynamicPose still returns UNKNOWN beyond it for hard checks.
  out.gp=Eigen::Vector2d(gx,gy);
  out.gv=gyaw*Eigen::Vector2d(-v.y(),v.x())/std::max(1e-8,v.squaredNorm());
  out.valid=std::isfinite(out.cost) && std::isfinite(out.gt) &&
            out.gp.allFinite() && out.gv.allFinite();
  return out;
}
}  // namespace coupling_geometry
