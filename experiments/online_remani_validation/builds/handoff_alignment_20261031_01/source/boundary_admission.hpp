#pragma once
// Isolated static ACTUAL FIXED backend-boundary admission. This is NOT a new safety
// certificate, optimizer constraint, fixed-p0 prediction, or controller policy.
#include "coupling_collision.hpp"
#include <Eigen/Core>
#include <limits>
#include <string>
#ifndef BOUNDARY_ADMISSION_NUMERICAL_ONLY
#include "state_contract_trace.hpp"
#endif

namespace boundary_admission {
// Reuse the EXISTING frozen coupling_bridge.hpp static acceptance tolerance.
// This is not a new margin relaxation: native rejection is strictly gap<-1e-8.
constexpr double same_native_gap_tolerance=1e-8;
enum class Category { invalid, full_margin, native_tolerance, margin_only, physical_collision };
inline const char* name(Category category) {
  switch(category) {
    case Category::full_margin:return "full_margin";
    case Category::native_tolerance:return "native_tolerance";
    case Category::margin_only:return "margin_only";
    case Category::physical_collision:return "physical_collision";
    default:return "invalid";
  }
}
inline double unknown() { return std::numeric_limits<double>::quiet_NaN(); }
inline Category classifyStaticClearance(double physical,double gap) {
  if(!std::isfinite(physical)||!std::isfinite(gap))return Category::invalid;
  if(physical<=0.)return Category::physical_collision;
  if(gap<-same_native_gap_tolerance)return Category::margin_only;
  if(gap<0.)return Category::native_tolerance;
  return Category::full_margin;
}
struct Evaluation {
  Category category=Category::invalid;
  std::string reason="missing_frame";
  bool valid=false,frame_fresh=false,static_obstacles_empty=false;
  bool nonzero_gear=false,heading_consistent=false;
  double physical=unknown(),gap=unknown(),velocity_body_yaw=unknown();
  coupling_collision::Evaluation geometry=coupling_collision::invalid();
};
struct Context {
  bool treatment=false,static_scene_declared=false;
  bool local_execution_owner=false,have_local_traj=false,dynamic_repair=false;
  bool exact_processed_measurement=false,request_scope_active=false;
  bool frozen_scope_valid=false;
  bool actual_fixed_heading_known=false,heading_matches_measurement=false;
};
struct Decision {
  bool eligible=false,would_defer=false,defer=false;
  std::string reason="invalid";
};
struct FixedHead {
  bool valid=false,heading_known=false;
  double x=unknown(),y=unknown(),vx=unknown(),vy=unknown(),yaw=unknown();
  int gear=0;
};
inline bool sameHeading(double left,double right) {
  if(!std::isfinite(left)||!std::isfinite(right))return false;
  const double delta=left-right;
  return std::cos(delta)>0.&&std::abs(std::sin(delta))<=64.*std::numeric_limits<double>::epsilon();
}
inline FixedHead fixedHead(const Eigen::MatrixXd& matrix,int gear) {
  FixedHead result;result.gear=gear;
  if(matrix.rows()<2||matrix.cols()!=4||!matrix.allFinite()||(gear!=-1&&gear!=0&&gear!=1))return result;
  result.valid=true;result.x=matrix(0,0);result.y=matrix(1,0);
  result.vx=matrix(0,1);result.vy=matrix(1,1);
  const double speed=Eigen::Vector2d(result.vx,result.vy).norm();
  if((gear==-1||gear==1)&&std::isfinite(speed)&&speed>0.) {
    result.yaw=std::atan2(gear*result.vy,gear*result.vx);result.heading_known=true;
  }
  return result;
}
inline bool exactBaseState(const FixedHead& actual,bool measurement_valid,
                          double x,double y,double vx,double vy,int gear) {
  return actual.valid&&measurement_valid&&actual.x==x&&actual.y==y&&
    actual.vx==vx&&actual.vy==vy&&actual.gear==gear;
}

// Physical==0 remains collision. Native accepts gap>=-1e-8, including negative
// numerical residuals: capture those separately, never defer them. The tiny
// heading tolerance covers floating arithmetic
// only (64 machine eps in sin(delta)); it is not a geometric safety tolerance.
inline Evaluation evaluate(const shared_collision::Snapshot* frame,double now,
                           double x,double y,double yaw,double vx,double vy,int gear) {
  Evaluation result;
  if(!std::isfinite(now)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||
     !std::isfinite(vx)||!std::isfinite(vy)||(gear!=-1&&gear!=0&&gear!=1)) {
    result.reason="nonfinite_or_invalid_actual_fixed_state";return result;
  }
  if(!frame)return result;
  result.static_obstacles_empty=frame->obstacles.empty();
  result.frame_fresh=std::isfinite(frame->stamp)&&std::isfinite(frame->max_age)&&
    frame->max_age>0.&&shared_collision::fresh(*frame,now);
  if(!result.frame_fresh){result.reason="stale_or_unhealthy_frame";return result;}
  const double speed=Eigen::Vector2d(vx,vy).norm();
  if(!std::isfinite(speed)){result.reason="nonfinite_actual_fixed_speed";return result;}
  result.nonzero_gear=gear==1||gear==-1;
  if(result.nonzero_gear && speed>0.) {
    result.velocity_body_yaw=std::atan2(gear*vy,gear*vx);
    result.heading_consistent=sameHeading(result.velocity_body_yaw,yaw);
  }
  result.geometry=coupling_collision::evaluate(*frame,x,y,yaw,0.,{true,false,0.});
  if(!result.geometry.valid||!std::isfinite(result.geometry.physical)||
     !std::isfinite(result.geometry.gap)) {
    result.reason="invalid_projected_static_geometry";return result;
  }
  result.valid=true;result.physical=result.geometry.physical;result.gap=result.geometry.gap;
  result.category=classifyStaticClearance(result.physical,result.gap);
  result.reason=name(result.category);
  return result;
}

inline Decision decide(const Evaluation& evaluation,const Context& context) {
  Decision result;
  if(!evaluation.valid){result.reason=evaluation.reason;return result;}
  if(!context.request_scope_active){result.reason="missing_deferred_request_scope";return result;}
  if(!context.frozen_scope_valid){result.reason="invalid_frozen_scope";return result;}
  if(!context.static_scene_declared){result.reason="static_scene_not_declared";return result;}
  if(!evaluation.static_obstacles_empty){result.reason="obstacle_records_exclude_static_trial";return result;}
  if(context.dynamic_repair){result.reason="dynamic_repair_excluded";return result;}
  if(!context.local_execution_owner||!context.have_local_traj) {
    result.reason="not_cooperative_measured_refresh";return result;
  }
  if(!context.actual_fixed_heading_known){result.reason="unknown_actual_fixed_heading";return result;}
  if(!context.exact_processed_measurement) {
    result.reason="actual_fixed_not_exact_current_processed_measurement";return result;
  }
  if(!context.heading_matches_measurement){result.reason="actual_fixed_heading_disagrees_measurement";return result;}
  if(!evaluation.nonzero_gear){result.reason="unknown_fixed_heading";return result;}
  if(!evaluation.heading_consistent){result.reason="actual_fixed_heading_disagrees_gear_velocity";return result;}
  if(evaluation.category==Category::physical_collision) {
    result.reason="physical_collision_not_margin_recovery";return result;
  }
  if(evaluation.category==Category::full_margin){result.reason="already_full_margin";return result;}
  if(evaluation.category==Category::native_tolerance||!(evaluation.gap<-same_native_gap_tolerance)) {
    result.reason="native_gap_tolerance_accepted";return result;
  }
  result.eligible=true;result.would_defer=true;result.defer=context.treatment;
  result.reason=result.defer?"defer_static_margin_only":"off_would_defer_static_margin_only";
  return result;
}

#ifndef BOUNDARY_ADMISSION_NUMERICAL_ONLY
inline bool enabled() { return state_contract_trace::flag("BOUNDARY_ADMISSION"); }
inline bool staticSceneDeclared() { return state_contract_trace::flag("BOUNDARY_ADMISSION_STATIC_SCENE"); }
inline void initialize(ros::NodeHandle&) {
  if(!enabled())return;
  bool sil=false,sim=false;
  ros::param::param("/online_validation/enabled",sil,false);
  ros::param::param("/use_sim_time",sim,false);
  if(!state_contract_trace::enabled()||!coupling_geometry::enabled()||!staticSceneDeclared()||!sil||!sim)
    throw std::runtime_error("BOUNDARY_ADMISSION requires explicit capture, engineering, declared static scene and isolated SIL");
}

class RequestContext;
inline const RequestContext*& currentRequest() { static thread_local const RequestContext* value=nullptr;return value; }
class RequestContext {
  const RequestContext* previous_=nullptr;bool active_=false;
 public:
  bool local_owner=false,have_local=false,dynamic_repair=false;
  std::shared_ptr<const shared_collision::Snapshot> outer_frame;
  double outer_frame_now=unknown();
  template<class Scope>
  RequestContext(bool local,bool have,bool dynamic,const Scope& scope) {
    if(!(active_=state_contract_trace::enabled()))return;
    previous_=currentRequest();local_owner=local;have_local=have;dynamic_repair=dynamic;
    outer_frame=scope.snapshot();outer_frame_now=scope.now();currentRequest()=this;
  }
  RequestContext(const RequestContext&)=delete;
  RequestContext& operator=(const RequestContext&)=delete;
  ~RequestContext(){if(active_)currentRequest()=previous_;}
};

template<class Left,class Right>
inline bool exactVector(const Left& left,const Right& right) {
  if(left.size()!=right.size())return false;
  for(Eigen::Index i=0;i<left.size();++i)if(left(i)!=right(i))return false;
  return true;
}

// Manager hook AFTER frontend/MINCO/warm/prefix/RRT have produced the actual
// fixed iniStates/gear arrays, BEFORE optimizer stage_begin or Optimize call.
// No requested-gear assumption is used: first gear and p/v are observed here.
// Nested Scope retains the SAME outer request frame (verified by identity/time).
template<class States,class P,class V,class A,class J>
inline Decision actualPrecheck(const States& initial,const std::vector<int>& signs,
    const P& desired_p,const V& desired_v,const A& desired_a,const J& desired_j,
    double desired_yaw,int desired_gear,double desired_epoch,
    double frontend_wall_ms,double frontend_ros_ms,int frontend_failure_count) {
  Decision decision;
  if(!state_contract_trace::enabled())return decision;
  coupling_geometry::Scope scope;
  const auto* request=currentRequest();
  const auto raw=state_contract::snapshot();const auto frame=scope.snapshot();
  const double check_now=ros::Time::now().toSec();
  const Eigen::MatrixXd head=initial.empty()?Eigen::MatrixXd():initial.front();
  const int gear=signs.empty()?0:signs.front();
  const auto actual=fixedHead(head,gear);
  Evaluation evaluation;
  if(initial.size()!=signs.size()||!actual.valid||head.rows()!=desired_p.size()||
     head.rows()!=desired_v.size()||head.rows()!=desired_a.size()||head.rows()!=desired_j.size())
    evaluation.reason="invalid_actual_fixed_head";
  else if(!actual.heading_known)evaluation.reason="unknown_actual_fixed_heading";
  else evaluation=evaluate(frame.get(),check_now,actual.x,actual.y,actual.yaw,actual.vx,actual.vy,gear);
  const bool same_outer_frame=request&&frame==request->outer_frame&&scope.now()==request->outer_frame_now;
  const bool measurement_valid=raw.received&&raw.raw_finite&&raw.seq!=0&&!raw.sequence_exhausted&&
    raw.processed_ready&&raw.processed_finite&&raw.processed_seq==raw.seq;
  Context context;
  context.treatment=enabled();context.static_scene_declared=staticSceneDeclared();
  context.local_execution_owner=request&&request->local_owner;context.have_local_traj=request&&request->have_local;
  context.dynamic_repair=request&&request->dynamic_repair;
  context.request_scope_active=request&&state_contract_trace::current()!=nullptr;
  context.frozen_scope_valid=scope.valid()&&same_outer_frame;
  context.actual_fixed_heading_known=actual.heading_known;
  context.exact_processed_measurement=exactBaseState(actual,measurement_valid,
    raw.x,raw.y,raw.processed_vx,raw.processed_vy,raw.processed_gear);
  context.heading_matches_measurement=actual.heading_known&&sameHeading(actual.yaw,raw.yaw);
  decision=decide(evaluation,context);
  const Eigen::VectorXd p=actual.valid?Eigen::VectorXd(head.col(0)):Eigen::VectorXd();
  const Eigen::VectorXd v=actual.valid?Eigen::VectorXd(head.col(1)):Eigen::VectorXd();
  const Eigen::VectorXd a=actual.valid?Eigen::VectorXd(head.col(2)):Eigen::VectorXd();
  const Eigen::VectorXd j=actual.valid?Eigen::VectorXd(head.col(3)):Eigen::VectorXd();
  auto fields=state_contract::boundaryFields(raw,p,v,actual.yaw,gear,desired_epoch);
  fields.str("boundary_admission_schema","boundary_admission_v2_actual_fixed")
    .str("admission_phase","after_frontend_before_optimizer")
    .add("planner_a",state_contract::vectorJson(a)).add("planner_j",state_contract::vectorJson(j))
    .add("actual_head_state",state_contract_trace::matrixJson(head))
    .add("initial_states",state_contract_trace::matricesJson(initial)).add("gears",state_contract_trace::signsJson(signs))
    .add("actual_first_gear",gear).add("actual_fixed_heading_known",actual.heading_known?"true":"false")
    .add("desired_p",state_contract::vectorJson(desired_p)).add("desired_v",state_contract::vectorJson(desired_v))
    .add("desired_a",state_contract::vectorJson(desired_a)).add("desired_j",state_contract::vectorJson(desired_j))
    .add("desired_gear",desired_gear)
    .add("frontend_failure_count_observed",frontend_failure_count)
    .add("actual_p_equals_desired",exactVector(p,desired_p)?"true":"false")
    .add("actual_v_equals_desired",exactVector(v,desired_v)?"true":"false")
    .add("actual_gear_equals_desired",gear==desired_gear?"true":"false")
    .add("actual_heading_matches_desired",actual.heading_known&&sameHeading(actual.yaw,desired_yaw)?"true":"false")
    .add("enabled",context.treatment?"true":"false")
    .add("static_scene_declared",context.static_scene_declared?"true":"false")
    .add("frame_obstacles_empty",evaluation.static_obstacles_empty?"true":"false")
    .add("frame_fresh_at_precheck",evaluation.frame_fresh?"true":"false")
    .add("scope_valid_at_freeze",scope.valid()?"true":"false")
    .add("frozen_scope_valid_for_admission",context.frozen_scope_valid?"true":"false")
    .add("same_outer_frozen_frame",same_outer_frame?"true":"false")
    .add("local_execution_owner",context.local_execution_owner?"true":"false")
    .add("have_local_traj",context.have_local_traj?"true":"false").add("dynamic_repair",context.dynamic_repair?"true":"false")
    .add("exact_processed_measurement",context.exact_processed_measurement?"true":"false")
    .str("exact_processed_measurement_scope","actual base xy/world vxvy/gear equal same-sequence raw/processed snapshot")
    .add("heading_matches_measurement",context.heading_matches_measurement?"true":"false")
    .add("nonzero_gear",gear==1||gear==-1?"true":"false")
    .add("heading_consistent",actual.heading_known?"true":"false")
    .str("gear_heading_status",actual.heading_known?"observed_actual_fixed_head_gear_velocity":"unknown_actual_fixed_heading")
    .add("final_backend_fixed_yaw_known",actual.heading_known?"true":"false")
    .str("fixed_heading_source","observed first iniStates_container head and singul_container gear after all frontend paths; atan2(actual_gear*actual_vy,actual_gear*actual_vx)")
    .str("classification",name(evaluation.category)).str("reason",decision.reason)
    .add("same_native_gap_tolerance",same_native_gap_tolerance)
    .str("native_gap_reject_rule","strict gap < -same_native_gap_tolerance; same frozen coupling_bridge validator")
    .add("native_gap_rejects_actual_fixed",evaluation.valid&&evaluation.gap<-same_native_gap_tolerance?"true":"false")
    .add("evaluation_valid",evaluation.valid?"true":"false")
    .str("skip_reason",decision.defer?decision.reason:"not_skipped")
    .add("eligible",decision.eligible?"true":"false").add("would_defer",decision.would_defer?"true":"false")
    .add("deferred",decision.defer?"true":"false")
    .str("action",decision.defer?"return_manager_false_before_optimizer":"continue_native_unchanged")
    .str("controller_policy","unchanged independently validated MPPI rules; no new zero-speed or reference publication")
    .str("scope","actual fixed backend first boundary; frozen projected static field only; not native 3D map replay or replacement candidate authorization");
  state_contract::number(fields,"physical",evaluation.physical);
  state_contract::number(fields,"gap",evaluation.gap);
  state_contract::number(fields,"velocity_body_yaw",evaluation.velocity_body_yaw);
  state_contract::number(fields,"actual_fixed_yaw",actual.yaw);
  state_contract::number(fields,"desired_yaw",desired_yaw);
  state_contract::number(fields,"frontend_wall_ms",frontend_wall_ms);
  state_contract::number(fields,"frontend_ros_ms",frontend_ros_ms);
  state_contract::number(fields,"frame_scope_now",scope.now());
  state_contract::number(fields,"precheck_now",check_now);
  state_contract::number(fields,"raw_receipt_age_at_precheck",raw.received?check_now-raw.receipt_ros.toSec():unknown());
  state_contract::number(fields,"raw_source_stamp_age_at_precheck",raw.received?check_now-raw.header_stamp.toSec():unknown());
  state_contract::number(fields,"frame_sequence",frame?frame->sequence:unknown());
  state_contract::number(fields,"frame_stamp",frame?frame->stamp:unknown());
  fields.add("actual_fixed_static",delivery_diag::evaluationJson(evaluation.geometry));
  if(frame&&raw.received&&raw.raw_finite)
    fields.add("raw_static",delivery_diag::evaluationJson(coupling_collision::evaluate(
      *frame,raw.x,raw.y,raw.yaw,0.,{true,false,0.})));
  else fields.add("raw_static","{\"valid\":false,\"reason\":\"missing_raw_or_frame\"}");
  auto record=state_contract_trace::record("boundary_admission_precheck",fields);
  const double frame_now=scope.now();
  record.postsolve=[frame,frame_now](dual_probe::Fields& output){
    output.add("precheck_frame",delivery_diag::directory()?
      delivery_diag::frameJson({frame,frame_now}):
      "{\"now\":"+delivery_diag::number(frame_now)+",\"snapshot\":null}");
    output.add("precheck_frame_saved",delivery_diag::directory()&&frame?"true":"false");
  };
  state_contract_trace::enqueue(std::move(record)); // No I/O; Request destructor flushes after decision.
  return decision;
}
#endif
} // namespace boundary_admission
