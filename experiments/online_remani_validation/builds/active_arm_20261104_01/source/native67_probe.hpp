#pragma once
// Read-only diagnostic counters and post-solver audits. No acceptance decisions.
#include <ros/ros.h>
#include <Eigen/Dense>
#include "failure_probe.hpp"
#include "delivery_diagnostics.hpp"
#include <array>
#include <limits>

namespace native67_probe {
struct Counters {
  unsigned long long evaluations=0,completed=0,line_search_trials=0,accepted=0;
  unsigned long long accepted_ls_sum=0;
  double callback_wall=0,callback_cpu=0,solver_wall=0,solver_cpu=0;
  std::array<double,8> first_cost{},last_cost{};bool cost_seen=false;
};
inline Counters*& active(){static thread_local Counters* p=nullptr;return p;}
inline double cpu(){return resource_probe::clockSeconds(CLOCK_THREAD_CPUTIME_ID);}
struct Callback {
  Counters* p=active();double w=p?failure_probe::wall():0,c=p?cpu():0;
  Callback(){if(p)++p->evaluations;}
  ~Callback(){if(p){p->callback_wall+=failure_probe::wall()-w;p->callback_cpu+=cpu()-c;}}
};
inline void trial(){if(active())++active()->line_search_trials;}
inline void completed(double smooth,const Eigen::VectorXd& costs,double time){
  auto*p=active();if(!p)return;++p->completed;
  std::array<double,8> v{};v[0]=smooth;for(int k=0;k<6;++k)v[k+1]=costs[k];v[7]=time;
  if(!p->cost_seen){p->first_cost=v;p->cost_seen=true;}p->last_cost=v;
}
inline std::string costJson(const std::array<double,8>& v){
  const char* names[]={"smooth","base_obstacle","arm_obstacle","self_collision","base_feasibility","arm_feasibility","other_integral","time"};
  std::ostringstream s;s<<'{';for(int i=0;i<8;++i){if(i)s<<',';s<<'"'<<names[i]<<"\":"<<delivery_diag::number(v[i]);}s<<'}';return s.str();
}
inline std::string countersJson(const Counters& c){
  std::ostringstream s;s<<std::setprecision(17)
    <<"\"evaluate_entries\":"<<c.evaluations<<",\"evaluate_completed\":"<<c.completed
    <<",\"evaluate_early_or_exception\":"<<(c.evaluations-c.completed)
    <<",\"line_search_trial_evaluations\":"<<c.line_search_trials
    <<",\"accepted_iterations\":"<<c.accepted<<",\"accepted_line_search_evaluations_sum\":"<<c.accepted_ls_sum
    <<",\"callback_wall_ms\":"<<c.callback_wall*1000<<",\"callback_thread_cpu_ms\":"<<c.callback_cpu*1000
    <<",\"solver_wall_ms\":"<<c.solver_wall*1000<<",\"solver_thread_cpu_ms\":"<<c.solver_cpu*1000
    <<",\"first_cost_components\":"<<costJson(c.first_cost)<<",\"last_completed_cost_components\":"<<costJson(c.last_cost)
    <<",\"counter_definitions\":\"evaluate_entries includes initial and all line-search trials, even timeout/exception; completed excludes early exits; accepted_iterations counts successful line-search returns before progress cancellation; no residual audit is timed as optimization\"";
  if(auto*f=failure_probe::active()){
    s<<",\"profile_wall_ms_inclusive\":{";
    for(int i=0;i<failure_probe::COUNT;++i){if(i)s<<',';s<<'"'<<failure_probe::partName(i)<<"\":"<<f->ms[i];}s<<'}';
  }
  return s.str();
}

// The same sample grid and algebra as IsTrajSafe/IsNotFeasibie. Full external
// sphere gaps have no early collision return. They are NOT continuous proof.
template<class Trajectory,class Model,class Map>
std::string residuals(const Trajectory& traj,Model& model,Map& map,double wheel_base,
    double wheel_radius,double wheel_limit,double alpha_limit,double arm_radius,double ground){
  const double began=failure_probe::wall();
  double max_l=0,max_r=0,max_al=0,max_ar=0;
  double base_gap=std::numeric_limits<double>::infinity(),arm_gap=base_gap,ground_gap=base_gap;
  int samples=0;std::array<int,4> hits{};
  const auto links=model.getLinkPoint();
  const int end=int(std::floor(traj.duration/.01));double t=.05;
  for(int k=5;k<end;++k,t+=.01){
    ++samples;const auto p=traj.getPos(t);
    const Eigen::Vector2d v=traj.getVel(t).head(2),a=traj.getAcc(t).head(2),j=traj.getJer(t).head(2);
    const double v2=std::max(1e-8,v.squaredNorm()),speed=std::max(1e-4,v.norm());
    const double cross=v.x()*a.y()-v.y()*a.x(),omega=cross/v2;
    const double alpha=(v.x()*j.y()-v.y()*j.x())/v2-2*cross*a.dot(v)/(v2*v2);
    const double sv=traj.getSingul(t)*speed,sa=traj.getSingul(t)*a.dot(v)/speed;
    max_l=std::max(max_l,std::abs((sv-.5*wheel_base*omega)/wheel_radius));
    max_r=std::max(max_r,std::abs((sv+.5*wheel_base*omega)/wheel_radius));
    max_al=std::max(max_al,std::abs((sa-.5*wheel_base*alpha)/wheel_radius));
    max_ar=std::max(max_ar,std::abs((sa+.5*wheel_base*alpha)/wheel_radius));
    const double yaw=traj.getCarAngle(t);const Eigen::Vector3d car(p[0],p[1],yaw);
    std::vector<Eigen::Vector3d> points;model.getCarPts(car,points);
    for(std::size_t i=0;i<points.size();++i)
      base_gap=std::min(base_gap,map.getPlanningDistance(points[i],true)-model.getBaseSphereRadius(i)-model.getMapResolution());
    std::vector<Eigen::Matrix4d> joints,unused;
    const Eigen::VectorXd q=p.tail(model.getManiDof());model.getJointTrans(q,joints,unused);
    Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
    pose(0,0)=std::cos(yaw);pose(0,1)=-std::sin(yaw);pose(1,0)=std::sin(yaw);pose(1,1)=std::cos(yaw);pose(0,3)=p[0];pose(1,3)=p[1];
    Eigen::Matrix4d transform=pose*model.getTq0();
    for(int i=0;i<model.getManiDof();++i){transform*=joints[i];
      for(int n=0;n<links[i].cols();++n){const Eigen::Vector3d point=(transform*links[i].col(n)).template head<3>();
        arm_gap=std::min(arm_gap,map.getPlanningDistance(point,true)-arm_radius);
        ground_gap=std::min(ground_gap,point.z()-ground);}}
    // Keep native collision-class observations, including the self predicates.
    double unused_distance=0;
    hits[0]+=model.checkCarPlanningObsCollision(car,true,false,unused_distance);
    hits[1]+=model.checkManiPlanningObsCollision(car,q,false,unused_distance);
    hits[2]+=model.checkCarManiCollision(q,false,unused_distance);
    hits[3]+=model.checkManiManiCollision(q,false,unused_distance);
  }
  std::ostringstream s;s<<std::setprecision(17)<<"\"full_grid_residual_audit\":{\"sample_count\":"<<samples
    <<",\"sample_dt_s\":0.01,\"sample_start_s\":0.05,\"duration_s\":"<<traj.duration
    <<",\"max_left_omega\":"<<max_l<<",\"max_right_omega\":"<<max_r
    <<",\"max_left_alpha\":"<<max_al<<",\"max_right_alpha\":"<<max_ar
    <<",\"omega_hard_limit_with_tolerance\":"<<wheel_limit*1.05
    <<",\"alpha_hard_limit_with_tolerance\":"<<alpha_limit*1.05
    <<",\"left_omega_excess\":"<<max_l-wheel_limit*1.05<<",\"right_omega_excess\":"<<max_r-wheel_limit*1.05
    <<",\"left_alpha_excess\":"<<max_al-alpha_limit*1.05<<",\"right_alpha_excess\":"<<max_ar-alpha_limit*1.05
    <<",\"min_base_hard_gap_m\":"<<delivery_diag::number(base_gap)<<",\"min_arm_hard_gap_m\":"<<delivery_diag::number(arm_gap)
    <<",\"min_arm_ground_gap_m\":"<<delivery_diag::number(ground_gap)
    <<",\"collision_samples_by_class\":["<<hits[0]<<','<<hits[1]<<','<<hits[2]<<','<<hits[3]<<']'
    <<",\"wall_s\":"<<failure_probe::wall()-began
    <<",\"budget_excluded\":true,\"scope\":\"native planning map/overlay, native hard geometry, sampled full path; not future live dynamic or continuous certification\"}";
  return s.str();
}
} // namespace native67_probe
