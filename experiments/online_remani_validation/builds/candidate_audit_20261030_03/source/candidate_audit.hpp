#pragma once
// Experiment-only observation. Expensive work runs after the online solver.
// This helper never changes a constraint, optimizer input, or acceptance result.
#include <Eigen/Dense>
#include <ros/ros.h>
#include "plan_env/grid_map.h"
#include <zlib.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace candidate_audit {
inline double wall() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::string number(double value) {
  if(!std::isfinite(value))return "null";
  std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();
}
inline std::string quote(const std::string& value) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:value){switch(c){case '\\':out<<"\\\\";break;case '"':out<<"\\\"";break;
    case '\n':out<<"\\n";break;case '\r':out<<"\\r";break;case '\t':out<<"\\t";break;
    default:if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else out<<char(c);}}
  out<<'"';return out.str();
}
template<class Vector> inline void vectorJson(std::ostream& out,const Vector& v){
  out<<'[';for(Eigen::Index i=0;i<v.size();++i){if(i)out<<',';out<<number(v[i]);}out<<']';
}
struct Step {
  std::string label;
  Eigen::VectorXd x;
  double cost=0,solver_elapsed_s=0;
  int iteration=0,line_search=0;
  bool cost_seen=false;
  std::array<double,8> costs{};
  std::string evaluation_begin_key="null",evaluation_end_key="null";

};
struct Session;
inline Session*& active(){static thread_local Session* ptr=nullptr;return ptr;}
struct Session {
  bool enabled;
  Session* previous;
  double start_wall;
  std::vector<Step> steps;
  std::function<std::string()> context;
  std::string solver_entry_key="null",solver_exit_key="null";
  std::string evaluation_begin_key="null",evaluation_end_key="null";
  unsigned long long evaluate_complete_count=0,evaluate_overlay_changed_count=0;

  bool cost_seen=false;
  std::array<double,8> costs{};
  explicit Session(bool on):enabled(on),previous(active()),start_wall(wall()) {
    if(enabled)active()=this;
  }

  void begin(){if(enabled){start_wall=wall();solver_entry_key=context?context():"null";}}
  void evaluationBegin(){if(enabled)evaluation_begin_key=context?context():"null";}

  ~Session(){if(enabled)active()=previous;}
  void initial(const Eigen::VectorXd& x,double cost){
    if(!enabled||!steps.empty()||!std::isfinite(cost)||!x.allFinite())return;
    steps.push_back({"initial",x,cost,wall()-start_wall,0,0});
    steps.back().evaluation_begin_key=evaluation_begin_key;
    steps.back().evaluation_end_key=evaluation_end_key;
    steps.back().cost_seen=cost_seen;steps.back().costs=costs;
  }
  void accepted(const Eigen::VectorXd& x,double cost,int iteration,int line_search){
    if(!enabled||!std::isfinite(cost)||!x.allFinite())return;
    steps.push_back({"accepted",x,cost,wall()-start_wall,iteration,line_search});
    steps.back().evaluation_begin_key=evaluation_begin_key;
    steps.back().evaluation_end_key=evaluation_end_key;
    steps.back().cost_seen=cost_seen;steps.back().costs=costs;
  }
  void writeSteps(const std::string& path) const {
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write accepted candidate steps");
    out<<"{\"schema\":\"candidate_audit_steps_v1\",\"steps\":[";
    for(std::size_t i=0;i<steps.size();++i){const auto&s=steps[i];if(i)out<<',';
      out<<"{\"label\":"<<quote(s.label)<<",\"iteration\":"<<s.iteration<<",\"line_search\":"<<s.line_search
        <<",\"solver_elapsed_s\":"<<number(s.solver_elapsed_s)<<",\"cost\":"<<number(s.cost)
        <<",\"cost_seen\":"<<(s.cost_seen?"true":"false")<<",\"cost_components\":[";
      for(int k=0;k<8;++k){if(k)out<<',';out<<number(s.costs[k]);}out<<"],\"x\":";

      vectorJson(out,s.x);out<<",\"evaluation_begin_key\":"<<s.evaluation_begin_key
        <<",\"evaluation_end_key\":"<<s.evaluation_end_key<<'}';}
    out<<"]}\n";if(!out)throw std::runtime_error("candidate steps write failed");
  }
};

inline void recordEvaluationBegin(){if(active())active()->evaluationBegin();}
inline std::string overlayContentKey(std::string key){
  for(const char* marker:{",\"frozen\":true",",\"frozen\":false"}){
    const std::string token=marker;const auto at=key.find(token);
    if(at!=std::string::npos)key.erase(at,token.size());
  }
  return key;
}
inline void recordInitial(const Eigen::VectorXd& x,double fx){if(active())active()->initial(x,fx);}
inline void recordAccepted(const Eigen::VectorXd& x,double fx,int iteration,int line_search){
  if(active())active()->accepted(x,fx,iteration,line_search);
}
template<class Vector> inline void recordCostComponents(double smooth,const Vector& costs,double time){
  auto*s=active();if(!s)return;s->cost_seen=true;s->costs[0]=smooth;
  s->evaluation_end_key=s->context?s->context():"null";
  ++s->evaluate_complete_count;
  s->evaluate_overlay_changed_count+=overlayContentKey(s->evaluation_begin_key)!=overlayContentKey(s->evaluation_end_key);
  for(int k=0;k<6;++k)s->costs[k+1]=costs[k];s->costs[7]=time;
}

struct Params {
  double wheel_base=0,wheel_radius=0,wheel_limit=0,alpha_limit=0,feas_tol=.05;
  double arm_radius=0,ground=0,base_safe_margin=0,arm_safe_margin=0;
  double joint_vel_limit=std::numeric_limits<double>::quiet_NaN();
  double joint_acc_limit=std::numeric_limits<double>::quiet_NaN();
};
struct Distances {
  double stat=0,dynamic=std::numeric_limits<double>::infinity(),planning=0;
  bool dynamic_available=false;
};
template<class Map> inline Distances distances(Map& map,const Eigen::Vector3d& point){
  Distances d;d.stat=map.getPreciseDistance(point);d.planning=d.stat;
  double dynamic=d.stat;
  d.dynamic_available=map.getDynamicPointPlanningDistance(point,std::max(d.stat,0.),dynamic,nullptr);
  if(d.dynamic_available){d.dynamic=dynamic;d.planning=std::min(d.stat,dynamic);}return d;
}
struct Sample {
  double t=0,x=0,y=0,yaw=0,v=0,omega=0,alpha=0;
  std::array<double,4> wheels{};
  double base_gap=INFINITY,base_static_gap=INFINITY,base_dynamic_gap=INFINITY;
  double arm_gap=INFINITY,arm_static_gap=INFINITY,arm_dynamic_gap=INFINITY,ground_gap=INFINITY;
  double base_soft_gap=INFINITY,arm_soft_gap=INFINITY,joint_vel=0,joint_acc=0;
  int base_index=-1;Eigen::Vector3d base_center=Eigen::Vector3d::Zero();double base_radius=0;
  Distances base_distance;
  std::array<bool,4> collisions{};
  bool collision_queried=false;
  bool kinematics_finite=true;
};
template<class Trajectory>
Sample motionSample(const Trajectory& traj,const Params& p,double t){
  Sample s;s.t=t;
  const Eigen::VectorXd pos=traj.getPos(t),vel=traj.getVel(t),acc=traj.getAcc(t),jer=traj.getJer(t);
  const Eigen::Vector2d v=vel.head(2),a=acc.head(2),j=jer.head(2);
  const double v2=std::max(1e-8,v.squaredNorm()),speed=std::max(1e-4,v.norm());
  const double cross=v.x()*a.y()-v.y()*a.x();
  s.x=pos[0];s.y=pos[1];s.yaw=traj.getCarAngle(t);s.v=traj.getSingul(t)*speed;
  s.omega=cross/v2;s.alpha=(v.x()*j.y()-v.y()*j.x())/v2-2*cross*a.dot(v)/(v2*v2);
  const double signed_a=traj.getSingul(t)*a.dot(v)/speed;
  s.wheels={{(s.v-.5*p.wheel_base*s.omega)/p.wheel_radius,(s.v+.5*p.wheel_base*s.omega)/p.wheel_radius,
    (signed_a-.5*p.wheel_base*s.alpha)/p.wheel_radius,(signed_a+.5*p.wheel_base*s.alpha)/p.wheel_radius}};
  s.kinematics_finite=pos.allFinite()&&vel.allFinite()&&acc.allFinite()&&jer.allFinite()&&
    std::isfinite(s.yaw)&&std::isfinite(s.omega)&&std::isfinite(s.alpha);
  for(double w:s.wheels)s.kinematics_finite=s.kinematics_finite&&std::isfinite(w);
  const int dof=int(pos.size())-2;
  if(dof){s.joint_vel=vel.tail(dof).template lpNorm<Eigen::Infinity>();s.joint_acc=acc.tail(dof).template lpNorm<Eigen::Infinity>();}
  return s;
}
template<class Trajectory,class Model,class Map>
Sample sample(const Trajectory& traj,Model& model,Map& map,const Params& p,double t){
  Sample s=motionSample(traj,p,t);s.collision_queried=true;
  const Eigen::VectorXd pos=traj.getPos(t),q=pos.tail(model.getManiDof());
  const int dof=model.getManiDof();
  const Eigen::Vector3d car(s.x,s.y,s.yaw);std::vector<Eigen::Vector3d> points;model.getCarPts(car,points);
  for(std::size_t i=0;i<points.size();++i){const auto d=distances(map,points[i]);
    const double radius=model.getBaseSphereRadius(i),hard=radius+model.getMapResolution(),gap=d.planning-hard;
    s.base_static_gap=std::min(s.base_static_gap,d.stat-hard);
    s.base_dynamic_gap=std::min(s.base_dynamic_gap,d.dynamic-hard);
    s.base_soft_gap=std::min(s.base_soft_gap,d.planning-radius-p.base_safe_margin);
    if(gap<s.base_gap){s.base_gap=gap;s.base_index=int(i);s.base_center=points[i];s.base_radius=radius;s.base_distance=d;}}
  const auto links=model.getLinkPoint();std::vector<Eigen::Matrix4d> joints,unused;model.getJointTrans(q,joints,unused);
  Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
  pose(0,0)=std::cos(s.yaw);pose(0,1)=-std::sin(s.yaw);pose(1,0)=std::sin(s.yaw);pose(1,1)=std::cos(s.yaw);
  pose(0,3)=s.x;pose(1,3)=s.y;Eigen::Matrix4d transform=pose*model.getTq0();
  for(int i=0;i<dof;++i){transform*=joints[i];for(int k=0;k<links[i].cols();++k){
    const Eigen::Vector3d point=(transform*links[i].col(k)).template head<3>();const auto d=distances(map,point);
    s.arm_gap=std::min(s.arm_gap,d.planning-p.arm_radius);
    s.arm_static_gap=std::min(s.arm_static_gap,d.stat-p.arm_radius);
    s.arm_dynamic_gap=std::min(s.arm_dynamic_gap,d.dynamic-p.arm_radius);
    s.arm_soft_gap=std::min(s.arm_soft_gap,d.planning-p.arm_radius-p.arm_safe_margin);
    s.ground_gap=std::min(s.ground_gap,point.z()-p.ground);}}
  // Native predicates retain their own ground and self-collision rules.
  double unused_distance=0;
  s.collisions[0]=model.checkCarPlanningObsCollision(car,true,false,unused_distance);
  s.collisions[1]=model.checkManiPlanningObsCollision(car,q,false,unused_distance);
  s.collisions[2]=model.checkCarManiCollision(q,false,unused_distance);
  s.collisions[3]=model.checkManiManiCollision(q,false,unused_distance);
  return s;
}
inline std::string sampleJson(const Sample&s){
  std::ostringstream o;o<<"{\"t\":"<<number(s.t)<<",\"x\":"<<number(s.x)<<",\"y\":"<<number(s.y)
    <<",\"yaw\":"<<number(s.yaw)<<",\"v\":"<<number(s.v)<<",\"omega\":"<<number(s.omega)<<",\"alpha\":"<<number(s.alpha);
  const char* names[]={"wheel_left_omega","wheel_right_omega","wheel_left_alpha","wheel_right_alpha"};
  for(int i=0;i<4;++i)o<<",\""<<names[i]<<"\":"<<number(s.wheels[i]);
  const char* gaps[]={"base_gap","base_static_gap","base_dynamic_gap","arm_gap","arm_static_gap","arm_dynamic_gap","ground_gap","base_soft_gap","arm_soft_gap"};
  const double values[]={s.base_gap,s.base_static_gap,s.base_dynamic_gap,s.arm_gap,s.arm_static_gap,s.arm_dynamic_gap,s.ground_gap,s.base_soft_gap,s.arm_soft_gap};
  for(int i=0;i<9;++i)o<<",\""<<gaps[i]<<"\":"<<number(values[i]);
  o<<",\"joint_vel_inf\":"<<number(s.joint_vel)<<",\"joint_acc_inf\":"<<number(s.joint_acc)
    <<",\"collision_queried\":"<<(s.collision_queried?"true":"false")
    <<",\"kinematics_finite\":"<<(s.kinematics_finite?"true":"false")
    <<",\"base_gap_finite\":"<<(std::isfinite(s.base_gap)?"true":"false")
    <<",\"arm_gap_finite\":"<<(std::isfinite(s.arm_gap)?"true":"false")
    <<",\"closest_base\":{\"index\":"<<s.base_index<<",\"center\":";vectorJson(o,s.base_center);
  o<<",\"radius\":"<<number(s.base_radius)<<",\"static_distance\":"<<number(s.base_distance.stat)
    <<",\"dynamic_distance\":"<<number(s.base_distance.dynamic)<<",\"dynamic_query_available\":"<<(s.base_distance.dynamic_available?"true":"false")
    <<",\"planning_distance\":"<<number(s.base_distance.planning)<<"},\"collision_classes\":[";
  for(int i=0;i<4;++i){if(i)o<<',';o<<(s.collisions[i]?"true":"false");}o<<"]}";return o.str();
}
struct Summary {
  int count=0,nonfinite_kinematics=0;std::array<double,4> max_wheels{},max_wheel_times{};std::array<int,4> hits{};
  double base_gap=INFINITY,arm_gap=INFINITY,ground_gap=INFINITY,base_t=0,arm_t=0;
  double static_base_gap=INFINITY,dynamic_base_gap=INFINITY,static_arm_gap=INFINITY,dynamic_arm_gap=INFINITY;
  double max_joint_v=0,max_joint_a=0;std::string first_failure;double first_failure_t=0;int first_collision_class=-1;
  void add(const Sample&s,const Params&p){
    ++count;nonfinite_kinematics+=!s.kinematics_finite;
    for(int i=0;i<4;++i){if(std::abs(s.wheels[i])>max_wheels[i]){max_wheels[i]=std::abs(s.wheels[i]);max_wheel_times[i]=s.t;}hits[i]+=s.collisions[i];}
    if(s.base_gap<base_gap){base_gap=s.base_gap;base_t=s.t;}if(s.arm_gap<arm_gap){arm_gap=s.arm_gap;arm_t=s.t;}
    ground_gap=std::min(ground_gap,s.ground_gap);static_base_gap=std::min(static_base_gap,s.base_static_gap);
    dynamic_base_gap=std::min(dynamic_base_gap,s.base_dynamic_gap);static_arm_gap=std::min(static_arm_gap,s.arm_static_gap);
    dynamic_arm_gap=std::min(dynamic_arm_gap,s.arm_dynamic_gap);max_joint_v=std::max(max_joint_v,s.joint_vel);max_joint_a=std::max(max_joint_a,s.joint_acc);
    if(!first_failure.empty())return;
    for(int i=0;i<4;++i)if(s.collisions[i]){first_failure="planning_collision";first_collision_class=i;first_failure_t=s.t;return;}
    const char* names[]={"left_wheel_speed","right_wheel_speed","left_wheel_acceleration","right_wheel_acceleration"};
    for(int i=0;i<4;++i)if(std::abs(s.wheels[i])>(1+p.feas_tol)*(i<2?p.wheel_limit:p.alpha_limit)){
      first_failure=names[i];first_failure_t=s.t;return;}
    if(std::isfinite(p.joint_vel_limit)&&s.joint_vel>p.joint_vel_limit*(1+p.feas_tol)){first_failure="joint_velocity";first_failure_t=s.t;return;}
    if(std::isfinite(p.joint_acc_limit)&&s.joint_acc>p.joint_acc_limit*(1+p.feas_tol)){first_failure="joint_acceleration";first_failure_t=s.t;}
  }
  std::string json(const Params&p,const std::string& grid="native_acceptance") const {
    std::ostringstream o;o<<"{\"sample_count\":"<<count<<",\"grid\":"<<quote(grid)
      <<",\"collision_queried\":"<<(grid=="optimizer_feasibility_grid"?"false":"true")
      <<",\"nonfinite_kinematics_samples\":"<<nonfinite_kinematics
      <<",\"omega_hard_limit_with_tolerance\":"<<number(p.wheel_limit*(1+p.feas_tol))
      <<",\"alpha_hard_limit_with_tolerance\":"<<number(p.alpha_limit*(1+p.feas_tol))
      <<",\"wheel_base\":"<<number(p.wheel_base)<<",\"wheel_radius\":"<<number(p.wheel_radius)
      <<",\"feasibility_tolerance\":"<<number(p.feas_tol)<<",\"arm_radius\":"<<number(p.arm_radius)<<",\"ground_hard_height\":"<<number(p.ground);
    const char* names[]={"left_omega","right_omega","left_alpha","right_alpha"};
    for(int i=0;i<4;++i)o<<",\"max_"<<names[i]<<"\":"<<number(max_wheels[i])<<",\"max_"<<names[i]<<"_t\":"<<number(max_wheel_times[i])
      <<",\""<<names[i]<<"_excess\":"<<number(max_wheels[i]-(1+p.feas_tol)*(i<2?p.wheel_limit:p.alpha_limit));
    o<<",\"min_base_hard_gap_m\":"<<number(base_gap)<<",\"min_base_hard_gap_t\":"<<number(base_t)
      <<",\"min_arm_hard_gap_m\":"<<number(arm_gap)<<",\"min_arm_hard_gap_t\":"<<number(arm_t)
      <<",\"min_arm_ground_gap_m\":"<<number(ground_gap)<<",\"min_base_static_gap_m\":"<<number(static_base_gap)
      <<",\"min_base_dynamic_gap_m\":"<<number(dynamic_base_gap)<<",\"min_arm_static_gap_m\":"<<number(static_arm_gap)
      <<",\"min_arm_dynamic_gap_m\":"<<number(dynamic_arm_gap)<<",\"max_joint_velocity_inf\":"<<number(max_joint_v)
      <<",\"max_joint_acceleration_inf\":"<<number(max_joint_a)<<",\"collision_samples_by_class\":[";
    for(int i=0;i<4;++i){if(i)o<<',';o<<hits[i];}
    o<<"],\"sampled_first_failure\":"<<quote(first_failure)<<",\"sampled_first_failure_t\":"<<number(first_failure_t)
      <<",\"sampled_first_collision_class\":"<<first_collision_class<<",\"sampled_conditions_all_pass\":"<<(first_failure.empty()?"true":"false")
      <<",\"scope\":\"class predicates and residuals on stated sample grid; original IsTrajSafe remains acceptance authority; finite sampling is not continuous certification\"}";
    return o.str();
  }
};
template<class Trajectory,class Model,class Map>
std::string audit(const Trajectory& traj,Model& model,Map& map,const Params&p,
    const std::vector<double>& optimizer_sample_times={},bool detail=true,
    const std::vector<double>& feasibility_sample_times={}){
  const double began=wall();Summary summary,opt_summary,feas_summary;std::ostringstream samples,opt,start,feas;
  samples<<'[';const int end=int(std::floor(traj.duration/.01));double t=.05;bool comma=false;
  for(int k=5;k<end;++k,t+=.01){const auto s=sample(traj,model,map,p,t);summary.add(s,p);
    if(detail){if(comma)samples<<',';samples<<sampleJson(s);comma=true;}}
  samples<<']';opt<<'[';comma=false;
  for(double at:optimizer_sample_times){if(!std::isfinite(at)||at<0||at>traj.duration)continue;
    const auto s=sample(traj,model,map,p,at);opt_summary.add(s,p);
    if(detail){if(comma)opt<<',';opt<<sampleJson(s);comma=true;}}
  opt<<']';start<<'[';comma=false;
  if(detail)for(int k=0;k<=20&&k*.01<=traj.duration;++k){if(comma)start<<',';start<<sampleJson(sample(traj,model,map,p,k*.01));comma=true;}
  start<<']';feas<<'[';comma=false;
  for(double at:feasibility_sample_times){if(!std::isfinite(at)||at<0||at>traj.duration)continue;
    const auto s=motionSample(traj,p,at);feas_summary.add(s,p);
    if(detail){if(comma)feas<<',';feas<<sampleJson(s);comma=true;}}
  feas<<']';std::ostringstream out;out<<"{\"schema\":\"candidate_audit_v1\",\"duration_s\":"<<number(traj.duration)
    <<",\"detail\":"<<(detail?"true":"false")<<",\"sample_start_s\":0.05,\"sample_dt_s\":0.01"
    <<",\"summary\":"<<summary.json(p)<<",\"samples\":"<<samples.str()
    <<",\"optimizer_samples\":"<<opt.str()<<",\"optimizer_sample_times_count\":"<<optimizer_sample_times.size()
    <<",\"optimizer_summary\":"<<opt_summary.json(p,"optimizer_cost_grid")
    <<",\"feasibility_optimizer_samples\":"<<feas.str()<<",\"feasibility_optimizer_sample_times_count\":"<<feasibility_sample_times.size()
    <<",\"feasibility_optimizer_summary\":"<<feas_summary.json(p,"optimizer_feasibility_grid")
    <<",\"start_samples\":"<<start.str()<<",\"post_solver_audit_wall_s\":"<<number(wall()-began)<<",\"budget_excluded\":true}";return out.str();
}

struct GzipWriter {
  gzFile file=nullptr;
  explicit GzipWriter(const std::string& path):file(gzopen(path.c_str(),"wb1")){
    if(!file)throw std::runtime_error("cannot open map gzip snapshot");
  }
  ~GzipWriter(){if(file)gzclose(file);}
  void write(const void* data,std::size_t bytes){
    const auto* p=static_cast<const unsigned char*>(data);
    while(bytes){const unsigned n=static_cast<unsigned>(std::min<std::size_t>(bytes,1U<<20));
      if(gzwrite(file,p,n)!=int(n))throw std::runtime_error("map gzip snapshot write failed");p+=n;bytes-=n;}
  }
  void close(){if(file){const int status=gzclose(file);file=nullptr;if(status!=Z_OK)throw std::runtime_error("map gzip close failed");}}
};
} // namespace candidate_audit

// The frozen GridMap already declares exactly this global friend class.
// Defining it here adds observation only and does not change its ABI/layout.
class GridMapParallelTestAccessor {
 public:
  static std::string readOverlayKey(GridMap& map){
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    std::ostringstream out;
    out<<"{\"sequence\":"<<map.dynamic_point_overlay_sequence_
      <<",\"refresh_nsec\":"<<map.dynamic_point_overlay_refresh_time_.toNSec()
      <<",\"size\":"<<(map.dynamic_point_overlay_cloud_?map.dynamic_point_overlay_cloud_->size():0)
      <<",\"radii_size\":"<<map.dynamic_point_overlay_radii_.size()
      <<",\"cloud_address\":"<<candidate_audit::quote(std::to_string(
          reinterpret_cast<std::uintptr_t>(map.dynamic_point_overlay_cloud_.get())))
      <<",\"enabled\":"<<(map.mp_.dynamic_point_overlay_enable_?"true":"false")
      <<",\"ready\":"<<(map.dynamic_point_overlay_ready_?"true":"false")
      <<",\"frozen\":"<<(map.dynamic_point_overlay_frozen_?"true":"false")<<'}';
    return out.str();
  }
  static bool freezeCurrentOverlay(GridMap& map){
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    if(!map.mp_.dynamic_point_overlay_enable_||map.dynamic_point_overlay_frozen_)return false;
    // Pin exactly the present query state, including an empty/unready layer.
    // beginDynamicPointPlanningSnapshot is deliberately not called: it can refresh.
    map.dynamic_point_overlay_frozen_=true;
    map.dynamic_point_overlay_frozen_sequence_=map.dynamic_point_overlay_sequence_;
    return true;
  }
  static void releaseCurrentOverlay(GridMap& map){
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    map.dynamic_point_overlay_frozen_=false;
    map.dynamic_point_overlay_frozen_sequence_=0;
  }

  static std::string writeSnapshot(GridMap& map,const std::string& prefix){
    using namespace candidate_audit;
    const std::string esdf_path=prefix+".esdf.f64.gz",overlay_path=prefix+".overlay.xyzw.f32.gz";
    std::uint64_t esdf_hash=14695981039346656037ULL,overlay_hash=esdf_hash;
    auto hash=[](std::uint64_t& h,const void* bytes,std::size_t size){const auto*p=static_cast<const unsigned char*>(bytes);
      for(std::size_t i=0;i<size;++i){h^=p[i];h*=1099511628211ULL;}};
    // The caller blocks the map's owning callback throughout its frozen audit.
    const auto&buffer=map.md_.distance_buffer_all_;GzipWriter esdf(esdf_path);
    esdf.write(buffer.data(),buffer.size()*sizeof(double));hash(esdf_hash,buffer.data(),buffer.size()*sizeof(double));esdf.close();
    std::ostringstream meta;meta<<"{\"schema\":\"candidate_native_map_v1\",\"esdf_path\":"<<quote(esdf_path)
      <<",\"esdf_dtype\":\"native_endian_float64\",\"esdf_linear_order\":\"(x*ny+y)*nz+z\",\"esdf_count\":"<<buffer.size()
      <<",\"esdf_fnv1a64\":"<<quote(std::to_string(esdf_hash))<<",\"map_origin\":";vectorJson(meta,map.mp_.map_origin_);
    meta<<",\"map_size\":";vectorJson(meta,map.mp_.map_size_);meta<<",\"map_voxel_num\":";vectorJson(meta,map.mp_.map_voxel_num_);
    meta<<",\"map_min_boundary\":";vectorJson(meta,map.mp_.map_min_boundary_);meta<<",\"map_max_boundary\":";vectorJson(meta,map.mp_.map_max_boundary_);
    meta<<",\"resolution\":"<<number(map.mp_.resolution_)<<",\"raw_seq\":"<<map.md_.dynamic_observation_seq_
      <<",\"esdf_seq\":"<<map.md_.esdf_observation_seq_<<",\"sim\":"<<number(ros::Time::now().toSec());
    {
      std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);GzipWriter overlay(overlay_path);
      const std::size_t count=map.dynamic_point_overlay_cloud_?map.dynamic_point_overlay_cloud_->size():0;
      if(count!=map.dynamic_point_overlay_radii_.size())throw std::runtime_error("snapshot cloud/radius mismatch");
      for(std::size_t i=0;i<count;++i){const auto&p=map.dynamic_point_overlay_cloud_->points[i];
        const float values[]={p.x,p.y,p.z,map.dynamic_point_overlay_radii_[i]};overlay.write(values,sizeof(values));hash(overlay_hash,values,sizeof(values));}
      overlay.close();meta<<",\"overlay_path\":"<<quote(overlay_path)<<",\"overlay_dtype\":\"native_endian_float32_xyzw_radius\""
        <<",\"overlay_count\":"<<count<<",\"overlay_fnv1a64\":"<<quote(std::to_string(overlay_hash))
        <<",\"overlay_enabled\":"<<(map.mp_.dynamic_point_overlay_enable_?"true":"false")
        <<",\"overlay_ready\":"<<(map.dynamic_point_overlay_ready_?"true":"false")
        <<",\"overlay_frozen\":"<<(map.dynamic_point_overlay_frozen_?"true":"false")
        <<",\"overlay_sequence\":"<<map.dynamic_point_overlay_sequence_
        <<",\"overlay_frozen_sequence\":"<<map.dynamic_point_overlay_frozen_sequence_
        <<",\"overlay_refresh_nsec\":"<<map.dynamic_point_overlay_refresh_time_.toNSec()
        <<",\"overlay_inflation\":"<<number(map.dynamic_point_overlay_inflation_)
        <<",\"overlay_influence_distance\":"<<number(map.mp_.dynamic_point_overlay_influence_distance_)
        <<",\"overlay_horizon\":"<<number(map.mp_.dynamic_point_overlay_horizon_)
        <<",\"overlay_time_step\":"<<number(map.mp_.dynamic_point_overlay_time_step_)
        <<",\"overlay_extra_inflation\":"<<number(map.mp_.dynamic_point_overlay_extra_inflation_)
        <<",\"overlay_max_inflation\":"<<number(map.mp_.dynamic_point_overlay_max_inflation_)
        <<",\"overlay_min\":";vectorJson(meta,map.dynamic_point_overlay_min_);
      meta<<",\"overlay_max\":";vectorJson(meta,map.dynamic_point_overlay_max_);
    }
    meta<<",\"scope\":\"actual native signed ESDF buffer and complete swept KD input cloud/radii; captured only after online solve in blocked owner callback\"}";
    const std::string json=meta.str();std::ofstream out(prefix+".json");if(!out)throw std::runtime_error("cannot write map snapshot metadata");
    out<<json<<'\n';if(!out)throw std::runtime_error("map snapshot metadata write failed");return json;
  }
};
