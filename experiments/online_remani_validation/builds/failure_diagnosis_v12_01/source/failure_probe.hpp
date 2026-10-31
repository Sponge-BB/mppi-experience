#pragma once
// Isolated experiment accounting; never included by production targets.
#include "resource_probe.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace failure_probe {
inline double wall() { return resource_probe::clockSeconds(CLOCK_MONOTONIC); }
enum Part { COST, INTEGRAL, OBSTACLE, BASE_FEAS, JOINT_FEAS, EDT, SMOOTH, PROPAGATE, COUNT };
inline const char* partName(int i) {
  static const char* names[]={"cost_callback","integral_cost","obstacle_cost","base_feasibility","joint_feasibility","edt_query","smoothness","gradient_propagation"};
  return names[i];
}
struct Frame;
inline Frame*& active() { static thread_local Frame* f=nullptr; return f; }
inline void emit(const std::string& row) {
  const char* path=std::getenv("FAILURE_TRACE_PATH");
  if(path){std::ofstream out(path,std::ios::app);out<<row<<'\n';}
}
struct Frame {
  Frame* previous;
  std::array<double,COUNT> ms{};
  std::array<unsigned long long,COUNT> calls{};
  std::string label, reason="not_validated";
  double first_cost=0, rejection_value=0, rejection_time=0;
  unsigned long long first_gradient_hash=0;
  bool first=false, complete=false, diagnostic_only=false;
  double start, cpu, wait, budget, wheel, sim;
  unsigned long long input, raw_seq, esdf_seq;
  std::string extra;
  Frame(double b,double w,unsigned long long h,unsigned long long raw,unsigned long long esdf):
      previous(active()),label(resource_probe::label()),start(wall()),
      cpu(resource_probe::clockSeconds(CLOCK_THREAD_CPUTIME_ID)),wait(resource_probe::scheduledWait()),
      budget(b),wheel(w),sim(ros::Time::now().toSec()),input(h),raw_seq(raw),esdf_seq(esdf){active()=this;}
  template<class T> void add(const std::string& key,const T& value){
    std::ostringstream s;s<<std::setprecision(17)<<",\""<<key<<"\":"<<value;extra+=s.str();
  }
  ~Frame(){
    std::ostringstream s;s<<std::setprecision(17)<<"{\"kind\":\"solve\",\"label\":\""<<label
      <<"\",\"input_hash\":\""<<input<<"\",\"budget_s\":"<<budget<<",\"wheel_limit\":"<<wheel
      <<",\"sim\":"<<sim<<",\"sim_end\":"<<ros::Time::now().toSec()<<",\"raw_seq\":"<<raw_seq
      <<",\"esdf_seq\":"<<esdf_seq<<",\"wall_ms\":"<<(wall()-start)*1000
      <<",\"thread_cpu_ms\":"<<(resource_probe::clockSeconds(CLOCK_THREAD_CPUTIME_ID)-cpu)*1000
      <<",\"runqueue_wait_ms\":"<<(resource_probe::scheduledWait()-wait)*1000
      <<",\"first_cost\":"<<first_cost<<",\"first_gradient_hash\":\""<<first_gradient_hash
      <<"\",\"reason\":\""<<reason<<"\",\"rejection_value\":"<<rejection_value
      <<",\"rejection_time\":"<<rejection_time<<",\"success\":"<<(complete?"true":"false")
      <<",\"diagnostic_only\":"<<(diagnostic_only?"true":"false")<<",\"profile\":{";
    for(int i=0;i<COUNT;++i){if(i)s<<',';s<<'\"'<<partName(i)<<"\":{\"ms\":"<<ms[i]<<",\"calls\":"<<calls[i]<<'}';}
    s<<'}'<<extra<<'}';emit(s.str());active()=previous;
  }
};
struct Timer {
  Frame* frame;int part;double start;
  explicit Timer(int p):frame(active()),part(p),start(wall()){}
  ~Timer(){if(frame){frame->ms[part]+=(wall()-start)*1000;++frame->calls[part];}}
};
inline void reject(const char* why,double value=0,double t=0){if(active()){
  active()->reason=why;active()->rejection_value=value;active()->rejection_time=t;}}
inline void firstCost(double cost,const Eigen::VectorXd& g){if(active()&&!active()->first){
  auto*f=active();f->first=true;f->first_cost=cost;f->first_gradient_hash=1469598103934665603ULL;
  resource_probe::hashBytes(f->first_gradient_hash,g.data(),g.size()*sizeof(double));}}
template<class Container> void matrices(std::ostream& out,const Container& values){
  out<<'[';bool comma=false;for(const auto&m:values){if(comma)out<<',';comma=true;
    out<<"{\"rows\":"<<m.rows()<<",\"cols\":"<<m.cols()<<",\"data_column_major\":[";
    for(long i=0;i<m.size();++i){if(i)out<<',';out<<m.data()[i];}out<<"]}";}out<<']';
}
template<class Trajectory> void trajectoryStats(const Trajectory& traj,double b,double r){
  double max_l=0,max_r=0,max_al=0,max_ar=0;unsigned long long h=1469598103934665603ULL;
  for(double t=0;t<=traj.duration;t+=.005){
    const Eigen::Vector2d v=traj.getVel(t).head(2),a=traj.getAcc(t).head(2),j=traj.getJer(t).head(2);
    const double v2=std::max(1e-8,v.squaredNorm()),speed=std::max(1e-4,v.norm());
    // B_h in this project is [0 -1; 1 0], hence a^T B_h v = vx*ay-vy*ax.
    const double cross=v.x()*a.y()-v.y()*a.x(),omega=cross/v2;
    const double alpha=(v.x()*j.y()-v.y()*j.x())/v2-2*cross*a.dot(v)/(v2*v2);
    const double signed_v=traj.getSingul(t)*speed;
    const double signed_a=traj.getSingul(t)*a.dot(v)/speed;
    double values[]={(signed_v-b*.5*omega)/r,(signed_v+b*.5*omega)/r,
                     (signed_a-b*.5*alpha)/r,(signed_a+b*.5*alpha)/r};
    max_l=std::max(max_l,std::abs(values[0]));max_r=std::max(max_r,std::abs(values[1]));
    max_al=std::max(max_al,std::abs(values[2]));max_ar=std::max(max_ar,std::abs(values[3]));
    resource_probe::hashBytes(h,values,sizeof(values));
  }
  if(active()){auto*f=active();f->add("max_left_omega",max_l);f->add("max_right_omega",max_r);
    f->add("max_left_alpha",max_al);f->add("max_right_alpha",max_ar);f->add("candidate_hash",h);f->add("candidate_duration_s",traj.duration);}
}
struct PauseOwned {
  std::vector<int> pids;
  PauseOwned(){
    const char* path=std::getenv("FAILURE_OWNED_PID_FILE");
    if(!path)throw std::runtime_error("missing experiment PID file");
    int sim=0,mppi=0;for(int i=0;i<50&&(!sim||!mppi);++i){std::ifstream in(path);in>>sim>>mppi;if(!sim||!mppi)usleep(100000);}
    try{for(auto item:{std::make_pair(sim,"algorithm_simulator"),std::make_pair(mppi,"base_mppi")}){
      if(item.first<=1)throw std::runtime_error("invalid owned PID");
      std::ifstream cmd("/proc/"+std::to_string(item.first)+"/cmdline");std::string text((std::istreambuf_iterator<char>(cmd)),{});
      if(text.find(std::string("__name:=")+item.second)==std::string::npos)throw std::runtime_error("owned node identity mismatch");
      if(kill(item.first,SIGSTOP)!=0)throw std::runtime_error("cannot pause owned diagnostic child");pids.push_back(item.first);
    }}catch(...){resume();throw;}
    // Drain already in-flight /clock messages; planner map commits remain on
    // its blocked ros::spin callback, not an asynchronous input refresh.
    usleep(150000);
  }
  void resume(){for(int pid:pids)kill(pid,SIGCONT);pids.clear();}
  ~PauseOwned(){resume();}
};
} // namespace failure_probe
