#pragma once
// Experimental orchestration helpers; no robot publications or budget resets.
#include "clearance_native.hpp"
#include "recovery_budget.hpp"
#include "dual_timescale_probe.hpp"
#include <algorithm>

namespace clearance_control {
constexpr double rho=1e5;
constexpr double validation_reserve=.04;
constexpr double minimum_restart_time=.02;
inline bool enabled() {
  const char* p=std::getenv("COUPLING_FEASIBILITY_RECOVERY");
  return p&&p[0]=='1'&&p[1]=='\0';
}
struct Frame;
inline Frame*& current(){static thread_local Frame* f=nullptr;return f;}
struct Frame {
  Frame* previous;
  recovery_budget::Window window;
  Frame(double start,double duration):previous(current()),
      window(start,duration,validation_reserve,minimum_restart_time){current()=this;}
  ~Frame(){current()=previous;}
  bool expired() const {return window.expired(ros::WallTime::now().toSec());}
};
inline bool stopRecovery() {
  return current()&&clearance_native::active()&&
      current()->window.recoveryExpired(ros::WallTime::now().toSec());
}
struct Scan {
  bool valid=false;
  double physical=std::numeric_limits<double>::infinity(),gap=physical;
  size_t samples=0;
  std::string reason="empty";
};
// Same block-wide .02s nodes, heading convention and full body as the final
// projected gate. It is a solve-snapshot STATIC check, never dynamic proof.
template<class Opts,class Signs> Scan scan(const Opts& opts,const Signs& signs,const Frame& control) {
  Scan out;
  coupling_geometry::Scope scope;
  if(!scope.valid()){out.reason="snapshot_invalid";return out;}
  if(opts.empty()||opts.size()!=signs.size())return out;
  for(size_t b=0;b<opts.size();++b){
    if(signs[b]!=1&&signs[b]!=-1){out.reason="gear_invalid";return out;}
    const auto& times=opts[b].get_T1();
    const double duration=times.sum();
    if(times.size()==0||!times.allFinite()||(times.array()<=0.).any()||
       !std::isfinite(duration)||duration/.02>1e6){out.reason="duration_invalid";return out;}
    const int steps=std::max(1,int(std::ceil(duration/.02)));
    const auto trajectory=opts[b].getTraj(signs[b]);
    for(int k=0;k<=steps;++k){
      if(k%16==0&&control.expired()){out.reason="deadline";return out;}
      const double t=duration*k/steps;
      const auto p=trajectory.getPos(t),v=trajectory.getVel(t);
      const double yaw=std::atan2(signs[b]*v(1),signs[b]*v(0));
      const auto value=coupling_geometry::staticProjectedPose(*scope.snapshot(),{p(0),p(1),yaw});
      if(!value.valid){out.reason="geometry_invalid";return out;}
      out.physical=std::min(out.physical,value.physical);out.gap=std::min(out.gap,value.gap);++out.samples;
    }
  }
  out.valid=out.samples>0&&!control.expired();out.reason=out.valid?"ok":"deadline";return out;
}
inline void emitScan(const char* event,const Scan& s,const Frame& control) {
  dual_probe::emit(event,dual_probe::Fields().add("enabled",int(enabled()))
    .add("valid",s.valid?"true":"false").str("reason",s.reason)
    .add("min_physical",delivery_diag::number(s.physical)).add("min_gap",delivery_diag::number(s.gap))
    .add("samples",s.samples).add("budget_s",control.window.duration())
    .add("elapsed_s",control.window.elapsed(ros::WallTime::now().toSec())));
}
// Copy only small coefficient arrays while the budget is running. Expensive
// JSON/snapshot I/O is deferred to this record's post-decision destructor.
class Record {
  struct Phase {std::string name;std::vector<Eigen::MatrixXd> coeffs;
    std::vector<Eigen::VectorXd> times;std::vector<int> signs;};
  std::vector<Phase> phases_;
  delivery_diag::Frame frame_;
  unsigned long long request_,scope_;
public:
  std::string outcome="incomplete";
  Record():request_(dual_probe::request()),scope_(delivery_diag::solve().scope_id) {
    coupling_geometry::Scope s;frame_={s.snapshot(),s.now()};
  }
  template<class Opts,class Signs>void capture(const char* name,const Opts& opts,const Signs& signs){
    if(!delivery_diag::directory())return;
    Phase p;p.name=name;p.signs.assign(signs.begin(),signs.end());
    for(const auto& opt:opts){p.coeffs.push_back(opt.get_b());p.times.push_back(opt.get_T1());}
    phases_.push_back(std::move(p));
  }
  ~Record() noexcept {
    if(!delivery_diag::directory()||phases_.empty())return;
    try{
      const std::string root=delivery_diag::directory();delivery_diag::mkdirChecked(root+"/clearance_recovery");
      std::string text="{\"schema\":\"clearance_recovery_v1\",\"request_id\":"+std::to_string(request_)+
        ",\"scope_id\":"+std::to_string(scope_)+",\"enabled\":"+(enabled()?"true":"false")+
        ",\"rho\":"+delivery_diag::number(rho)+",\"outcome\":"+delivery_diag::quote(outcome)+
        ",\"frame\":"+delivery_diag::frameJson(frame_)+",\"phases\":[";
      for(size_t i=0;i<phases_.size();++i){if(i)text+=',';const auto& p=phases_[i];
        text+="{\"phase\":"+delivery_diag::quote(p.name)+",\"blocks\":[";
        for(size_t b=0;b<p.coeffs.size();++b){if(b)text+=',';
          text+="{\"gear\":"+std::to_string(p.signs[b])+",\"basis\":\"ascending_rows_per_8\",\"times\":"+
            delivery_diag::vector(p.times[b])+",\"coeffs\":"+delivery_diag::matrix(p.coeffs[b])+"}";
        }text+="]}";
      }text+="]}";
      delivery_diag::writeExclusive(delivery_diag::nextName("clearance_recovery"),text);
    }catch(const std::exception& e){delivery_diag::unavailable(e.what());}
    catch(...){delivery_diag::unavailable("clearance record failure");}
  }
};
}
