#pragma once
// Isolated SIL diagnostics only. No ROS publisher; no relaxed constraint.
// These are NEW captures with all solves paused, not historical replay and
// not navigation/performance measurements. Post-solver audits are untimed.
#include "coupling_bridge.hpp"
#include "failure_probe.hpp"
#include "boundary_admission.hpp"
#include <functional>
#include <limits>

namespace delivery_replay {
inline bool& running(){static thread_local bool b=false;return b;}
inline bool& used(){static bool b=false;return b;}
inline unsigned long long& refreshOrdinal(){static unsigned long long n=0;return n;}
inline bool& auditFailed(){static thread_local bool b=false;return b;}
inline std::string& phase(){static thread_local std::string s;return s;}
inline bool requested(double budget){
  const char* log=std::getenv("DELIVERY_REPLAY_LOG");
  if(!log||!*log||running()||used()||resource_probe::label()!="normal"||
     budget<.59||budget>.61)return false;
  const char* id=std::getenv("DELIVERY_REPLAY_REQUEST_ID");
  const char* kind=std::getenv("DELIVERY_REPLAY_KIND");
  const char* ordinal=std::getenv("DELIVERY_REPLAY_REFRESH_ORDINAL");
  if(!kind||std::string(kind)!="first_refresh"||coupling_bridge::referenceRevision()==0)return false;
  const auto current=++refreshOrdinal();
  const auto chosen=ordinal&&*ordinal?std::stoull(ordinal):1ULL;
  if(chosen<1)throw std::invalid_argument("DELIVERY_REPLAY_REFRESH_ORDINAL must be positive");
  return current==chosen&&(!(id&&*id)||dual_probe::request()==std::stoull(id));
}
inline std::string num(double v){return delivery_diag::number(v);}
inline std::string quote(const std::string&s){return delivery_diag::quote(s);}
// Content serialization deliberately avoids hashing Eigen object padding or
// std::string pointers. Nanoseconds and IDs remain exact integers.
template<class Snapshot> std::string nativeDynamicText(const Snapshot& s){
  std::ostringstream o;o<<std::setprecision(17)<<"{\"schema\":\"native_dynamic_point_snapshot_v1\""
    <<",\"valid\":"<<(s.valid?"true":"false")<<",\"sequence\":"<<s.sequence
    <<",\"stamp_nsec\":"<<s.stamp.toNSec()<<",\"receive_time_nsec\":"<<s.receive_time.toNSec()
    <<",\"frame_id\":"<<quote(s.frame_id)<<",\"invalid_reason\":"<<quote(s.invalid_reason)
    <<",\"count\":"<<s.points.size()<<",\"point_columns\":\"x,y,z,vx,vy,vz,track_id,confidence\",\"points\":[";
  for(size_t i=0;i<s.points.size();++i){if(i)o<<',';const auto&p=s.points[i];o<<'[';
    for(int k=0;k<3;++k){if(k)o<<',';o<<num(p.position(k));}
    for(int k=0;k<3;++k)o<<','<<num(p.velocity(k));
    o<<','<<p.track_id<<','<<num(p.confidence)<<']';}
  o<<"]}";return o.str();
}
struct Finally{
  std::function<void()> action;
  explicit Finally(std::function<void()> f):action(std::move(f)){}
  Finally(const Finally&)=delete;Finally& operator=(const Finally&)=delete;
  ~Finally(){action();}
};
inline void emit(const std::string& body){
  const char* path=std::getenv("DELIVERY_REPLAY_LOG");
  if(!path||!*path)throw std::runtime_error("missing delivery replay log");
  std::ofstream f(path,std::ios::app);
  if(!f)throw std::runtime_error("cannot open delivery replay log");
  f<<"{\"schema\":\"delivery_replay_v1\",\"request_id\":"<<dual_probe::request()
   <<",\"phase\":"<<quote(phase())<<','<<body<<"}\n";
  if(!f)throw std::runtime_error("cannot write delivery replay log");
}
inline unsigned long long vectorHash(const Eigen::VectorXd& x){
  unsigned long long h=1469598103934665603ULL;
  resource_probe::hashBytes(h,x.data(),x.size()*sizeof(double));return h;
}
inline std::string predictionText(){
  auto*p=coupling_bridge::activePrediction();if(!p)return "null";
  std::ostringstream s;s<<std::setprecision(17)<<"{\"valid\":"<<(p->valid?"true":"false")
   <<",\"reason\":"<<quote(p->reason)<<",\"request_time\":"<<p->request_time
   <<",\"delay\":"<<p->delay<<",\"seed_stamp\":"<<p->seed.stamp
   <<",\"seed_revision\":"<<p->seed.revision<<",\"seed_generation\":"<<p->seed.generation
   <<",\"rows\":[";
  for(size_t i=0;i<p->seed.rows.size();++i){if(i)s<<',';s<<'[';
    for(size_t j=0;j<p->seed.rows[i].size();++j){if(j)s<<',';s<<p->seed.rows[i][j];}s<<']';}
  s<<"],\"execution_targets\":[";
  auto target=[&](const coupling_objective::Target&t){s<<'['<<t.pos.x()<<','<<t.pos.y()<<','
    <<t.yaw<<','<<t.speed<<','<<t.time<<','<<t.gear<<']';};
  for(size_t i=0;i<p->targets.execution.size();++i){if(i)s<<',';target(p->targets.execution[i]);}
  s<<"],\"has_delivery\":"<<(p->targets.has_delivery?"true":"false")<<",\"delivery\":";
  if(p->targets.has_delivery)target(p->targets.delivery);else s<<"null";
  s<<",\"checked_begin\":"<<p->targets.checked_begin<<",\"checked_end\":"<<p->targets.checked_end<<'}';
  return s.str();
}
struct Signature{bool first=false;double cost=0.;unsigned long long x=0,g=0;};
struct RestoreFailure{
  failure_probe::Frame* d=failure_probe::active();std::string reason=d?d->reason:"missing";
  bool complete=d&&d->complete;double value=d?d->rejection_value:0.,time=d?d->rejection_time:0.;
  ~RestoreFailure(){if(d){d->reason=reason;d->complete=complete;d->rejection_value=value;d->rejection_time=time;}}
};
inline std::vector<Signature>& signatures(){static thread_local std::vector<Signature> s;return s;}
struct Accepted{
  Eigen::VectorXd x,g;double cost=0,elapsed=0,step=0;int iteration=0,line_search=0;
  std::string stage;
};
struct Trial;
inline Trial*& active(){static thread_local Trial*p=nullptr;return p;}
struct Trial{
  Trial* previous=nullptr;bool enabled=false,first_seen=false;
  double solver_start=0.;std::vector<Accepted> accepted;
  size_t accepted_total=0;bool truncated=false;
  std::function<std::string(const Accepted&)> reconstruct;
  Signature signature;unsigned long long input=0,raw=0,esdf=0;
  double budget=0,wheel=0,sim=0,frame_now=0;std::string prediction;
  std::string effective_initial="null",effective_final="null";
  std::shared_ptr<const shared_collision::Snapshot> frame;
  Trial(unsigned long long h,double b,double w,unsigned long long r,unsigned long long e):
    previous(active()),enabled(running()),input(h),raw(r),esdf(e),budget(b),wheel(w){
    if(!enabled)return;active()=this;accepted.reserve(512);
    sim=ros::Time::now().toSec();frame=coupling_geometry::currentSnapshot();
    frame_now=coupling_geometry::currentTime();prediction=predictionText();
  }
  void begin(){if(enabled)solver_start=failure_probe::wall();}
  void first(double fx,const Eigen::VectorXd&x,const Eigen::VectorXd&g){
    if(!enabled||first_seen)return;first_seen=true;signature={true,fx,vectorHash(x),vectorHash(g)};
    accepted.push_back({x,g,fx,failure_probe::wall()-solver_start,0.,0,0,"initial"});
  }
  void progress(const Eigen::VectorXd&x,const Eigen::VectorXd&g,double fx,double step,int k,int ls){
    if(!enabled)return;++accepted_total;
    if(accepted.size()>=4096){truncated=true;return;}
    accepted.push_back({x,g,fx,failure_probe::wall()-solver_start,step,k,ls,"accepted"});
  }
  ~Trial() noexcept{
    if(!enabled)return;
    RestoreFailure restore;
    try{
      // Save the actual solver verdict BEFORE the expensive canonical audits.
      auto*d=failure_probe::active();const std::string reason=d?d->reason:"missing";
      const bool success=d&&d->complete;const double reject=d?d->rejection_value:0.,rt=d?d->rejection_time:0.;
      signatures().push_back(signature);
      emit("\"kind\":\"solve_summary\",\"input_hash\":"+quote(std::to_string(input))+
        ",\"budget_s\":"+num(budget)+",\"wheel_limit\":"+num(wheel)+
        ",\"sim\":"+num(sim)+",\"sim_end\":"+num(ros::Time::now().toSec())+
        ",\"raw_seq\":"+std::to_string(raw)+",\"esdf_seq\":"+std::to_string(esdf)+
        ",\"frame_now\":"+num(frame_now)+",\"frame_sequence\":"+num(frame?frame->sequence:0)+
        ",\"prediction_hash\":"+quote(delivery_diag::hash(prediction))+
        ",\"effective_initial_states\":"+effective_initial+",\"effective_final_states\":"+effective_final+
        ",\"first_seen\":"+(signature.first?"true":"false")+
        ",\"first_cost\":"+num(signature.cost)+",\"first_x_hash\":"+quote(std::to_string(signature.x))+
        ",\"first_gradient_hash\":"+quote(std::to_string(signature.g))+
        ",\"native_return_success\":"+(success?"true":"false")+",\"reason\":"+quote(reason)+
        ",\"rejection_value\":"+num(reject)+",\"rejection_time\":"+num(rt)+
        ",\"accepted_total\":"+std::to_string(accepted_total)+",\"accepted_truncated\":"+(truncated?"true":"false")+
        ",\"capture_scope\":\"paused same-process optimizer diagnostic, not online publication\"");
      const double audit_start=failure_probe::wall();
      for(const auto&a:accepted){
        // Reconstruction uses canonical x, never the last trial callback cache.
        const std::string rebuilt=reconstruct?reconstruct(a):"\"reconstructed\":false";
        emit("\"kind\":\"canonical_candidate\",\"stage\":"+quote(a.stage)+
          ",\"iteration\":"+std::to_string(a.iteration)+",\"line_search\":"+std::to_string(a.line_search)+
          ",\"solver_elapsed_s\":"+num(a.elapsed)+",\"cost\":"+num(a.cost)+",\"step\":"+num(a.step)+
          ",\"x\":"+delivery_diag::vector(a.x)+",\"g\":"+delivery_diag::vector(a.g)+','+rebuilt);
      }
      emit("\"kind\":\"post_solver_audit_end\",\"audit_wall_s\":"+num(failure_probe::wall()-audit_start)+
        ",\"audited_candidates\":"+std::to_string(accepted.size())+
        ",\"not_part_of_lbfgs_budget\":true,\"does_not_certify_future_live_publication\":true");
      // A diagnostic candidate audit must not overwrite the solver's reason.
      if(d){d->reason=reason;d->complete=success;d->rejection_value=reject;d->rejection_time=rt;}
    }catch(const std::exception&e){
      auditFailed()=true;
      try{emit("\"kind\":\"audit_error\",\"reason\":"+quote(e.what()));}catch(...){}
    }catch(...){
      auditFailed()=true;
      try{emit("\"kind\":\"audit_error\",\"reason\":\"unknown exception\"");}catch(...){}
    }
    active()=previous;
  }
};
inline void first(double fx,const Eigen::VectorXd&x,const Eigen::VectorXd&g){if(active())active()->first(fx,x,g);}
inline void progress(const Eigen::VectorXd&x,const Eigen::VectorXd&g,double fx,double step,int k,int ls){
  if(active())active()->progress(x,g,fx,step,k,ls);
}
inline bool sameFirst(){
  const auto&s=signatures();if(s.size()!=3)return false;
  for(const auto&v:s)if(!v.first||v.cost!=s[0].cost||v.x!=s[0].x||v.g!=s[0].g)return false;
  return true;
}
} // namespace delivery_replay
