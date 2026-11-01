#pragma once
// Isolated SIL: retain actual accepted iterates and audit one predetermined
// dynamic refresh. Paused diagnostics never constitute online timing results.
#include "candidate_collision_repair.hpp"
#include "early_delivery_gate.hpp"
#include "domain_lbfgs.hpp"
#include <Eigen/Dense>
#include <cstdlib>
#include <sys/stat.h>
#include <cerrno>

namespace candidate_exit_audit {
inline bool flag(const char* name){const char* p=std::getenv(name);return p&&std::string(p)=="1";}
inline bool selected(bool eligible){
  static bool used=false;
  if(!flag("CANDIDATE_EXIT_AUDIT")||!eligible||used)return false;
  used=true;dual_probe::emit("candidate_exit_audit_selected",dual_probe::Fields()
    .str("selection","first eligible dynamic refresh, before knowing optimizer outcome"));return true;
}
inline bool canRecover(bool eligible,int result,int force_stop,bool usable){
  // Only a finite actual iterate after line-search exhaustion is eligible.
  // It still must pass the full native gate and the unchanged latest gate.
  return flag("CANDIDATE_EXIT_RECOVER")&&eligible&&result==domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH&&force_stop==0&&usable;
}
struct Evaluation {
  bool valid=false;Eigen::VectorXd x,gradient;double cost=0.;
  unsigned long long x_hash=0,gradient_hash=0;
};
struct Scope;
inline Scope*& current(){static thread_local Scope* value=nullptr;return value;}
struct Scope {
  Scope* previous;bool enabled;Evaluation initial,evaluated,accepted;
  explicit Scope(bool on):previous(current()),enabled(on){current()=this;}
  ~Scope(){current()=previous;}
  Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
};
inline void completed(const Eigen::VectorXd& x,const Eigen::VectorXd& gradient,double cost){
  auto* s=current();if(!s||!s->enabled)return;
  auto& e=s->evaluated;e.valid=x.allFinite()&&gradient.allFinite()&&std::isfinite(cost);
  if(!e.valid)return;e.x=x;e.gradient=gradient;e.cost=cost;
  e.x_hash=delivery_candidate::hash(x);e.gradient_hash=delivery_candidate::hash(gradient);
  if(!s->initial.valid)s->initial=e;
  if(!s->accepted.valid)s->accepted=e;
}
inline void accepted(const Eigen::VectorXd& x,double cost){
  auto* s=current();if(!s||!s->enabled)return;
  if(s->evaluated.valid&&s->evaluated.x_hash==delivery_candidate::hash(x)&&s->evaluated.cost==cost)
    s->accepted=s->evaluated;
}
inline std::string evaluationJson(const Evaluation& e){
  if(!e.valid)return "null";
  return std::string("{\"cost\":")+delivery_diag::number(e.cost)+
    ",\"x_hash\":"+delivery_diag::quote(std::to_string(e.x_hash))+
    ",\"gradient_hash\":"+delivery_diag::quote(std::to_string(e.gradient_hash))+
    ",\"x\":"+delivery_diag::vector(e.x)+",\"gradient\":"+delivery_diag::vector(e.gradient)+"}";
}
struct Verdict {
  bool available=false,native_pass=false;std::string reason="not_reconstructed";
  double value=0.,time=0.,reconstruct_ms=0.,native_ms=0.;
};
inline std::string verdictJson(const Verdict& v){
  return std::string("{\"available\":")+(v.available?"true":"false")+
    ",\"native_pass\":"+(v.native_pass?"true":"false")+
    ",\"reason\":"+delivery_diag::quote(v.reason)+",\"rejection_value\":"+delivery_diag::number(v.value)+
    ",\"rejection_time\":"+delivery_diag::number(v.time)+
    ",\"reconstruction_ms\":"+delivery_diag::number(v.reconstruct_ms)+
    ",\"native_ms\":"+delivery_diag::number(v.native_ms)+"}";
}
inline std::string directory(){
  const char* directory=std::getenv("CANDIDATE_EXIT_AUDIT_DIR");
  if(!directory||!*directory)throw std::runtime_error("missing candidate exit audit directory");
  if(mkdir(directory,0775)!=0&&errno!=EEXIST)throw std::runtime_error("cannot create candidate exit audit directory");
  return directory;
}
inline void append(const std::string& name,const std::string& text){
  std::ofstream out(directory()+"/"+name,std::ios::app);
  if(!out)throw std::runtime_error("cannot open candidate exit audit output");
  out<<text<<'\n';if(!out)throw std::runtime_error("cannot write candidate exit audit output");
}
} // namespace candidate_exit_audit
