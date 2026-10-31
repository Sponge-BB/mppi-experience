#pragma once
// Isolated delivery experiment: retain an actual solver iterate, never a trial
// callback's polynomial cache. No collision checks or file output in callbacks.
#include <Eigen/Eigen>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace delivery_candidate {
inline bool requested() {
  const char* value=std::getenv("DELIVERY_ACCEPT_CANDIDATE");
  return value && std::strcmp(value,"1")==0;
}
inline unsigned long long hash(const Eigen::VectorXd& x) {
  unsigned long long value=1469598103934665603ULL;
  const auto* bytes=reinterpret_cast<const unsigned char*>(x.data());
  for(std::size_t i=0;i<std::size_t(x.size())*sizeof(double);++i) {
    value^=bytes[i];value*=1099511628211ULL;
  }
  return value;
}
struct Scope;
inline Scope*& active() { static thread_local Scope* value=nullptr;return value; }
struct Scope {
  Scope* previous;
  const bool enabled;
  bool available=false, invalid=false, complete_evaluation=false, initial=true;
  unsigned long long evaluated_hash=0;
  double evaluated_cost=0., cost=0.;
  int accepted_steps=0;
  Eigen::VectorXd x;
  explicit Scope(bool on=requested()):previous(active()),enabled(on) { active()=this; }
  ~Scope() { active()=previous; }
  Scope(const Scope&)=delete;
  Scope& operator=(const Scope&)=delete;
  void beginEvaluation() { if(enabled)complete_evaluation=false; }
  void fail() { if(enabled) { invalid=true;complete_evaluation=false; } }
  void completed(const Eigen::VectorXd& trial,const Eigen::VectorXd& gradient,double f) {
    if(!enabled||invalid)return;
    if(!trial.allFinite()||!gradient.allFinite()||!std::isfinite(f)) { fail();return; }
    complete_evaluation=true;evaluated_hash=hash(trial);evaluated_cost=f;
    // The initial point is a legitimate fully evaluated solver iterate even if
    // the time budget expires before the first accepted line-search step.
    if(!available) { x=trial;cost=f;available=true; }
  }
  void accepted(const Eigen::VectorXd& point,double f) {
    if(!enabled||invalid||!complete_evaluation||!available||
       f!=evaluated_cost||hash(point)!=evaluated_hash)return;
    x=point;cost=f;initial=false;++accepted_steps;
  }
  bool usable() const { return enabled&&available&&!invalid; }
};
inline void beginEvaluation() { if(active())active()->beginEvaluation(); }
inline void fail() { if(active())active()->fail(); }
inline void completed(const Eigen::VectorXd&x,const Eigen::VectorXd&g,double f) {
  if(active())active()->completed(x,g,f);
}
inline void accepted(const Eigen::VectorXd&x,double f) { if(active())active()->accepted(x,f); }

struct Part { Eigen::MatrixXd points,initial,final;Eigen::VectorXd times; };
// Same decision-vector layout and time/gear equations as costFunctionCallback.
// Input boundaries are the effective optimizer boundaries AFTER speed clipping.
inline bool decode(const Eigen::VectorXd& x,const std::vector<int>& pieces,int dim,
                   const std::vector<Eigen::MatrixXd>& initial,
                   const std::vector<Eigen::MatrixXd>& final,double non_singular_speed,
                   std::vector<Part>& out) {
  const int count=pieces.size();
  if(count<1||dim<2||initial.size()!=pieces.size()||final.size()!=pieces.size()||!x.allFinite())return false;
  int expected=(dim+1)*(count-1);
  for(int n:pieces) { if(n<1)return false;expected+=dim*(n-1)+n; }
  if(x.size()!=expected)return false;
  out.clear();out.resize(count);int at=0;
  for(int n=0;n<count;++n) {
    if(initial[n].rows()!=dim||final[n].rows()!=dim||initial[n].cols()!=4||final[n].cols()!=4)return false;
    out[n].points=Eigen::Map<const Eigen::MatrixXd>(x.data()+at,dim,pieces[n]-1);
    at+=dim*(pieces[n]-1);
    out[n].initial=initial[n];out[n].final=final[n];
  }
  for(int n=0;n<count;++n) {
    out[n].times.resize(pieces[n]);
    for(int j=0;j<pieces[n];++j) {
      const double v=x[at++];
      out[n].times[j]=v>0.?((.5*v+1.)*v+1.):1./((.5*v-1.)*v+1.);
    }
    if(!out[n].times.allFinite()||(out[n].times.array()<=0.).any())return false;
  }
  const int gear_at=at,angle_at=at+dim*(count-1);
  for(int n=0;n<count;++n) {
    if(n>0) {
      out[n].initial.col(0)=Eigen::Map<const Eigen::VectorXd>(x.data()+gear_at+dim*(n-1),dim);
      const double angle=x[angle_at+n-1];
      out[n].initial.col(1).head(2)=Eigen::Vector2d(-non_singular_speed*std::cos(angle),-non_singular_speed*std::sin(angle));
    }
    if(n<count-1) {
      out[n].final.col(0)=Eigen::Map<const Eigen::VectorXd>(x.data()+gear_at+dim*n,dim);
      const double angle=x[angle_at+n];
      out[n].final.col(1).head(2)=Eigen::Vector2d(non_singular_speed*std::cos(angle),non_singular_speed*std::sin(angle));
    }
  }
  return true;
}
} // namespace delivery_candidate
