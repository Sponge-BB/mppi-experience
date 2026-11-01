#pragma once
// Isolated diagnostic coordinates only: no objective, boundary or safety changes.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include "feasibility_stop_gate.hpp"

namespace budget_scaling {
using Vector=Eigen::VectorXd;
inline bool scaledPhase(const std::string& phase){return phase=="budget_scaled_300";}
inline bool vetoPhase(const std::string& phase){
  return phase=="budget_reference_060" || phase=="budget_extended_300" ||
         phase=="budget_scaled_300" || phase=="budget_repeat_060";
}
inline bool identical(const Vector& a,const Vector& b){
  return a.size()==b.size() &&
    !std::memcmp(a.data(),b.data(),std::size_t(a.size())*sizeof(double));
}
struct CanonicalSession;
inline CanonicalSession*& canonicalSession(){static thread_local CanonicalSession* s=nullptr;return s;}
struct CanonicalSession {
  CanonicalSession* previous;
  bool enabled=false,evaluated=false,accepted=false;
  Vector last_x,last_accepted_x;
  explicit CanonicalSession(bool active):previous(canonicalSession()),enabled(active){
    if(enabled)canonicalSession()=this;
  }
  ~CanonicalSession(){if(enabled)canonicalSession()=previous;}
};
inline void recordCanonical(const Vector& x){
  auto* s=canonicalSession();if(!s)return;
  if(!x.allFinite())throw std::runtime_error("nonfinite canonical evaluation record");
  s->last_x=x;s->evaluated=true;
  if(!s->accepted){s->last_accepted_x=x;s->accepted=true;}
}
inline void recordAccepted(const Vector& x){
  auto* s=canonicalSession();if(!s)return;
  if(!s->evaluated || !identical(x,s->last_x))
    throw std::runtime_error("canonical accepted/native-cache mismatch");
  s->last_accepted_x=x;s->accepted=true;
}
struct FinalState {
  bool solver_result_known=false,native_known=false,native_pass=false;
  bool projected_known=false,projected_pass=false,output_constructed=false,allowed=false;
  int solver_result=0;
  unsigned long long output_hash=0;
  double decision_elapsed_s=-1.;
};
struct OnExit {
  bool enabled;
  std::function<void()> report;
  OnExit(bool active,std::function<void()> callback):enabled(active),report(std::move(callback)){}
  ~OnExit()noexcept{if(enabled)try{report();}catch(...){/* Missing marker fails the external audit. */}}
};
struct Plan {
  Vector diagonal;
  double minimum_time=0.,maximum_time=0.;
  // A numerical-only lower bound, chosen from machine precision, not outcomes.
  double numerical_floor=std::sqrt(std::numeric_limits<double>::epsilon());
  std::size_t floor_count=0,scaled_count=0;
};
inline Plan makePlan(const std::vector<Vector>& times,int dimension=8,int base_dof=2){
  if(times.empty() || dimension<base_dof || base_dof!=2)
    throw std::invalid_argument("invalid coordinate-scaling layout");
  Plan p;p.minimum_time=std::numeric_limits<double>::infinity();
  int pieces=0,positions=0;
  for(const auto& t:times){
    if(t.size()<1 || !t.allFinite() || (t.array()<=0.).any())
      throw std::invalid_argument("coordinate scaling requires positive finite initial times");
    p.minimum_time=std::min(p.minimum_time,t.minCoeff());
    p.maximum_time=std::max(p.maximum_time,t.maxCoeff());
    pieces+=t.size();positions+=dimension*(t.size()-1);
  }
  p.diagonal=Vector::Ones(positions+pieces+(dimension+1)*(times.size()-1));
  const auto pairScale=[&](double left,double right){
    // MinSnap's integral of squared fourth derivative scales as T^-7.
    // Log-sum-exp avoids overflow for an extreme but finite initial time ratio.
    const double a=7.*(std::log(p.maximum_time)-std::log(left));
    const double b=7.*(std::log(p.maximum_time)-std::log(right));
    const double m=std::max(a,b);
    const double log_d=.5*(std::log(2.)-m-std::log(std::exp(a-m)+std::exp(b-m)));
    if(log_d<std::log(p.numerical_floor))return p.numerical_floor;
    return std::min(1.,std::exp(log_d));
  };
  const auto assign=[&](int at,double value){
    for(int axis=0;axis<base_dof;++axis){
      p.diagonal[at+axis]=value;
      p.floor_count+=value==p.numerical_floor;p.scaled_count+=value<1.;
    }
  };
  int offset=0;
  for(const auto& t:times){
    for(int node=0;node<t.size()-1;++node)
      assign(offset+dimension*node,pairScale(t[node],t[node+1]));
    offset+=dimension*(t.size()-1);
  }
  offset+=pieces; // Virtual-time coordinates remain unscaled.
  for(std::size_t block=0;block+1<times.size();++block)
    assign(offset+dimension*block,pairScale(times[block][times[block].size()-1],times[block+1][0]));
  // Gear angles and fixed-arm coordinates remain unscaled.
  if(!p.diagonal.allFinite() || (p.diagonal.array()<=0.).any())
    throw std::runtime_error("noninvertible coordinate diagonal");
  return p;
}

struct Context {
  using Evaluate=double(*)(void*,const Vector&,Vector&);
  using Progress=int(*)(void*,const Vector&,const Vector&,double,double,int,int);
  using Stopped=bool(*)(void*);
  void* native;
  Evaluate native_evaluate;
  Progress native_progress;
  Stopped native_stopped;
  Vector origin,diagonal,last_z,last_x,last_g,last_gz,last_progress_x;
  bool valid=false;
  std::size_t evaluations=0,progress_calls=0,stopped_progress=0;
  double maximum_canonical_step=0.;
  Context(void* object,const Vector& initial,const Vector& d,Evaluate evaluate,
          Progress progress,Stopped stopped):native(object),native_evaluate(evaluate),
      native_progress(progress),native_stopped(stopped),origin(initial),diagonal(d),
      last_progress_x(initial){
    if(!native_evaluate || !native_progress || !native_stopped ||
       origin.size()!=diagonal.size() || !origin.allFinite() ||
       !diagonal.allFinite() || (diagonal.array()<=0.).any())
      throw std::invalid_argument("invalid scaled callback context");
  }
  Vector canonical(const Vector& z)const {
    if(z.size()!=origin.size() || !z.allFinite())
      throw std::invalid_argument("invalid scaled coordinate vector");
    Vector x=origin;
    // Copy zero coordinates verbatim: -0 in the original must stay -0 at z0.
    for(int i=0;i<z.size();++i)if(z[i]!=0.)x[i]+=diagonal[i]*z[i];
    if(!x.allFinite())throw std::runtime_error("nonfinite canonical coordinates");
    return x;
  }
  static double evaluate(void* raw,const Vector& z,Vector& gz){
    auto& c=*static_cast<Context*>(raw);++c.evaluations;
    const Vector x=c.canonical(z);Vector gx=Vector::Zero(z.size());
    // Native domain rejection propagates; it must not overwrite a good cache.
    const double cost=c.native_evaluate(c.native,x,gx);
    gz=c.diagonal.array()*gx.array();
    if(!c.native_stopped(c.native)){
      if(!std::isfinite(cost) || !gx.allFinite() || !gz.allFinite())
        throw std::runtime_error("unflagged nonfinite native evaluation");
      c.last_z=z;c.last_x=x;c.last_g=gx;c.last_gz=gz;c.valid=true;
      // Original callback already recorded canonical x. The solver and stop
      // hook see z, while the native polynomial cache still belongs to x(z).
      feasibility_stop_gate::recordEvaluated(z);
    }
    return cost;
  }
  static int progress(void* raw,const Vector& z,const Vector& gz,double cost,
                      double step,int k,int ls){
    auto& c=*static_cast<Context*>(raw);
    // A timeout/error callback deliberately does not certify its cache. Keep
    // that original verdict, rather than masking it with a cache exception.
    if(c.native_stopped(c.native)){++c.stopped_progress;return 1;}
    if(!c.valid || !identical(z,c.last_z) || !identical(gz,c.last_gz))
      throw std::runtime_error("scaled accepted coordinate/cache mismatch");
    ++c.progress_calls;
    c.maximum_canonical_step=std::max(c.maximum_canonical_step,(c.last_x-c.last_progress_x).norm());
    c.last_progress_x=c.last_x;
    // Forward original canonical gradient, never a rounded inverse transform.
    return c.native_progress(c.native,c.last_x,c.last_g,cost,step,k,ls);
  }
};
} // namespace budget_scaling
