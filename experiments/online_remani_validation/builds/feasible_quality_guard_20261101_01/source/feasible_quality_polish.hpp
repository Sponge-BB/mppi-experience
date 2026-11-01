#pragma once
// Isolated SIL: retain a certified incumbent while the unchanged objective
// continues under its original deadline. This helper performs no safety check.
#include "delivery_accepted_candidate.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace feasible_quality_polish {
inline bool requested(){
  const char* p=std::getenv("FEASIBLE_QUALITY_POLISH");return p&&std::string(p)=="1";
}
struct Snapshot {
  bool valid=false;
  unsigned long long x_hash=0;
  double smooth=0.,time=0.,geometry=0.,prediction=0.,total=0.,gradient_norm=0.;
  std::array<double,6> components{};
};
struct Scope;
inline Scope*& current(){static thread_local Scope* p=nullptr;return p;}
struct Scope {
  Scope* previous;Snapshot initial,evaluated;
  Scope():previous(current()){current()=this;}
  ~Scope(){current()=previous;}
  Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
};
inline void completed(const Eigen::VectorXd& x,const Eigen::VectorXd& gradient,
                      double smooth,const Eigen::VectorXd& components,double time,
                      double geometry,double prediction,double total){
  auto* scope=current();if(!scope)return;
  Snapshot value;value.valid=components.size()==6&&x.allFinite()&&gradient.allFinite()&&
    std::isfinite(smooth)&&std::isfinite(time)&&std::isfinite(geometry)&&
    std::isfinite(prediction)&&std::isfinite(total)&&components.allFinite();
  if(value.valid){
    value.x_hash=delivery_candidate::hash(x);value.smooth=smooth;value.time=time;
    value.geometry=geometry;value.prediction=prediction;value.total=total;
    value.gradient_norm=gradient.norm();
    for(int i=0;i<6;++i)value.components[i]=components[i];
  }
  scope->evaluated=value;if(!scope->initial.valid&&value.valid)scope->initial=value;
}
inline Snapshot matching(const Eigen::VectorXd& x,double accepted_cost){
  const auto* scope=current();
  if(scope&&scope->evaluated.valid&&scope->evaluated.x_hash==delivery_candidate::hash(x)&&
     scope->evaluated.total==accepted_cost)return scope->evaluated;
  return Snapshot();
}
// Pure selection policy. The caller supplies an actual accepted point and the
// result of its ordinary native check; no synthetic verdict certifies motion.
struct Checkpoint {
  bool available=false;
  Eigen::VectorXd first_x,best_x;
  double first_cost=std::numeric_limits<double>::infinity();
  double best_cost=std::numeric_limits<double>::infinity();
  Snapshot first_snapshot,best_snapshot;
  int improvements=0,native_checks=0;
  int first_iteration=0,best_iteration=0;
  double first_elapsed_s=0.,best_elapsed_s=0.;
  bool better(double cost)const{return std::isfinite(cost)&&(!available||cost<best_cost);}
  bool consider(const Eigen::VectorXd& x,double cost,bool native_pass,bool within_budget,
                const Snapshot& snapshot,int iteration=0,double elapsed_s=0.){
    if(!native_pass||!within_budget||!x.allFinite()||!better(cost))return false;
    if(!available){first_x=x;first_cost=cost;first_snapshot=snapshot;
      first_iteration=iteration;first_elapsed_s=elapsed_s;}
    else ++improvements;
    best_x=x;best_cost=cost;best_snapshot=snapshot;best_iteration=iteration;
    best_elapsed_s=elapsed_s;available=true;return true;
  }
};
inline std::string number(double value){
  if(!std::isfinite(value))return "null";
  std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();
}
inline std::string snapshotJson(const Snapshot& s){
  if(!s.valid)return "null";
  const char* names[]={"base_obstacle","arm_obstacle","self_collision",
                       "base_feasibility","arm_feasibility","other_integral"};
  std::ostringstream out;out<<std::setprecision(17)<<"{\"valid\":true,\"x_hash\":\""<<s.x_hash
    <<"\",\"smooth\":"<<number(s.smooth)<<",\"time\":"<<number(s.time)
    <<",\"geometry\":"<<number(s.geometry)<<",\"prediction\":"<<number(s.prediction)
    <<",\"total\":"<<number(s.total)<<",\"gradient_norm\":"<<number(s.gradient_norm);
  for(int i=0;i<6;++i)out<<",\""<<names[i]<<"\":"<<number(s.components[i]);
  return out.str()+"}";
}
} // namespace feasible_quality_polish
