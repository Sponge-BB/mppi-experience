#pragma once
// One same-request continuation after an environmental native collision reject.
// This header is used only by the isolated numerical-SIL experimental build.
#include "temporal_collision_capture.hpp"
#include "delivery_accepted_candidate.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <map>
#include <utility>

namespace candidate_collision_repair {
inline bool requested(){const char* p=std::getenv("CANDIDATE_COLLISION_REPAIR");return p&&std::string(p)=="1";}
constexpr double factor=10.;
struct PieceStats {
  double base_cost=0.,arm_cost=0.,self_cost=0.;
  double position_gradient_norm_sum=0.,velocity_gradient_norm_sum=0.,temporal_gradient_abs_sum=0.;
  unsigned long long samples=0,penalized_samples=0;
};
struct Objective {
  bool valid=false;unsigned long long hash=0;double total=0.,smooth=0.,time=0.,geometry=0.,prediction=0.,gradient_norm=0.;
  Eigen::VectorXd components;
  std::map<std::pair<int,int>,PieceStats> pieces;
};
struct Scope;
inline Scope*& current(){static thread_local Scope* value=nullptr;return value;}
struct Scope {
  Scope* previous;bool enabled=requested(),repairing=false;
  int target_trajectory=-1,target_piece=-1;
  std::map<std::pair<int,int>,PieceStats> pieces;
  Objective evaluated,accepted;
  Scope():previous(current()){current()=this;}
  ~Scope(){current()=previous;}
  Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
  void target(const std::vector<Eigen::VectorXd>& times,double t){
    double elapsed=0.;
    for(std::size_t n=0;n<times.size();++n)for(int j=0;j<times[n].size();++j){
      const double end=elapsed+times[n][j];
      if(t<end+1e-10){target_trajectory=n;target_piece=j;return;}elapsed=end;
    }
  }
  const Objective* matching(const Eigen::VectorXd& x)const{
    const auto h=delivery_candidate::hash(x);
    if(evaluated.valid&&evaluated.hash==h)return &evaluated;
    if(accepted.valid&&accepted.hash==h)return &accepted;
    return nullptr;
  }
};
inline void beginEvaluation(){if(current())current()->pieces.clear();}
inline void completed(const Eigen::VectorXd& x,const Eigen::VectorXd& gradient,double total,double smooth,
                      const Eigen::VectorXd& components,double time,double geometry,double prediction){
  auto* s=current();if(!s)return;
  auto& e=s->evaluated;e.valid=x.allFinite()&&gradient.allFinite()&&std::isfinite(total);
  e.hash=delivery_candidate::hash(x);e.total=total;e.smooth=smooth;e.components=components;
  e.time=time;e.geometry=geometry;e.prediction=prediction;e.gradient_norm=gradient.norm();e.pieces=s->pieces;
}
inline void accepted(const Eigen::VectorXd& x){auto* s=current();if(s&&s->evaluated.valid&&s->evaluated.hash==delivery_candidate::hash(x))s->accepted=s->evaluated;}
inline double& sample_weight(){static thread_local double value=1.;return value;}
inline double weight(int trajectory,int piece){auto* s=current();return s&&s->repairing&&s->target_trajectory==trajectory&&s->target_piece==piece?factor:1.;}
inline void sample(int trajectory,int piece,double integration,double base,double arm,double self,
                   const Eigen::VectorXd& gp,const Eigen::VectorXd& gv,double temporal){
  auto* s=current();if(!s)return;auto& p=s->pieces[{trajectory,piece}];
  ++p.samples;if(base>0.||arm>0.)++p.penalized_samples;
  p.base_cost+=integration*base;p.arm_cost+=integration*arm;p.self_cost+=integration*self;
  p.position_gradient_norm_sum+=integration*gp.norm();p.velocity_gradient_norm_sum+=integration*gv.norm();
  p.temporal_gradient_abs_sum+=integration*std::abs(temporal);
}
inline dual_probe::Fields diagnostics(const Scope& s,const Eigen::VectorXd& x){
  dual_probe::Fields f;const auto* o=s.matching(x);f.add("objective_matches_x",o?"true":"false");
  if(!o)return f;
  f.add("objective_total",o->total).add("objective_smooth",o->smooth).add("objective_time",o->time)
   .add("objective_geometry",o->geometry).add("objective_prediction",o->prediction)
   .add("objective_gradient_norm",o->gradient_norm).add("objective_components",delivery_diag::vector(o->components));
  const auto it=o->pieces.find({s.target_trajectory,s.target_piece});
  if(it!=o->pieces.end()){const auto& p=it->second;
    f.add("piece_base_soft_cost",p.base_cost).add("piece_arm_soft_cost",p.arm_cost).add("piece_self_soft_cost",p.self_cost)
     .add("piece_position_gradient_norm_sum",p.position_gradient_norm_sum)
     .add("piece_velocity_gradient_norm_sum",p.velocity_gradient_norm_sum)
     .add("piece_temporal_gradient_abs_sum",p.temporal_gradient_abs_sum)
     .add("piece_sample_count",p.samples).add("piece_penalized_sample_count",p.penalized_samples);
  }
  return f;
}
template<class Data>void candidate(const std::string& stage,const Data& data,bool success,const std::string& reason){
  if(!temporal_capture::enabled())return;const auto plan=temporal_capture::currentPlan();
  if(!plan||plan->request!=dual_probe::request())return;
  const auto copied=data;
  temporal_capture::enqueue(stage,[plan,copied,success,reason](){return std::string("\"plan_ordinal\":")+
    std::to_string(plan->ordinal)+",\"native_success\":"+(success?"true":"false")+
    ",\"native_reason\":"+delivery_diag::quote(reason)+",\"candidate\":"+temporal_capture::candidateJson(copied);});
}
} // namespace candidate_collision_repair
