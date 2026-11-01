#pragma once
// Accepted-iterate gate only. No objective, gradient, native cache or physical
// constraint is changed. Native checks run on independently owned MINCO data.
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "delivery_accepted_candidate.hpp"
#include "feasible_quality_polish.hpp"
#include "traj_utils/plan_container.hpp"

namespace early_delivery {
inline double monotonic(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline bool requested(){const char* s=std::getenv("DELIVERY_EARLY_EXIT");return s&&std::strcmp(s,"1")==0;}
struct Check {
  int accepted_iteration=0;unsigned long long candidate_hash=0;
  int collision_type=-1;double rejection_time=0.;
  double begin_mono_s=0.,end_mono_s=0.,elapsed_begin_s=0.,elapsed_end_s=0.;
  double reconstruction_s=0.,wheel_s=0.,native_s=0.;
  bool wheel_pass=false,native_checked=false,native_pass=false,selected=false,budget_expired=false;
  std::string reason="unchecked",native_reason="not_checked";
};
struct NativeVerdict { bool pass=false;std::string reason="unknown";
  int collision_type=-1;double rejection_time=0.; };

class Checker {
  std::vector<int> pieces_,signs_;
  std::vector<Eigen::MatrixXd> initial_,final_;
  std::vector<poly_traj::MinSnapOpt<8>> snaps_;
  double non_singular_,base_,radius_,omega_,alpha_;
 public:
  Checker(const std::vector<int>& pieces,const std::vector<int>& signs,
      const std::vector<Eigen::MatrixXd>& initial,const std::vector<Eigen::MatrixXd>& final,
      double non_singular,double base,double radius,double omega,double alpha):
      pieces_(pieces),signs_(signs),initial_(initial),final_(final),non_singular_(non_singular),
      base_(base),radius_(radius),omega_(omega),alpha_(alpha){
    snaps_.resize(pieces.size());for(std::size_t i=0;i<pieces.size();++i)snaps_[i].reset(pieces[i]);
  }
  Checker(const Checker&)=delete;Checker& operator=(const Checker&)=delete;
  remani_planner::SingulTrajData trajectory(const Eigen::VectorXd& x){
    std::vector<delivery_candidate::Part> parts;
    if(!delivery_candidate::decode(x,pieces_,8,initial_,final_,non_singular_,parts))throw std::runtime_error("early gate cannot decode accepted x");
    remani_planner::SingulTrajData out;out.clearSingulTraj();double prefix=0.;
    for(std::size_t i=0;i<parts.size();++i){
      snaps_[i].generate(parts[i].points,parts[i].times,parts[i].initial,parts[i].final);
      out.addSingulTraj(snaps_[i].getTraj(signs_[i]),prefix);prefix=out.duration;
    }return out;
  }
  bool wheelPass(const remani_planner::SingulTrajData& t)const{
    // Same native t=.05, dt=.01, excluded final endpoint and 5% tolerance.
    const double end=std::floor(t.duration/.01)*.01;
    const double omega2=omega_*omega_*1.05*1.05,alpha2=alpha_*alpha_*1.05*1.05;
    for(double at=.05;at<end;at+=.01){
      const Eigen::Vector2d v=t.getVel(at).head(2),a=t.getAcc(at).head(2),j=t.getJer(at).head(2);
      const double norm=std::max(1e-4,v.norm()),vv=std::max(1e-8,v.squaredNorm());
      const double av=a.dot(v),abv=a.y()*v.x()-a.x()*v.y(),jbv=j.y()*v.x()-j.x()*v.y();
      const double yaw=abv/vv,yaw_acc=jbv/vv-2.*abv*av/(vv*vv),sign=t.getSingul(at);
      const double wl=(2.*sign*norm-base_*yaw)/(2.*radius_),wr=(2.*sign*norm+base_*yaw)/(2.*radius_);
      const double al=(2.*sign*av/norm-base_*yaw_acc)/(2.*radius_),ar=(2.*sign*av/norm+base_*yaw_acc)/(2.*radius_);
      if(wl*wl>omega2||wr*wr>omega2||al*al>alpha2||ar*ar>alpha2)return false;
    }return true;
  }
};

struct Session;
inline Session*& active(){static thread_local Session* s=nullptr;return s;}
struct Session {
  using Elapsed=std::function<double()>;
  using Native=std::function<NativeVerdict(const remani_planner::SingulTrajData&)>;
  Session* previous;bool enabled=false,selected=false;
  const bool polish=feasible_quality_polish::requested();
  feasible_quality_polish::Checkpoint quality;
  Checker* checker=nullptr;Elapsed elapsed;Native native;double budget=0.;
  double last_native_start=-std::numeric_limits<double>::infinity();
  Eigen::VectorXd selected_x;std::vector<Check> checks;
  Session(bool on,Checker* c,Elapsed e,double max_time,Native n):previous(active()),enabled(on),checker(c),elapsed(std::move(e)),native(std::move(n)),budget(max_time){if(enabled)active()=this;}
  ~Session(){if(enabled)active()=previous;}
  Session(const Session&)=delete;Session& operator=(const Session&)=delete;
  bool accepted(const Eigen::VectorXd& x,int iteration){
    if(!enabled)return false;
    if(selected&&!polish)return true;
    Check c;c.accepted_iteration=iteration;c.candidate_hash=delivery_candidate::hash(x);
    c.begin_mono_s=monotonic();c.elapsed_begin_s=elapsed();
    const auto* actual=delivery_candidate::active();
    const bool accepted_matches=actual&&actual->usable()&&!actual->initial&&
      actual->complete_evaluation&&actual->evaluated_hash==c.candidate_hash&&
      delivery_candidate::hash(actual->x)==c.candidate_hash&&actual->cost==actual->evaluated_cost;
    const double accepted_cost=accepted_matches?actual->cost:std::numeric_limits<double>::infinity();
    const auto objective=feasible_quality_polish::matching(x,accepted_cost);
    if(polish&&(!accepted_matches||!objective.valid||!quality.better(accepted_cost))){
      c.reason=!accepted_matches?"quality_missing_accepted_cost":
        !objective.valid?"quality_missing_objective":"quality_not_better";
      c.elapsed_end_s=elapsed();c.end_mono_s=monotonic();checks.push_back(c);return false;
    }
    if(c.elapsed_begin_s>=budget){c.budget_expired=true;c.reason="budget_before_check";}
    else{
      auto began=monotonic();const auto candidate=checker->trajectory(x);c.reconstruction_s=monotonic()-began;
      began=monotonic();c.wheel_pass=checker->wheelPass(candidate);c.wheel_s=monotonic()-began;
      if(!c.wheel_pass)c.reason="wheel_precheck_reject";
      else if(elapsed()>=budget){c.budget_expired=true;c.reason="budget_after_wheel";}
      else if(monotonic()-last_native_start<.1)c.reason="native_check_interval";
      else{
        c.native_checked=true;++quality.native_checks;last_native_start=monotonic();began=last_native_start;
        const auto verdict=native(candidate);c.native_pass=verdict.pass;c.native_reason=verdict.reason;c.collision_type=verdict.collision_type;
        c.rejection_time=verdict.rejection_time;c.native_s=monotonic()-began;
        c.budget_expired=elapsed()>=budget;
        c.selected=c.native_pass&&!c.budget_expired;
        c.reason=c.budget_expired?"budget_after_native":(c.native_pass?(polish?"native_pass_incumbent":"native_pass_early_exit"):"native_gate_reject");
        if(c.selected){
          selected=true;selected_x=x;
          quality.consider(x,accepted_cost,true,true,objective,iteration,elapsed());
        }
      }
    }
    c.elapsed_end_s=elapsed();c.end_mono_s=monotonic();checks.push_back(c);return selected&&!polish;
  }
};
inline bool accepted(const Eigen::VectorXd& x,int iteration){auto* s=active();return s&&s->accepted(x,iteration);}
} // namespace early_delivery
