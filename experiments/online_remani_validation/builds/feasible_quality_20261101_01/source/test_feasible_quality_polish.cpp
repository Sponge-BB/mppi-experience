#include "feasible_quality_polish.hpp"
#include <iostream>

int main(){
  using namespace feasible_quality_polish;
  Checkpoint value;Eigen::VectorXd first(2),next(2);first<<1.,2.;next<<3.,4.;
  Snapshot snapshot;snapshot.valid=true;snapshot.total=100.;snapshot.x_hash=delivery_candidate::hash(first);
  const bool initial=value.consider(first,100.,true,true,snapshot)&&value.available;
  const bool unsafe=!value.consider(next,80.,false,true,snapshot)&&value.best_x==first&&value.best_cost==100.;
  const bool late=!value.consider(next,70.,true,false,snapshot)&&value.best_x==first;
  const bool worse=!value.consider(next,110.,true,true,snapshot)&&value.best_cost==100.;
  snapshot.total=90.;snapshot.x_hash=delivery_candidate::hash(next);
  const bool improved=value.consider(next,90.,true,true,snapshot)&&value.improvements==1&&
    value.first_x==first&&value.first_cost==100.&&value.best_x==next&&value.best_cost==90.;
  const bool equal=!value.consider(first,90.,true,true,snapshot)&&value.best_x==next;
  auto nan=next;nan[0]=std::numeric_limits<double>::quiet_NaN();
  const bool finite=!value.consider(nan,1.,true,true,snapshot)&&value.best_x==next;
  Scope scope;Eigen::VectorXd gradient=Eigen::VectorXd::Ones(2),components=Eigen::VectorXd::Constant(6,2.);
  completed(first,gradient,3.,components,4.,5.,6.,30.);
  const bool matched=matching(first,30.).valid&&!matching(next,30.).valid&&!matching(first,29.).valid;
  completed(next,gradient,1.,components,2.,3.,4.,22.);
  const bool snapshots=scope.initial.x_hash==delivery_candidate::hash(first)&&scope.initial.total==30.&&
    matching(next,22.).valid&&scope.evaluated.total==22.;
  unsetenv("FEASIBLE_QUALITY_POLISH");const bool off=!requested();
  setenv("FEASIBLE_QUALITY_POLISH","1",1);const bool on=requested();unsetenv("FEASIBLE_QUALITY_POLISH");
  const bool pass=initial&&unsafe&&late&&worse&&improved&&equal&&finite&&matched&&snapshots&&off&&on;
  std::cout<<"{\"pass\":"<<(pass?"true":"false")
    <<",\"initial_retained\":"<<(initial?"true":"false")
    <<",\"unsafe_or_late_cannot_replace\":"<<(unsafe&&late?"true":"false")
    <<",\"strictly_better_only\":"<<(worse&&equal?"true":"false")
    <<",\"safe_improvement_keeps_first\":"<<(improved?"true":"false")
    <<",\"objective_snapshot_matches_x_and_cost\":"<<(matched&&snapshots?"true":"false")
    <<",\"scope\":\"checkpoint bookkeeping only; artificial verdicts do not test collision safety\"}\n";
  return pass?0:1;
}
