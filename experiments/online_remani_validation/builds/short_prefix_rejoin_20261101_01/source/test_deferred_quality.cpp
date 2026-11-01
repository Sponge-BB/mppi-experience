#include "early_delivery_gate.hpp"
#include <iostream>
#include <map>

// Actual Session and polynomial reconstruction; native predicates and clock are
// synthetic, so these tests verify routing/budget policy, not collision safety.
struct Fixture {
  delivery_candidate::Scope accepted{true};
  feasible_quality_polish::Scope objectives;
  std::vector<int> pieces{1},signs{1};
  std::vector<Eigen::MatrixXd> initial{Eigen::MatrixXd::Zero(8,4)};
  std::vector<Eigen::MatrixXd> final{Eigen::MatrixXd::Zero(8,4)};
  std::unique_ptr<early_delivery::Checker> checker;
  double now=.2,native_duration=.02;int native_calls=0;bool native_pass=true;
  Fixture(){
    initial[0](0,1)=.2;final[0](0,0)=1.;final[0](0,1)=.2;
    checker.reset(new early_delivery::Checker(pieces,signs,initial,final,.1,.5,.1,1000.,1000.));
  }
  Eigen::VectorXd point(double t){Eigen::VectorXd x(1);x<<t;return x;}
  void record(const Eigen::VectorXd& x,double f){
    const Eigen::VectorXd g=Eigen::VectorXd::Ones(1),parts=Eigen::VectorXd::Zero(6);
    delivery_candidate::beginEvaluation();delivery_candidate::completed(x,g,f);
    feasible_quality_polish::completed(x,g,0.,parts,0.,0.,0.,f);
    delivery_candidate::accepted(x,f);
  }
  early_delivery::NativeVerdict native(const remani_planner::SingulTrajData&){
    ++native_calls;now+=native_duration;
    early_delivery::NativeVerdict v;v.pass=native_pass;v.reason=native_pass?"test_pass":"test_reject";return v;
  }
};

int main(){
  setenv("FEASIBLE_QUALITY_POLISH","1",1);
  std::map<std::string,bool> tests;
  for(int mode=0;mode<2;++mode){
    setenv("DEFERRED_QUALITY_CHECK",mode?"1":"0",1);
    Fixture f;early_delivery::Session s(true,f.checker.get(),[&]{return f.now;},1.2,
      [&](const remani_planner::SingulTrajData& t){return f.native(t);});
    s.polish=false;const auto x=f.point(1.);f.record(x,100.);
    const bool stop=s.accepted(x,1);s.finalize(true);
    tests[mode?"q1_deferred_same_first_safe":"q1_control_same_first_safe"]=
      stop&&f.native_calls==1&&s.selected&&!s.deferredEnabled()&&!s.pending.available&&!s.reserveStopDue();
  }
  {
    setenv("DEFERRED_QUALITY_CHECK","0",1);Fixture f;
    early_delivery::Session s(true,f.checker.get(),[&]{return f.now;},1.2,
      [&](const remani_planner::SingulTrajData& t){return f.native(t);});
    auto first=f.point(1.),next=f.point(1.1);f.record(first,100.);const bool first_continues=!s.accepted(first,1);
    f.now=.5;s.last_native_start=-std::numeric_limits<double>::infinity();
    f.record(next,90.);const bool next_continues=!s.accepted(next,2);s.finalize(true);
    tests["control_retains_repeated_native_route"]=first_continues&&next_continues&&f.native_calls==2&&
      s.quality.best_cost==90.&&!s.pending.available&&!s.deferred_check_attempted;
  }
  for(int verdict=0;verdict<4;++verdict){
    setenv("DEFERRED_QUALITY_CHECK","1",1);Fixture f;
    early_delivery::Session s(true,f.checker.get(),[&]{return f.now;},1.2,
      [&](const remani_planner::SingulTrajData& t){return f.native(t);});
    const auto first=f.point(1.),second=f.point(1.1),last=f.point(1.2);
    f.record(first,100.);const bool first_continues=!s.accepted(first,1);
    f.now=.6;f.record(second,90.);const bool deferred_no_native=!s.accepted(second,2)&&f.native_calls==1;
    f.now=1.06;f.record(last,80.);const bool cutoff=s.accepted(last,3)&&s.reserve_stop_requested;
    const auto actual_hash=delivery_candidate::hash(f.accepted.x);
    if(verdict==1)f.native_pass=false;
    if(verdict==2)f.native_duration=.2;
    if(verdict==3)f.accepted.fail();
    s.finalize(verdict!=3);
    const bool intact=delivery_candidate::hash(f.accepted.x)==actual_hash&&f.accepted.cost==80.;
    const bool common=first_continues&&deferred_no_native&&cutoff&&intact&&s.pending.cost==80.;
    if(verdict==0)tests["defer_once_promotes_matching_pending"]=common&&f.native_calls==2&&s.promoted&&
      s.quality.best_cost==80.&&s.quality.first_cost==100.&&s.deferred_check_attempted&&s.deferred_check.native_pass;
    if(verdict==1)tests["failed_native_falls_back_without_forging_last_accepted"]=common&&f.native_calls==2&&
      !s.promoted&&s.quality.best_cost==100.&&delivery_candidate::hash(s.selected_x)!=actual_hash;
    if(verdict==2)tests["native_budget_overrun_cannot_promote"]=common&&f.native_calls==2&&
      !s.promoted&&s.quality.best_cost==100.&&s.deferred_check.budget_expired&&s.deferred_check.native_pass;
    if(verdict==3)tests["invalid_scope_is_not_recovered"]=common&&f.native_calls==1&&
      !s.promoted&&!s.deferred_check_attempted&&f.accepted.invalid&&!f.accepted.usable();
  }
  {
    Fixture f;early_delivery::Session s(true,f.checker.get(),[&]{return f.now;},1.2,
      [&](const remani_planner::SingulTrajData& t){return f.native(t);});
    const auto first=f.point(1.);f.record(first,100.);s.accepted(first,1);
    f.now=.4;auto bad=f.point(1.3);f.record(bad,90.);
    // Mismatched snapshot cannot become pending even when accepted Scope exists.
    f.objectives.evaluated.x_hash=0;s.accepted(bad,2);s.finalize(true);
    tests["mismatched_snapshot_cannot_become_pending"]=!s.pending.available&&f.native_calls==1&&s.quality.best_cost==100.;
  }
  {
    Fixture f;early_delivery::Session s(true,f.checker.get(),[&]{return f.now;},1.2,
      [&](const remani_planner::SingulTrajData& t){return f.native(t);});
    f.now=1.06;const auto x=f.point(1.);f.record(x,100.);const bool stop=s.accepted(x,1);s.finalize(true);
    tests["late_first_safe_returns_without_extra_native"]=stop&&f.native_calls==1&&s.selected&&
      !s.pending.available&&!s.deferred_check_attempted;
  }
  bool pass=true;for(const auto& test:tests)pass=pass&&test.second;
  std::cout<<"{\"pass\":"<<(pass?"true":"false");
  for(const auto& test:tests)std::cout<<",\""<<test.first<<"\":"<<(test.second?"true":"false");
  std::cout<<",\"scope\":\"actual Session and polynomial checker with synthetic clock/native predicate; no map-safety claim\"}\n";
  return pass?0:1;
}
