#include "early_collision_repair.hpp"
#include <iostream>

int main(){
  setenv("EARLY_COLLISION_REPAIR","1",1);setenv("CANDIDATE_COLLISION_REPAIR","1",1);
  setenv("DYNAMIC_WINDOW_CONTINUITY","1",1);
  time_indexed_collision::Context context;context.active=true;context.full_coverage=true;
  auto* previous=time_indexed_collision::current();time_indexed_collision::current()=&context;
  candidate_collision_repair::Scope repair;
  early_collision_repair::Session policy;
  early_delivery::Session native(true,nullptr,[]{return .1;},1.2,{});
  Eigen::VectorXd x=Eigen::VectorXd::Ones(4);
  early_delivery::Check c;c.accepted_iteration=5;c.candidate_hash=delivery_candidate::hash(x);
  c.wheel_pass=true;c.native_checked=true;c.native_reason="planning_collision";c.collision_type=0;
  native.checks.push_back(c);
  int failed=0;
  auto expect=[&](bool value){if(!value)++failed;};
  native.checks.back().collision_type=2;expect(!early_collision_repair::shouldYield(x,5));
  native.checks.back().collision_type=3;expect(!early_collision_repair::shouldYield(x,5));
  native.checks.back().collision_type=-1;expect(!early_collision_repair::shouldYield(x,5));
  native.checks.back().collision_type=0;
  expect(!early_collision_repair::shouldYield(x,6));
  auto other=x;other[0]+=.01;expect(!early_collision_repair::shouldYield(other,5));
  native.checks.back().budget_expired=true;expect(!early_collision_repair::shouldYield(x,5));
  native.checks.back().budget_expired=false;native.checks.back().native_pass=true;
  expect(!early_collision_repair::shouldYield(x,5));native.checks.back().native_pass=false;
  native.checks.back().native_checked=false;expect(!early_collision_repair::shouldYield(x,5));
  native.checks.back().native_checked=true;
  repair.repairing=true;expect(!early_collision_repair::shouldYield(x,5));repair.repairing=false;
  policy.repairing=true;expect(!early_collision_repair::shouldYield(x,5));policy.repairing=false;
  policy.enabled=false;expect(!early_collision_repair::shouldYield(x,5));policy.enabled=true;
  expect(early_collision_repair::shouldYield(x,5));expect(policy.yielded&&(policy.x-x).norm()==0.);
  expect(!early_collision_repair::shouldYield(x,5));
  time_indexed_collision::current()=previous;
  std::cout<<"{\"pass\":"<<(failed?"false":"true")<<",\"failed\":"<<failed
    <<",\"scope\":\"environment only; current physical accepted iterate; once; never while repairing or expired\"}\n";
  return failed?1:0;
}
