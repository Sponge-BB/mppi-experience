// Isolated synthetic contracts for stop eligibility, not trajectory feasibility.
// Parent and patched optimizer headers are included from frozen build snapshots.
#include <atomic>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "feasibility_stop_gate.hpp"

#ifndef PARENT_LBFGS_HEADER
#error "Define PARENT_LBFGS_HEADER as the immutable original header path"
#endif
#ifndef PATCHED_LBFGS_HEADER
#error "Define PATCHED_LBFGS_HEADER as the frozen patched header path"
#endif
#define domain_lbfgs parent_domain_lbfgs
#include PARENT_LBFGS_HEADER
#undef domain_lbfgs
#undef DOMAIN_STEP_LBFGS_HPP
#include PATCHED_LBFGS_HEADER

namespace gate = feasibility_stop_gate;
using Vector = Eigen::VectorXd;

void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
template<class F> std::string throws(F callback) {
  try { callback(); } catch (const std::exception& error) { return error.what(); }
  throw std::runtime_error("Expected an exception, but callback returned");
}
Vector point() { Vector x(3); x << .2, -.4, .7; return x; }
gate::Check verdict(bool native=true, bool projected=true, bool expired=false) {
  gate::Check c; c.native_known=true; c.native_pass=native;
  c.projected_known=true; c.projected_pass=projected; c.budget_expired=expired;
  c.native_wall_s=.001; c.projected_wall_s=.002; c.total_wall_s=.003;
  c.reason=expired?"synthetic_expired":native&&projected?"synthetic_pass":"synthetic_reject";
  return c;
}
bool identical(const Vector& a, const Vector& b) {
  return a.size()==b.size() && !std::memcmp(a.data(),b.data(),a.size()*sizeof(double));
}
std::string quote(const std::string& value) {
  std::string out="\"";
  for(char c:value){
    if(c=='\\'||c=='\"')out+='\\';
    if(c=='\n')out+="\\n";else out+=c;
  }
  return out+'\"';
}

struct Evaluation { Vector x,g; double cost; };
struct Accepted { Vector x,g; double cost,step; int k,ls; };
struct Quadratic {
  std::vector<Evaluation> evaluated;
  std::vector<Accepted> accepted;
};
double objective(void* instance, const Vector& x, Vector& gradient) {
  auto& log=*static_cast<Quadratic*>(instance);
  gradient.resize(x.size()); double cost=0.;
  for(int i=0;i<x.size();++i){ const double h=double((i+1)*(i+1));
    cost+=.5*h*x(i)*x(i); gradient(i)=h*x(i); }
  log.evaluated.push_back({x,gradient,cost});
  gate::recordEvaluated(x);
  return cost;
}
int progress(void* instance,const Vector& x,const Vector& gradient,double cost,
             double step,int k,int ls) {
  static_cast<Quadratic*>(instance)->accepted.push_back({x,gradient,cost,step,k,ls});
  return 0;
}
struct Solve { Vector x; double cost=0.; int status=0; Quadratic log; };
Solve solve(bool parent, bool stationary=false, int max_iterations=40) {
  Solve out; out.x.resize(5);
  if(stationary)out.x.setZero();else out.x<<3.,-2.,1.,-.4,.8;
  if(parent){
    parent_domain_lbfgs::lbfgs_parameter_t p;
    p.g_epsilon=0.;p.past=1;p.delta=1.e12;p.max_iterations=max_iterations;
    out.status=parent_domain_lbfgs::lbfgs_optimize(out.x,out.cost,objective,nullptr,progress,&out.log,p);
  }else{
    domain_lbfgs::lbfgs_parameter_t p;
    p.g_epsilon=0.;p.past=1;p.delta=1.e12;p.max_iterations=max_iterations;
    out.status=domain_lbfgs::lbfgs_optimize(out.x,out.cost,objective,nullptr,progress,&out.log,p);
  }
  return out;
}
void identical_solve(const Solve& a,const Solve& b) {
  check(a.status==b.status && a.cost==b.cost && identical(a.x,b.x),"OFF final result differs from parent");
  check(a.log.evaluated.size()==b.log.evaluated.size(),"OFF evaluation count differs");
  check(a.log.accepted.size()==b.log.accepted.size(),"OFF accepted count differs");
  for(std::size_t i=0;i<a.log.evaluated.size();++i){
    const auto& x=a.log.evaluated[i];const auto& y=b.log.evaluated[i];
    check(x.cost==y.cost&&identical(x.x,y.x)&&identical(x.g,y.g),"OFF evaluation x/f/g differs");
  }
  for(std::size_t i=0;i<a.log.accepted.size();++i){
    const auto& x=a.log.accepted[i];const auto& y=b.log.accepted[i];
    check(x.cost==y.cost&&x.step==y.step&&x.k==y.k&&x.ls==y.ls&&
          identical(x.x,y.x)&&identical(x.g,y.g),"OFF accepted trajectory differs");
  }
}

void test_no_session() {
  check(gate::current()==nullptr,"Leaked session before test");
  check(gate::allowCostStop(point()),"Default OFF must allow original stop");
  check(!gate::abortRequested(),"Default OFF must not abort");
  gate::recordEvaluated(point());
}
void test_disabled_session() {
  int checks=0,reports=0;
  {
    gate::Session s(false,[&](const Vector&){++checks;return verdict(false,false);},
      [&](const gate::Statistics& stats){++reports;check(!stats.enabled&&stats.checks==0,"OFF stats changed");});
    check(gate::current()==nullptr,"Disabled session became active");
    check(gate::allowCostStop(point()),"Disabled session vetoed stop");
  }
  check(checks==0&&reports==1,"Disabled predicate called or reporter count wrong");
}
void test_false_then_true() {
  int calls=0,reports=0;gate::Statistics captured;
  {
    gate::Session s(true,[&](const Vector&){++calls;return verdict(calls>1,calls>2);},
      [&](const gate::Statistics& stats){++reports;captured=stats;});
    const auto x=point();gate::recordEvaluated(x);
    check(!gate::allowCostStop(x),"Native violation admitted");
    check(!gate::allowCostStop(x),"Projection violation admitted");
    check(gate::allowCostStop(x),"Both gates passing was blocked");
    check(!gate::abortRequested(),"Nondeadline rejection requested abort");
  }
  check(calls==3&&reports==1&&captured.checks==3&&captured.blocked==2&&captured.allowed==1,
        "Wrong gate accounting");
  check(captured.native_pass==2&&captured.projected_pass==1&&!captured.fatal,"Wrong gate pass counters");
  check(std::abs(captured.total_wall_s-.009)<1.e-15,"Gate wall accounting wrong");
}
void test_missing_cache() {
  int reports=0;gate::Statistics captured;
  {
    gate::Session s(true,[](const Vector&){return verdict();},
      [&](const gate::Statistics& stats){++reports;captured=stats;});
    check(throws([&](){gate::allowCostStop(point());}).find("cache mismatch")!=std::string::npos,
          "Missing cache not rejected");
  }
  check(reports==1&&captured.fatal&&captured.checks==1,"Missing cache fatal not reported once");
}
void test_changed_cache() {
  gate::Session s(true,[](const Vector&){return verdict();},{});
  auto x=point();gate::recordEvaluated(x);x(0)+=.01;
  check(throws([&](){gate::allowCostStop(x);}).find("cache mismatch")!=std::string::npos,
        "Changed accepted x not rejected");
}
void test_shape_cache() {
  gate::Session s(true,[](const Vector&){return verdict();},{});
  gate::recordEvaluated(point());Vector x(2);x.setZero();
  check(throws([&](){gate::allowCostStop(x);}).find("cache mismatch")!=std::string::npos,
        "Changed accepted dimension not rejected");
}
void test_nonfinite_cache() {
  gate::Session s(true,[](const Vector&){return verdict();},{});
  auto x=point();x(0)=std::numeric_limits<double>::quiet_NaN();
  check(throws([&](){gate::recordEvaluated(x);}).find("nonfinite")!=std::string::npos,
        "Nonfinite evaluated x not rejected");
}
void test_nested_session() {
  gate::Session* outer_pointer=nullptr;int outer_calls=0,inner_calls=0;
  {
    gate::Session outer(true,[&](const Vector&){++outer_calls;return verdict(false,false);},{});
    outer_pointer=&outer;const auto x=point();gate::recordEvaluated(x);
    {
      gate::Session inner(true,[&](const Vector&){++inner_calls;return verdict();},{});
      check(gate::current()==&inner,"Inner session not selected");
      gate::recordEvaluated(x);check(gate::allowCostStop(x),"Inner verdict not used");
    }
    check(gate::current()==outer_pointer,"Outer session not restored");
    {
      gate::Session disabled(false,{},{});
      check(gate::current()==outer_pointer,"Nested disabled session masked outer");
      check(!gate::allowCostStop(x),"Outer verdict not restored");
    }
    check(outer_calls==1&&inner_calls==1,"Nested predicate accounting wrong");
  }
  check(gate::current()==nullptr,"Outermost session not restored");
}
void test_thread_local() {
  gate::Session outer(true,[](const Vector&){return verdict(false,false);},{});
  std::exception_ptr thread_error;std::atomic<int> done{0};
  std::thread worker([&](){try{
    check(gate::current()==nullptr,"Worker inherited main-thread session");
    {gate::Session s(true,[](const Vector&){return verdict();},{});
     const auto x=point();gate::recordEvaluated(x);check(gate::allowCostStop(x),"Worker predicate failed");}
    check(gate::current()==nullptr,"Worker session leaked");++done;
  }catch(...){thread_error=std::current_exception();}});
  worker.join();if(thread_error)std::rethrow_exception(thread_error);
  check(done==1&&gate::current()==&outer,"Thread-local main session corrupted");
}
void test_budget_abort() {
  {
    gate::Session s(true,[](const Vector&){return verdict(true,true,true);},{});
    const auto x=point();gate::recordEvaluated(x);
    check(!gate::allowCostStop(x),"Expired budget admitted");
    check(gate::abortRequested(),"Expired budget not requesting abort");
    check(s.stats.budget_expired==1,"Budget expiry counter wrong");
  }
  check(!gate::abortRequested(),"Abort flag leaked after session");
}
void test_predicate_exception() {
  int reports=0;gate::Statistics captured;
  const auto message=throws([&](){
    gate::Session s(true,[](const Vector&)->gate::Check{throw std::runtime_error("synthetic_predicate_fatal");},
      [&](const gate::Statistics& stats){++reports;captured=stats;});
    const auto x=point();gate::recordEvaluated(x);gate::allowCostStop(x);
  });
  check(message=="synthetic_predicate_fatal"&&reports==1&&captured.fatal&&
        captured.fatal_reason==message,"Fatal predicate exception not preserved/reported");
  check(gate::current()==nullptr,"Session leaked while unwinding fatal predicate");
}
void test_reporter_unwind_once() {
  int reports=0;
  check(throws([&](){gate::Session s(true,[](const Vector&){return verdict();},
    [&](const gate::Statistics&){++reports;});throw std::runtime_error("outside_failure");})=="outside_failure",
    "Session swallowed unrelated exception");
  check(reports==1&&gate::current()==nullptr,"Unwind reporter/session wrong");
}
void test_throwing_reporter() {
  int reports=0;
  {gate::Session s(true,[](const Vector&){return verdict();},
    [&](const gate::Statistics&){++reports;throw std::runtime_error("reporter_failure");});}
  check(reports==1&&gate::current()==nullptr,"Throwing reporter corrupted cleanup");
}
void test_empty_predicate() {
  check(throws([](){gate::Session s(true,{},{});}).find("empty feasibility")!=std::string::npos,
        "Enabled empty predicate accepted");
  check(gate::current()==nullptr,"Invalid construction changed thread-local session");
}
void test_patched_off_parent_exact() {
  const auto parent=solve(true);const auto without=solve(false);
  identical_solve(parent,without);
  int reports=0;
  {
    gate::Session off(false,{},[&](const gate::Statistics& stats){++reports;check(stats.checks==0,"OFF consulted gate");});
    identical_solve(parent,solve(false));
  }
  check(parent.status==parent_domain_lbfgs::LBFGS_STOP&&parent.log.accepted.size()==1&&reports==1,
        "Synthetic OFF case did not exercise original past/delta stop");
}
void test_patched_on_continues() {
  const auto off=solve(false);int calls=0;gate::Statistics stats;Solve on;
  {
    gate::Session s(true,[&](const Vector&){return verdict(++calls>=4,true);},
      [&](const gate::Statistics& value){stats=value;});
    on=solve(false);
  }
  check(on.status==domain_lbfgs::LBFGS_STOP&&stats.blocked==3&&stats.allowed==1&&calls==4,
        "ON did not veto three delta stops then allow fourth");
  check(on.log.accepted.size()==4&&on.cost<off.cost,"ON did not continue numerical progress");
  check(identical(on.log.accepted.front().x,off.log.accepted.front().x)&&
        on.log.accepted.front().cost==off.log.accepted.front().cost,"ON altered pre-veto optimizer trajectory");
  for(const auto& e:on.log.evaluated){Vector gradient;Quadratic scratch;
    // Direct recomputation without an active session certifies the same f/g.
    const double cost=objective(&scratch,e.x,gradient);
    check(cost==e.cost&&identical(gradient,e.g),"ON changed quadratic objective/gradient");}
  std::cout<<"{\"kind\":\"synthetic_optimizer_comparison\",\"scope\":\"logic_only_not_robot_feasibility\","
    <<"\"off_accepted\":"<<off.log.accepted.size()<<",\"on_accepted\":"<<on.log.accepted.size()
    <<",\"off_cost\":"<<std::setprecision(17)<<off.cost<<",\"on_cost\":"<<on.cost
    <<",\"blocked\":"<<stats.blocked<<",\"allowed\":"<<stats.allowed<<"}\n";
}
void test_patched_expiry_cancels() {
  gate::Statistics stats;Solve result;
  {
    gate::Session s(true,[](const Vector&){return verdict(false,false,true);},
      [&](const gate::Statistics& value){stats=value;});result=solve(false);
  }
  check(result.status==domain_lbfgs::LBFGS_CANCELED&&stats.blocked==1&&stats.budget_expired==1,
        "Expired gate did not return original cancellation status");
  check(result.log.accepted.size()==1,"Expired gate allowed another optimizer step");
}
void test_other_terminations_unchanged() {
  const auto parent=solve(true,true);int calls=0;Solve stationary;
  {
    gate::Session s(true,[&](const Vector&){++calls;return verdict(false,false);},{});
    stationary=solve(false,true);
  }
  identical_solve(parent,stationary);
  check(calls==0&&stationary.status==domain_lbfgs::LBFGS_CONVERGENCE,
        "Delta-only patch unexpectedly intercepted gradient/stationary termination");
  gate::Statistics stats;Solve limited;
  {
    gate::Session s(true,[](const Vector&){return verdict(false,false);},
      [&](const gate::Statistics& value){stats=value;});limited=solve(false,false,3);
  }
  check(limited.status==domain_lbfgs::LBFGSERR_MAXIMUMITERATION&&
        limited.log.accepted.size()==3&&stats.blocked==3,
        "Gate bypassed original maximum-iteration termination");
}

int main() {
  const std::vector<std::pair<std::string,std::function<void()>>> tests={
    {"no_session_default_off",test_no_session},{"disabled_session",test_disabled_session},
    {"native_and_projected_false_then_true",test_false_then_true},{"missing_cache_fatal",test_missing_cache},
    {"changed_x_cache_fatal",test_changed_cache},{"changed_shape_cache_fatal",test_shape_cache},
    {"nonfinite_cache_rejected",test_nonfinite_cache},{"nested_session_restore",test_nested_session},
    {"thread_local_isolation",test_thread_local},{"budget_abort_flag",test_budget_abort},
    {"fatal_predicate_unwind",test_predicate_exception},{"reporter_once_on_unwind",test_reporter_unwind_once},
    {"throwing_reporter_cleanup",test_throwing_reporter},{"empty_predicate_rejected",test_empty_predicate},
    {"patched_off_bitwise_parent_equal",test_patched_off_parent_exact},
    {"patched_on_veto_then_continue",test_patched_on_continues},
    {"patched_budget_immediate_cancel",test_patched_expiry_cancels},
    {"non_delta_termination_preserved",test_other_terminations_unchanged}};
  std::size_t failures=0;
  for(const auto& test:tests){
    try{test.second();check(gate::current()==nullptr,"Session leaked after test");
      std::cout<<"{\"kind\":\"unit_test\",\"name\":"<<quote(test.first)<<",\"passed\":true}\n";
    }catch(const std::exception& error){++failures;
      std::cout<<"{\"kind\":\"unit_test\",\"name\":"<<quote(test.first)
        <<",\"passed\":false,\"error\":"<<quote(error.what())<<"}\n";
    }catch(...){++failures;
      std::cout<<"{\"kind\":\"unit_test\",\"name\":"<<quote(test.first)
        <<",\"passed\":false,\"error\":\"unknown exception\"}\n";}
  }
  std::cout<<"{\"kind\":\"summary\",\"tests\":"<<tests.size()<<",\"failures\":"<<failures
    <<",\"passed\":"<<(failures?"false":"true")
    <<",\"scope\":\"synthetic contracts, not proof of real trajectory delivery improvement\"}\n";
  return failures?1:0;
}
