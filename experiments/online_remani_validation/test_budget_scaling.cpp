// Synthetic native-callback/actual-domain-LBFGS contracts, not feasibility proof.
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include "budget_scaling.hpp"
#ifndef NATIVE_LBFGS_HEADER
#error "Define NATIVE_LBFGS_HEADER to the frozen domain_lbfgs.hpp path"
#endif
#include NATIVE_LBFGS_HEADER
using Vector=Eigen::VectorXd;
namespace bs=budget_scaling;
namespace gate=feasibility_stop_gate;
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
template<class F>std::string throws(F fn){try{fn();}catch(const std::exception& e){return e.what();}throw std::runtime_error("expected exception");}
Vector vec(std::initializer_list<double> values){Vector x(values.size());int i=0;for(double value:values)x[i++]=value;return x;}
struct Row{Vector x,g;double f,step;int k,ls;};
struct Native {
  bool stopped=false,stop_on_eval=false,nonfinite=false,domain=false;
  double target=0.;Vector weights;
  std::vector<Row> evaluations,accepted;
};
double native_cost(void* raw,const Vector& x,Vector& g){
  auto& n=*static_cast<Native*>(raw);
  if(n.domain && x[0]>=.8)throw domain_step::RejectedTrial(x[0],0.,-10.,-10.,.8,10.);
  if(n.stop_on_eval){n.stopped=true;g=Vector::Zero(x.size());return 1.e100;}
  if(n.nonfinite){g=Vector::Constant(x.size(),std::numeric_limits<double>::quiet_NaN());return g[0];}
  g.resize(x.size());double f=0.;
  for(int i=0;i<x.size();++i){const double w=n.weights.size()?n.weights[i]:double((i+1)*(i+1));
    g[i]=w*(x[i]-n.target);f+=.5*w*(x[i]-n.target)*(x[i]-n.target);}
  n.evaluations.push_back({x,g,f,0.,0,0});
  bs::recordCanonical(x);gate::recordEvaluated(x);return f;
}
bool native_stopped(void* raw){return static_cast<Native*>(raw)->stopped;}
int native_progress(void* raw,const Vector& x,const Vector& g,double f,double step,int k,int ls){
  auto& n=*static_cast<Native*>(raw);bs::recordAccepted(x);n.accepted.push_back({x,g,f,step,k,ls});return n.stopped;
}
bs::Context context(Native& n,const Vector& x,const Vector& d){
  return bs::Context(&n,x,d,native_cost,native_progress,native_stopped);
}
void test_layout_formula(){
  const auto p=bs::makePlan({vec({.5,.5}),vec({2.,2.,2.})});
  check(p.diagonal.size()==38,"wrong native variable layout");
  check(std::abs(p.diagonal[0]-1./128.)<1.e-16,"wrong MinSnap time-order scaling");
  check(p.diagonal[1]==p.diagonal[0],"x/y scaling not isotropic");
  check(std::abs(p.diagonal[29]-std::sqrt(2./(16384.+1.)))<1.e-16,"wrong gear-position time pair");
  for(int i=0;i<38;++i)if(i!=0&&i!=1&&i!=29&&i!=30)
    check(p.diagonal[i]==1.,"time/angle/arm/long coordinate unexpectedly scaled");
  check(p.scaled_count==4&&p.floor_count==0,"wrong scale count");
}
void test_equal_times(){check((bs::makePlan({vec({1.,1.,1.})}).diagonal.array()==1.).all(),"equal-time identity lost");}
void test_numerical_floor(){
  const auto p=bs::makePlan({vec({1.e-200,1.e200})});
  check(p.floor_count==2&&p.diagonal[0]==std::sqrt(std::numeric_limits<double>::epsilon()),"machine-derived floor failed");
  check(p.diagonal.allFinite()&&(p.diagonal.array()>0.).all(),"floor produced singular transform");
}
void test_bad_plan(){
  check(throws([](){bs::makePlan({});}).find("layout")!=std::string::npos,"empty layout accepted");
  check(throws([](){bs::makePlan({vec({0.,1.})});}).find("positive")!=std::string::npos,"zero time accepted");
  check(throws([](){bs::makePlan({vec({-1.,1.})});}).find("positive")!=std::string::npos,"negative time accepted");
}
void test_zero_identity(){
  Native n;const auto x=vec({-0.,.12345678901234567,-2.,0.});auto c=context(n,x,vec({.001,.2,1.,1.}));
  check(bs::identical(c.canonical(Vector::Zero(x.size())),x),"initial canonical bits/-0 changed");
}
void test_first_fg(){
  Native a,b;const auto x=vec({-0.,.23456789012345678,-2.});Vector ga,gb;
  const double fa=native_cost(&a,x,ga);auto c=context(b,x,vec({.003,.01,1.}));
  const double fb=bs::Context::evaluate(&c,Vector::Zero(3),gb);
  check(fa==fb&&bs::identical(a.evaluations[0].x,b.evaluations[0].x)&&
    bs::identical(a.evaluations[0].g,b.evaluations[0].g),"initial native x/f/g differ");
}
void test_chain_gradient(){
  Native n;auto c=context(n,vec({.3,-.5,.9}),vec({.001,.2,1.}));
  auto z=vec({2.,-.3,.2});Vector g;bs::Context::evaluate(&c,z,g);
  for(int i=0;i<3;++i){auto hi=z,lo=z;hi[i]+=1.e-4;lo[i]-=1.e-4;Vector scratch;
    const double numeric=(bs::Context::evaluate(&c,hi,scratch)-bs::Context::evaluate(&c,lo,scratch))/2.e-4;
    check(std::abs(g[i]-numeric)<1.e-9,"transformed derivative chain mismatch");}
}
void test_progress_original_gradient(){
  Native n;bs::CanonicalSession session(true);auto c=context(n,vec({.7,-.4}),vec({.003,.11}));
  auto z=vec({.1,.2});Vector gz;const auto f=bs::Context::evaluate(&c,z,gz);
  const auto original=n.evaluations.back();bs::Context::progress(&c,z,gz,f,.2,1,2);
  check(bs::identical(n.accepted[0].x,original.x)&&bs::identical(n.accepted[0].g,original.g),"progress did not use original cached canonical x/g");
  check(bs::identical(session.last_x,original.x)&&bs::identical(session.last_accepted_x,original.x),"canonical trace mismatch");
}
void test_gate_solver_identity(){
  Native n;auto c=context(n,vec({.7,-.4}),vec({.003,.11}));
  gate::Session session(true,[](const Vector&){gate::Check v;v.native_known=v.native_pass=v.projected_known=v.projected_pass=true;return v;},{});
  const auto z=vec({.1,.2});Vector gz;bs::Context::evaluate(&c,z,gz);
  check(gate::allowCostStop(z)&&!session.stats.fatal,"gate used canonical x instead of solver z");
}
void test_cache_mismatch(){
  Native n;auto c=context(n,vec({.7,-.4}),vec({.003,.11}));auto z=vec({.1,.2});Vector gz;
  const auto f=bs::Context::evaluate(&c,z,gz);z[0]+=.1;
  check(throws([&](){bs::Context::progress(&c,z,gz,f,1.,1,1);}).find("mismatch")!=std::string::npos,"accepted stale x admitted");
}
void test_timeout_preserved(){
  Native n;n.stop_on_eval=true;auto c=context(n,vec({.7,-.4}),vec({.003,.11}));Vector gz;
  const auto z=Vector::Zero(2);const auto f=bs::Context::evaluate(&c,z,gz);
  check(n.stopped&&!c.valid&&f==1.e100,"native timeout flag/cache changed");
  check(bs::Context::progress(&c,z,gz,f,1.,1,1)==1&&c.stopped_progress==1,"native timeout became cache error");
}
void test_nonfinite_unflagged(){
  Native n;n.nonfinite=true;auto c=context(n,vec({.7}),vec({.1}));Vector gz;
  check(throws([&](){bs::Context::evaluate(&c,vec({0.}),gz);}).find("nonfinite")!=std::string::npos,"unflagged nonfinite admitted");
}
void test_domain_cache(){
  Native n;n.domain=true;auto c=context(n,vec({.1}),vec({1.}));Vector gz;
  bs::Context::evaluate(&c,vec({0.}),gz);const auto old=c.last_x;
  check(throws([&](){bs::Context::evaluate(&c,vec({1.}),gz);}).find("domain")!=std::string::npos,"native domain exception swallowed");
  check(bs::identical(c.last_x,old),"rejected native trial overwrote successful cache");
  bs::Context::evaluate(&c,vec({.2}),gz);check(c.last_x[0]>.29&&c.last_x[0]<.31,"recovery callback failed");
}
struct Solve{Vector x;double f;int status;Native native;};
Solve solve(bool wrapper,bool gate_on=false,bool domain=false){
  Solve s;s.x=domain?vec({.1}):vec({3.,-2.,1.,-.4,.8});s.native.domain=domain;s.native.target=domain?1.:0.;
  domain_lbfgs::lbfgs_parameter_t p;p.g_epsilon=0.;p.past=1;p.delta=1.e12;p.max_iterations=30;
  bs::CanonicalSession trace(true);
  gate::Session gate_session(gate_on,[](const Vector&){gate::Check v;v.native_known=v.native_pass=v.projected_known=v.projected_pass=true;return v;},{});
  if(wrapper){auto c=context(s.native,s.x,Vector::Ones(s.x.size()));auto z=Vector::Zero(s.x.size()).eval();
    s.status=domain_lbfgs::lbfgs_optimize(z,s.f,bs::Context::evaluate,nullptr,bs::Context::progress,&c,p);s.x=c.canonical(z);
  }else s.status=domain_lbfgs::lbfgs_optimize(s.x,s.f,native_cost,nullptr,native_progress,&s.native,p);
  check(!gate_session.stats.fatal,"native solver gate fatal");return s;
}
void test_off_solver_repeat(){
  const auto a=solve(false),b=solve(false);
  check(a.status==b.status&&a.f==b.f&&bs::identical(a.x,b.x),"unwrapped OFF result not deterministic");
  check(a.native.evaluations.size()==b.native.evaluations.size(),"OFF eval count changed");
  for(std::size_t i=0;i<a.native.evaluations.size();++i)
    check(bs::identical(a.native.evaluations[i].x,b.native.evaluations[i].x)&&
      bs::identical(a.native.evaluations[i].g,b.native.evaluations[i].g)&&
      a.native.evaluations[i].f==b.native.evaluations[i].f,"OFF numerical path changed");
}
void test_actual_solver_gate_and_domain(){
  domain_step::reset();const auto s=solve(true,true,true);
  check(domain_step::rejectedTrials()>0&&domain_step::acceptedStepsAfterRejection()>0,"domain solver did not test backtracking recovery");
  check(s.status==domain_lbfgs::LBFGS_STOP&&s.x[0]<.8,"native domain/stop behavior changed");
}
void test_final_marker_unwind(){
  int called=0;const auto message=throws([&](){bs::OnExit report(true,[&](){++called;});throw std::runtime_error("native_failure");});
  check(called==1&&message=="native_failure","final marker did not run on early exception");
  {bs::OnExit report(false,[&](){++called;});}check(called==1,"normal disabled marker ran");
}
int main(){
  const std::vector<std::pair<const char*,std::function<void()>>> tests={
    {"layout_formula",test_layout_formula},{"equal_times",test_equal_times},{"numerical_floor",test_numerical_floor},
    {"bad_plan",test_bad_plan},{"zero_identity",test_zero_identity},{"first_fg",test_first_fg},
    {"chain_gradient",test_chain_gradient},{"progress_original_gradient",test_progress_original_gradient},
    {"gate_solver_identity",test_gate_solver_identity},{"cache_mismatch",test_cache_mismatch},
    {"timeout_preserved",test_timeout_preserved},{"nonfinite_unflagged",test_nonfinite_unflagged},
    {"domain_cache",test_domain_cache},{"off_solver_repeat",test_off_solver_repeat},
    {"actual_solver_gate_and_domain",test_actual_solver_gate_and_domain},{"final_marker_unwind",test_final_marker_unwind}};
  int failed=0;for(const auto& t:tests)try{t.second();std::cout<<"PASS "<<t.first<<"\n";}catch(const std::exception& e){++failed;std::cout<<"FAIL "<<t.first<<": "<<e.what()<<"\n";}
  std::cout<<"RESULT "<<tests.size()-failed<<"/"<<tests.size()<<"\n";return failed?1:0;
}
