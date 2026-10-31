#include "clearance_penalty.hpp"
#include <array>
#include <cassert>
#include <iostream>
#include <random>

using clearance_penalty::Evaluation;
using clearance_penalty::Snapshot;
static Snapshot fixture() {
  Snapshot m;
  m.gx=-3.;m.gy=-3.;m.res=.1;m.nx=61;m.ny=61;
  m.margin=.1;m.comfort=.32;m.weight=17.;
  m.body={{.31,.12,.18},{-.27,-.08,.30},{.04,.25,.28}};
  for(int iy=0;iy<m.ny;++iy)for(int ix=0;ix<m.nx;++ix) {
    const double x=m.gx+ix*m.res,y=m.gy+iy*m.res;
    m.field.push_back(.36+.02*x+.025*y+.005*x*y);
  }
  return m;
}
static void near(double a,double b,double tolerance=2e-6) {
  if(!std::isfinite(a)||!std::isfinite(b)||
      std::abs(a-b)>tolerance*(1.+std::abs(a)+std::abs(b))) {
    std::cerr<<"clearance mismatch "<<a<<" vs "<<b<<'\n';std::abort();
  }
}
static Evaluation at(const Snapshot& m,const std::array<double,3>& q,
                     const std::vector<double>* lambda=nullptr) {
  const auto e=clearance_penalty::evaluate(m,q[0],q[1],q[2],1e5,
      lambda ? lambda->data() : nullptr,lambda ? lambda->size() : 0);
  assert(e.valid);return e;
}
static void gradients(const Snapshot& m,const std::array<double,3>& q,
                      const std::vector<double>* lambda=nullptr) {
  const auto e=at(m,q,lambda);const double analytic[]={e.gx,e.gy,e.gyaw};
  for(int d=0;d<3;++d) {
    auto p=q,n=q;p[d]+=1e-6;n[d]-=1e-6;
    near(analytic[d],(at(m,p,lambda).cost-at(m,n,lambda).cost)/2e-6);
  }
}
int main() {
  static_assert(clearance_penalty::fixed_rho==1e5,"predeclared recovery rho");
  auto m=fixture();const std::array<double,3> q={.017,-.033,.73};
  const auto e=at(m,q);assert(e.cost>0. && e.gap<0.);near(e.gt,0.,0.);
  coupling_collision::Options options;options.include_dynamic=false;
  const auto original=coupling_collision::evaluate(m,q[0],q[1],q[2],0.,options);
  near(e.physical,original.physical,0.);near(e.gap,original.gap,0.);
  double cost=0.,gx=0.,gy=0.,gyaw=0.;int active=0;
  for(const auto& body:m.body) {
    auto one=m;one.body={body};const auto s=at(one,q);
    cost+=s.cost;gx+=s.gx;gy+=s.gy;gyaw+=s.gyaw;active+=s.gap<0.;
    near(s.cost,.5*clearance_penalty::fixed_rho*std::pow(std::max(0.,-s.gap),2));
  }
  assert(active>0 && active<int(m.body.size()));
  near(e.cost,cost);near(e.gx,gx);near(e.gy,gy);near(e.gyaw,gyaw);
  gradients(m,q);
  std::mt19937 generator(7919);
  std::uniform_real_distribution<double> xy(-.12,.12),yaw(-2.8,2.8);
  for(int i=0;i<100;++i)gradients(m,{xy(generator),xy(generator),yaw(generator)});
  // Penalty is unrelated to comfort/soft weight or dynamic obstacle geometry.
  auto changed=m;changed.comfort=.7;changed.weight=999.;
  changed.obstacles={{NAN,NAN,NAN,NAN,-1.,2.}};changed.max_prediction=NAN;
  const auto unaffected=at(changed,q);
  near(e.cost,unaffected.cost,0.);near(e.gx,unaffected.gx,0.);
  near(e.gy,unaffected.gy,0.);near(e.gyaw,unaffected.gyaw,0.);
  const auto doubled=clearance_penalty::evaluate(m,q[0],q[1],q[2],2e5);
  assert(doubled.valid);near(doubled.cost,2*e.cost);near(doubled.gyaw,2*e.gyaw);
  // C1 hinge: exactly zero at the boundary and throughout the safe side.
  auto hinge=m;hinge.body={{0.,0.,.25}};hinge.field.assign(61*61,.35);
  const auto zero=at(hinge,{0.,0.,0.});near(zero.cost,0.,1e-20);near(zero.gx,0.,0.);
  hinge.field.assign(61*61,.5);const auto safe=at(hinge,{0.,0.,0.});
  assert(safe.cost==0. && safe.gx==0. && safe.gy==0. && safe.gyaw==0.);
  const std::vector<double> lambda={2000.,3500.,900.},zeros(3,0.);
  gradients(m,q,&lambda);near(at(m,q,&zeros).cost,e.cost,0.);
  double al_cost=0.;
  for(size_t i=0;i<m.body.size();++i) {
    auto one=m;one.body={m.body[i]};const double h=-at(one,q).gap;
    al_cost+=(std::pow(std::max(0.,lambda[i]+1e5*h),2)-lambda[i]*lambda[i])/(2e5);
  }
  near(at(m,q,&lambda).cost,al_cost);
  for(int i=0;i<50;++i)gradients(m,{xy(generator),xy(generator),yaw(generator)},&lambda);
  const double invalid_lambda[]={-1.,NAN,INFINITY};
  for(double value:invalid_lambda) {
    const double bad_lambda[]={0.,value,0.};
    assert(!clearance_penalty::evaluate(m,0.,0.,0.,1e5,bad_lambda,3).valid);
  }
  assert(!clearance_penalty::evaluate(m,0.,0.,0.,1e5,lambda.data(),2).valid);
  const std::vector<double> safe_lambda={1000.};
  const auto al_safe=at(hinge,{0.,0.,0.},&safe_lambda);
  near(al_safe.cost,-5.);assert(al_safe.gx==0. && al_safe.gy==0. && al_safe.gyaw==0.);
  for(double bad:{0.,-1.,double(NAN),double(INFINITY)})
    assert(!clearance_penalty::evaluate(m,0.,0.,0.,bad).valid);
  const auto outside=clearance_penalty::evaluate(m,3.,0.,0.);
  assert(!outside.valid && outside.failure==Evaluation::Failure::static_map_domain);
  assert(!clearance_penalty::evaluate(m,NAN,0.,0.).valid);
  assert(!clearance_penalty::evaluate(m,1e100,0.,0.).valid);
  auto bad=m;bad.field.pop_back();
  assert(clearance_penalty::evaluate(bad,0.,0.,0.).failure==Evaluation::Failure::malformed);
  bad=m;bad.body[1].r=NAN;assert(!clearance_penalty::evaluate(bad,0.,0.,0.).valid);
  bad=m;bad.field.assign(61*61,-1.);assert(!clearance_penalty::evaluate(bad,0.,0.,0.).valid);
  std::cout<<"CLEARANCE_PENALTY_PASSED raw static pose/yaw gradients; rotated full body; "
               "mixed hinge activity; zero/nonzero AL; fixed rho; unchanged physical/domain semantics\n";
}
