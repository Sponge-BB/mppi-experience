// Full actual MPPI header harness. It has no ROS node/publisher/robot I/O.
#include "online_kernel.hpp"
#include <cassert>
#include <cstring>

using namespace experiment;

// The two otherwise-unresolved legacy CV utilities use exactly the frozen
// implementation. Native-3D prediction remains the real header path above.
bool plan_env::DynamicPointBuffer::predictPosition(const plan_env::DynamicPoint&point,
    const ros::Time&stamp,const ros::Time&time,double horizon,Eigen::Vector3d&out,double&dt){
  dt=(time-stamp).toSec();
  if(!std::isfinite(dt)||dt< -1e-6||dt>horizon+1e-6)return false;
  dt=std::max(0.,std::min(horizon,dt));out=point.position+dt*point.velocity;return out.allFinite();
}
double plan_env::DynamicPointBuffer::predictionUncertainty(double dt,double pos,double vel,double acc){
  dt=std::max(0.,dt);return std::max(0.,pos)+std::max(0.,vel)*dt+.5*std::max(0.,acc)*dt*dt;
}
static void near(double a,double b,double tolerance=1e-11){
  if(a==b)return;
  if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>tolerance*(1+std::abs(a)+std::abs(b))){
    std::cerr<<"mismatch "<<std::setprecision(17)<<a<<" vs "<<b<<'\n';std::abort();}
}
static C config(){
  C c{.28,1.2,.6,2.0,.4,.3,.8,0.,.35,3.,0.,0.,0.,.06,.10,.02};
  c.body={{{0.,0.},.13}};return c;
}
static std::vector<double> payload(int kind=0,int nb=160){
  std::vector<double> p(32,0.);p[0]=3;p[1]=1;p[2]=1;p[3]=100.;p[4]=1;
  p[5]=85;p[6]=45;p[7]=18;p[8]=-1.;p[9]=-2.;p[10]=-.1;p[11]=.1;
  p[12]=nb;p[13]=kind==1?1:kind==2?2:0;p[14]=1;p[15]=.35;p[16]=3.;
  p[17]=100.;p[18]=100.;p[19]=1;p[20]=1;p[21]=.01;p[22]=1;p[23]=1;p[24]=100.;p[28]=1;
  for(int k=0;k<nb;++k){const double b[]={.04*(k%5-2),.04*((k/5)%4-1.5),.10+.04*(k/20),.08,.18,50000.,0.};
    p.insert(p.end(),std::begin(b),std::end(b));}
  if(kind==1||kind==2){const double o[]={1.05,0.,.20,-.18,0.,0.,1.};p.insert(p.end(),std::begin(o),std::end(o));}
  if(kind==2){const double o[]={.7,-.65,.25,0.,.22,0.,2.};p.insert(p.end(),std::begin(o),std::end(o));}
  for(int x=0;x<int(p[5]);++x)for(int y=0;y<int(p[6]);++y)for(int z=0;z<int(p[7]);++z){
    const double wx=p[8]+(x+.5)*p[11],wy=p[9]+(y+.5)*p[11];
    p.push_back(kind==3?0.:kind==4?std::hypot(wx-1.,wy)-.2:2.);
  }
  return p;
}
static native3d::State state(const std::vector<double>&p){
  native3d::State v;v.now=100.;assert(native3d::bind(v,p.data(),p.size()));return v;
}
static std::vector<std::vector<O>> future(const Z&z,const C&c,int n){
  std::vector<std::vector<O>> f;for(int k=0;k<=2*n;++k)f.push_back(predict(z,k*c.rdt/2,c));return f;
}
static void sameMerit(const MPPI::DynamicMerit&a,const MPPI::DynamicMerit&b,bool compare_cost=true){
  assert(a.merit.feasible==b.merit.feasible);if(compare_cost)near(a.merit.cost,b.merit.cost);
  near(a.merit.min_gap,b.merit.min_gap);near(a.merit.progress,b.merit.progress);
  near(a.merit.deficit,b.merit.deficit);near(a.merit.conflict,b.merit.conflict);
  near(a.merit.lateral,b.merit.lateral);near(a.merit.end_gap,b.merit.end_gap);
  near(a.merit.end_closing,b.merit.end_closing);near(a.terminal_side,b.terminal_side);
  assert(a.merit.lane==b.merit.lane&&a.side==b.side);
}
static void cacheBoundaries(){
  auto c=config();auto sc=scene("empty",c);auto ns=state(payload(1));native3d::Scope scope(ns);
  MPPI m(c,11);m.d_origin={0.,0.};m.d_forward={1.,0.};m.d_lateral_needed=true;
  S s{.1,0.,0.,.17,.13};Z z{{{{1.05,0.},{-.18,0.},.2,true}},0.,0.,true};
  auto a=m.arcs(sc);std::vector<V> us(20,V{.19,.31});auto f=future(z,c,20);
  ::setenv("NATIVE_DYNAMIC_FAST","0",1);auto original=m.dynamicScore(s,sc,a,us,f,false,0,false);
  ::setenv("NATIVE_DYNAMIC_FAST","1",1);native_dynamic_fast::NominalCache cache;
  auto recorded=m.dynamicScore(s,sc,a,us,f,false,0,false,&cache,true);sameMerit(original,recorded);
  assert(cache.matches(s,us,f)&&cache.samples.size()==40);
  // Store the actual two-half-step state, never the proposal's full step.
  auto half=step(s,us.front(),c.rdt/2,c);half=step(half,us.front(),c.rdt/2,c);
  assert(native_dynamic_fast::sameState(cache.samples[1].state,half));
  auto whole=step(s,us.front(),c.rdt,c);
  // They can differ by rounding even with equal integration subdivision.
  assert(std::abs(cache.samples[1].state.x-whole.x)<1e-12);
  ::setenv("NATIVE_DYNAMIC_FAST","0",1);auto intervention=m.dynamicScore(s,sc,a,us,f,true,1,false);
  ::setenv("NATIVE_DYNAMIC_FAST","1",1);auto reused=m.dynamicScore(s,sc,a,us,f,true,1,false,&cache);
  sameMerit(intervention,reused); // The intervention-dependent cost is recomputed.
  assert(std::abs(original.merit.cost-intervention.merit.cost)>1e-6);
  auto changed=s;changed.x=std::nextafter(s.x,1.);assert(!cache.matches(changed,us,f));
  auto controls=us;controls[0].x=std::nextafter(us[0].x,1.);assert(!cache.matches(s,controls,f));
  auto shifted=f;shifted[1][0].absolute_query_time=std::nextafter(f[1][0].absolute_query_time,200.);
  assert(!cache.matches(s,us,shifted));shifted=f;shifted[1].clear();assert(!cache.matches(s,us,shifted));
  const auto entry=cache.samples.front();
  assert(cache.at(0,entry.state,entry.key.time,entry.key.dynamic));
  assert(!cache.at(0,entry.state,entry.key.time,!entry.key.dynamic));
  assert(!cache.at(0,entry.state,std::nextafter(entry.key.time,200.),entry.key.dynamic));
  assert(!cache.at(1000,entry.state,entry.key.time,entry.key.dynamic));
  // Different snapshot/scope, including a nested scope over the same input,
  // may not borrow this cache. Restoring the immutable outer scope is safe.
  auto other=state(payload(2));{native3d::Scope nested(other);assert(!cache.matches(s,us,f));}
  assert(cache.matches(s,us,f));{native3d::Scope nested(ns);assert(!cache.matches(s,us,f));}
  assert(cache.matches(s,us,f));
  ::setenv("NATIVE_DYNAMIC_FAST","0",1);assert(!cache.matches(s,us,f));
  ::setenv("NATIVE_DYNAMIC_FAST","1",1);::setenv("NATIVE_COLLISION_3D","0",1);
  assert(!cache.matches(s,us,f));::setenv("NATIVE_COLLISION_3D","1",1);
  native_dynamic_fast::NominalCache empty;assert(!empty.matches(s,us,f));
}
static void earlyAndUnknown(){
  auto c=config();auto sc=scene("empty",c);MPPI m(c,17);auto a=m.arcs(sc);
  auto p=payload(0,2);p[32]=0.;p[39]=.25;
  // Nonconstant signed field: first sphere is just inside, next is deeper.
  std::size_t start=32+2*7;
  for(int x=0;x<int(p[5]);++x)for(int y=0;y<int(p[6]);++y)for(int z=0;z<int(p[7]);++z)
    p[start++]=.06-.1*(p[8]+(x+.5)*p[11]);
  auto ns=state(p);S s{};std::vector<V> us(3,V{});Z z;
  ::setenv("NATIVE_DYNAMIC_FAST","1",1);
  {native3d::Scope scope(ns);auto f=future(z,c,3);native_dynamic_fast::NominalCache cache;
    auto nm=m.dynamicScore(s,sc,a,us,f,false,0,false,&cache,true);
    assert(!nm.merit.feasible);assert(cache.samples.size()==6);
    assert(cache.samples[0].clearance_gap<0&&cache.samples[0].complete.valid);
    assert(cache.samples[0].complete.gap<cache.samples[0].clearance_gap-.01);
    near(nm.merit.min_gap,cache.samples[0].clearance_gap);
    auto reused=m.dynamicScore(s,sc,a,us,f,false,0,false,&cache);sameMerit(nm,reused);
  }
  // Early collision is valid but the later body sample lies outside ESDF.
  p[39]=20.;p[14]=2;ns=state(p);
  {native3d::Scope scope(ns);auto f=future(z,c,3);native_dynamic_fast::NominalCache cache;
    auto nm=m.dynamicScore(s,sc,a,us,f,false,0,false,&cache,true);
    const auto&first=cache.samples.front();assert(first.clearance_gap<0&&first.clearance_gap> -1.);
    assert(!first.complete.valid&&first.complete.cost==1e12);
    near(nm.merit.min_gap,first.clearance_gap);
    auto reused=m.dynamicScore(s,sc,a,us,f,false,0,false,&cache);sameMerit(nm,reused);
  }
}
static void earlyPruneCounts(){
  auto c=config();auto sc=scene("empty",c);MPPI m(c,11);auto a=m.arcs(sc);
  auto ns=state(payload(3));S s{};Z z;std::vector<V>us(20,V{.2,.1});
  native_realtime::Stats before,after;MPPI::DynamicMerit old_value,new_value;
  ::setenv("NATIVE_DYNAMIC_FAST","0",1);
  {native3d::Scope scope(ns);native_realtime::Session stats(before,true);auto f=future(z,c,20);
    old_value=m.dynamicScore(s,sc,a,us,f,false);}
  ::setenv("NATIVE_DYNAMIC_FAST","1",1);
  {native3d::Scope scope(ns);native_realtime::Session stats(after,true);auto f=future(z,c,20);
    new_value=m.dynamicScore(s,sc,a,us,f,false);}
  assert(!old_value.merit.feasible&&!new_value.merit.feasible);
  near(old_value.merit.min_gap,new_value.merit.min_gap);
  assert(before.values[native_realtime::body_visits]==161);
  assert(after.values[native_realtime::body_visits]==1);
  assert(before.values[native_realtime::query_calls]==2&&after.values[native_realtime::query_calls]==1);
  assert(old_value.merit.cost>new_value.merit.cost); // Only rejected-route diagnostic cost changes.
}
static void fullPlans(){
  auto c=config();auto sc=scene("empty",c);S s{.1,0.,0.,.17,0.};
  for(int kind:{0,1,2,3,4}){
    auto ns=state(payload(kind));Z z;
    if(kind==1||kind==2)z.os.push_back({{1.05,0.},{-.18,0.},.2,true});
    if(kind==2)z.os.push_back({{.7,-.65},{0.,.22},.2,true});
    // Force the dynamic scorer even when observations clear, just as a
    // controller continuing after a previously seen moving obstacle does.
    MPPI old(c,11),fast(c,11);old.trace_enabled=fast.trace_enabled=true;
    old.d_had_dynamic=fast.d_had_dynamic=true;
    for(int turn=0;turn<3;++turn){V u0,u1;
      ::setenv("NATIVE_DYNAMIC_FAST","0",1);{native3d::Scope scope(ns);u0=old.plan(s,z,sc);}
      ::setenv("NATIVE_DYNAMIC_FAST","1",1);{native3d::Scope scope(ns);u1=fast.plan(s,z,sc);}
      near(u0.x,u1.x);near(u0.y,u1.y);assert(old.search_trace.size()==fast.search_trace.size());
      assert(old.warm.size()==fast.warm.size());for(std::size_t k=0;k<old.warm.size();++k){near(old.warm[k].x,fast.warm[k].x);near(old.warm[k].y,fast.warm[k].y);}
      for(std::size_t k=0;k<old.search_trace.size();++k){const auto&a=old.search_trace[k];const auto&b=fast.search_trace[k];
        assert(a.feasible==b.feasible&&a.selected==b.selected&&a.controls.size()==b.controls.size());
        for(std::size_t j=0;j<a.controls.size();++j){near(a.controls[j].x,b.controls[j].x);near(a.controls[j].y,b.controls[j].y);}
        if(a.feasible){near(a.cost,b.cost);near(a.gap,b.gap);}
      }
      for(std::size_t k=0;k<old.decision_debug.size();++k)near(old.decision_debug[k],fast.decision_debug[k]);
      s=step(s,u0,c.dt,c);
    }
  }
}
int main(){
  ::setenv("NATIVE_COLLISION_3D","1",1);::setenv("NATIVE_REALTIME_DEMAND","1",1);
  ::setenv("NATIVE_REALTIME_PROFILE","1",1);::setenv("NATIVE_REALTIME_REPLAY","1",1);
  for(const char*value:{"0","","true","11"}){::setenv("NATIVE_DYNAMIC_FAST",value,1);assert(!native_dynamic_fast::enabled());}
  ::setenv("NATIVE_DYNAMIC_FAST","1",1);assert(native_dynamic_fast::enabled());
  cacheBoundaries();earlyAndUnknown();earlyPruneCounts();fullPlans();
  std::cout<<"native dynamic fast: scope/exact-key cache, legacy partial gap, unknown fail-closed, half-step states, intervention re-score, early-prune work and 5 full-plan fixtures passed\n";
}
