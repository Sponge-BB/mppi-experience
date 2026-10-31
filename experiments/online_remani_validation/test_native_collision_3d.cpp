#include "native_collision_3d.hpp"
#include <cassert>
#include <iostream>
#include <random>

using native3d::Point;
static void near(double a,double b,double tolerance=1e-10){
  if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>tolerance*(1+std::abs(a)+std::abs(b))){
    std::cerr<<"native3d mismatch "<<a<<" vs "<<b<<'\n';std::abort();
  }
}
static double field(const Point&p){return .27+.02*p.x+.03*p.y+.01*p.z+.012*p.x*p.y-.009*p.y*p.z;}
static std::vector<double> fixture(){
  std::vector<double> p(32,0.);p[0]=3;p[1]=1;p[2]=1;p[3]=100.;p[4]=1;
  p[5]=17;p[6]=17;p[7]=13;p[8]=-.8;p[9]=-.8;p[10]=-.4;p[11]=.1;
  p[12]=2;p[13]=0;p[14]=1;p[15]=.35;p[16]=3.;p[17]=10.;p[18]=100.;
  p[19]=1;p[20]=1;p[21]=.05;p[22]=1;p[23]=1;p[24]=2.;p[28]=1;
  const double bodies[]={.1,-.04,.2,.2,.35,17.,0., -.1,.08,.4,.1,.18,3.4,1.};
  p.insert(p.end(),std::begin(bodies),std::end(bodies));
  for(int x=0;x<int(p[5]);++x)for(int y=0;y<int(p[6]);++y)for(int z=0;z<int(p[7]);++z)
    p.push_back(field({p[8]+(x+.5)*p[11],p[9]+(y+.5)*p[11],p[10]+(z+.5)*p[11]}));
  return p;
}
static std::vector<double> withDynamic(std::vector<double> p,const std::vector<std::array<double,7>>&points){
  const std::size_t offset=32+7*std::size_t(p[12]);p[13]=points.size();
  for(auto it=points.rbegin();it!=points.rend();++it)p.insert(p.begin()+offset,it->begin(),it->end());
  p[2]+=1;return p;
}
static native3d::State makeState(const std::vector<double>&p){
  native3d::State state;state.now=100.;assert(native3d::bind(state,p.data(),p.size()));return state;
}
static native3d::Evaluation query(const native3d::State&state,double x=0,double y=0,double yaw=0,double t=100.,bool dynamic=true){
  native3d::Scope scope(state);return native3d::query(x,y,yaw,t,dynamic);
}
int main(){
  auto payload=fixture();auto state=makeState(payload);
  auto out=query(state);assert(out.valid&&out.gap>0);
  // Independent analytic trilinear reference includes mixed terms and the
  // native voxel-centre half-cell convention (not corner-index interpolation).
  std::mt19937 rng(101);std::uniform_real_distribution<double> pos(-.3,.3),angle(-3.,3.);
  for(int iteration=0;iteration<300;++iteration){
    const double x=pos(rng),y=pos(rng),yaw=angle(rng);out=query(state,x,y,yaw,100.,false);
    assert(out.valid);double minimum=1e9,cost=0;
    for(const auto&body:state.snapshot->body){
      const Point p{x+std::cos(yaw)*body.p.x-std::sin(yaw)*body.p.y,
        y+std::sin(yaw)*body.p.x+std::cos(yaw)*body.p.y,body.p.z};
      const double d=field(p);minimum=std::min(minimum,d-body.hard_radius);
      if(body.type==1)minimum=std::min(minimum,p.z-payload[21]);
      const double error=std::max(0.,body.soft_radius-d);cost+=body.weight*error*error*error;
    }
    near(out.gap,minimum);near(out.cost,cost);
  }
  assert(!query(state,100.,0.,0.,100.,false).valid); // Unknown crop is not free.
  // No new uncertainty dilation: configured non-zero values fail explicitly.
  for(int index:{25,26,27}){auto bad=payload;bad[index]=.001;auto copy=state;
    assert(!native3d::bind(copy,bad.data(),bad.size()));assert(!copy.snapshot);}
  // A new version validates every raw ESDF cell, including signed distances.
  auto bad=payload;bad[14]=2;bad.back()=std::numeric_limits<double>::quiet_NaN();
  assert(!native3d::bind(state,bad.data(),bad.size()));assert(!query(state).valid);
  assert(native3d::bind(state,payload.data(),payload.size()));
  auto delta=payload;delta.resize(32+14);delta[20]=0;delta[2]=5;delta[3]=100.01;
  const auto cached=state.cached_map;assert(native3d::bind(state,delta.data(),delta.size()));
  assert(state.cached_map==cached);near(query(state).gap,query(makeState(payload)).gap);
  auto empty=native3d::State{};assert(!native3d::bind(empty,delta.data(),delta.size()));
  // A moving crop does not necessarily advance the global ESDF sequence.
  // Full new-crop data must bind; an unmatched delta must never reuse it.
  auto recrop=payload;recrop[8]+=.1;recrop[2]=6;
  assert(native3d::bind(state,recrop.data(),recrop.size()));
  assert(state.cached_map->sequence==payload[14]);
  assert(state.cached_map->origin.x==recrop[8]);
  assert(!native3d::bind(state,delta.data(),delta.size()));
  assert(native3d::bind(state,payload.data(),payload.size()));
  bad=delta;bad[14]=2;assert(!native3d::bind(state,bad.data(),bad.size()));
  assert(native3d::bind(state,delta.data(),delta.size()));
  // Map acquisition/joint audit timestamps can be old for an actually locked
  // arm and immutable ESDF. Dynamic/publication heartbeat may not be stale.
  state.now=100.2;assert(query(state).valid);state.now=100.36;assert(!query(state).valid);
  state.now=100.;bad=payload;bad[22]=0;auto ground=makeState(bad);assert(!query(ground).valid);
  bad=payload;bad[23]=0;auto self=makeState(bad);assert(!query(self).valid);
  // Group by both track and exact velocity. Retain actual cloud points rather
  // than replacing a hollow obstacle with its enclosing sphere.
  const std::vector<std::array<double,7>> points={
    {{.3,.0,.2,-.04,.01,0.,10.}},{{.3,.1,.2,-.04,.01,0.,10.}},
    {{-.3,.0,.7,.02,0.,-.01,10.}},{{.0,.3,.4,0.,-.02,0.,11.}}};
  auto dynamic=withDynamic(payload,points);auto dyn=makeState(dynamic);
  assert(dyn.snapshot->groups->size()==3);
  for(int iteration=0;iteration<300;++iteration){
    const double x=pos(rng),y=pos(rng),yaw=angle(rng),dt=iteration/100.;
    const auto value=query(dyn,x,y,yaw,100.+dt);assert(value.valid);
    double brute=1e9,cost=0,hard=1e9;
    for(const auto&body:dyn.snapshot->body){
      Point p{x+std::cos(yaw)*body.p.x-std::sin(yaw)*body.p.y,
        y+std::sin(yaw)*body.p.x+std::cos(yaw)*body.p.y,body.p.z};
      double distance=1e9;for(const auto&obstacle:points)
        distance=std::min(distance,std::sqrt(native3d::squaredDistance(p,
          {obstacle[0]+dt*obstacle[3],obstacle[1]+dt*obstacle[4],obstacle[2]+dt*obstacle[5]})));
      brute=std::min(brute,distance-body.hard_radius);
      hard=std::min(hard,std::min(field(p),distance)-body.hard_radius);
      if(body.type==1)hard=std::min(hard,p.z-payload[21]);
      const double error=std::max(0.,body.soft_radius-std::min(field(p),distance));
      cost+=body.weight*error*error*error;
    }
    near(value.dynamic_gap,brute);near(value.gap,hard);near(value.cost,cost);
  }
  assert(!query(dyn,0.,0.,0.,103.01).valid);
  assert(query(dyn,0.,0.,0.,103.01,false).valid);
  assert(query(makeState(payload),0.,0.,0.,105.).valid); // Empty actual cloud needs no CV extrapolation.
  const auto groups=dyn.snapshot->groups;
  assert(native3d::bind(dyn,dynamic.data(),dynamic.size()));assert(dyn.snapshot->groups==groups);
  // A scope owns immutable inputs across another bind and restores nesting.
  {native3d::Scope scope(dyn);const auto frozen=native3d::context().snapshot;
    assert(native3d::hasDynamic());assert(native3d::bind(dyn,payload.data(),payload.size()));
    assert(native3d::context().snapshot==frozen);
    {native3d::Scope inner(dyn);assert(!native3d::hasDynamic());}
    assert(native3d::hasDynamic());}
  assert(!native3d::context().active);
  // Same-pose gap/cost reuse, interleaved reference query and scope invalidation.
  {native3d::Scope scope(ground);assert(!native3d::query(0,0,0,100.).valid);}
  {native3d::Scope scope(dyn);const auto full=native3d::query(*dyn.snapshot,0.,0.,0.,100.,true,false);
    native3d::query(0.,0.,0.,100.,true,true);
    native3d::query(.01,0.,0.,100.,true,true);
    const auto value=native3d::query(0.,0.,0.,100.,true,false);
    assert(value.valid==full.valid);near(value.gap,full.gap);near(value.cost,full.cost);}
  std::cout<<"native collision 3D: payload, native-centre interpolation, soft/hard separation, KD exactness, clocks, scope and fail-closed tests passed\n";
}
