// Offline local-control experiment. No ros::init, NodeHandle, publishers or drivers.
// CV and its uncertainty are linked directly from the workspace implementation.
#include <plan_env/dynamic_point_buffer.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace experiment {
using Distance = double (*)(double,double);
Distance map_distance=nullptr;
constexpr double pi=3.14159265358979323846;
double clip(double x,double a,double b){return std::max(a,std::min(b,x));}
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
struct V{double x=0,y=0;};
V operator+(V a,V b){return {a.x+b.x,a.y+b.y};}
V operator-(V a,V b){return {a.x-b.x,a.y-b.y};}
V operator*(double a,V b){return {a*b.x,a*b.y};}
double dot(V a,V b){return a.x*b.x+a.y*b.y;}
double norm(V a){return std::hypot(a.x,a.y);}
struct Disk{V p;double r;};
struct S{double x=0,y=0,yaw=0,v=0,w=0;};
struct O{V p,vel;double r=.23;bool moving=true;};
struct C{
  double vmax,wmax,accel,angular_accel,track,wheel_v,wheel_a;
  double margin,max_age,max_prediction,pos_unc,vel_unc,acc_unc;
  double goal_pos,goal_yaw,goal_speed;
  double dt=.05,rdt=.10,tau=.12,alpha=1.2;
  int samples=192,horizon=15;
  std::vector<Disk> body;
};
V wheels(V u,const C&c){return {u.x-.5*c.track*u.y,u.x+.5*c.track*u.y};}
V bound(V u,const C&c){
  double k=std::max({1.,std::abs(u.x)/c.vmax,std::abs(u.y)/c.wmax,
    std::abs(wheels(u,c).x)/c.wheel_v,std::abs(wheels(u,c).y)/c.wheel_v});
  return (1/k)*u;
}
S step(S s,V u,double dt,const C&c,double tau=-1,double sub=.01){
  u=bound(u,c);if(tau<0)tau=c.tau;
  int n=std::max(1,int(std::ceil(dt/sub-1e-10)));double h=dt/n;
  for(int j=0;j<n;++j){
    V a={(u.x-s.v)/tau,(u.y-s.w)/tau};auto wa=wheels(a,c);
    double k=std::max({1.,std::abs(a.x)/c.accel,std::abs(a.y)/c.angular_accel,
      std::abs(wa.x)/c.wheel_a,std::abs(wa.y)/c.wheel_a});a=(1/k)*a;
    double vn=s.v+h*a.x,wn=s.w+h*a.y,vm=.5*(s.v+vn),wm=.5*(s.w+wn);
    double heading=s.yaw+.5*h*wm;s.x+=h*vm*std::cos(heading);s.y+=h*vm*std::sin(heading);
    s.yaw=wrap(s.yaw+h*wm);s.v=vn;s.w=wn;
  }return s;
}
V bodypoint(const S&s,V p){return {s.x+std::cos(s.yaw)*p.x-std::sin(s.yaw)*p.y,
  s.y+std::sin(s.yaw)*p.x+std::cos(s.yaw)*p.y};}
struct Ref{V p;double yaw,v;};
struct Scene{
  std::string name;std::vector<Ref> path;double duration=32,ymax=1.8,delay=0,drop0=-1,drop1=-1,plant_tau=.12;
  double velocity_error=0;
};
Scene scene(const std::string&name,const C&c){
  Scene sc;sc.name=name;
  if(name=="delayed_crossing"){sc.delay=.15;sc.drop0=4;sc.drop1=4.45;sc.velocity_error=.05;}
  if(name=="lag_mismatch"){sc.delay=.10;sc.plant_tau=.28;}
  const double speed=.85*std::min(c.vmax,c.wheel_v);
  for(int i=0;i<=120;++i){
    double t=i/120.,x=3*t,y=0,yaw=0,v=speed;
    if(name=="curve"){x=2.8*t;y=.45*(1-std::cos(pi*t));yaw=std::atan2(.45*pi*std::sin(pi*t),2.8);}
    if(name=="reverse"){x=-.8*t;v=-speed;}
    if(i==120)v=0;
    sc.path.push_back({{x,y},yaw,v});
  }return sc;
}
std::vector<O> truth(const Scene&sc,double t){
  if(sc.name=="empty"||sc.name=="curve"||sc.name=="reverse")return {};
  if(sc.name=="new_static")return t<2 ? std::vector<O>{}:std::vector<O>{{{1.55,0},{0,0},.25,false}};
  if(sc.name=="late_intrusion")return t<4.5 ? std::vector<O>{}:std::vector<O>{{{1.48,0},{0,0},.25,false}};
  if(sc.name=="close_clearance")return t<4.5 ? std::vector<O>{}:std::vector<O>{{{1.88,0},{0,0},.25,false}};
  if(sc.name=="blocked"){
    std::vector<O> os;for(int i=-3;i<=3;++i)os.push_back({{1.65,.52*i},{0,0},.29,false});return os;
  }
  return {{{1.45,-1.70+.28*t},{0,.28},.23,true}};
}
struct Z{std::vector<O> os;double t=0,stamp=0;bool fresh=true;};
Z observe(const Scene&sc,double t,const C&c){
  double stamp=std::floor((t-sc.delay+1e-9)/.1)*.1;
  if(t>=sc.drop0&&t<sc.drop1)stamp=std::floor((sc.drop0-sc.delay-1e-6)/.1)*.1;
  Z z{truth(sc,stamp),t,stamp,t-stamp<=c.max_age+1e-9};
  for(auto&o:z.os)if(o.moving)o.vel.x+=sc.velocity_error;
  return z;
}
std::vector<O> predict(const Z&z,double ahead,const C&c){
  auto os=z.os;
  for(auto&o:os){
    if(!o.moving){o.r+=c.pos_unc;continue;}
    plan_env::DynamicPoint p;p.position={o.p.x,o.p.y,0};p.velocity={o.vel.x,o.vel.y,0};
    Eigen::Vector3d result;double dt;
    if(!plan_env::DynamicPointBuffer::predictPosition(p,ros::Time(1000+z.stamp),
        ros::Time(1000+z.t+ahead),c.max_prediction,result,dt))
      throw std::runtime_error("prediction exceeds production time validity");
    o.p={result.x(),result.y()};
    o.r+=plan_env::DynamicPointBuffer::predictionUncertainty(dt,c.pos_unc,c.vel_unc,c.acc_unc);
  }return os;
}
double clearance(const S&s,const std::vector<O>&os,const C&c,double ymax,double margin=0){
  double d=1e6;for(const auto&b:c.body){auto p=bodypoint(s,b.p);
    d=std::min(d,ymax-std::abs(p.y)-b.r-margin);
    if(map_distance)d=std::min(d,map_distance(p.x,p.y)-b.r-margin);
    for(const auto&o:os)d=std::min(d,norm(p-o.p)-b.r-o.r-margin);
  }return d;
}
std::vector<double> barrier(const S&s,const Z&z,double ahead,const C&c,double ymax){
  auto os=predict(z,ahead,c);std::vector<double> hs;
  for(const auto&b:c.body){auto p=bodypoint(s,b.p);
    for(const auto&o:os){double r=b.r+o.r+c.margin;hs.push_back(dot(p-o.p,p-o.p)-r*r);}
    hs.push_back(ymax-p.y-b.r-c.margin);hs.push_back(ymax+p.y-b.r-c.margin);
    if(map_distance)hs.push_back(map_distance(p.x,p.y)-b.r-c.margin);
  }return hs;
}
struct Constraint{V a;double b;};
std::vector<Constraint> bounds(const C&c){
  double h=.5*c.track;
  return {{{1,0},-c.vmax},{{-1,0},-c.vmax},{{0,1},-c.wmax},{{0,-1},-c.wmax},
    {{1,h},-c.wheel_v},{{-1,-h},-c.wheel_v},{{1,-h},-c.wheel_v},{{-1,h},-c.wheel_v}};
}
struct QP{V u;bool ok=false;};
QP qp(V nominal,const std::vector<Constraint>&cs){
  QP best;double value=1e100;
  auto consider=[&](V u){
    if(!std::isfinite(u.x)||!std::isfinite(u.y))return;
    for(auto con:cs)if(dot(con.a,u)<con.b-1e-8)return;
    V d=u-nominal;double f=d.x*d.x+.2*d.y*d.y;if(f<value){value=f;best={u,true};}
  };
  consider(nominal);
  for(size_t i=0;i<cs.size();++i){auto a=cs[i].a;double den=a.x*a.x+5*a.y*a.y;
    if(den>1e-18){double k=(cs[i].b-dot(a,nominal))/den;consider(nominal+V{k*a.x,5*k*a.y});}
    for(size_t j=0;j<i;++j){auto b=cs[j].a;double det=a.x*b.y-a.y*b.x;
      if(std::abs(det)>1e-12)consider({(cs[i].b*b.y-a.y*cs[j].b)/det,(a.x*cs[j].b-cs[i].b*b.x)/det});}
  }return best;
}
struct F{V u;bool intervention=false,qp_failed=false,unverified=false;};
F filter(const S&s,V nominal,const Z&z,const C&c,double ymax){
  auto h0=barrier(s,z,0,c,ymax);
  auto residual=[&](V u){auto hs=barrier(step(s,u,c.dt,c),z,c.dt,c,ymax);
    for(size_t i=0;i<hs.size();++i){hs[i]-=std::exp(-c.alpha*c.dt)*h0[i];}
    return hs;};
  auto check=[&](V u){for(double r:residual(u))if(!std::isfinite(r)||r< -1e-7)return false;
    S p=s;for(int k=1;k<=10;++k){p=step(p,u,c.dt/10,c,-1,c.dt/10);
      if(clearance(p,predict(z,k*c.dt/10,c),c,ymax,c.margin)<-1e-7)return false;}return true;};
  if(check(nominal))return {nominal};
  // Linearize near actual velocity: derivatives of a saturated velocity target
  // are often zero. Each QP solution still needs the original nonlinear check.
  V u=bound({s.v,s.w},c);constexpr double eps=1e-4;
  for(int iter=0;iter<5;++iter){auto r=residual(u),rx=residual(u+V{eps,0}),ry=residual(u+V{0,eps});
    auto cs=bounds(c);for(size_t i=0;i<r.size();++i){V a={(rx[i]-r[i])/eps,(ry[i]-r[i])/eps};cs.push_back({a,dot(a,u)-r[i]});}
    auto q=qp(nominal,cs);if(!q.ok)break;u=q.u;
    if(check(u))return {u,norm(u-nominal)>1e-6,false,false};
  }
  return {{0,0},true,true,!check({0,0})}; // A stop request is NOT a safety proof.
}
int nearest(const S&s,const Scene&sc){int best=0;double d=1e100;
  for(size_t i=0;i<sc.path.size();++i){double v=dot(sc.path[i].p-V{s.x,s.y},sc.path[i].p-V{s.x,s.y});if(v<d){d=v;best=int(i);}}return best;}
Ref trackerReference(const S&s,const Scene&sc){
  int i=nearest(s,sc);int j=i;
  while(j+1<int(sc.path.size())&&norm(sc.path[j].p-sc.path[i].p)<.07)++j;
  Ref r=sc.path[j];double remaining=norm(sc.path.back().p-V{s.x,s.y});
  r.v=std::copysign(std::min(std::abs(r.v),.8*remaining),r.v);
  // Let the original Tracker align a large heading change without latching a
  // moving-reference fault; this is the shared reference adapter, not a planner.
  if(std::abs(wrap(r.yaw-s.yaw))>.45){r.p={s.x,s.y};r.v=0;}
  return r;
}
// Isolated experiment: path-relative MPPI, no CBF, no safety certificate.
// CV mean + present uncertainty is hard-costed; future uncertainty is soft risk.
struct BaseMPPI {
  int proposal_divisor=1;
  const C& c; std::mt19937 rng; std::normal_distribution<double> normal{0,1};
  std::vector<V> warm; std::array<double,8> debug{};
  BaseMPPI(const C& cc,int seed):c(cc),rng(seed),warm(c.horizon){}
  void reset(){std::fill(warm.begin(),warm.end(),V{});}
  struct Projection {int index; double distance,arc,yaw,speed;};
  Projection project(const S& s,const Scene& sc,const std::vector<double>& arc,int hint=-1)const {
    Projection out{0,1e9,0,0,0};double best_fraction=0;
    int lo=hint<0?0:std::max(0,hint-8),hi=hint<0?int(sc.path.size())-1:std::min(int(sc.path.size())-1,hint+45);
    for(int i=lo;i<hi;++i){V a=sc.path[i].p,b=sc.path[i+1].p,d=b-a;double length=arc[i+1]-arc[i];if(length<1e-9)continue;
      double f=clip(dot(V{s.x,s.y}-a,d)/(length*length),0,1);V delta=V{s.x,s.y}-(a+f*d);double dist=dot(delta,delta);
      if(dist<out.distance){out={i,dist,arc[i]+f*length,0,0};best_fraction=f;}
    }
    out.distance=std::sqrt(out.distance);int i=out.index;double f=best_fraction;
    out.yaw=wrap(sc.path[i].yaw+f*wrap(sc.path[i+1].yaw-sc.path[i].yaw));out.speed=(1-f)*sc.path[i].v+f*sc.path[i+1].v;
    return out;
  }
  int direction(const Scene& sc,int i)const {
    for(int j=i;j<std::min(int(sc.path.size()),i+12);++j)if(std::abs(sc.path[j].v)>.005)return sc.path[j].v>0?1:-1;
    for(int j=i;j>=0;--j)if(std::abs(sc.path[j].v)>.005)return sc.path[j].v>0?1:-1;
    return 1;
  }
  V guide(const S& s,const Scene& sc,const std::vector<double>& arc,const std::vector<V>& targets,int& hint)const {
    auto p=project(s,sc,arc,hint);hint=p.index;int sign=direction(sc,p.index);
    double remaining=arc.back()-p.arc,goal=norm(sc.path.back().p-V{s.x,s.y});
    // Terminal proposal is evaluated by the same optimizer, never overrides it.
    if(goal<.025)return bound({0,1.8*wrap(sc.path.back().yaw-s.yaw)},c);
    int target=p.index;double lookahead=.20+.35*std::abs(s.v);
    while(target+1<int(sc.path.size())&&arc[target]<p.arc+lookahead){
      if(std::abs(sc.path[target].v)>.005&&direction(sc,target)!=sign)break;
      ++target;
    }
    V delta=targets[target]-V{s.x,s.y};double travel_yaw=s.yaw+(sign<0?pi:0);
    // Align to the attainable lookahead, not to the nearest tangent behind a
    // blocked bend. The old tangent gate undid an obstacle-avoidance turn.
    double heading=wrap(std::atan2(delta.y,delta.x)-travel_yaw);
    if(std::abs(heading)>1.2)return bound({0,1.8*heading},c);
    double lateral=-std::sin(travel_yaw)*delta.x+std::cos(travel_yaw)*delta.y;
    double speed=std::max(std::abs(p.speed),std::abs(sc.path[target].v));
    speed=std::min({speed,.85*c.wheel_v,.9*std::max(goal,remaining)});
    speed*=std::max(.2,std::cos(heading));
    return bound({sign*speed,2*speed*lateral/std::max(.015,dot(delta,delta))},c);
  }
  struct Score {double total=0,path=0,risk=0,collision=0,direction=0;};
  virtual Score cost(S s,const std::vector<V>& us,const std::vector<std::vector<O>>& nominal,
             const std::vector<std::vector<O>>& uncertain,const Scene& sc,const std::vector<double>& arc,int initial_hint)const {
    Score value;V prev{s.v,s.w};auto p=project(s,sc,arc,initial_hint);double initial=p.arc,terminal_adherence=1;
    for(size_t k=0;k<us.size();++k){s=step(s,us[k],c.rdt,c);p=project(s,sc,arc,p.index);
      double goal=norm(sc.path.back().p-V{s.x,s.y});double d=clearance(s,nominal[k],c,sc.ymax,c.margin);
      double risk=d; // v3: identical nominal/uncertain geometry
      double speed=std::copysign(std::min(std::abs(p.speed),.9*goal),p.speed);
      double yaw_error=wrap((goal<.05?sc.path.back().yaw:p.yaw)-s.yaw);
      // Do not tightly track a reference pose which this very same local
      // collision model considers blocked. Relax locally, recover full weight
      // in free space. Collision penalties themselves are not weakened.
      S reference_pose{s.x,s.y,p.yaw,0,0};
      V pa=sc.path[p.index].p,pb=sc.path[p.index+1].p;
      double f=clip((p.arc-arc[p.index])/std::max(1e-9,arc[p.index+1]-arc[p.index]),0,1);
      V rp=pa+f*(pb-pa);reference_pose.x=rp.x;reference_pose.y=rp.y;
      double reference_clearance=clearance(reference_pose,nominal[k],c,sc.ymax,c.margin);
      double adherence=clip(reference_clearance/.10,.10,1.);
      terminal_adherence=adherence;
      double path_cost=adherence*(100*p.distance*p.distance+2*yaw_error*yaw_error)
        +200*std::pow(std::max(0.,p.distance-.25),2)+8*std::pow(s.v-speed,2);
      double collision=d<0?1600+9000*d*d:0,near=std::max(0.,.04-d);
      double risk_cost=2*std::pow(clip(-risk,0.,.5),2)+15*near*near;
      double wrong=std::max(0.,-direction(sc,p.index)*s.v);
      double direction_cost=150*wrong*wrong+100*std::pow(std::max(0.,initial-p.arc),2);
      value.path+=c.rdt*path_cost;value.risk+=c.rdt*risk_cost;value.collision+=c.rdt*collision;value.direction+=c.rdt*direction_cost;
      V du=us[k]-prev;value.total+=.02*dot(du,du);prev=us[k];
    }
    // Along-path progress avoids a straight-line attraction across curve chords.
    value.total+=value.path+value.risk+value.collision+value.direction+30*terminal_adherence*p.distance*p.distance-4*(p.arc-initial);
    double goal=norm(sc.path.back().p-V{s.x,s.y});
    if(arc.back()-p.arc<.15){value.total+=30*goal*goal;if(goal<.05)value.total+=2*std::pow(wrap(s.yaw-sc.path.back().yaw),2)+4*s.v*s.v;}
    return value;
  }
  virtual void adjustTargets(const S&,const Z&,const Scene&,const std::vector<double>&,std::vector<V>&)const {}
  V plan(const S& s,const Z& z,const Scene& sc){
    if(!z.fresh||sc.path.size()<2)return {};
    // Static bends may need a rotate-then-advance maneuver longer than 1.5 s.
    // Keep dynamic prediction within its existing validity contract. Trade
    // samples for horizon, preserving approximately the rollout-step budget.
    const int horizon=z.os.empty()?30:std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));
    const int samples=std::max(32,c.samples*c.horizon/horizon/proposal_divisor);
    warm.resize(horizon);
    std::vector<double> arc(sc.path.size(),0);for(size_t i=1;i<arc.size();++i)arc[i]=arc[i-1]+norm(sc.path[i].p-sc.path[i-1].p);
    auto initial_projection=project(s,sc,arc);
    // Collision-consistent proposal targets, NOT replacements for the native
    // path or actual rollout checks. Offset only within a bounded normal band.
    // Precompute once per cycle; never query obstacle truth.
    std::vector<V> targets;targets.reserve(sc.path.size());
    for(size_t i=0;i<sc.path.size();++i){const auto& r=sc.path[i];V target=r.p;
      if(map_distance && arc[i]>=initial_projection.arc-.05 && arc[i]<=initial_projection.arc+1.2){
        auto distance_at=[&](V p){return clearance(S{p.x,p.y,r.yaw,0,0},{},c,sc.ymax,c.margin);};
        double best=distance_at(target);V normal_to_path{-std::sin(r.yaw),std::cos(r.yaw)};
        if(best<.04)for(int n=1;n<=6;++n){
          V plus=r.p+(.05*n)*normal_to_path,minus=r.p-(.05*n)*normal_to_path;
          double dp=distance_at(plus),dm=distance_at(minus);
          if(std::max(dp,dm)>best){best=std::max(dp,dm);target=dp>=dm?plus:minus;}
          if(best>=.04)break;
        }
      }targets.push_back(target);
    }
    adjustTargets(s,z,sc,arc,targets);
    std::vector<std::vector<O>> nominal,uncertain;double age=std::max(0.,z.t-z.stamp);
    double observed_unc=plan_env::DynamicPointBuffer::predictionUncertainty(age,c.pos_unc,c.vel_unc,c.acc_unc);
    for(int k=0;k<horizon;++k){double ahead=(k+1)*c.rdt;auto future=predict(z,ahead,c);uncertain.push_back(future);
      double future_unc=plan_env::DynamicPointBuffer::predictionUncertainty(age+ahead,c.pos_unc,c.vel_unc,c.acc_unc);
      for(size_t j=0;j<future.size();++j)if(z.os[j].moving)future[j].r-=std::max(0.,future_unc-observed_unc);
      nominal.push_back(future);
    }
    std::vector<std::vector<V>> candidates(samples,std::vector<V>(horizon));std::vector<Score> scores(samples);int best=0;
    int initial_hint=initial_projection.index;
    for(int j=0;j<samples;++j){S roll=s;V noise{};int hint=initial_hint;
      for(int k=0;k<horizon;++k){V g=guide(roll,sc,arc,targets,hint),u=g;noise=.75*noise+V{.045*normal(rng),.18*normal(rng)};
        if(j==0)u=warm[k];else if(j==1)u={0,0};else if(j==2)u=g;else if(j==3)u=.5*g;
        else if(j==4||j==5)u={.65*g.x,g.y+(j==4?.35:-.35)};
        // Reference-relative reverse cost, not a global reverse prohibition.
        else if(j==6)u={-direction(sc,hint)*.10,0};
        else if(j==7||j==8)u={0,j==7?.45:-.45};
        else if(j==9||j==10)u=k<12?V{0,j==9?.75:-.75}:g;
        else if(j==11||j==12)u=k<(j==11?4:8)?V{0,0}:g;
        else u=(j%2?g:.5*g+.5*warm[k])+noise;
        candidates[j][k]=bound(u,c);roll=step(roll,candidates[j][k],c.rdt,c);
      }
      scores[j]=cost(s,candidates[j],nominal,uncertain,sc,arc,initial_hint);if(scores[j].total<scores[best].total)best=j;
    }
    std::vector<V> mean(horizon);double total=0;
    for(int j=0;j<samples;++j){double weight=std::exp(-(scores[j].total-scores[best].total)/.12);total+=weight;for(int k=0;k<horizon;++k)mean[k]=mean[k]+weight*candidates[j][k];}
    for(auto& u:mean)u=bound((1/total)*u,c);
    auto selected=cost(s,mean,nominal,uncertain,sc,arc,initial_hint);int selected_index=-1;
    if(selected.total>scores[best].total+.03||selected.collision>scores[best].collision+1e-8){mean=candidates[best];selected=scores[best];selected_index=best;}
    debug={selected.total,selected.path,selected.risk,selected.collision,selected.direction,double(selected_index),scores[1].total,scores[6].total};
    warm=mean;return mean.front();
  }
  void feedback(V actual){double fraction=c.dt/c.rdt;for(size_t k=0;k+1<warm.size();++k)warm[k]=(1-fraction)*warm[k]+fraction*warm[k+1];warm[0]=.5*warm[0]+.5*actual;}
};

// A geometric local route proposal + MPPI execution, not pure MPPI and not a
// safety certificate. The native safety/abort chain remains authoritative.
struct LocalGuideMPPI : BaseMPPI {
  Scene route;std::array<double,8> status{};int side=0,cycles=0;
  double envelope=0;bool failed=false;
  LocalGuideMPPI(const C& config,int seed):BaseMPPI(config,seed){
    if(c.pos_unc!=0||c.vel_unc!=0||c.acc_unc!=0)
      throw std::runtime_error("v4 requires no_uncertainty profile");
  }
  void reset(){BaseMPPI::reset();route.path.clear();side=cycles=0;status={};}
  virtual double freeDistance(V p)const{return map_distance?map_distance(p.x,p.y)-envelope-c.margin:1e3;}
  virtual bool segment(V a,V b)const{
    int n=std::max(1,int(std::ceil(norm(b-a)/.04)));
    for(int i=0;i<=n;++i)if(freeDistance(a+(double(i)/n)*(b-a))<.015)return false;
    return true;
  }
  bool usable(const S&s)const{
    if(route.path.size()<2)return false;
    size_t closest=0;double best=1e9;
    for(size_t i=0;i<route.path.size();++i){double d=norm(route.path[i].p-V{s.x,s.y});if(d<best){best=d;closest=i;}}
    if(best>.5||!segment({s.x,s.y},route.path[closest].p))return false;
    for(size_t i=closest;i+1<route.path.size();++i)
      if(!segment(route.path[i].p,route.path[i+1].p))return false;
    return true;
  }
  // Small bounded Dijkstra/A* grid for proposal topology. Timing used here is
  // approximate; actual rollouts are still checked against time-indexed CV.
  virtual size_t chooseJoin(const S&,const Scene& sc)const{return sc.path.size()-1;}
  bool makeRoute(const S&s,const Z&z,const Scene&sc){
    const int n=77,mid=n/2,total=n*n;const double res=.10;
    V origin{s.x-mid*res,s.y-mid*res};size_t join=sc.path.size()-1;
    // Rejoin before the terminal pose so that the final reference heading is
    // approached along the original path, not imposed as an abrupt endpoint.
    join=chooseJoin(s,sc);
    V goal=sc.path[join].p;
    int tx=std::lround((goal.x-origin.x)/res),ty=std::lround((goal.y-origin.y)/res);
    if(tx<1||ty<1||tx>=n-1||ty>=n-1)return false;
    auto point=[&](int k){return origin+V{res*(k%n),res*(k/n)};};
    int start=mid*n+mid,end=ty*n+tx;
    std::vector<double> gs(total,1e12);std::vector<int> parent(total,-1);
    std::vector<unsigned char> closed(total,0);
    std::vector<double> clearance_grid(total);
    for(int k=0;k<total;++k)clearance_grid[k]=freeDistance(point(k));
    if(clearance_grid[start]<.015||clearance_grid[end]<.015||!segment(point(end),goal))return false;
    // Heap entries are immutable pairs; avoid mutating keys already enqueued.
    using Entry=std::pair<double,int>;
    std::vector<Entry> queue;
    auto greater=[](const Entry&a,const Entry&b){return a.first>b.first;};
    gs[start]=0;queue.push_back({norm(goal-V{s.x,s.y}),start});
    std::vector<std::vector<O>> future;for(int k=0;k<=15;++k)future.push_back(predict(z,.1*k,c));
    V baseline=goal-V{s.x,s.y};double length=std::max(.01,norm(baseline));
    for(int expanded=0;!queue.empty()&&expanded<total;++expanded){
      std::pop_heap(queue.begin(),queue.end(),greater);int u=queue.back().second;queue.pop_back();
      if(closed[u]){--expanded;continue;}closed[u]=1;if(u==end)break;
      int ux=u%n,uy=u/n;
      for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
        if(!dx&&!dy)continue;int x=ux+dx,y=uy+dy;if(x<0||y<0||x>=n||y>=n)continue;
        int v=y*n+x;if(closed[v]||clearance_grid[v]<.015)continue;
        if(dx&&dy&&(clearance_grid[uy*n+x]<.015||clearance_grid[y*n+ux]<.015))continue;
        V p=point(v),delta=p-V{s.x,s.y};double distance=norm(delta);
        double arrival=distance/std::max(.10,std::abs(s.v));double risk=0;
        // No prediction beyond 1.5 s, and never use obstacle truth.
        if(arrival<=1.5){int t=std::min(15,int(std::lround(arrival*10)));
          for(const auto&o:future[t])risk=std::max(risk,std::max(0.,.20-(norm(p-o.p)-o.r-envelope-c.margin))*12.);
        }
        double lateral=(baseline.x*delta.y-baseline.y*delta.x)/length;
        double switch_cost=side&&side*lateral<-.10?.25:0.;
        double edge=res*std::hypot(dx,dy)*(1.+risk+switch_cost+.02/std::max(.04,clearance_grid[v]));
        double g=gs[u]+edge;if(g>=gs[v])continue;gs[v]=g;parent[v]=u;
        queue.push_back({g+norm(goal-p),v});std::push_heap(queue.begin(),queue.end(),greater);
      }
    }
    if(parent[end]<0)return false;
    std::vector<V> reverse;for(int k=end;k!=start;k=parent[k]){if(k<0)return false;reverse.push_back(point(k));}
    reverse.push_back({s.x,s.y});std::reverse(reverse.begin(),reverse.end());reverse.push_back(goal);
    // Line-of-sight shortcut with the same conservative footprint model.
    std::vector<V> corners{reverse.front()};size_t i=0;
    while(i+1<reverse.size()){size_t j=reverse.size()-1;while(j>i+1&&!segment(reverse[i],reverse[j]))--j;
      if(!segment(reverse[i],reverse[j]))return false;corners.push_back(reverse[j]);i=j;
    }
    Scene candidate=sc;candidate.path.clear();int gear=direction(sc,0);
    for(size_t j=0;j+1<corners.size();++j){V a=corners[j],b=corners[j+1],d=b-a;
      int count=std::max(1,int(std::ceil(norm(d)/.04)));double yaw=wrap(std::atan2(d.y,d.x)+(gear<0?pi:0));
      for(int k=0;k<count;++k)candidate.path.push_back({a+(double(k)/count)*d,yaw,gear*.18});
    }
    for(size_t k=join;k<sc.path.size();++k)candidate.path.push_back(sc.path[k]);
    double most=0;for(const auto&r:candidate.path){V d=r.p-V{s.x,s.y};double lateral=(baseline.x*d.y-baseline.y*d.x)/length;if(std::abs(lateral)>std::abs(most))most=lateral;}
    if(std::abs(most)>.1)side=most>0?1:-1;
    route=std::move(candidate);return true;
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    if(sc.path.size()<2||!z.fresh)return {};
    envelope=0;for(const auto&b:c.body)envelope=std::max(envelope,norm(b.p)+b.r);
    std::vector<double> arc(sc.path.size());for(size_t i=1;i<arc.size();++i)arc[i]=arc[i-1]+norm(sc.path[i].p-sc.path[i-1].p);
    auto p=project(s,sc,arc);bool blocked=false;
    for(size_t i=p.index;i<sc.path.size();++i)if(freeDistance(sc.path[i].p)<.04){blocked=true;break;}
    bool active=!route.path.empty();failed=false;
    // Cached complete detour, not independent left/right offsets per point.
    if((active&&!usable(s))||(!active&&blocked)){
      if(!makeRoute(s,z,sc)){failed=true;route.path.clear();}
      else active=true;
    }
    active=!route.path.empty();
    auto u=BaseMPPI::plan(s,z,active?route:sc);
    status={double(active?1:0),double(side),p.distance,-1,0,double(route.path.size()),double(failed),0};
    return u;
  }
};

// Local proposal geometry matches MPPI body disks. The fixed physical margin,
// raster correction, native abort and final rollout checks are not removed.
struct V5MPPI : LocalGuideMPPI {
  double guide_yaw=0;int gear=1,clear_ticks=0,strategy=0,phase=0;
  std::vector<Ref> published_route;
  V5MPPI(const C& config,int seed):LocalGuideMPPI(config,seed){}
  void reset(){LocalGuideMPPI::reset();clear_ticks=strategy=phase=0;published_route.clear();}
  double poseDistance(V p,double yaw)const{
    return clearance(S{p.x,p.y,yaw,0,0},{},c,100.,c.margin);
  }
  double freeDistance(V p)const override{return poseDistance(p,guide_yaw);}
  bool segment(V a,V b)const override{
    double yaw=norm(b-a)<1e-5?guide_yaw:wrap(std::atan2(b.y-a.y,b.x-a.x)+(gear<0?pi:0));
    int n=std::max(1,int(std::ceil(norm(b-a)/.04)));
    for(int i=0;i<=n;++i)if(poseDistance(a+(double(i)/n)*(b-a),yaw)<.015)return false;
    return true;
  }
  std::vector<double> arcs(const Scene&sc)const{
    std::vector<double>a(sc.path.size());for(size_t i=1;i<a.size();++i)a[i]=a[i-1]+norm(sc.path[i].p-sc.path[i-1].p);return a;
  }
  size_t chooseJoin(const S&s,const Scene&sc)const override{
    auto a=arcs(sc);auto p=project(s,sc,a);double last=p.arc;
    for(size_t i=p.index;i<sc.path.size();++i){
      if(a[i]>p.arc+2.5)break;
      if(poseDistance(sc.path[i].p,sc.path[i].yaw)<.03)last=a[i];
    }
    for(size_t i=p.index;i<sc.path.size();++i){
      if(a[i]<std::max(p.arc+.3,last+.22))continue;
      bool clear=true;
      for(size_t k=i;k<sc.path.size()&&a[k]<a[i]+.35;++k)
        if(poseDistance(sc.path[k].p,sc.path[k].yaw)<.03){clear=false;break;}
      if(clear)return i;
    }
    return sc.path.size()-1;
  }
  Scene connector(const S&s,const Scene&sc,size_t j)const{
    Scene out=sc;out.path.clear();V a{s.x,s.y},b=sc.path[j].p,d=b-a;
    int n=std::max(1,int(std::ceil(norm(d)/.04)));double yaw=wrap(std::atan2(d.y,d.x)+(gear<0?pi:0));
    for(int k=0;k<n;++k)out.path.push_back({a+(double(k)/n)*d,yaw,gear*.18});
    out.path.insert(out.path.end(),sc.path.begin()+j,sc.path.end());return out;
  }
  double remaining(const S&s,const Scene&sc)const{
    if(sc.path.size()<2)return 1e6;auto a=arcs(sc);return a.back()-project(s,sc,a).arc;
  }
  // Same objective for proposals tracked against different reference paths.
  // Comparing their original tracking costs directly would be meaningless.
  double compareRollout(S s,const Z&z,const Scene&original,const std::vector<V>&us)const{
    auto a=arcs(original);auto p=project(s,original,a);double start=p.arc,cost=0;V previous{s.v,s.w};
    for(size_t k=0;k<us.size();++k){
      s=step(s,us[k],c.rdt,c);p=project(s,original,a,p.index);
      double d=clearance(s,predict(z,(k+1)*c.rdt,c),c,100.,c.margin);
      double reverse=std::max(0.,-gear*s.v);V du=us[k]-previous;previous=us[k];
      cost+=c.rdt*(2*p.distance*p.distance+10*reverse*reverse+20*std::pow(std::max(0.,.04-d),2));
      if(d<0)cost+=c.rdt*(1600+9000*d*d);
      cost+=.02*dot(du,du);
    }
    return cost-8*(p.arc-start);
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    if(sc.path.size()<2||!z.fresh)return {};
    auto a=arcs(sc);auto p=project(s,sc,a);gear=direction(sc,p.index);guide_yaw=s.yaw;
    // Only risk ranking in the inherited grid uses this radius; geometric
    // feasibility itself is now the exact disk union above.
    envelope=0;for(const auto&b:c.body)envelope=std::max(envelope,norm(b.p)+b.r);
    bool blocked=false;for(size_t i=p.index;i<sc.path.size()&&a[i]<=p.arc+2.5;++i)
      if(poseDistance(sc.path[i].p,sc.path[i].yaw)<.03){blocked=true;break;}
    size_t join=chooseJoin(s,sc);bool active=!route.path.empty();failed=false;
    if(active){
      bool shortcut=segment({s.x,s.y},sc.path[join].p);
      clear_ticks=shortcut?clear_ticks+1:0;
      if(clear_ticks>=4){
        if(!blocked&&p.distance<.04){route.path.clear();side=0;phase=0;}
        else{
          auto direct=connector(s,sc,join);
          if(remaining(s,direct)+.12<remaining(s,route)){route=std::move(direct);phase=2;}
        }
      }
    }
    active=!route.path.empty();
    if((active&&!usable(s))||(!active&&blocked)){
      if(!makeRoute(s,z,sc)){route.path.clear();failed=true;}
      else phase=1;
    }
    active=!route.path.empty();strategy=active?1:0;V u{};
    if(active&&!z.os.empty()){
      proposal_divisor=2;auto previous=warm;
      V direct=BaseMPPI::plan(s,z,sc);auto direct_us=warm;auto direct_debug=debug;
      double direct_cost=compareRollout(s,z,sc,direct_us);
      warm=previous;V detour=BaseMPPI::plan(s,z,route);double detour_cost=compareRollout(s,z,sc,warm);
      // Prefer the simpler original/wait policy unless detour improves the
      // common short-horizon score. Neither option bypasses safety gates.
      if(direct_cost<=detour_cost+.10){u=direct;warm=std::move(direct_us);debug=direct_debug;strategy=std::abs(u.x)<.03?2:0;}
      else u=detour;
      proposal_divisor=1;
    }else u=BaseMPPI::plan(s,z,active?route:sc);
    published_route=strategy==1?route.path:std::vector<Ref>{};
    status={double(active?phase:0),double(side),p.distance,a[join],double(clear_ticks),double(published_route.size()),double(failed),double(strategy)};
    return u;
  }
};

// Local SIL prototype. Monitor early; act only on time-indexed conflict.
// Retains V5 static detours, original footprint/margin and native safety gates.
struct V7MPPI : V5MPPI {
  std::array<double,8> avoidance_debug{};
  std::array<double,12> decision_debug{};
  int committed_side=0,clear_count=0;
  // Opt-in observation only: no RNG calls or score changes. Rollout geometry
  // is reconstructed by the getter OUTSIDE the timed planning call.
  struct SearchTrace{std::vector<V> controls;double cost,gap;bool feasible,selected;};
  bool trace_enabled=false;
  S trace_start{};
  std::vector<SearchTrace> search_trace;
  void capture(const std::vector<V>&us,double cost,double gap,bool feasible,bool selected=false){
    if(trace_enabled)search_trace.push_back({us,cost,gap,feasible,selected});
  }
  V7MPPI(const C& config,int seed):V5MPPI(config,seed){}
  void reset(){V5MPPI::reset();avoidance_debug={};decision_debug={};committed_side=clear_count=0;search_trace.clear();}
  struct Merit{double cost=0,min_gap=1e9,progress=0,deficit=0,conflict=1e9,lateral=0,end_gap=1e9,end_closing=0;bool feasible=true;int lane=0;};
  Merit evaluate(S s,const Z&z,const Scene&sc,const std::vector<double>&a,const std::vector<V>&us,bool prune=true)const{
    Merit m;auto p=project(s,sc,a);double initial=p.arc;V previous{s.v,s.w};double last_gap=1e9,closing=0;
    double previous_gap=clearance(s,predict(z,0,c),c,sc.ymax,c.margin);
    bool restore=z.os.empty() && previous_gap<0 && previous_gap+c.margin>1e-6;
    const bool started_restoration=restore;
    for(size_t k=0;k<us.size();++k){
      double old_arc=p.arc;
      for(int j=0;j<2;++j){s=step(s,us[k],c.rdt/2,c);double t=(k+.5*(j+1))*c.rdt;
        double d=clearance(s,predict(z,t,c),c,sc.ymax,c.margin);
        m.min_gap=std::min(m.min_gap,d);
        if(restore){
          if(d<previous_gap-1e-9 || d+c.margin<=1e-6)m.feasible=false;
          if(d>=0)restore=false;
        }else if(d<0)m.feasible=false;
        previous_gap=d;
        closing=last_gap<1e8?std::max(0.,(last_gap-d)/(c.rdt/2)):0.;last_gap=d;
        if(d<.04)m.conflict=std::min(m.conflict,t);
        m.cost+=.5*c.rdt*40*std::pow(std::max(0.,.18-d),2);
        if(prune&&!m.feasible)return m;
      }
      p=project(s,sc,a,p.index);double goal=norm(sc.path.back().p-V{s.x,s.y});
      double wanted=std::min(std::abs(p.speed),.9*goal);
      // Arc increases along either forward or reverse reference. Sideways
      // body motion no longer earns the same credit as useful path progress.
      double rate=(p.arc-old_arc)/c.rdt;
      double deficit=wanted>.08?std::pow(clip((wanted-rate)/wanted,0.,2.),2):0.;
      m.deficit+=c.rdt*deficit;
      V delta=V{s.x,s.y}-sc.path[p.index].p;
      double lateral=-std::sin(p.yaw)*delta.x+std::cos(p.yaw)*delta.y;
      if(std::abs(lateral)>std::abs(m.lateral))m.lateral=lateral;
      m.cost+=c.rdt*(5*p.distance*p.distance+.6*std::pow(wrap(s.yaw-p.yaw),2)+1.5*deficit);
      V du=us[k]-previous;m.cost+=.04*dot(du,du);previous=us[k];
    }
    m.progress=p.arc-initial;m.lane=std::abs(m.lateral)>.08?(m.lateral>0?1:-1):0;
    m.cost+=12*p.distance*p.distance+.5*std::pow(wrap(s.yaw-p.yaw),2)-12*m.progress;
    // Discourage ending the horizon close AND still closing. Uses only
    // in-horizon separation rate; not an extended CV prediction or proof that
    // braking is safe against a moving obstacle. Physical margin is unchanged.
    double response=c.tau+c.dt+std::abs(s.v)/c.accel;
    m.cost+=200*std::pow(std::max(0.,.03+closing*response-last_gap),2)
      +30*std::pow(std::max(0.,.18-last_gap),2);
    if(committed_side&&m.lane&&m.lane!=committed_side)m.cost+=.4;
    if(started_restoration && last_gap<0)m.feasible=false;
    m.end_gap=last_gap;m.end_closing=closing;
    return m;
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    avoidance_debug={};decision_debug={};search_trace.clear();trace_start=s;
    if(!z.fresh||sc.path.size()<2)return {};
    int horizon=z.os.empty()?30:std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));
    auto a=arcs(sc);auto p=project(s,sc,a);int sign=direction(sc,p.index);
    std::vector<V> targets;for(const auto&r:sc.path)targets.push_back(r.p);
    auto following=[&](double scale){std::vector<V> us;S roll=s;int hint=p.index;
      for(int k=0;k<horizon;++k){V u=guide(roll,sc,a,targets,hint);u=bound(scale*u,c);us.push_back(u);roll=step(roll,u,c.rdt,c);}return us;};
    auto nominal=following(1.);auto nm=evaluate(s,z,sc,a,nominal,false);
    capture(nominal,nm.cost,nm.min_gap,nm.feasible);
    // Conservative maneuver budget from angular/linear acceleration, with
    // actuator lag and one control cycle. Not a universal TTC stop threshold.
    double aa=std::min(c.angular_accel,2*c.wheel_a/c.track),ww=std::min(c.wmax,2*c.wheel_v/c.track);
    double turn=.5<=ww*ww/aa?2*std::sqrt(.5/aa):.5/ww+ww/aa;
    double budget=std::max(std::abs(s.v)/std::min(c.accel,c.wheel_a),turn+std::abs(s.w)/aa)+c.tau+c.dt;
    bool displaced=p.distance>.04||std::abs(wrap(s.yaw-p.yaw))>.15;
    bool lateral_needed=nm.conflict<=budget||displaced;
    if(nm.min_gap>.10){if(++clear_count>=4&&!displaced)committed_side=0;}else clear_count=0;
    // Explicit checked REJOIN policy. Do not let a high-progress warm start
    // keep travelling parallel to, or overshooting, a now-safe reference.
    if(displaced&&nm.feasible&&nm.min_gap>.10&&clear_count>=4){
      capture(nominal,nm.cost,nm.min_gap,nm.feasible,true);
      warm=nominal;published_route.clear();S roll=s;published_route.push_back({{s.x,s.y},s.yaw,s.v});
      for(const auto&u:nominal){roll=step(roll,u,c.rdt,c);published_route.push_back({{roll.x,roll.y},roll.yaw,roll.v});}
      avoidance_debug={nm.min_gap,1.,1.,0.,nm.min_gap,nm.progress,nm.deficit,0.};
      decision_debug={3.,nm.conflict<1e8?nm.conflict:-1.,budget,nm.min_gap,double(committed_side),0.,0.,p.distance,nm.progress/(horizon*c.rdt),nm.cost,double(clear_count),nm.lateral};
      status={2.,double(committed_side),p.distance,p.arc,double(clear_count),double(published_route.size()),0.,8.};return nominal.front();
    }
    // Preserve the established free-path speed policy. The mere existence of
    // a distant track should not cap the vehicle to the slower guide proposal.
    // Recheck the original controller's full rollout and reject lateral drift.
    if(nm.min_gap>.30&&!displaced){
      V u=V5MPPI::plan(s,z,sc);auto fm=evaluate(s,z,sc,a,warm);
      capture(warm,fm.cost,fm.min_gap,fm.feasible);
      if(fm.feasible&&fm.min_gap>.12&&std::abs(fm.lateral)<.03){
        capture(warm,fm.cost,fm.min_gap,fm.feasible,true);
        published_route.clear();avoidance_debug={nm.min_gap,1.,1.,0.,fm.min_gap,fm.progress,fm.deficit,0.};
        decision_debug={0.,nm.conflict<1e8?nm.conflict:-1.,budget,nm.min_gap,double(committed_side),0.,0.,p.distance,fm.progress/(horizon*c.rdt),fm.cost,double(clear_count),fm.lateral};
        status={0.,double(committed_side),p.distance,p.arc,double(clear_count),0.,0.,0.};return u;
      }
    }
    std::vector<std::vector<V>> candidates{nominal};
    for(double scale:{.8,.55,.3,0.})candidates.push_back(following(scale));
    bool safe_tracking=false;for(const auto&us:candidates)if(evaluate(s,z,sc,a,us).feasible){safe_tracking=true;break;}
    lateral_needed=lateral_needed||!safe_tracking;
    // Feedback guide automatically bends back toward the original reference;
    // there is no straight-parallel-offset default after conflict clears.
    if(lateral_needed){
      auto saved=warm;V5MPPI::plan(s,z,sc);
      if(z.os.empty()){
        auto detour=evaluate(s,z,sc,a,warm);
        if(detour.feasible&&detour.progress>.02&&(detour.end_gap>=.05||detour.end_closing<=.005)){
          capture(warm,detour.cost,detour.min_gap,true,true);
          avoidance_debug={nm.min_gap,1.,1.,0.,detour.min_gap,detour.progress,detour.deficit,0.};
          decision_debug={2.,nm.conflict<1e8?nm.conflict:-1.,budget,nm.min_gap,double(committed_side),0.,1.,p.distance,
            detour.progress/(warm.size()*c.rdt),detour.cost,double(clear_count),detour.lateral};
          return warm.front();
        }
      }
      candidates.push_back(warm);warm=saved;
      saved.resize(horizon);candidates.push_back(saved);
      candidates.push_back(std::vector<V>(horizon,V{-sign*.1,0}));
      for(double angle:{-1.2,-.8,-.4,.4,.8,1.2})for(double delay:{0.,.4,.8})for(double hold:{.6,1.2}){
        S roll=s;int hint=p.index;std::vector<V> us;
        for(int k=0;k<horizon;++k){double t=k*c.rdt;V u=guide(roll,sc,a,targets,hint);
          if(t>=delay&&t<delay+hold){
            auto q=project(roll,sc,a,hint);double error=wrap(q.yaw+angle-roll.yaw);
            u=bound({sign*.16*std::max(.65,std::cos(error)),clip(2*error,-c.wmax,c.wmax)},c);
          }us.push_back(u);roll=step(roll,u,c.rdt,c);
        }candidates.push_back(std::move(us));
      }
    }
    std::vector<Merit> scores;int best=-1,nfeasible=0;
    auto score_last=[&](){auto m=evaluate(s,z,sc,a,candidates[scores.size()]);capture(candidates[scores.size()],m.cost,m.min_gap,m.feasible);scores.push_back(m);
      if(m.feasible){++nfeasible;if(best<0||m.cost<scores[best].cost)best=int(scores.size())-1;}};
    while(scores.size()<candidates.size())score_last();
    // On an unobstructed reference, tracking should not be distorted merely
    // because a distant moving track exists. Sample only during intervention.
    if(lateral_needed&&best>=0){auto seed=candidates[best];
      for(int j=0;j<24;++j){auto us=seed;V noise{};for(auto&u:us){noise=.7*noise+V{.015*normal(rng),.08*normal(rng)};u=bound(u+noise,c);}
        candidates.push_back(std::move(us));score_last();}
    }
    avoidance_debug={nm.min_gap,1.,double(nfeasible),double(best),0.,0.,0.,0.};
    decision_debug={0.,nm.conflict<1e8?nm.conflict:-1.,budget,nm.min_gap,double(committed_side),double(best),double(lateral_needed),p.distance,0.,0.,double(clear_count),0.};
    if(best<0){warm.assign(horizon,V{});published_route.clear();avoidance_debug[7]=1;decision_debug[0]=4;
      status={3.,double(committed_side),p.distance,p.arc,0.,0.,1.,5.};return {};}
    std::vector<V> selected=candidates[best];auto sm=scores[best];
    if(lateral_needed){std::vector<V> mean(horizon);double total=0;
      for(size_t j=0;j<scores.size();++j)if(scores[j].feasible&&scores[j].lane==sm.lane){
        double w=std::exp(-(scores[j].cost-sm.cost)/.12);total+=w;for(int k=0;k<horizon;++k)mean[k]=mean[k]+w*candidates[j][k];}
      for(auto&u:mean)u=bound((1/total)*u,c);auto mm=evaluate(s,z,sc,a,mean);capture(mean,mm.cost,mm.min_gap,mm.feasible);
      if(mm.feasible&&mm.cost<=sm.cost+.03){selected=std::move(mean);sm=mm;}
    }
    int mode=displaced&&nm.min_gap>.04?3:sm.lane&&lateral_needed?2:best==0?0:1;
    capture(selected,sm.cost,sm.min_gap,sm.feasible,true);
    if(mode==2&&sm.lane)committed_side=sm.lane;
    warm=selected;published_route.clear();S roll=s;published_route.push_back({{s.x,s.y},s.yaw,s.v});
    for(const auto&u:selected){roll=step(roll,u,c.rdt,c);published_route.push_back({{roll.x,roll.y},roll.yaw,roll.v});}
    avoidance_debug[4]=sm.min_gap;avoidance_debug[5]=sm.progress;avoidance_debug[6]=sm.deficit;
    decision_debug[0]=mode;decision_debug[4]=committed_side;decision_debug[8]=sm.progress/(horizon*c.rdt);decision_debug[9]=sm.cost;decision_debug[11]=sm.lateral;
    status={3.,double(committed_side),p.distance,p.arc,double(clear_count),double(published_route.size()),0.,double(mode==0?0:mode==1?7:mode==3?8:6)};
    return selected.front();
  }
};

// Experiment-only candidate coverage/recovery extension; not a safety proof.
// Original V7 is retained verbatim as a proposal and free-space policy.
struct MPPI : V7MPPI {
  std::array<double,10> recovery_debug{};
  std::array<double,4> margin_debug{};
  bool recovering=false;int recovery_clear=0;
  MPPI(const C&config,int seed):V7MPPI(config,seed){}
  void reset(){V7MPPI::reset();recovery_debug={};margin_debug={};recovering=false;recovery_clear=0;}
  struct Option{std::vector<V> us;Merit merit;double backup=-1e9;int continuations=0;bool terminal_ok=true;};
  // Near the horizon end, closing motion can consume the remaining clearance
  // immediately after that horizon. Rank non-closing/open terminal states
  // first when available, without extrapolating CV beyond its valid age.
  bool terminalOpen(const Merit&m)const{return m.end_gap>=.05||m.end_closing<=.005;}
  // Test alternative actions after 0.8 s, using only the remaining original
  // CV horizon. This measures a limited escape reserve, not infinite viability.
  double continuation(const S&start,const Z&z,const Scene&sc,Option&o){
    int split=std::min(8,int(o.us.size())/2);S s=start;
    for(int k=0;k<split;++k)s=step(s,o.us[k],c.rdt,c);
    double best=-1e9;int n=0;
    for(V target:std::vector<V>{{0,0},{-.22,0},{.22,0},{-.16,-.5},{-.16,.5},{.16,-.5},{.16,.5}}){
      S roll=s;double gap=1e9;target=bound(target,c);
      for(int k=split;k<int(o.us.size());++k)for(int j=0;j<2;++j){
        roll=step(roll,target,c.rdt/2,c);
        gap=std::min(gap,clearance(roll,predict(z,(k+.5*(j+1))*c.rdt,c),c,sc.ymax,c.margin));
      }
      best=std::max(best,gap);if(gap>=0)++n;
    }
    o.backup=best;o.continuations=n;
    return 150*std::pow(std::max(0.,.05-best),2)+(n==0?1.5:0.);
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    recovery_debug={};auto previous=warm;V original=V7MPPI::plan(s,z,sc);
    double static_gap=clearance(s,{},c,sc.ymax,c.margin);
    bool restoring=z.os.empty() && static_gap<0 && static_gap+c.margin>1e-6;
    margin_debug={static_gap,double(restoring),double(restoring && avoidance_debug[7]==0),0.};
    if(!z.fresh||sc.path.size()<2){recovering=false;recovery_clear=0;return original;}
    bool failed=avoidance_debug[7]>0;
    if(!failed&&avoidance_debug[0]>.10){if(++recovery_clear>=4)recovering=false;}else recovery_clear=0;
    if(!recovering&&!failed&&avoidance_debug[0]>.04)return original;
    int horizon=z.os.empty()?30:std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));
    auto a=arcs(sc);auto p=project(s,sc,a);std::vector<V> targets;
    for(const auto&r:sc.path)targets.push_back(r.p);
    // Preserve a checked existing policy when it still has an alternative
    // continuation. Extra primitive coverage must not arbitrarily replace a
    // working maneuver merely because it offers short-horizon progress.
    if(!failed&&(!recovering||(z.os.empty()&&avoidance_debug[5]>.02))){
      Option base{warm,evaluate(s,z,sc,a,warm)};continuation(s,z,sc,base);
      recovery_debug[5]=base.backup;recovery_debug[6]=base.continuations;
      if(base.merit.feasible&&base.continuations>0&&terminalOpen(base.merit)){if(z.os.empty()){recovering=false;recovery_clear=0;}return original;}
    }
    recovering=true;
    for(auto&r:search_trace)r.selected=false;
    std::vector<Option> options;int base_feasible=0;
    auto add=[&](std::vector<V>us){
      if(us.size()!=size_t(horizon))us.resize(horizon);
      auto m=evaluate(s,z,sc,a,us);capture(us,m.cost,m.min_gap,m.feasible);
      if(m.feasible){bool open=terminalOpen(m);options.push_back({std::move(us),m,-1e9,0,open});}
    };
    if(!failed){add(warm);base_feasible=int(options.size());}
    if(!previous.empty())add(previous);
    int count=0;
    // Independent samples cover both signs of v and w. They do not require
    // any original candidate to be feasible, unlike V7's local perturbations.
    for(double v:{-.22,-.16,-.08,0.,.08,.16,.22})for(double w:{-.8,-.4,0.,.4,.8}){
      V u=bound({v,w},c);add(std::vector<V>(horizon,u));++count;
      if(std::abs(v)<.01)continue;
      for(int kind=0;kind<2;++kind){S roll=s;int hint=p.index;std::vector<V> us;
        for(int k=0;k<horizon;++k){
          V command=k<8?u:kind==0?bound({v,0},c):guide(roll,sc,a,targets,hint);
          us.push_back(command);roll=step(roll,command,c.rdt,c);
        }add(std::move(us));++count;
      }
    }
    // Heading-hold arcs keep the retreat/side-exit direction coherent across
    // receding horizons instead of alternating fixed yaw-rate primitives.
    for(double v:{-.22,-.16,.16,.22})for(double angle:{-.9,-.5,.5,.9}){
      S roll=s;std::vector<V>us;double heading=wrap(p.yaw+angle);
      for(int k=0;k<horizon;++k){V u=bound({v,clip(2*wrap(heading-roll.yaw),-c.wmax,c.wmax)},c);
        us.push_back(u);roll=step(roll,u,c.rdt,c);}
      add(std::move(us));++count;
    }
    // Broad two-phase random coverage runs even if ALL deterministic samples
    // fail; no feasible-seed precondition. Inputs remain wheel/actuator bound.
    for(int j=0;j<24;++j){
      V first=bound({.22*std::tanh(normal(rng)),.8*std::tanh(normal(rng))},c);
      V second=bound({.22*std::tanh(normal(rng)),.8*std::tanh(normal(rng))},c);
      std::vector<V>us;for(int k=0;k<horizon;++k)us.push_back(k<8?first:second);
      add(std::move(us));++count;
    }
    recovery_debug[0]=1;recovery_debug[1]=failed;recovery_debug[2]=count;
    recovery_debug[3]=options.size();recovery_debug[4]=base_feasible;
    if(options.empty()){
      warm.assign(horizon,V{});published_route.clear();avoidance_debug[7]=1;decision_debug[0]=4;
      recovery_debug[9]=1;return {};
    }
    std::sort(options.begin(),options.end(),[](const Option&a,const Option&b){
      if(a.terminal_ok!=b.terminal_ok)return a.terminal_ok;
      return a.merit.cost<b.merit.cost;
    });
    int n=std::min(8,int(options.size())),best=0;double score=1e99;
    if(options[0].terminal_ok)while(n>1&&!options[n-1].terminal_ok)--n;
    for(int i=0;i<n;++i){
      double total=options[i].merit.cost+continuation(s,z,sc,options[i]);
      if(total<score){score=total;best=i;}
    }
    auto&selected=options[best];warm=selected.us;
    margin_debug[2]=restoring; margin_debug[3]=selected.merit.end_gap;
    capture(warm,score,selected.merit.min_gap,true,true);
    published_route.clear();S roll=s;published_route.push_back({{s.x,s.y},s.yaw,s.v});
    for(const auto&u:warm){roll=step(roll,u,c.rdt,c);published_route.push_back({{roll.x,roll.y},roll.yaw,roll.v});}
    avoidance_debug[2]=options.size();avoidance_debug[4]=selected.merit.min_gap;
    avoidance_debug[5]=selected.merit.progress;avoidance_debug[6]=selected.merit.deficit;avoidance_debug[7]=0;
    int mode=std::abs(warm.front().x)<.02?1:selected.merit.lane?2:1;
    decision_debug[0]=mode;decision_debug[5]=best;decision_debug[6]=1;
    decision_debug[8]=selected.merit.progress/(horizon*c.rdt);decision_debug[9]=score;decision_debug[11]=selected.merit.lateral;
    if(selected.merit.lane)committed_side=selected.merit.lane;
    recovery_debug[5]=selected.backup;recovery_debug[6]=selected.continuations;
    recovery_debug[7]=selected.merit.cost;recovery_debug[8]=score;
    return warm.front();
  }
};

} // namespace experiment
