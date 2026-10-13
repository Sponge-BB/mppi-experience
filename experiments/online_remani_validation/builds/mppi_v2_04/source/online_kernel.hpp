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
struct MPPI {
  const C& c; std::mt19937 rng; std::normal_distribution<double> normal{0,1};
  std::vector<V> warm; std::array<double,8> debug{};
  MPPI(const C& cc,int seed):c(cc),rng(seed),warm(c.horizon){}
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
  V guide(const S& s,const Scene& sc,const std::vector<double>& arc,int& hint)const {
    auto p=project(s,sc,arc,hint);hint=p.index;int sign=direction(sc,p.index);
    double remaining=arc.back()-p.arc,goal=norm(sc.path.back().p-V{s.x,s.y});
    // Terminal proposal is evaluated by the same optimizer, never overrides it.
    if(goal<.025)return bound({0,1.8*wrap(sc.path.back().yaw-s.yaw)},c);
    double heading=wrap(p.yaw-s.yaw);if(std::abs(heading)>.8)return bound({0,1.8*heading},c);
    int target=p.index;double lookahead=.20+.35*std::abs(s.v);
    while(target+1<int(sc.path.size())&&arc[target]<p.arc+lookahead){
      if(std::abs(sc.path[target].v)>.005&&direction(sc,target)!=sign)break;
      ++target;
    }
    V delta=sc.path[target].p-V{s.x,s.y};double travel_yaw=s.yaw+(sign<0?pi:0);
    double lateral=-std::sin(travel_yaw)*delta.x+std::cos(travel_yaw)*delta.y;
    double speed=std::max(std::abs(p.speed),std::abs(sc.path[target].v));
    speed=std::min({speed,.85*c.wheel_v,.9*std::max(goal,remaining)});
    return bound({sign*speed,2*speed*lateral/std::max(.015,dot(delta,delta))},c);
  }
  struct Score {double total=0,path=0,risk=0,collision=0,direction=0;};
  Score cost(S s,const std::vector<V>& us,const std::vector<std::vector<O>>& nominal,
             const std::vector<std::vector<O>>& uncertain,const Scene& sc,const std::vector<double>& arc,int initial_hint)const {
    Score value;V prev{s.v,s.w};auto p=project(s,sc,arc,initial_hint);double initial=p.arc;
    for(size_t k=0;k<us.size();++k){s=step(s,us[k],c.rdt,c);p=project(s,sc,arc,p.index);
      double goal=norm(sc.path.back().p-V{s.x,s.y});double d=clearance(s,nominal[k],c,sc.ymax,c.margin);
      double risk=clearance(s,uncertain[k],c,sc.ymax,c.margin);
      double speed=std::copysign(std::min(std::abs(p.speed),.9*goal),p.speed);
      double yaw_error=wrap((goal<.05?sc.path.back().yaw:p.yaw)-s.yaw);
      double path_cost=100*p.distance*p.distance+2*yaw_error*yaw_error+8*std::pow(s.v-speed,2);
      double collision=d<0?1600+9000*d*d:0,near=std::max(0.,.04-d);
      double risk_cost=2*std::pow(clip(-risk,0.,.5),2)+15*near*near;
      double wrong=std::max(0.,-direction(sc,p.index)*s.v);
      double direction_cost=150*wrong*wrong+100*std::pow(std::max(0.,initial-p.arc),2);
      value.path+=c.rdt*path_cost;value.risk+=c.rdt*risk_cost;value.collision+=c.rdt*collision;value.direction+=c.rdt*direction_cost;
      V du=us[k]-prev;value.total+=.02*dot(du,du);prev=us[k];
    }
    // Along-path progress avoids a straight-line attraction across curve chords.
    value.total+=value.path+value.risk+value.collision+value.direction+30*p.distance*p.distance-4*(p.arc-initial);
    double goal=norm(sc.path.back().p-V{s.x,s.y});
    if(arc.back()-p.arc<.15){value.total+=30*goal*goal;if(goal<.05)value.total+=2*std::pow(wrap(s.yaw-sc.path.back().yaw),2)+4*s.v*s.v;}
    return value;
  }
  V plan(const S& s,const Z& z,const Scene& sc){
    if(!z.fresh||sc.path.size()<2)return {};
    // Static bends may need a rotate-then-advance maneuver longer than 1.5 s.
    // Keep dynamic prediction within its existing validity contract. Trade
    // samples for horizon, preserving approximately the rollout-step budget.
    const int horizon=z.os.empty()?30:c.horizon;
    const int samples=std::max(32,c.samples*c.horizon/horizon);
    warm.resize(horizon);
    std::vector<double> arc(sc.path.size(),0);for(size_t i=1;i<arc.size();++i)arc[i]=arc[i-1]+norm(sc.path[i].p-sc.path[i-1].p);
    std::vector<std::vector<O>> nominal,uncertain;double age=std::max(0.,z.t-z.stamp);
    double observed_unc=plan_env::DynamicPointBuffer::predictionUncertainty(age,c.pos_unc,c.vel_unc,c.acc_unc);
    for(int k=0;k<horizon;++k){double ahead=(k+1)*c.rdt;auto future=predict(z,ahead,c);uncertain.push_back(future);
      double future_unc=plan_env::DynamicPointBuffer::predictionUncertainty(age+ahead,c.pos_unc,c.vel_unc,c.acc_unc);
      for(size_t j=0;j<future.size();++j)if(z.os[j].moving)future[j].r-=std::max(0.,future_unc-observed_unc);
      nominal.push_back(future);
    }
    std::vector<std::vector<V>> candidates(samples,std::vector<V>(horizon));std::vector<Score> scores(samples);int best=0;
    int initial_hint=project(s,sc,arc).index;
    for(int j=0;j<samples;++j){S roll=s;V noise{};int hint=initial_hint;
      for(int k=0;k<horizon;++k){V g=guide(roll,sc,arc,hint),u=g;noise=.75*noise+V{.045*normal(rng),.18*normal(rng)};
        if(j==0)u=warm[k];else if(j==1)u={0,0};else if(j==2)u=g;else if(j==3)u=.5*g;
        else if(j==4||j==5)u={.65*g.x,g.y+(j==4?.35:-.35)};
        // Reference-relative reverse cost, not a global reverse prohibition.
        else if(j==6)u={-direction(sc,hint)*.10,0};
        else if(j==7||j==8)u={0,j==7?.45:-.45};else u=(j%2?g:.5*g+.5*warm[k])+noise;
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

} // namespace experiment
