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
    for(const auto&o:os)d=std::min(d,norm(p-o.p)-b.r-o.r-margin);
  }return d;
}
std::vector<double> barrier(const S&s,const Z&z,double ahead,const C&c,double ymax){
  auto os=predict(z,ahead,c);std::vector<double> hs;
  for(const auto&b:c.body){auto p=bodypoint(s,b.p);
    for(const auto&o:os){double r=b.r+o.r+c.margin;hs.push_back(dot(p-o.p,p-o.p)-r*r);}
    hs.push_back(ymax-p.y-b.r-c.margin);hs.push_back(ymax+p.y-b.r-c.margin);
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
struct MPPI{
  const C&c;std::mt19937 rng;std::normal_distribution<double> normal{0,1};std::vector<V>warm;
  MPPI(const C&cc,int seed):c(cc),rng(seed),warm(c.horizon){}
  double cost(S s,const std::vector<V>&us,const std::vector<std::vector<O>>&pred,const Scene&sc)const{
    double value=0;V prev{s.v,s.w};int idx=nearest(s,sc);
    for(int k=0;k<c.horizon;++k){s=step(s,us[k],c.rdt,c);
      // Monotone local nearest search, with a small backward allowance.
      int best=idx;double dist=1e100;
      for(int j=std::max(0,idx-5);j<std::min(int(sc.path.size()),idx+18);++j){double d=norm(sc.path[j].p-V{s.x,s.y});if(d<dist){dist=d;best=j;}}idx=best;
      double d=clearance(s,pred[k],c,sc.ymax,c.margin),deficit=std::max(0.,.22-d);
      double speed=sc.path[idx].v,remaining=norm(sc.path.back().p-V{s.x,s.y});
      speed=std::copysign(std::min(std::abs(speed),remaining),speed);
      value+=c.rdt*(1.0*dist*dist+.12*std::pow(wrap(s.yaw-sc.path[idx].yaw),2)+
        30*deficit*deficit+(d<0?1600+9000*d*d:0)+.5*std::pow(s.v-speed,2));
      auto du=us[k]-prev;value+=.01*dot(du,du);prev=us[k];
    }
    double remain=norm(sc.path.back().p-V{s.x,s.y});value+=16*remain;
    if(remain<.25)value+=4*std::pow(wrap(s.yaw-sc.path.back().yaw),2)+8*s.v*s.v;
    return value;
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    if(!z.fresh)return {};
    // Explicit terminal pose controller, shared by MPPI and MPPI+CBF. Short
    // sampling horizons otherwise trade a few cm lateral error against final
    // yaw and stall. First close position, then align yaw; filters still apply.
    V delta=sc.path.back().p-V{s.x,s.y};double distance=norm(delta);
    // Switch within one horizon's maximum travel distance. A fixed 0.20 m
    // switch can leave curved-path lateral errors just outside the terminal
    // controller while the short-horizon heading cost discourages correction.
    const double terminal_radius=std::max(.20,std::min(c.vmax,c.wheel_v)*c.horizon*c.rdt);
    if(distance<terminal_radius){
      if(distance<.7*c.goal_pos)
        return bound({0,clip(1.5*wrap(sc.path.back().yaw-s.yaw),-c.wmax,c.wmax)},c);
      double error=wrap(std::atan2(delta.y,delta.x)-s.yaw);double sign=1;
      if(std::abs(error)>pi/2){error=wrap(error+pi);sign=-1;}
      return bound({std::abs(error)>.35?0:sign*.65*distance*std::cos(error),1.8*error},c);
    }
    std::vector<std::vector<O>>pred;for(int k=0;k<c.horizon;++k)pred.push_back(predict(z,(k+1)*c.rdt,c));
    std::vector<std::vector<V>>cand(c.samples,std::vector<V>(c.horizon));std::vector<double>costs(c.samples);
    auto ref=trackerReference(s,sc);double sign=sc.name=="reverse"?-1:1;double cruise=.85*std::min(c.vmax,c.wheel_v);
    V guided=bound({ref.v*std::cos(wrap(ref.yaw-s.yaw)),2*wrap(ref.yaw-s.yaw)},c);
    int bi=0;double best=1e100;
    for(int j=0;j<c.samples;++j){V noise;
      for(int k=0;k<c.horizon;++k){noise=.88*noise+V{.10*normal(rng),.30*normal(rng)};V u=warm[k]+noise;
        if(j==0)u=warm[k];
        if(j==1)u={0,0};
        if(j==2)u=guided;
        if(j==3||j==4)u={sign*cruise,(j==3?1:-1)*.6};
        if(j==5)u={-sign*cruise,0};
        if(j==6||j==7)u={0,(j==6?1:-1)*.7};
        if(j==8)u={sign*cruise,0};
        cand[j][k]=bound(u,c);
      }costs[j]=cost(s,cand[j],pred,sc);if(costs[j]<best){best=costs[j];bi=j;}
    }
    std::vector<V>mean(c.horizon);double total=0;
    for(int j=0;j<c.samples;++j){double weight=std::exp(-(costs[j]-best)/.30);total+=weight;for(int k=0;k<c.horizon;++k)mean[k]=mean[k]+weight*cand[j][k];}
    for(auto&u:mean)u=bound((1/total)*u,c);
    if(cost(s,mean,pred,sc)>best+.3)mean=cand[bi];
    warm=mean;return mean[0];
  }
  void feedback(V actual){double f=c.dt/c.rdt;for(int k=0;k<c.horizon-1;++k)warm[k]=(1-f)*warm[k]+f*warm[k+1];warm[0]=.5*warm[0]+.5*actual;}
};
// Common experiment guard for ALL three methods. Not REMANI's production TTC.
// Short reaction + braking rollout, with moving obstacles and delay accounted for.
bool guard(const S&s,V u,const Z&z,const C&c,const Scene&sc){
  if(!z.fresh)return true;
  S p=s;
  for(int k=1;k<=16;++k){double t=.05*k;p=step(p,t<=.15?u:V{},.05,c);
    if(clearance(p,predict(z,t,c),c,sc.ymax,.05)<0)return true;
  }return false;
}
using Tracker=void(*)(int,double,const double*,double*);
bool has(const std::string&list,const std::string&key){return (","+list+",").find(","+key+",")!=std::string::npos;}
void run(const std::filesystem::path&out,C base,int seeds,const std::string&names,const std::string&profiles,Tracker tracker){
  std::ofstream sums(out/"runs.csv"),refs(out/"references.csv"),objects(out/"obstacles.csv"),forecasts(out/"predictions.csv");
  sums<<"scenario,profile,mode,seed,reached,collision,elapsed,stop_seconds,reverse_seconds,path_length,min_clearance,qp_failed,unverified,interventions,guard_cycles,stale_cycles,deadline_misses,tracker_faults,cycles\n";
  refs<<"scenario,index,x,y,yaw,v\n";objects<<"scenario,t,id,x,y,r\n";
  forecasts<<"scenario,profile,t,ahead,id,x,y,r\n";
  for(const auto&name:{"empty","curve","reverse","new_static","crossing","delayed_crossing","blocked","late_intrusion","close_clearance","lag_mismatch"}){
    if(!has(names,name))continue;
    Scene sc=scene(name,base);
    for(size_t i=0;i<sc.path.size();++i){auto r=sc.path[i];refs<<name<<','<<i<<','<<r.p.x<<','<<r.p.y<<','<<r.yaw<<','<<r.v<<'\n';}
    for(int k=0;k<=int(sc.duration/base.dt);++k){auto os=truth(sc,k*base.dt);for(size_t i=0;i<os.size();++i)objects<<name<<','<<k*base.dt<<','<<i<<','<<os[i].p.x<<','<<os[i].p.y<<','<<os[i].r<<'\n';}
    for(const auto&profile:{"workspace","sensitivity"}){
      if(!has(profiles,profile))continue;
      if(std::string(profile)=="sensitivity"&&std::string(name)!="crossing"&&std::string(name)!="delayed_crossing"&&std::string(name)!="lag_mismatch")continue;
      C c=base;if(std::string(profile)=="sensitivity"){c.pos_unc=.02;c.vel_unc=.03;c.acc_unc=.02;}
      for(int tick=0;tick<=int(sc.duration/.5);++tick){double t=tick*.5;auto z=observe(sc,t,c);if(!z.fresh)continue;
        for(double ahead:{0.,.5,1.,1.5}){auto os=predict(z,ahead,c);for(size_t i=0;i<os.size();++i)
          forecasts<<name<<','<<profile<<','<<t<<','<<ahead<<','<<i<<','<<os[i].p.x<<','<<os[i].p.y<<','<<os[i].r<<'\n';}}
      for(const auto&mode:{"tracker","mppi","mppi_cbf"})for(int seed=0;seed<seeds;++seed){
        std::string id=sc.name+"__"+profile+"__"+mode+"__"+std::to_string(seed);
        std::ofstream f(out/(id+".csv"));f<<std::setprecision(12);
        f<<"t,x,y,yaw,v,w,nominal_v,nominal_w,filtered_v,filtered_w,command_v,command_w,clearance,obs_age,total_ms,controller_ms,cbf_ms,guard_ms,intervention,qp_failed,unverified,guard,stale,deadline_miss,tracker_fault\n";
        S s;MPPI planner(c,seed);double min_clear=1e6,stopped=0,reverse=0,pathlen=0;
        int qpf=0,unverified=0,interventions=0,guards=0,stales=0,misses=0,faults=0,cycles=0;bool hit=false,reached=false;
        double dummy[10]={},answer[3]={};if(std::string(mode)=="tracker")tracker(1,0,dummy,answer);
        for(int tick=0;tick<int(sc.duration/c.dt);++tick){double t=tick*c.dt;
          auto start=std::chrono::steady_clock::now();Z z=observe(sc,t,c);V nominal;int fault=0;
          if(std::string(mode)=="tracker"){
            auto r=trackerReference(s,sc);double input[]={s.x,s.y,s.yaw,s.v,s.w,r.p.x,r.p.y,r.yaw,r.v,0};
            tracker(0,t,input,answer);
            if(!std::isfinite(answer[0])||!std::isfinite(answer[1])||answer[2]<0)throw std::runtime_error("original Tracker callback failed");
            nominal=bound({answer[0],answer[1]},c);fault=int(answer[2]);
          }else nominal=planner.plan(s,z,sc);
          auto mid=std::chrono::steady_clock::now();F fr{nominal};
          if(std::string(mode)=="mppi_cbf"&&z.fresh)fr=filter(s,nominal,z,c,sc.ymax);
          auto filtered=std::chrono::steady_clock::now();V applied=fr.u;
          bool stopped_by_guard=guard(s,applied,z,c,sc);if(stopped_by_guard)applied={0,0};
          auto end=std::chrono::steady_clock::now();
          double m1=std::chrono::duration<double,std::milli>(mid-start).count();
          double m2=std::chrono::duration<double,std::milli>(filtered-mid).count(),m3=std::chrono::duration<double,std::milli>(end-filtered).count();
          bool deadline=m1+m2+m3>c.dt*1000;if(deadline)applied={0,0};
          planner.feedback(applied);double d=clearance(s,truth(sc,t),c,sc.ymax);V old{s.x,s.y};
          for(int sub=1;sub<=10;++sub){s=step(s,applied,c.dt/10,c,sc.plant_tau,c.dt/10);d=std::min(d,clearance(s,truth(sc,t+sub*c.dt/10),c,sc.ymax));}
          min_clear=std::min(min_clear,d);hit=d<0;pathlen+=norm(V{s.x,s.y}-old);
          if(t>=1&&std::abs(s.v)<.025&&norm(sc.path.back().p-V{s.x,s.y})>.15)stopped+=c.dt;
          if(s.v<-.025)reverse+=c.dt;
          bool uv=std::string(mode)=="mppi_cbf"&&(fr.unverified||stopped_by_guard||deadline||!z.fresh);
          qpf+=fr.qp_failed;unverified+=uv;interventions+=fr.intervention;guards+=stopped_by_guard;stales+=!z.fresh;misses+=deadline;faults+=fault;++cycles;
          f<<t+c.dt<<','<<s.x<<','<<s.y<<','<<s.yaw<<','<<s.v<<','<<s.w<<','<<nominal.x<<','<<nominal.y<<','<<fr.u.x<<','<<fr.u.y<<','<<applied.x<<','<<applied.y<<','<<d<<','<<t-z.stamp<<','<<m1+m2+m3<<','<<m1<<','<<m2<<','<<m3<<','<<fr.intervention<<','<<fr.qp_failed<<','<<uv<<','<<stopped_by_guard<<','<<!z.fresh<<','<<deadline<<','<<fault<<'\n';
          if(hit)break;
          if(norm(sc.path.back().p-V{s.x,s.y})<c.goal_pos&&std::abs(wrap(s.yaw-sc.path.back().yaw))<c.goal_yaw&&std::abs(s.v)<c.goal_speed&&std::abs(s.w)<.05){reached=true;break;}
        }
        sums<<sc.name<<','<<profile<<','<<mode<<','<<seed<<','<<reached<<','<<hit<<','<<cycles*c.dt<<','<<stopped<<','<<reverse<<','<<pathlen<<','<<min_clear<<','<<qpf<<','<<unverified<<','<<interventions<<','<<guards<<','<<stales<<','<<misses<<','<<faults<<','<<cycles<<'\n';sums.flush();
        std::cout<<id<<" reached="<<reached<<" collision="<<hit<<" min_clear="<<min_clear<<" qp_failed="<<qpf<<" guard="<<guards<<" deadlines="<<misses<<std::endl;
      }
    }
  }
}
} // namespace experiment
extern "C" int run_comparison(const char*out,const double*p,const char*geometry,int seeds,int samples,const char*scenes,const char*profiles,experiment::Tracker tracker){
  try{using namespace experiment;
    C c{p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],p[9],p[10],p[11],p[12],p[13],p[14],p[15],.05,.10,.12,1.2,192,15,{}};c.samples=samples;
    if(seeds<1||samples<9||c.dt/c.rdt>1||c.horizon*c.rdt+c.max_age>c.max_prediction)throw std::runtime_error("invalid run configuration");
    std::ifstream in(geometry);std::string line;while(std::getline(in,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream ss(line);Disk d;if(ss>>d.p.x>>d.p.y>>d.r)c.body.push_back(d);}
    if(c.body.empty())throw std::runtime_error("empty collision model");
    run(out,c,seeds,scenes,profiles,tracker);return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
extern "C" int self_test(){
  try{using namespace experiment;
    auto require=[](bool b){if(!b)throw std::runtime_error("numerical self-test failed");};
    require(!qp({0,0},{{{1,0},1},{{-1,0},0}}).ok);
    auto q=qp({0,0},{{{1,1},1}});require(q.ok&&std::abs(q.u.x-1./6)<1e-8&&std::abs(q.u.y-5./6)<1e-8);
    C c{.3,.8,.35,1,.37,.2296,.2975,.08,.35,2,.08,.25,.75,.05,.1,.05,.05,.10,.12,1.2,192,15,{}};c.body={{{0,0},.36}};
    std::mt19937 rng(42);std::uniform_real_distribution<double> uni(-1,1);S s;
    for(int i=0;i<300;++i){V u=bound({uni(rng),2*uni(rng)},c);auto ww=wheels(u,c);require(std::max(std::abs(ww.x),std::abs(ww.y))<=c.wheel_v+1e-9);
      S n=step(s,u,.05,c);auto a=wheels({(n.v-s.v)/.05,(n.w-s.w)/.05},c);require(std::max(std::abs(a.x),std::abs(a.y))<=c.wheel_a+1e-8);s=n;
      auto cs=bounds(c);for(int k=0;k<4;++k)cs.push_back({{uni(rng),uni(rng)},-.3});auto r=qp(u,cs);require(r.ok);
      for(auto con:cs)require(dot(con.a,r.u)>=con.b-1e-7);
    }
    Z z;z.t=.2;z.os={{{1,0},{.5,0},.2,true}};auto os=predict(z,.3,c);require(std::abs(os[0].p.x-1.25)<1e-8);
    require(std::abs(os[0].r-(.2+.08+.25*.5+.5*.75*.25))<1e-8);
    auto free=filter({}, {.1,0},Z{},c,2);require(!free.intervention);
    z=Z{};z.os={{{0,0},{0,0},.3,false}};auto bad=filter({}, {.1,0},z,c,2);require(bad.qp_failed&&bad.unverified);
    require(step({}, {-.2,0},.5,c).x<0);
    std::cout<<"PASS: QP, 300 coupled wheel/acceleration samples, production CV/uncertainty, free/unsafe barrier, reverse\n";return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
