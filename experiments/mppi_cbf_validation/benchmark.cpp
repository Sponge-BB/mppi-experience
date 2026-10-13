// Standalone screening experiment. No ROS, sockets, drivers or robot commands.
// Linearized discrete CBF constraints are numerically verified, NOT a safety proof.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

constexpr double PI = 3.14159265358979323846;
double clip(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
struct Vec { double x=0, y=0; };
Vec operator+(Vec a, Vec b) { return {a.x+b.x,a.y+b.y}; }
Vec operator-(Vec a, Vec b) { return {a.x-b.x,a.y-b.y}; }
Vec operator*(double k, Vec a) { return {k*a.x,k*a.y}; }
double dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y; }
double norm(Vec a) { return std::sqrt(dot(a,a)); }
struct Disk { Vec p; double r; };
struct State { double x=0,y=0,yaw=0,v=0,w=0; };
struct Obs { Vec p,vel; double r=.23; };
struct Config {
  double dt=.05, rollout_dt=.10, vmax=.30, wmax=.80, accel=.35, angular_accel=1.;
  double plant_tau=.12, margin=.08, max_age=.35, alpha=1.2;
  int samples=256, horizon=18;
  std::vector<Disk> body;
};
struct Scenario {
  std::string name;
  double delay=0, vel_error=0, dropout_begin=-1, dropout_end=-1;
  double pos_unc=.02, vel_unc=.03, acc_unc=.02, duration=24., ymax=1.5;
  bool workspace_uncertainty=false;
  Vec goal{3.,0};
};
Vec worldPoint(const State&s,Vec p) {
  const double c=std::cos(s.yaw), sn=std::sin(s.yaw);
  return {s.x+c*p.x-sn*p.y,s.y+sn*p.x+c*p.y};
}
// Velocity-target actuator with bounded acceleration and a first-order lag.
// Both rollout and closed-loop plant use 10 ms substeps; truth collision uses 5 ms.
State step(State s, Vec command, double dt, const Config& c, double sub=.01) {
  command={clip(command.x,-c.vmax,c.vmax),clip(command.y,-c.wmax,c.wmax)};
  const int n=std::max(1,int(std::ceil(dt/sub-1e-9))); const double h=dt/n;
  for(int i=0;i<n;++i) {
    const double av=clip((command.x-s.v)/c.plant_tau,-c.accel,c.accel);
    const double aw=clip((command.y-s.w)/c.plant_tau,-c.angular_accel,c.angular_accel);
    const double vnew=s.v+av*h, wnew=s.w+aw*h;
    const double vm=.5*(s.v+vnew), wm=.5*(s.w+wnew), mid=s.yaw+.5*wm*h;
    s.x+=h*vm*std::cos(mid); s.y+=h*vm*std::sin(mid);
    s.yaw=angle(s.yaw+h*wm); s.v=vnew; s.w=wnew;
  }
  return s;
}
std::vector<Obs> truth(const Scenario& sc,double t) {
  if(sc.name=="empty") return {};
  if(sc.name=="near_miss") return {{{1.55,1.12},{0,0},.20}};
  if(sc.name=="blocked") {
    std::vector<Obs> out;
    for(int i=-2;i<=2;++i) out.push_back({{1.65,.5*i},{0,0},.27});
    return out;
  }
  if(sc.name=="new_static") return t<1.5 ? std::vector<Obs>{} : std::vector<Obs>{{{1.65,0},{0,0},.25}};
  if(sc.name=="late_intrusion") return t<3.5 ? std::vector<Obs>{} : std::vector<Obs>{{{1.75,0},{0,0},.25}};
  // Same exogenous obstacle motion for every controller and random seed.
  return {{{1.55,-1.85+.34*t},{0,.34},.23}};
}
struct Snapshot {
  std::vector<Obs> obstacles;
  double stamp=0, age=0, pos_unc=0, vel_unc=0, acc_unc=0;
  bool fresh=true;
};
Snapshot observe(const Scenario& sc, double t, const Config& c) {
  // 10 Hz sensor, then deterministic transport delay; no future observations.
  double stamp=std::floor((t-sc.delay+1e-9)/.1)*.1;
  if(t>=sc.dropout_begin && t<sc.dropout_end)
    stamp=std::floor((sc.dropout_begin-sc.delay-1e-6)/.1)*.1;
  Snapshot z; z.stamp=stamp; z.age=t-stamp; z.fresh=z.age<=c.max_age;
  z.obstacles=truth(sc,stamp);
  z.pos_unc=sc.pos_unc; z.vel_unc=sc.vel_unc; z.acc_unc=sc.acc_unc;
  for(auto &o:z.obstacles) o.vel.x+=sc.vel_error;
  return z;
}
double uncertainty(const Snapshot& z, double ahead) {
  const double a=std::max(0.,z.age+ahead);
  return z.pos_unc+z.vel_unc*a+.5*z.acc_unc*a*a;
}
std::vector<Obs> predict(const Snapshot&z,double ahead) {
  auto os=z.obstacles;
  for(auto&o:os) { o.p=o.p+(z.age+ahead)*o.vel; o.r+=uncertainty(z,ahead); }
  return os;
}
double clearance(const State&s,const std::vector<Obs>&os,const Config&c,double ymax,double margin=0) {
  double result=1e6;
  for(const auto&b:c.body) {
    const auto p=worldPoint(s,b.p);
    result=std::min(result,ymax-std::abs(p.y)-b.r-margin);
    for(const auto&o:os) result=std::min(result,norm(p-o.p)-b.r-o.r-margin);
  }
  return result;
}
// h uses smooth squared circle distance plus two linear wall distances.
std::vector<double> barriers(const State&s,const Snapshot&z,double ahead,const Config&c,double ymax) {
  const auto os=predict(z,ahead); std::vector<double> h;
  for(const auto&b:c.body) {
    const auto p=worldPoint(s,b.p);
    for(const auto&o:os) {
      const double radius=b.r+o.r+c.margin;
      h.push_back(dot(p-o.p,p-o.p)-radius*radius);
    }
    h.push_back(ymax-p.y-b.r-c.margin);
    h.push_back(ymax+p.y-b.r-c.margin);
  }
  return h;
}
// Exact active-set enumeration for a strictly convex two-variable QP:
// min (u-u0)' diag(1, .20) (u-u0), subject to A*u >= b.
struct Constraint { Vec a; double b; };
struct QP { Vec u; bool feasible=false; };
QP solveQP(Vec u0,const std::vector<Constraint>& cs) {
  QP best; double value=1e100;
  auto consider=[&](Vec u) {
    if(!std::isfinite(u.x)||!std::isfinite(u.y)) return;
    for(const auto&c:cs) if(dot(c.a,u)<c.b-1e-8) return;
    const Vec d=u-u0; const double f=d.x*d.x+.20*d.y*d.y;
    if(f<value) {value=f;best={u,true};}
  };
  consider(u0);
  for(size_t i=0;i<cs.size();++i) {
    auto a=cs[i].a; const double den=a.x*a.x+5*a.y*a.y;
    if(den>1e-18) {
      const double k=(cs[i].b-dot(a,u0))/den;
      consider(u0+Vec{k*a.x,5*k*a.y});
    }
    for(size_t j=0;j<i;++j) {
      const auto b=cs[j].a; const double det=a.x*b.y-a.y*b.x;
      if(std::abs(det)>1e-12)
        consider({(cs[i].b*b.y-a.y*cs[j].b)/det,(a.x*cs[j].b-cs[i].b*b.x)/det});
    }
  }
  return best;
}
struct FilterResult { Vec u; bool infeasible=false, intervention=false, verified=true; };
FilterResult filter(const State&s,Vec nominal,const Snapshot&z,const Config&c,double ymax) {
  if(!z.fresh) return {{0,0},false,true,false};
  const auto h0=barriers(s,z,0,c,ymax);
  auto residual=[&](Vec u) {
    const auto next=step(s,u,c.dt,c);
    auto h=barriers(next,z,c.dt,c,ymax);
    const double decay=std::exp(-c.alpha*c.dt);
    for(size_t i=0;i<h.size();++i) h[i]-=decay*h0[i];
    return h;
  };
  auto verified=[&](Vec u) {
    const auto res=residual(u);
    for(double r:res) if(!std::isfinite(r)||r< -1e-7) return false;
    // Extra sampled collision verification, including between control endpoints.
    State p=s;
    for(int k=1;k<=10;++k) {
      p=step(p,u,c.dt/10,c,c.dt/10);
      if(clearance(p,predict(z,k*c.dt/10),c,ymax,c.margin)< -1e-7) return false;
    }
    return true;
  };
  Vec u=nominal;
  if(verified(u)) return {u,false,false,true};
  constexpr double eps=1e-4;
  for(int iter=0;iter<4;++iter) {
    const auto r=residual(u), rx=residual(u+Vec{eps,0}), ry=residual(u+Vec{0,eps});
    std::vector<Constraint> cs={{{1,0},-c.vmax},{{-1,0},-c.vmax},{{0,1},-c.wmax},{{0,-1},-c.wmax}};
    for(size_t i=0;i<r.size();++i) {
      Vec a{(rx[i]-r[i])/eps,(ry[i]-r[i])/eps};
      cs.push_back({a,dot(a,u)-r[i]});
    }
    const auto q=solveQP(nominal,cs);
    if(!q.feasible) break;
    u=q.u;
    if(verified(u)) return {u,false,norm(u-nominal)>1e-5,true};
  }
  // Explicit failure, not softened collision constraints or a claim of safe stop.
  return {{0,0},true,true,verified({0,0})};
}
struct MPPI {
  const Config&c; std::mt19937 rng; std::normal_distribution<double> normal{0,1};
  std::vector<Vec> warm;
  MPPI(const Config&config,int seed):c(config),rng(seed),warm(c.horizon,Vec{.25,0}){}
  double cost(State s,const std::vector<Vec>&us,const std::vector<std::vector<Obs>>&pred,const Scenario&sc) const {
    double total=0; Vec previous{s.v,s.w};
    for(int k=0;k<c.horizon;++k) {
      s=step(s,us[k],c.rollout_dt,c);
      const double d=clearance(s,pred[k],c,sc.ymax,c.margin);
      const double deficit=std::max(0.,.25-d);
      const double dx=sc.goal.x-s.x;
      total+=c.rollout_dt*(.8*s.y*s.y+.15*s.yaw*s.yaw+28*deficit*deficit+
              (d<0 ? 1200+8000*d*d : 0)+.3*std::pow(s.v-.28,2));
      const auto du=us[k]-previous;
      total+=.015*dot(du,du); previous=us[k];
      if(dx<-.05) total+=5*dx*dx;
    }
    total+=15*norm(Vec{s.x,s.y}-sc.goal)+.4*s.yaw*s.yaw;
    if(norm(Vec{s.x,s.y}-sc.goal)<.2) total+=8*s.v*s.v;
    return total;
  }
  Vec plan(const State&s,const Snapshot&z,const Scenario&sc) {
    if(!z.fresh) return {0,0};
    if(norm(Vec{s.x,s.y}-sc.goal)<.09) return {0,clip(-1.5*s.yaw,-c.wmax,c.wmax)};
    std::vector<std::vector<Obs>> pred;
    for(int k=0;k<c.horizon;++k) pred.push_back(predict(z,(k+1)*c.rollout_dt));
    std::vector<std::vector<Vec>> candidates(c.samples,std::vector<Vec>(c.horizon));
    std::vector<double> costs(c.samples); double best=1e100; int best_i=0;
    // Correlated Gaussian perturbations. Seeded left/right/brake proposals avoid
    // relying on one stochastic mode; this is a guided practical MPPI variant.
    for(int j=0;j<c.samples;++j) {
      Vec noise{};
      for(int k=0;k<c.horizon;++k) {
        noise=.90*noise+Vec{.11*normal(rng),.32*normal(rng)};
        Vec u=warm[k]+noise;
        if(j==0) u=warm[k];
        if(j==1) u={0,0};
        if(j==2||j==3) u={.25,(j==2?1:-1)*.65};
        if(j==4) u={-.20,0};
        candidates[j][k]={clip(u.x,-c.vmax,c.vmax),clip(u.y,-c.wmax,c.wmax)};
      }
      costs[j]=cost(s,candidates[j],pred,sc);
      if(costs[j]<best) {best=costs[j];best_i=j;}
    }
    std::vector<Vec> mean(c.horizon); double total=0;
    const double temperature=.35;
    for(int j=0;j<c.samples;++j) {
      const double weight=std::exp(-(costs[j]-best)/temperature); total+=weight;
      for(int k=0;k<c.horizon;++k) mean[k]=mean[k]+weight*candidates[j][k];
    }
    for(auto&u:mean) u=(1/total)*u;
    // The weighted mean can cross obstacles even when contributing samples do not.
    if(cost(s,mean,pred,sc)>best+.5) mean=candidates[best_i];
    warm=mean;
    return mean.front();
  }
  void feedback(Vec executed) {
    // Re-anchor to filtered/held action, not an unexecuted nominal command.
    const double fraction=c.dt/c.rollout_dt;
    for(int k=0;k<c.horizon-1;++k) warm[k]=(1-fraction)*warm[k]+fraction*warm[k+1];
    warm.front()=.5*warm.front()+.5*executed;
  }
};
void selfTest() {
  auto require=[](bool ok,const char*msg){if(!ok) throw std::runtime_error(msg);};
  auto q=solveQP({2,0},{{{1,0},-1},{{-1,0},-1},{{0,1},-1},{{0,-1},-1}});
  require(q.feasible && std::abs(q.u.x-1)<1e-8,"QP projection");
  require(!solveQP({0,0},{{{1,0},1},{{-1,0},0}}).feasible,"QP infeasible");
  q=solveQP({0,0},{{{1,1},1}});
  require(q.feasible&&std::abs(q.u.x-1./6)<1e-8&&std::abs(q.u.y-5./6)<1e-8,"QP weighted optimum");
  Config c; c.body={{{0,0},.36}};
  auto s=step({}, {.3,.8},.05,c);
  require(s.v<=c.accel*.05+1e-9 && s.w<=c.angular_accel*.05+1e-9,"actuator acceleration");
  auto back=step({}, {-.3,0},.5,c); require(back.x<0,"reverse motion");
  Snapshot z; z.age=.2;z.obstacles={{{1,0},{1,0},.2}};
  require(std::abs(predict(z,.3)[0].p.x-1.5)<1e-8,"CV includes observation age");
  z.fresh=false; require(filter({}, {.3,0},z,c,2).intervention,"stale fallback");
  z=Snapshot{}; auto f=filter({}, {.2,0},z,c,2);
  require(f.verified&&!f.intervention,"free-space filter passthrough");
  z.obstacles={{{0,0},{0,0},.3}}; f=filter({}, {.2,0},z,c,2);
  require(f.infeasible&&!f.verified,"unsafe initial state not certified");
  Scenario sc; sc.name="delayed_crossing";sc.delay=.15;sc.dropout_begin=4;sc.dropout_end=4.45;
  auto delayed=observe(sc,4.4,c); require(!delayed.fresh,"dropout ages past maximum");
  sc.name="new_static";require(truth(sc,1.49).empty()&&!truth(sc,1.5).empty(),"obstacle appearance");
  std::mt19937 rng(42);std::uniform_real_distribution<double> uni(-1,1);
  for(int trial=0;trial<80;++trial) {
    Vec desired{2*uni(rng),2*uni(rng)};
    std::vector<Constraint> cs={{{1,0},-1},{{-1,0},-1},{{0,1},-1},{{0,-1},-1}};
    for(int k=0;k<5;++k)cs.push_back({{uni(rng),uni(rng)},-.5});
    auto solution=solveQP(desired,cs);require(solution.feasible,"random QP feasible origin");
    const Vec d=solution.u-desired;double optimal=dot(Vec{d.x,.2*d.y},d);
    for(int ix=-10;ix<=10;++ix)for(int iy=-10;iy<=10;++iy) {
      Vec point{ix*.1,iy*.1};bool feasible=true;
      for(const auto&con:cs)feasible=feasible&&(dot(con.a,point)>=con.b-1e-9);
      Vec e=point-desired;
      if(feasible)require(optimal<=e.x*e.x+.2*e.y*e.y+1e-8,"QP dominates feasible grid");
    }
  }
  std::cout<<"PASS: 10 numerical/model cases + 80 randomized QP grid checks\n";
}
int main(int argc,char**argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {selfTest();return 0;}
    if(argc<4) throw std::runtime_error("usage: benchmark OUT_DIR GEOMETRY_CSV SEEDS [SAMPLES] [SCENARIOS comma-separated]");
    const std::filesystem::path out=argv[1]; std::filesystem::create_directories(out);
    Config c; const int seeds=std::stoi(argv[3]); if(argc>4)c.samples=std::stoi(argv[4]);
    if(seeds<1||c.samples<8) throw std::runtime_error("invalid seed/sample count");
    std::ifstream geometry(argv[2]); std::string line;
    while(std::getline(geometry,line)) {std::replace(line.begin(),line.end(),',',' ');std::istringstream ss(line);Disk b;if(ss>>b.p.x>>b.p.y>>b.r)c.body.push_back(b);}
    if(c.body.empty()) throw std::runtime_error("missing body geometry");
    std::vector<Scenario> scenarios;
    for(const auto&name:{"empty","near_miss","new_static","crossing","delayed_crossing","blocked","workspace_uncertainty","late_intrusion"}) {
      Scenario sc;sc.name=name;
      if(sc.name=="delayed_crossing") {sc.delay=.15;sc.vel_error=.05;sc.dropout_begin=4.;sc.dropout_end=4.45;}
      if(sc.name=="workspace_uncertainty") {sc.delay=.10;sc.pos_unc=.08;sc.vel_unc=.25;sc.acc_unc=.75;sc.workspace_uncertainty=true;}
      if(argc>5 && (","+std::string(argv[5])+",").find(","+sc.name+",")==std::string::npos) continue;
      scenarios.push_back(sc);
    }
    if(scenarios.empty())throw std::runtime_error("no matching scenarios");
    std::ofstream objects(out/"obstacles.csv");
    objects<<"scenario,t,id,x,y,r\n";
    for(const auto&sc:scenarios)for(int k=0;k<=int(sc.duration/c.dt);++k) {
      const auto os=truth(sc,k*c.dt);
      for(size_t id=0;id<os.size();++id)
        objects<<sc.name<<','<<k*c.dt<<','<<id<<','<<os[id].p.x<<','<<os[id].p.y<<','<<os[id].r<<'\n';
    }
    objects.close();
    std::ofstream summary(out/"runs.csv");
    summary<<"scenario,mode,seed,reached,collision,elapsed,stop_seconds,min_clearance,qp_infeasible,unverified,interventions,stale_cycles,deadline_misses,cycles\n";
    for(const auto&sc:scenarios)for(const auto&mode:{"mppi","mppi_cbf"})for(int seed=0;seed<seeds;++seed) {
      const std::string run=sc.name+"__"+mode+"__"+std::to_string(seed);
      std::ofstream f(out/(run+".csv"));
      f<<"t,x,y,yaw,v,w,nominal_v,nominal_w,command_v,command_w,clearance,total_ms,mppi_ms,cbf_ms,intervention,infeasible,verified,stale,deadline_miss\n";
      f<<std::setprecision(10);
      State state; MPPI planner(c,seed); double min_clear=1e6,stopped=0,t=0;
      int failures=0,unverified=0,interventions=0,stale=0,misses=0,cycles=0;
      bool hit=false,reached=false;
      for(;t<sc.duration-1e-9;t+=c.dt) {
        const auto start=std::chrono::steady_clock::now();
        const auto z=observe(sc,t,c);
        const Vec nominal=planner.plan(state,z,sc);
        const auto mid=std::chrono::steady_clock::now();
        FilterResult r{nominal,false,false,false};
        if(std::string(mode)=="mppi_cbf")r=filter(state,nominal,z,c,sc.ymax);
        if(!z.fresh) r={{0,0},false,true,false};
        const auto end=std::chrono::steady_clock::now();
        const double mp_ms=std::chrono::duration<double,std::milli>(mid-start).count();
        const double cbf_ms=std::chrono::duration<double,std::milli>(end-mid).count();
        const bool deadline=mp_ms+cbf_ms>1000*c.dt;
        // No hidden slowed-down simulator: late results are dropped, braking is
        // requested for this cycle. This models deadline rejection, not CPU queues.
        if(deadline)r={{0,0},false,true,false};
        const Vec applied=r.u; planner.feedback(applied);
        double cycle_clear=clearance(state,truth(sc,t),c,sc.ymax);
        for(int sub=1;sub<=10;++sub) {
          state=step(state,applied,c.dt/10,c,c.dt/10);
          cycle_clear=std::min(cycle_clear,clearance(state,truth(sc,t+sub*c.dt/10),c,sc.ymax));
        }
        min_clear=std::min(min_clear,cycle_clear);hit=hit||cycle_clear<0;
        if(t>1&&std::abs(state.v)<.025&&norm(Vec{state.x,state.y}-sc.goal)>.15) stopped+=c.dt;
        failures+=r.infeasible;unverified+=std::string(mode)=="mppi_cbf"&&!r.verified;
        interventions+=r.intervention;stale+=!z.fresh;misses+=deadline;++cycles;
        f<<t+c.dt<<','<<state.x<<','<<state.y<<','<<state.yaw<<','<<state.v<<','<<state.w<<','
         <<nominal.x<<','<<nominal.y<<','<<applied.x<<','<<applied.y<<','<<cycle_clear<<','
         <<mp_ms+cbf_ms<<','<<mp_ms<<','<<cbf_ms<<','<<r.intervention<<','<<r.infeasible<<','<<r.verified<<','<<!z.fresh<<','<<deadline<<'\n';
        if(hit) break;
        if(norm(Vec{state.x,state.y}-sc.goal)<.10&&std::abs(state.v)<.04) {reached=true;break;}
      }
      summary<<sc.name<<','<<mode<<','<<seed<<','<<reached<<','<<hit<<','<<cycles*c.dt<<','<<stopped<<','<<min_clear<<','<<failures<<','<<unverified<<','<<interventions<<','<<stale<<','<<misses<<','<<cycles<<'\n';
      summary.flush();
      std::cout<<run<<" reached="<<reached<<" collision="<<hit<<" min_clearance="<<min_clear<<" qp_fail="<<failures<<" deadline_misses="<<misses<<std::endl;
    }
  } catch(const std::exception&e) {std::cerr<<e.what()<<'\n';return 1;}
}
