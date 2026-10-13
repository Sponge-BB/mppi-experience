// Local SIL prototype. Monitor early; act only on time-indexed conflict.
// Retains V5 static detours, original footprint/margin and native safety gates.
struct MPPI : V5MPPI {
  std::array<double,8> avoidance_debug{};
  std::array<double,12> decision_debug{};
  int committed_side=0,clear_count=0;
  MPPI(const C& config,int seed):V5MPPI(config,seed){}
  void reset(){V5MPPI::reset();avoidance_debug={};decision_debug={};committed_side=clear_count=0;}
  struct Merit{double cost=0,min_gap=1e9,progress=0,deficit=0,conflict=1e9,lateral=0;bool feasible=true;int lane=0;};
  Merit evaluate(S s,const Z&z,const Scene&sc,const std::vector<double>&a,const std::vector<V>&us)const{
    Merit m;auto p=project(s,sc,a);double initial=p.arc;V previous{s.v,s.w};double last_gap=1e9,closing=0;
    for(size_t k=0;k<us.size();++k){
      double old_arc=p.arc;
      for(int j=0;j<2;++j){s=step(s,us[k],c.rdt/2,c);double t=(k+.5*(j+1))*c.rdt;
        double d=clearance(s,predict(z,t,c),c,sc.ymax,c.margin);
        m.min_gap=std::min(m.min_gap,d);if(d<0)m.feasible=false;
        closing=last_gap<1e8?std::max(0.,(last_gap-d)/(c.rdt/2)):0.;last_gap=d;
        if(d<.04)m.conflict=std::min(m.conflict,t);
        m.cost+=.5*c.rdt*40*std::pow(std::max(0.,.18-d),2);
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
    return m;
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    avoidance_debug={};decision_debug={};
    if(!z.fresh||sc.path.size()<2)return {};
    if(z.os.empty()){committed_side=clear_count=0;return V5MPPI::plan(s,z,sc);}
    int horizon=std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));
    auto a=arcs(sc);auto p=project(s,sc,a);int sign=direction(sc,p.index);
    std::vector<V> targets;for(const auto&r:sc.path)targets.push_back(r.p);
    auto following=[&](double scale){std::vector<V> us;S roll=s;int hint=p.index;
      for(int k=0;k<horizon;++k){V u=guide(roll,sc,a,targets,hint);u=bound(scale*u,c);us.push_back(u);roll=step(roll,u,c.rdt,c);}return us;};
    auto nominal=following(1.);auto nm=evaluate(s,z,sc,a,nominal);
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
      if(fm.feasible&&fm.min_gap>.12&&std::abs(fm.lateral)<.03){
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
      auto saved=warm;V5MPPI::plan(s,z,sc);candidates.push_back(warm);warm=saved;
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
    auto score_last=[&](){auto m=evaluate(s,z,sc,a,candidates[scores.size()]);scores.push_back(m);
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
      for(auto&u:mean)u=bound((1/total)*u,c);auto mm=evaluate(s,z,sc,a,mean);
      if(mm.feasible&&mm.cost<=sm.cost+.03){selected=std::move(mean);sm=mm;}
    }
    int mode=displaced&&nm.min_gap>.04?3:sm.lane&&lateral_needed?2:best==0?0:1;
    if(mode==2&&sm.lane)committed_side=sm.lane;
    warm=selected;published_route.clear();S roll=s;published_route.push_back({{s.x,s.y},s.yaw,s.v});
    for(const auto&u:selected){roll=step(roll,u,c.rdt,c);published_route.push_back({{roll.x,roll.y},roll.yaw,roll.v});}
    avoidance_debug[4]=sm.min_gap;avoidance_debug[5]=sm.progress;avoidance_debug[6]=sm.deficit;
    decision_debug[0]=mode;decision_debug[4]=committed_side;decision_debug[8]=sm.progress/(horizon*c.rdt);decision_debug[9]=sm.cost;decision_debug[11]=sm.lateral;
    status={3.,double(committed_side),p.distance,p.arc,double(clear_count),double(published_route.size()),0.,double(mode==0?0:mode==1?7:mode==3?8:6)};
    return selected.front();
  }
};
