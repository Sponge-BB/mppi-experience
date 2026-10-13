// Experimental predictive proposal MPPI. No CBF, no override of native gates.
// Retains measured occupancy (including moving returns), body disks and margin.
struct MPPI : V5MPPI {
  std::array<double,8> avoidance_debug{};
  int release_ticks=0;bool avoiding=false;
  MPPI(const C& config,int seed):V5MPPI(config,seed){}
  void reset(){V5MPPI::reset();avoiding=false;release_ticks=0;avoidance_debug={};}
  struct Merit {double cost=0,min_gap=1e9,progress=0,low_speed=0;bool feasible=true;};
  Merit evaluate(S s,const Z&z,const Scene&sc,const std::vector<double>&a,
                 const std::vector<V>&us)const{
    Merit m;auto p=project(s,sc,a);double start=p.arc;int sign=direction(sc,p.index);
    V previous{s.v,s.w};double terminal_gap=1e9;
    for(size_t k=0;k<us.size();++k){
      // Midpoint and endpoint checks, same lag/acceleration model as control.
      for(int j=0;j<2;++j){
        s=step(s,us[k],c.rdt/2,c);double t=(k+(j+1)*.5)*c.rdt;
        double d=clearance(s,predict(z,t,c),c,sc.ymax,c.margin);
        m.min_gap=std::min(m.min_gap,d);if(d<0)m.feasible=false;
        terminal_gap=d;
        // Smooth early-risk ranking, not an inflated collision boundary.
        m.cost+=c.rdt*.5*40*std::pow(std::max(0.,.30-d),2);
      }
      p=project(s,sc,a,p.index);double goal=norm(sc.path.back().p-V{s.x,s.y});
      double desired=std::min(std::abs(p.speed),.9*goal),forward=sign*s.v;
      double low=desired>.08?std::pow(std::max(0.,.10-forward)/.10,2):0.;
      m.low_speed+=c.rdt*low;
      m.cost+=c.rdt*(.7*p.distance*p.distance+.15*std::pow(wrap(s.yaw-p.yaw),2)
        +2*std::pow(forward-desired,2)+3*low+10*std::pow(std::max(0.,-forward),2));
      V du=us[k]-previous;m.cost+=.04*dot(du,du);previous=us[k];
    }
    m.progress=p.arc-start;
    m.cost+=3*p.distance*p.distance+.2*std::pow(wrap(s.yaw-p.yaw),2)-12*m.progress
      +20*std::pow(std::max(0.,.25-terminal_gap),2);
    // Crossing encounter terminal preference: being ahead of an approaching
    // pedestrian costs more than passing behind it. This is a soft preference,
    // not an opposite-velocity command and not prediction beyond CV validity.
    // Ignore parallel traffic and spatially remote tracks.
    for(const auto&o:predict(z,us.size()*c.rdt,c)){
      double speed=norm(o.vel);if(!o.moving||speed<.08)continue;
      V tangent{std::cos(p.yaw),std::sin(p.yaw)},n{-tangent.y,tangent.x};
      if(std::abs(dot(n,o.vel))<.65*speed)continue;
      V rel=V{s.x,s.y}-o.p;double ahead=dot(rel,o.vel)/speed;
      double across=std::abs(rel.x*o.vel.y-rel.y*o.vel.x)/speed;
      double relevance=clip((1.5-across)/.7,0.,1.);
      m.cost+=8*relevance*std::pow(clip(ahead,0.,1.5),2);
    }
    return m;
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    avoidance_debug={};
    if(!z.fresh||sc.path.size()<2)return {};
    if(z.os.empty()){avoiding=false;release_ticks=0;return V5MPPI::plan(s,z,sc);}
    int horizon=std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));
    auto a=arcs(sc);auto p=project(s,sc,a);int sign=direction(sc,p.index);
    std::vector<V> targets;for(const auto&r:sc.path)targets.push_back(r.p);
    std::vector<V> nominal(horizon);S roll=s;int hint=p.index;
    for(int k=0;k<horizon;++k){nominal[k]=guide(roll,sc,a,targets,hint);roll=step(roll,nominal[k],c.rdt,c);}
    auto nominal_merit=evaluate(s,z,sc,a,nominal);
    // Warning distance activates proposals, NOT extra collision inflation.
    if(nominal_merit.min_gap<.40){avoiding=true;release_ticks=0;}
    else if(avoiding&&nominal_merit.min_gap>.48&&++release_ticks>=4)avoiding=false;
    avoidance_debug[0]=nominal_merit.min_gap;
    if(!avoiding||norm(sc.path.back().p-V{s.x,s.y})<.15)return V5MPPI::plan(s,z,sc);
    auto previous_warm=warm;
    V base=V5MPPI::plan(s,z,sc);auto base_sequence=warm;
    std::vector<std::vector<V>> candidates;
    candidates.push_back(base_sequence);candidates.push_back(nominal);
    previous_warm.resize(horizon);candidates.push_back(previous_warm);
    candidates.push_back(std::vector<V>(horizon,V{0,0}));
    candidates.push_back(std::vector<V>(horizon,V{-sign*.10,0}));
    // Curved forward proposals in BOTH directions. No rule tying the selected
    // side to obstacle velocity. Timing and full footprint decide feasibility.
    for(double angle:{-1.5,-1.2,-.9,-.6,-.3,0.,.3,.6,.9,1.2,1.5})
      for(double speed:{.10,.16,.21})for(double hold:{.6,1.2,2.0}){
        std::vector<V> us;roll=s;hint=p.index;
        for(int k=0;k<horizon;++k){
          auto q=project(roll,sc,a,hint);hint=q.index;
          double offset=angle*clip((hold+.5-k*c.rdt)/.5,0.,1.);
          double error=wrap(q.yaw+offset-roll.yaw);
          // Saturate heading feedback before joint wheel scaling; otherwise
          // a large desired heading divides linear speed almost to zero.
          V u=bound({sign*speed*std::max(.65,std::cos(error)),clip(2*error,-c.wmax,c.wmax)},c);
          us.push_back(u);roll=step(roll,u,c.rdt,c);
        }candidates.push_back(std::move(us));
      }
    std::vector<Merit> scores;int best=-1,feasible=0;
    auto score_last=[&](){auto m=evaluate(s,z,sc,a,candidates[scores.size()]);scores.push_back(m);
      if(m.feasible){++feasible;if(best<0||m.cost<scores[best].cost)best=int(scores.size())-1;}};
    while(scores.size()<candidates.size())score_last();
    if(best>=0){
      auto seed=candidates[best];
      for(int j=0;j<32;++j){auto us=seed;V noise{};
        for(auto&u:us){noise=.75*noise+V{.02*normal(rng),.10*normal(rng)};u=bound(u+noise,c);}
        candidates.push_back(std::move(us));score_last();
      }
    }
    avoidance_debug[1]=1;avoidance_debug[2]=feasible;avoidance_debug[3]=best;
    if(best<0){
      // None of the tested controls is collision-free under this model. Brake
      // request only; never report it as a verified safe trajectory.
      warm.assign(horizon,V{});published_route.clear();strategy=5;
      avoidance_debug[7]=1;status[5]=0;status[7]=strategy;return {};
    }
    std::vector<V> mean(horizon);double total=0;
    for(size_t j=0;j<candidates.size();++j)if(scores[j].feasible){
      double w=std::exp(-(scores[j].cost-scores[best].cost)/.12);total+=w;
      for(int k=0;k<horizon;++k)mean[k]=mean[k]+w*candidates[j][k];
    }
    for(auto&u:mean)u=bound((1/total)*u,c);
    auto selected=evaluate(s,z,sc,a,mean);
    if(!selected.feasible||selected.cost>scores[best].cost+.03){mean=candidates[best];selected=scores[best];}
    warm=mean;V u=mean.front();
    // Publish executed rollout, not an unselected geometric guide.
    published_route.clear();roll=s;published_route.push_back({{s.x,s.y},s.yaw,s.v});
    for(const auto&v:mean){roll=step(roll,v,c.rdt,c);published_route.push_back({{roll.x,roll.y},roll.yaw,roll.v});}
    strategy=std::abs(u.x)<.03?(std::abs(u.y)<.05?2:3):(sign*u.x<0?4:6);
    status={3.,double(side),p.distance,a[chooseJoin(s,sc)],double(release_ticks),double(published_route.size()),0.,double(strategy)};
    avoidance_debug[4]=selected.min_gap;avoidance_debug[5]=selected.progress;avoidance_debug[6]=selected.low_speed;
    return u;
  }
};
