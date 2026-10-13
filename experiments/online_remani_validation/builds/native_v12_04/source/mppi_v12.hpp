// Isolated SIL V12: obstacle-aware passing proposals with a sampled MPPI
// update. The proposals are guidance, not collision certificates. Every
// rollout uses the existing body disks, map field and physical margin; the
// caller must still perform its publication-time trajectory/braking checks.
struct MPPI : LegacyMPPI {
  bool d_active=false;
  bool d_lateral_needed=false,d_had_dynamic=false,d_using_new=false;
  int d_clear=0;
  V d_origin{},d_forward{1,0};
  MPPI(const C&config,int seed):LegacyMPPI(config,seed){}
  void referenceChanged(){d_active=false;d_clear=0;committed_side=0;}
  void reset(){LegacyMPPI::reset();d_had_dynamic=false;d_lateral_needed=false;d_using_new=false;referenceChanged();}
  bool goalCapture(const S&s,const Scene&sc)const{
    if(sc.path.size()<2)return false;
    V delta=sc.path.back().p-V{s.x,s.y};double yaw=sc.path.back().yaw;
    return norm(delta)<.20&&std::abs(-std::sin(yaw)*delta.x+std::cos(yaw)*delta.y)<.06;
  }

  struct DynamicMerit {
    Merit merit;
    double terminal_side=0;
    int side=0;
  };
  // Cached forecasts belong to this planning call. A candidate never queries
  // obstacle truth, extends the CV validity, or changes the collision shape.
  DynamicMerit dynamicScore(S s,const Scene&sc,const std::vector<double>&a,
      const std::vector<V>&us,const std::vector<std::vector<O>>&future,
      bool intervention,int desired_side=0,bool prune=true)const {
    DynamicMerit out;auto&m=out.merit;auto p=project(s,sc,a);
    double start=p.arc,last=1e9,closing=0;V prev{s.v,s.w};
    V lateral{-d_forward.y,d_forward.x};
    for(size_t k=0;k<us.size();++k){double old=p.arc;
      for(int j=0;j<2;++j){s=step(s,us[k],c.rdt/2,c);
        double gap=clearance(s,future[2*k+j+1],c,sc.ymax,c.margin);
        m.min_gap=std::min(m.min_gap,gap);
        if(gap<0)m.feasible=false;
        if(gap<.04)m.conflict=std::min(m.conflict,(k+.5*(j+1))*c.rdt);
        closing=last<1e8?std::max(0.,(last-gap)/(c.rdt/2)):0.;last=gap;
        // Terminal closing must not be ignored merely because a candidate
        // ends just before an incoming target reaches the footprint.
        m.cost+=c.rdt/2*70*std::pow(std::max(0.,.25-gap),2);
        if(prune&&!m.feasible)return out;
      }
      p=project(s,sc,a,p.index);
      double rate=(p.arc-old)/c.rdt;
      double wanted=std::min(std::abs(p.speed),.9*norm(sc.path.back().p-V{s.x,s.y}));
      double deficit=wanted>.08?std::pow(clip((wanted-rate)/wanted,0.,2.),2):0.;
      m.deficit+=c.rdt*deficit;
      double lat=dot(V{s.x,s.y}-d_origin,lateral);
      if(std::abs(lat)>std::abs(m.lateral))m.lateral=lat;
      // During a real predicted conflict, useful side displacement may precede
      // native arc progress. Strong native adherence here selected retreat or
      // standstill before the side-pass could become useful.
      bool side_pass=intervention&&d_lateral_needed;
      double adherence=side_pass?.35:5.;
      double yaw_weight=side_pass?.10:.6;
      double target_yaw=p.yaw;
      double goal_distance=norm(sc.path.back().p-V{s.x,s.y});
      if(!side_pass&&goalCapture(s,sc))target_yaw=sc.path.back().yaw;
      else if(!side_pass&&(p.distance>.08||goal_distance<.25)){
        // A displaced robot must face the attainable connection, rather than
        // being penalized for turning away from the tangent of a remote line.
        int look=p.index;
        while(look+1<int(sc.path.size())&&a[look]<p.arc+.35)++look;
        V connection=sc.path[look].p-V{s.x,s.y};
        target_yaw=wrap(std::atan2(connection.y,connection.x)+(direction(sc,p.index)<0?pi:0));
      }
      m.cost+=c.rdt*(adherence*p.distance*p.distance+
        yaw_weight*std::pow(wrap(s.yaw-target_yaw),2)+
        (intervention?.12:1.5)*deficit+
        2*std::pow(std::max(0.,-direction(sc,p.index)*s.v),2));
      V du=us[k]-prev;m.cost+=.02*dot(du,du);prev=us[k];
    }
    m.progress=p.arc-start;m.end_gap=last;m.end_closing=closing;
    out.terminal_side=dot(V{s.x,s.y}-d_origin,lateral);
    m.lane=std::abs(out.terminal_side)>.05?(out.terminal_side>0?1:-1):0;
    // The actual terminal displacement defines a side. In particular, a
    // reverse arc bends to the opposite side of its heading-angle sign.
    out.side=m.lane?m.lane:desired_side;
    bool side_pass=intervention&&d_lateral_needed;
    m.cost+=(side_pass?1.5:12.)*p.distance*p.distance-
      (side_pass?7.:12.)*m.progress;
    double response=c.tau+c.dt+std::abs(s.v)/std::min(c.accel,c.wheel_a);
    m.cost+=250*std::pow(std::max(0.,.06+closing*response-last),2)+
      60*std::pow(std::max(0.,.22-last),2);
    if(side_pass){
      // Reward reaching an escape lane only up to the physical passing width.
      // It is not an unbounded lateral attraction and collision costs dominate.
      double extent=0;for(const auto&b:c.body)extent=std::max(extent,norm(b.p)+b.r);
      double obstacle_radius=0.;for(const auto&o:future.front())obstacle_radius=std::max(obstacle_radius,o.r);
      double lane=extent+c.margin+obstacle_radius+.06;
      m.cost-=3.0*std::min(lane,std::abs(out.terminal_side));
      if(committed_side&&out.side&&out.side!=committed_side)m.cost+=.7;
    }
    return out;
  }

  V plan(const S&s,const Z&z,const Scene&sc){
    bool moving=false;for(const auto&o:z.os)moving=moving||o.moving;
    if(!moving){d_active=false;d_clear=0;d_using_new=false;
      if(!d_had_dynamic||!goalCapture(s,sc))return LegacyMPPI::plan(s,z,sc);}
    d_using_new=true;d_had_dynamic=d_had_dynamic||moving;
    avoidance_debug={};decision_debug={};recovery_debug={};margin_debug={};
    search_trace.clear();trace_start=s;
    if(!z.fresh||sc.path.size()<2){warm.clear();published_route.clear();return {};}
    double valid=c.max_prediction-std::max(0.,z.t-z.stamp)-.01;
    int horizon=std::max(1,std::min(40,int(std::floor(valid/c.rdt))));
    if(valid<c.rdt){warm.clear();published_route.clear();avoidance_debug[7]=1;return {};}
    std::vector<std::vector<O>> future;future.reserve(2*horizon+1);
    for(int k=0;k<=2*horizon;++k)future.push_back(predict(z,k*c.rdt/2,c));
    auto a=arcs(sc);auto p=project(s,sc,a);int gear=direction(sc,p.index);
    std::vector<V> targets;for(const auto&r:sc.path)targets.push_back(r.p);
    auto following=[&](double speed){std::vector<V>us;S roll=s;int hint=p.index;
      for(int k=0;k<horizon;++k){V proposal;
        V to_goal=sc.path.back().p-V{roll.x,roll.y};
        if(goalCapture(roll,sc)){
          // Locally align the endpoint pose with small signed corrections.
          // The optimizer still checks the complete proposal; this is not a
          // goal override and does not introduce a forward-only restriction.
          double along=std::cos(roll.yaw)*to_goal.x+std::sin(roll.yaw)*to_goal.y;
          double across=-std::sin(roll.yaw)*to_goal.x+std::cos(roll.yaw)*to_goal.y;
          double error=wrap(sc.path.back().yaw-roll.yaw);
          double alignment=std::pow(std::max(0.,std::cos(error)),2);
          double speed=clip(1.2*along,-.08,.08)*alignment;
          double correction_sign=speed<0?-1:1;
          proposal={speed,clip(1.8*error+2*correction_sign*across,-c.wmax,c.wmax)};
        }else proposal=guide(roll,sc,a,targets,hint);
        V u=bound(speed*proposal,c);
        us.push_back(u);roll=step(roll,u,c.rdt,c);}return us;};
    if(!d_active){d_origin={s.x,s.y};d_forward=gear*V{std::cos(p.yaw),std::sin(p.yaw)};}
    auto nominal=following(1.);
    auto nm=dynamicScore(s,sc,a,nominal,future,false,0,false);
    // Only an actual frontal closing conflict needs a persistent escape
    // lane. A crossing target must not receive a reward for chasing its
    // transverse motion. Identify threats on the continuing nominal rollout.
    d_lateral_needed=false;
    S continuing=s;V normal_axis{-d_forward.y,d_forward.x};
    for(int k=0;k<horizon&&!d_lateral_needed;++k)for(int j=0;j<2;++j){
      continuing=step(continuing,nominal[k],c.rdt/2,c);
      for(const auto&o:future[2*k+j+1]){
        double longitudinal=dot(o.vel,d_forward),transverse=dot(o.vel,normal_axis);
        if(!o.moving||longitudinal>=-.04||-longitudinal<=.65*std::abs(transverse))continue;
        for(const auto&b:c.body)
          if(norm(bodypoint(continuing,b.p)-o.p)-b.r-o.r-c.margin<.20){d_lateral_needed=true;break;}
      }
    }
    if(!d_lateral_needed)committed_side=0;
    // The full valid CV horizon, rather than a fixed 1.6s TTC threshold,
    // determines whether a passing maneuver is actually needed.
    if(nm.merit.min_gap<.20){d_active=true;d_clear=0;}
    else if(d_active&&nm.merit.feasible&&nm.merit.min_gap>.25){
      if(++d_clear>=4){d_active=false;committed_side=0;}
    }else d_clear=0;
    bool intervention=d_active;
    std::vector<std::vector<V>> candidates;
    std::vector<DynamicMerit> scores;int best=-1,nfeasible=0;
    auto add=[&](std::vector<V>us,int side=0){
      if(us.size()!=size_t(horizon))us.resize(horizon);
      auto m=dynamicScore(s,sc,a,us,future,intervention,side);
      capture(us,m.merit.cost,m.merit.min_gap,m.merit.feasible);
      int index=int(scores.size());scores.push_back(m);candidates.push_back(std::move(us));
      if(m.merit.feasible){++nfeasible;
        if(best<0||m.merit.cost<scores[best].merit.cost)best=index;}
    };
    add(nominal);for(double scale:{.7,.35,0.})add(following(scale));
    if(!warm.empty())add(warm);
    if(intervention){
      V lateral{-d_forward.y,d_forward.x};
      double width=0;for(const auto&b:c.body)width=std::max(width,norm(b.p)+b.r);
      double obstacle_radius=.0;for(const auto&o:z.os)obstacle_radius=std::max(obstacle_radius,o.r);
      double lane=width+obstacle_radius+c.margin+.06;
      // Obstacle-aware waypoint proposals persist on one world-space passing
      // lane. They do not revert to the blocked native tangent every 0.6s.
      if(d_lateral_needed)for(int side:{-1,1})for(double speed:{.16,.22})for(double advance:{.15,.45}){
        S roll=s;int hint=p.index;std::vector<V>us;
        double x0=dot(V{s.x,s.y}-d_origin,d_forward);
        V target=d_origin+(x0+advance)*d_forward+(side*lane)*lateral;
        for(int k=0;k<horizon;++k){
          double lat=dot(V{roll.x,roll.y}-d_origin,lateral);
          V u;
          if(side*lat<lane-.07){V delta=target-V{roll.x,roll.y};
            double desired=wrap(std::atan2(delta.y,delta.x)+(gear<0?pi:0));
            double err=wrap(desired-roll.yaw);
            u=bound({gear*speed*std::max(.25,std::cos(err)),clip(2*err,-c.wmax,c.wmax)},c);
          }else{
            // Once lateral clearance is achieved, progress parallel to the
            // real reference; a checked nominal rollout controls later rejoin.
            double desired=wrap(std::atan2(d_forward.y,d_forward.x)+(gear<0?pi:0));
            u=bound({gear*speed,clip(2*wrap(desired-roll.yaw),-c.wmax,c.wmax)},c);
          }
          us.push_back(u);roll=step(roll,u,c.rdt,c);
        }add(std::move(us),side);
      }
      // Wide coherent headings include near-sideways trajectories, and reverse
      // remains a valid option. All are bounded by the differential wheel model.
      if(d_lateral_needed)for(double angle:{-1.6,-1.15,-.7,.7,1.15,1.6})for(double speed:{.16,.22,-.16}){
        S roll=s;std::vector<V>us;double heading=wrap(std::atan2(d_forward.y,d_forward.x)+angle);
        for(int k=0;k<horizon;++k){V u=bound({speed,clip(2*wrap(heading-roll.yaw),-c.wmax,c.wmax)},c);
          us.push_back(u);roll=step(roll,u,c.rdt,c);}add(std::move(us),angle*speed>0?1:-1);
      }
      if(!d_lateral_needed){
        // Crossing: bounded time-yield and short steering proposals, checked
        // against every target at its own future time. Slow motion can be
        // preferable to either stopping or following the obstacle sideways.
        for(double delay:{.2,.5,.8,1.2,2.0})for(double scale:{.7,1.}){
          S roll=s;int hint=p.index;std::vector<V>us;
          for(int k=0;k<horizon;++k){V u=k*c.rdt<delay?V{}:bound(scale*guide(roll,sc,a,targets,hint),c);
            us.push_back(u);roll=step(roll,u,c.rdt,c);}add(std::move(us));
        }
        for(double angle:{-.5,.5})for(double duration:{.4,.8})for(double speed:{.12,.18}){
          S roll=s;int hint=p.index;std::vector<V>us;double heading=wrap(p.yaw+angle);
          for(int k=0;k<horizon;++k){V u=k*c.rdt<duration?
            bound({gear*speed,clip(2*wrap(heading-roll.yaw),-c.wmax,c.wmax)},c):guide(roll,sc,a,targets,hint);
            us.push_back(u);roll=step(roll,u,c.rdt,c);}add(std::move(us));
        }
      }
      add(std::vector<V>(horizon,V{-.22,0}));
      // If deterministic proposals are all blocked, independent bounded
      // two-phase samples still explore without a feasible-seed prerequisite.
      if(best<0)for(int j=0;j<12;++j){
        V first=bound({.22*std::tanh(normal(rng)),.8*std::tanh(normal(rng))},c);
        V second=bound({.22*std::tanh(normal(rng)),.8*std::tanh(normal(rng))},c);
        std::vector<V>us;for(int k=0;k<horizon;++k)us.push_back(k<horizon/2?first:second);add(std::move(us));
      }
      // Fixed work budget: cached future obstacles, bounded candidate count,
      // and early pruning keep 2/3/4s comparisons from multiplying the workload.
      if(best>=0){auto seed=candidates[best];int side=scores[best].side;
        int count=std::max(8,24*19/horizon);
        for(int j=0;j<count;++j){auto us=seed;V noise{};
          for(auto&u:us){noise=.75*noise+V{.012*normal(rng),.09*normal(rng)};u=bound(u+noise,c);}
          add(std::move(us),side);}
      }
    }
    avoidance_debug={nm.merit.min_gap,1.,double(nfeasible),double(best),0.,0.,0.,0.};
    decision_debug={0.,nm.merit.conflict<1e8?nm.merit.conflict:-1.,horizon*c.rdt,nm.merit.min_gap,
      double(committed_side),double(best),double(intervention),p.distance,0.,0.,double(d_clear),0.};
    if(best<0){warm.assign(horizon,V{});published_route.clear();avoidance_debug[7]=1;decision_debug[0]=4;
      status={3.,double(committed_side),p.distance,p.arc,0.,0.,1.,5.};return {};}
    auto selected=candidates[best];auto sm=scores[best];
    if(intervention){std::vector<V>mean(horizon);double total=0;
      for(size_t j=0;j<scores.size();++j)if(scores[j].merit.feasible&&scores[j].side==sm.side){
        double weight=std::exp(-(scores[j].merit.cost-sm.merit.cost)/.12);total+=weight;
        for(int k=0;k<horizon;++k)mean[k]=mean[k]+weight*candidates[j][k];}
      for(auto&u:mean)u=bound((1/total)*u,c);
      auto mm=dynamicScore(s,sc,a,mean,future,true,sm.side);
      capture(mean,mm.merit.cost,mm.merit.min_gap,mm.merit.feasible);
      if(mm.merit.feasible&&mm.merit.cost<=sm.merit.cost+.03){selected=std::move(mean);sm=mm;}
    }
    capture(selected,sm.merit.cost,sm.merit.min_gap,true,true);
    if(intervention&&d_lateral_needed&&sm.side)committed_side=sm.side;
    warm=selected;published_route.clear();S roll=s;
    published_route.push_back({{s.x,s.y},s.yaw,s.v});
    for(const auto&u:warm){roll=step(roll,u,c.rdt,c);published_route.push_back({{roll.x,roll.y},roll.yaw,roll.v});}
    avoidance_debug[4]=sm.merit.min_gap;avoidance_debug[5]=sm.merit.progress;avoidance_debug[6]=sm.merit.deficit;
    int mode=intervention?2:p.distance>.04?3:0;
    decision_debug[0]=mode;decision_debug[4]=committed_side;decision_debug[8]=sm.merit.progress/(horizon*c.rdt);
    decision_debug[9]=sm.merit.cost;decision_debug[11]=sm.merit.lateral;
    status={3.,double(committed_side),p.distance,p.arc,double(d_clear),double(published_route.size()),0.,double(mode==2?6:mode==3?8:0)};
    return warm.front();
  }
};
