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
  Score cost(S s,const std::vector<V>& us,const std::vector<std::vector<O>>& nominal,
             const std::vector<std::vector<O>>& uncertain,const Scene& sc,const std::vector<double>& arc,int initial_hint)const {
    Score value;V prev{s.v,s.w};auto p=project(s,sc,arc,initial_hint);double initial=p.arc,terminal_adherence=1;
    for(size_t k=0;k<us.size();++k){s=step(s,us[k],c.rdt,c);p=project(s,sc,arc,p.index);
      double goal=norm(sc.path.back().p-V{s.x,s.y});double d=clearance(s,nominal[k],c,sc.ymax,c.margin);
      double risk=clearance(s,uncertain[k],c,sc.ymax,c.margin);
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
  V plan(const S& s,const Z& z,const Scene& sc){
    if(!z.fresh||sc.path.size()<2)return {};
    // Static bends may need a rotate-then-advance maneuver longer than 1.5 s.
    // Keep dynamic prediction within its existing validity contract. Trade
    // samples for horizon, preserving approximately the rollout-step budget.
    const int horizon=z.os.empty()?30:c.horizon;
    const int samples=std::max(32,c.samples*c.horizon/horizon);
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
