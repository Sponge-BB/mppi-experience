// Experimental no-uncertainty revision. BaseMPPI retains v2 model, sampling,
// physical footprint, fixed margin and actual rollout collision costs.
struct MPPI : BaseMPPI {
  enum Phase { TRACK=0, AVOID=1, REJOIN=2 };
  Phase phase=TRACK;int side=0,clear_cycles=0;
  double conflict_time=-1,path_error=0;
  std::array<double,8> status{};
  MPPI(const C& config,int seed):BaseMPPI(config,seed){
    if(c.pos_unc!=0||c.vel_unc!=0||c.acc_unc!=0)
      throw std::runtime_error("v3 requires the shared no_uncertainty profile");
  }
  void reset(){BaseMPPI::reset();phase=TRACK;side=0;clear_cycles=0;status={};}
  Ref at(const Scene& sc,const std::vector<double>& arc,double progress)const {
    auto it=std::upper_bound(arc.begin(),arc.end(),progress);
    int j=std::max(0,std::min(int(arc.size())-2,int(it-arc.begin())-1));
    double f=clip((progress-arc[j])/std::max(1e-9,arc[j+1]-arc[j]),0,1);
    return {sc.path[j].p+f*(sc.path[j+1].p-sc.path[j].p),
      wrap(sc.path[j].yaw+f*wrap(sc.path[j+1].yaw-sc.path[j].yaw)),
      (1-f)*sc.path[j].v+f*sc.path[j+1].v};
  }
  double referenceDistance(const Ref& r,const Z& z,double ahead,double offset=0)const {
    V normal{-std::sin(r.yaw),std::cos(r.yaw)},p=r.p+offset*normal;
    return clearance(S{p.x,p.y,r.yaw,0,0},predict(z,ahead,c),c,100.,c.margin);
  }
  V plan(const S& s,const Z& z,const Scene& sc){
    if(sc.path.size()<2||!z.fresh)return {};
    std::vector<double> arc(sc.path.size());
    for(size_t i=1;i<arc.size();++i)arc[i]=arc[i-1]+norm(sc.path[i].p-sc.path[i-1].p);
    auto p=project(s,sc,arc);path_error=p.distance;
    double velocity=std::max({.10,std::abs(s.v),std::abs(p.speed)});
    bool blocked=false;conflict_time=-1;double left=0,right=0;
    // No extrapolation outside the existing CV validity window.
    for(int k=0;k<=15;++k){double t=.1*k;Ref r=at(sc,arc,p.arc+velocity*t);
      if(referenceDistance(r,z,t)<.08){
        blocked=true;if(conflict_time<0)conflict_time=t;
        for(double shift:{.10,.20,.30}){
          left+=std::min(.15,referenceDistance(r,z,t,shift));
          right+=std::min(.15,referenceDistance(r,z,t,-shift));
        }
      }
    }
    if(blocked){
      clear_cycles=0;phase=AVOID;
      // Hysteresis discourages left/right alternation, but permits changing
      // side when the old side becomes substantially worse.
      if(side==0)side=left>=right?1:-1;
      else if(side>0&&right>left+.4)side=-1;
      else if(side<0&&left>right+.4)side=1;
    }else if(++clear_cycles>=4){
      phase=p.distance>.03?REJOIN:TRACK;if(phase==TRACK)side=0;
    }
    auto u=BaseMPPI::plan(s,z,sc);
    status={double(phase),double(side),path_error,conflict_time,double(clear_cycles),0,0,0};
    return u;
  }
  void adjustTargets(const S& s,const Z& z,const Scene& sc,const std::vector<double>& arc,std::vector<V>& targets)const override {
    if(phase!=AVOID||z.os.empty())return; // v2 handles measured static geometry.
    auto p=project(s,sc,arc);double speed=std::max({.10,std::abs(s.v),std::abs(p.speed)});
    for(size_t i=0;i<targets.size();++i){
      double delta=arc[i]-p.arc;if(delta<0||delta>speed*1.5+.25)continue;
      double ahead=clip(delta/speed,0,1.5);const Ref& r=sc.path[i];
      double best=referenceDistance(r,z,ahead);if(best>=.04)continue;
      V n{-std::sin(r.yaw),std::cos(r.yaw)};
      for(int j=1;j<=6;++j){double shift=side*.05*j;double d=referenceDistance(r,z,ahead,shift);
        if(d>best){best=d;targets[i]=r.p+shift*n;}if(best>=.04)break;
      }
    }
  }
  Score cost(S s,const std::vector<V>& us,const std::vector<std::vector<O>>& predicted,
             const std::vector<std::vector<O>>& duplicate,const Scene& sc,const std::vector<double>& arc,int hint)const override {
    auto score=BaseMPPI::cost(s,us,predicted,duplicate,sc,arc,hint);
    if(phase!=REJOIN)return score;
    double return_cost=0;Projection p=project(s,sc,arc,hint);
    for(size_t k=0;k<us.size();++k){
      s=step(s,us[k],c.rdt,c);p=project(s,sc,arc,p.index);Ref r=at(sc,arc,p.arc);
      // Increase attraction only to reference poses feasible under the same
      // collision model. Returning to a blocked reference is not required.
      double d=clearance(S{r.p.x,r.p.y,r.yaw,0,0},predicted[k],c,sc.ymax,c.margin);
      if(d>.08)return_cost+=c.rdt*(60*p.distance*p.distance+2*std::pow(wrap(s.yaw-r.yaw),2));
    }
    Ref r=at(sc,arc,p.arc);
    if(clearance(S{r.p.x,r.p.y,r.yaw,0,0},predicted.back(),c,sc.ymax,c.margin)>.08)
      return_cost+=40*p.distance*p.distance;
    score.path+=return_cost;score.total+=return_cost;return score;
  }
};
