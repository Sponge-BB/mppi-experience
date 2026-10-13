// Experiment-only candidate coverage/recovery extension; not a safety proof.
// Original V7 is retained verbatim as a proposal and free-space policy.
struct MPPI : V7MPPI {
  std::array<double,10> recovery_debug{};
  bool recovering=false;int recovery_clear=0;
  MPPI(const C&config,int seed):V7MPPI(config,seed){}
  void reset(){V7MPPI::reset();recovery_debug={};recovering=false;recovery_clear=0;}
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
    if(!z.fresh||sc.path.size()<2||z.os.empty()){recovering=false;recovery_clear=0;return original;}
    bool failed=avoidance_debug[7]>0;
    if(!failed&&avoidance_debug[0]>.10){if(++recovery_clear>=4)recovering=false;}else recovery_clear=0;
    if(!recovering&&!failed&&avoidance_debug[0]>.04)return original;
    int horizon=std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));
    auto a=arcs(sc);auto p=project(s,sc,a);std::vector<V> targets;
    for(const auto&r:sc.path)targets.push_back(r.p);
    // Preserve a checked existing policy when it still has an alternative
    // continuation. Extra primitive coverage must not arbitrarily replace a
    // working maneuver merely because it offers short-horizon progress.
    if(!failed&&!recovering){
      Option base{warm,evaluate(s,z,sc,a,warm)};continuation(s,z,sc,base);
      recovery_debug[5]=base.backup;recovery_debug[6]=base.continuations;
      if(base.merit.feasible&&base.continuations>0&&terminalOpen(base.merit))return original;
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
