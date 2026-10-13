// Local proposal geometry matches MPPI body disks. The fixed physical margin,
// raster correction, native abort and final rollout checks are not removed.
struct MPPI : LocalGuideMPPI {
  double guide_yaw=0;int gear=1,clear_ticks=0,strategy=0,phase=0;
  std::vector<Ref> published_route;
  MPPI(const C& config,int seed):LocalGuideMPPI(config,seed){}
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
