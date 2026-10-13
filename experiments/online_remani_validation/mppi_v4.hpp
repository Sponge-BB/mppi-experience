// A geometric local route proposal + MPPI execution, not pure MPPI and not a
// safety certificate. The native safety/abort chain remains authoritative.
struct MPPI : BaseMPPI {
  Scene route;std::array<double,8> status{};int side=0,cycles=0;
  double envelope=0;bool failed=false;
  MPPI(const C& config,int seed):BaseMPPI(config,seed){
    if(c.pos_unc!=0||c.vel_unc!=0||c.acc_unc!=0)
      throw std::runtime_error("v4 requires no_uncertainty profile");
  }
  void reset(){BaseMPPI::reset();route.path.clear();side=cycles=0;status={};}
  double freeDistance(V p)const{return map_distance?map_distance(p.x,p.y)-envelope-c.margin:1e3;}
  bool segment(V a,V b)const{
    int n=std::max(1,int(std::ceil(norm(b-a)/.04)));
    for(int i=0;i<=n;++i)if(freeDistance(a+(double(i)/n)*(b-a))<.015)return false;
    return true;
  }
  bool usable(const S&s)const{
    if(route.path.size()<2)return false;
    size_t closest=0;double best=1e9;
    for(size_t i=0;i<route.path.size();++i){double d=norm(route.path[i].p-V{s.x,s.y});if(d<best){best=d;closest=i;}}
    if(best>.5||!segment({s.x,s.y},route.path[closest].p))return false;
    for(size_t i=closest;i+1<route.path.size();++i)
      if(!segment(route.path[i].p,route.path[i+1].p))return false;
    return true;
  }
  // Small bounded Dijkstra/A* grid for proposal topology. Timing used here is
  // approximate; actual rollouts are still checked against time-indexed CV.
  bool makeRoute(const S&s,const Z&z,const Scene&sc){
    const int n=77,mid=n/2,total=n*n;const double res=.10;
    V origin{s.x-mid*res,s.y-mid*res};size_t join=sc.path.size()-1;
    // Rejoin before the terminal pose so that the final reference heading is
    // approached along the original path, not imposed as an abrupt endpoint.
    for(size_t k=0;k<sc.path.size();++k)
      if(norm(sc.path[k].p-sc.path.back().p)<.45&&freeDistance(sc.path[k].p)>.08){join=k;break;}
    V goal=sc.path[join].p;
    int tx=std::lround((goal.x-origin.x)/res),ty=std::lround((goal.y-origin.y)/res);
    if(tx<1||ty<1||tx>=n-1||ty>=n-1)return false;
    auto point=[&](int k){return origin+V{res*(k%n),res*(k/n)};};
    int start=mid*n+mid,end=ty*n+tx;
    std::vector<double> gs(total,1e12);std::vector<int> parent(total,-1);
    std::vector<unsigned char> closed(total,0);
    std::vector<double> clearance_grid(total);
    for(int k=0;k<total;++k)clearance_grid[k]=freeDistance(point(k));
    if(clearance_grid[start]<.015||clearance_grid[end]<.015||!segment(point(end),goal))return false;
    // Heap entries are immutable pairs; avoid mutating keys already enqueued.
    using Entry=std::pair<double,int>;
    std::vector<Entry> queue;
    auto greater=[](const Entry&a,const Entry&b){return a.first>b.first;};
    gs[start]=0;queue.push_back({norm(goal-V{s.x,s.y}),start});
    std::vector<std::vector<O>> future;for(int k=0;k<=15;++k)future.push_back(predict(z,.1*k,c));
    V baseline=goal-V{s.x,s.y};double length=std::max(.01,norm(baseline));
    for(int expanded=0;!queue.empty()&&expanded<total;++expanded){
      std::pop_heap(queue.begin(),queue.end(),greater);int u=queue.back().second;queue.pop_back();
      if(closed[u]){--expanded;continue;}closed[u]=1;if(u==end)break;
      int ux=u%n,uy=u/n;
      for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
        if(!dx&&!dy)continue;int x=ux+dx,y=uy+dy;if(x<0||y<0||x>=n||y>=n)continue;
        int v=y*n+x;if(closed[v]||clearance_grid[v]<.015)continue;
        if(dx&&dy&&(clearance_grid[uy*n+x]<.015||clearance_grid[y*n+ux]<.015))continue;
        V p=point(v),delta=p-V{s.x,s.y};double distance=norm(delta);
        double arrival=distance/std::max(.10,std::abs(s.v));double risk=0;
        // No prediction beyond 1.5 s, and never use obstacle truth.
        if(arrival<=1.5){int t=std::min(15,int(std::lround(arrival*10)));
          for(const auto&o:future[t])risk=std::max(risk,std::max(0.,.20-(norm(p-o.p)-o.r-envelope-c.margin))*12.);
        }
        double lateral=(baseline.x*delta.y-baseline.y*delta.x)/length;
        double switch_cost=side&&side*lateral<-.10?.25:0.;
        double edge=res*std::hypot(dx,dy)*(1.+risk+switch_cost+.08/std::max(.04,clearance_grid[v]));
        double g=gs[u]+edge;if(g>=gs[v])continue;gs[v]=g;parent[v]=u;
        queue.push_back({g+norm(goal-p),v});std::push_heap(queue.begin(),queue.end(),greater);
      }
    }
    if(parent[end]<0)return false;
    std::vector<V> reverse;for(int k=end;k!=start;k=parent[k]){if(k<0)return false;reverse.push_back(point(k));}
    reverse.push_back({s.x,s.y});std::reverse(reverse.begin(),reverse.end());reverse.push_back(goal);
    // Line-of-sight shortcut with the same conservative footprint model.
    std::vector<V> corners{reverse.front()};size_t i=0;
    while(i+1<reverse.size()){size_t j=reverse.size()-1;while(j>i+1&&!segment(reverse[i],reverse[j]))--j;
      if(!segment(reverse[i],reverse[j]))return false;corners.push_back(reverse[j]);i=j;
    }
    Scene candidate=sc;candidate.path.clear();int gear=direction(sc,0);
    for(size_t j=0;j+1<corners.size();++j){V a=corners[j],b=corners[j+1],d=b-a;
      int count=std::max(1,int(std::ceil(norm(d)/.04)));double yaw=wrap(std::atan2(d.y,d.x)+(gear<0?pi:0));
      for(int k=0;k<count;++k)candidate.path.push_back({a+(double(k)/count)*d,yaw,gear*.18});
    }
    for(size_t k=join;k<sc.path.size();++k)candidate.path.push_back(sc.path[k]);
    double most=0;for(const auto&r:candidate.path){V d=r.p-V{s.x,s.y};double lateral=(baseline.x*d.y-baseline.y*d.x)/length;if(std::abs(lateral)>std::abs(most))most=lateral;}
    if(std::abs(most)>.1)side=most>0?1:-1;
    route=std::move(candidate);return true;
  }
  V plan(const S&s,const Z&z,const Scene&sc){
    if(sc.path.size()<2||!z.fresh)return {};
    envelope=0;for(const auto&b:c.body)envelope=std::max(envelope,norm(b.p)+b.r);
    std::vector<double> arc(sc.path.size());for(size_t i=1;i<arc.size();++i)arc[i]=arc[i-1]+norm(sc.path[i].p-sc.path[i-1].p);
    auto p=project(s,sc,arc);bool blocked=false;
    for(size_t i=p.index;i<sc.path.size();++i)if(freeDistance(sc.path[i].p)<.04){blocked=true;break;}
    bool active=!route.path.empty();failed=false;
    // Cached complete detour, not independent left/right offsets per point.
    if((active&&!usable(s))||(!active&&blocked)){
      if(!makeRoute(s,z,sc)){failed=true;route.path.clear();}
      else active=true;
    }
    active=!route.path.empty();
    auto u=BaseMPPI::plan(s,z,active?route:sc);
    status={double(active?1:0),double(side),p.distance,-1,0,double(route.path.size()),double(failed),0};
    return u;
  }
};
