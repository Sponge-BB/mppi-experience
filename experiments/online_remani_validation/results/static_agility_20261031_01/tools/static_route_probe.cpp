// Diagnostic-only wrapper: same compiled source, no ROS or command publication.
#ifndef ROUTE_PROBE_API
#define ROUTE_PROBE_API "/home/l/remani_ws/experiments/online_remani_validation/builds/static_agility_20261031_01/source/online_api.cpp"
#endif
#include ROUTE_PROBE_API
#include <fstream>
#include <iomanip>

struct RouteProbe : V5MPPI {
  mutable std::vector<std::array<double,8>> segments;
  explicit RouteProbe(const C& c):V5MPPI(c,11){}
  bool segment(V a,V b)const override {
    const bool ok=V5MPPI::segment(a,b);
    double yaw=norm(b-a)<1e-5?guide_yaw:wrap(std::atan2(b.y-a.y,b.x-a.x)+(gear<0?pi:0));
    double gap=1e9;int n=std::max(1,int(std::ceil(norm(b-a)/.04)));
    for(int i=0;i<=n;++i)gap=std::min(gap,poseDistance(a+(double(i)/n)*(b-a),yaw));
    segments.push_back({a.x,a.y,b.x,b.y,yaw,double(ok),gap,norm(b-a)});
    return ok;
  }
};

extern "C" int probe_route(void* ptr,const double*state,const double*refs,int nr,const char* output) {
  try {
    auto& h=*static_cast<Online*>(ptr);native3d::Scope scope(h.native_collision);
    if(!scope.valid()||nr<2)return 1;
    S s{state[0],state[1],state[2],state[3],state[4]};Scene sc;sc.ymax=100.;
    for(int i=0;i<nr;++i)sc.path.push_back({{refs[4*i],refs[4*i+1]},refs[4*i+2],refs[4*i+3]});
    Z z;z.fresh=true;z.t=0;z.stamp=0;
    RouteProbe p(h.c);auto arcs=p.arcs(sc);auto projection=p.project(s,sc,arcs);
    p.gear=p.direction(sc,projection.index);p.guide_yaw=s.yaw;p.envelope=native3d::planarExtent();
    const size_t join=p.chooseJoin(s,sc);const V goal=sc.path[join].p;
    const V origin{s.x-3.8,s.y-3.8};const int tx=std::lround((goal.x-origin.x)/.1),ty=std::lround((goal.y-origin.y)/.1);
    const V endpoint=origin+V{.1*tx,.1*ty};
    int blocked=0;double minref=1e9;
    for(size_t i=projection.index;i<sc.path.size()&&arcs[i]<=projection.arc+2.5;++i){double d=p.poseDistance(sc.path[i].p,sc.path[i].yaw);minref=std::min(minref,d);if(d<.03)++blocked;}
    const bool made=p.makeRoute(s,z,sc);const auto make_segments=p.segments;
    const bool usable=made&&p.usable(s);
    std::ofstream out(output);out<<std::setprecision(17);
    out<<"{\"state\":["<<s.x<<","<<s.y<<","<<s.yaw<<","<<s.v<<","<<s.w<<"],\"blocked_samples\":"<<blocked
       <<",\"min_reference_gap\":"<<minref<<",\"join\":"<<join<<",\"join_xy\":["<<goal.x<<","<<goal.y
       <<"],\"grid_target\":["<<tx<<","<<ty<<"],\"start_gap\":"<<p.freeDistance({s.x,s.y})
       <<",\"end_gap\":"<<p.freeDistance(endpoint)<<",\"made\":"<<(made?"true":"false")
       <<",\"usable\":"<<(usable?"true":"false")<<",\"route_size\":"<<p.route.path.size()<<",\"make_segments\":[";
    for(size_t i=0;i<make_segments.size();++i){if(i)out<<",";out<<"[";for(int j=0;j<8;++j){if(j)out<<",";out<<make_segments[i][j];}out<<"]";}
    out<<"],\"route\":[";for(size_t i=0;i<p.route.path.size();++i){if(i)out<<",";const auto&r=p.route.path[i];out<<"["<<r.p.x<<","<<r.p.y<<","<<r.yaw<<","<<r.v<<"]";}out<<"]}";
    return 0;
  }catch(...){return 2;}
}
