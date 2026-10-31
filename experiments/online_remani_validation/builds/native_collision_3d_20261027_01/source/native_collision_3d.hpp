#pragma once
// Fixed-arm native-geometry MPPI experiment. No ROS calls in a point query.
// Wire v1: 32 doubles; body[7] = xyz, hard_r, soft_r, weight, type(0/1);
// dynamic[7] = xyz, velocity xyz, track id; optional raw x-major ESDF follows.
// Header: magic=3, version=1, seq, publication_stamp, healthy, nx,ny,nz,
// crop_edge_origin xyz, resolution, nbody, ndynamic, esdf_seq, max_age,
// max_prediction, esdf_acquisition_stamp, dynamic_stamp, dynamic_healthy,
// has_full_esdf, ground_safe_distance, ground_ok, self_collision_ok,
// observed_joint_stamp, pos_unc, vel_unc, acc_bound, dynamic_enabled, 0,0,0.
// ESDF/joint stamps are honest audit metadata, not live heartbeat timestamps.
// Original fixed Piper self-collision policy is retained, NOT repaired here.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace native3d {
constexpr std::size_t header_size=32, body_stride=7, dynamic_stride=7;
inline bool enabled(){const char*p=std::getenv("NATIVE_COLLISION_3D");return p&&p[0]=='1'&&p[1]=='\0';}
struct Point { double x=0,y=0,z=0; };
inline double coordinate(const Point&p,int axis){return axis==0?p.x:axis==1?p.y:p.z;}
inline double squaredDistance(const Point&a,const Point&b){
  const double x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;return x*x+y*y+z*z;
}
struct Body { Point p;double hard_radius=0,soft_radius=0,weight=0;int type=0; };
struct KDTree {
  struct Node {Point p;int left=-1,right=-1,axis=0;};
  std::vector<Node> nodes;int root=-1;
  int build(std::vector<Point>&points,int lo,int hi,int depth){
    if(lo>=hi)return -1;int mid=lo+(hi-lo)/2,axis=depth%3;
    std::nth_element(points.begin()+lo,points.begin()+mid,points.begin()+hi,
      [axis](const Point&a,const Point&b){return coordinate(a,axis)<coordinate(b,axis);});
    const int index=int(nodes.size());nodes.push_back({points[mid],-1,-1,axis});
    const int left=build(points,lo,mid,depth+1),right=build(points,mid+1,hi,depth+1);
    nodes[index].left=left;nodes[index].right=right;return index;
  }
  explicit KDTree(std::vector<Point>points={}){nodes.reserve(points.size());root=build(points,0,int(points.size()),0);}
  void search(int index,const Point&q,double&best)const{
    if(index<0)return;const auto&n=nodes[index];best=std::min(best,squaredDistance(q,n.p));
    const double delta=coordinate(q,n.axis)-coordinate(n.p,n.axis);
    search(delta<0?n.left:n.right,q,best);
    if(delta*delta<best)search(delta<0?n.right:n.left,q,best);
  }
  double distance(const Point&q)const{
    double best=std::numeric_limits<double>::infinity();search(root,q,best);return std::sqrt(best);
  }
};
struct DynamicGroup {double track=0;Point velocity;KDTree tree;};
struct Map {
  int nx=0,ny=0,nz=0;Point origin;double resolution=0,sequence=0;
  std::vector<double> distance;
  std::size_t address(int x,int y,int z)const{return (std::size_t(x)*ny+y)*nz+z;}
};
struct Snapshot {
  double sequence=0,stamp=0,max_age=0,max_prediction=0,map_stamp=0,dynamic_stamp=0,joint_stamp=0;
  double ground_safe_distance=0;
  bool healthy=false,dynamic_healthy=false,ground_ok=false,self_ok=false,dynamic_enabled=false;
  std::shared_ptr<const Map> map;std::vector<Body> body;
  std::vector<double> dynamic_wire;
  std::shared_ptr<const std::vector<DynamicGroup>> groups;
};
struct State {std::shared_ptr<const Snapshot> snapshot;std::shared_ptr<const Map> cached_map;double now=0;};
inline bool integer(double value,double minimum,double maximum){
  return std::isfinite(value)&&value>=minimum&&value<=maximum&&std::floor(value)==value;
}
inline bool sameMap(const Map&m,const double*h){
  return m.sequence==h[14]&&m.nx==h[5]&&m.ny==h[6]&&m.nz==h[7]&&
    m.origin.x==h[8]&&m.origin.y==h[9]&&m.origin.z==h[10]&&m.resolution==h[11];
}
// Binding failure invalidates current authority. It never silently uses old inputs.
inline bool bind(State&state,const double*p,std::size_t count){
  auto fail=[&](){state.snapshot.reset();return false;};
  if(!p||count<header_size||count>32000000)return fail();
  for(std::size_t i=0;i<header_size;++i)if(!std::isfinite(p[i]))return fail();
  if(p[0]!=3||p[1]!=1||!integer(p[2],1,9007199254740991.)||p[3]<=0||
    !integer(p[4],0,1)||!integer(p[5],2,2000)||!integer(p[6],2,2000)||!integer(p[7],2,2000)||
    p[11]<=0||!integer(p[12],1,10000)||!integer(p[13],0,2000000)||
    !integer(p[14],1,9007199254740991.)||p[15]<=0||p[15]>.35||p[16]<=0||p[16]>4.||
    p[17]<=0||p[18]<=0||!integer(p[19],0,1)||!integer(p[20],0,1)||p[21]<0||
    !integer(p[22],0,1)||!integer(p[23],0,1)||p[24]<=0||
    p[25]!=0||p[26]!=0||p[27]!=0||!integer(p[28],0,1)||p[29]!=0||p[30]!=0||p[31]!=0)return fail();
  const std::size_t nb=std::size_t(p[12]),nd=std::size_t(p[13]);
  const std::size_t cells=std::size_t(p[5])*std::size_t(p[6])*std::size_t(p[7]);
  const std::size_t prefix=header_size+body_stride*nb+dynamic_stride*nd;
  if(cells>32000000||count!=prefix+(p[20]==1?cells:0))return fail();
  for(std::size_t i=header_size;i<prefix;++i)if(!std::isfinite(p[i]))return fail();
  std::shared_ptr<const Map> map=state.cached_map;
  // esdf_seq denotes an immutable raw field. Repeated self-contained Python
  // payloads reuse it without copying/scanning millions of cells each cycle.
  if(map&&map->sequence==p[14]&&!sameMap(*map,p))return fail();
  if(p[20]==1&&(!map||!sameMap(*map,p))){
    for(std::size_t i=prefix;i<count;++i)if(!std::isfinite(p[i]))return fail();
    auto next=std::make_shared<Map>();next->nx=int(p[5]);next->ny=int(p[6]);next->nz=int(p[7]);
    next->origin={p[8],p[9],p[10]};next->resolution=p[11];next->sequence=p[14];
    next->distance.assign(p+prefix,p+count);map=next;
  }else if(!map||!sameMap(*map,p))return fail();
  auto frame=std::make_shared<Snapshot>();frame->sequence=p[2];frame->stamp=p[3];frame->healthy=p[4]==1;
  frame->map=map;frame->max_age=p[15];frame->max_prediction=p[16];frame->map_stamp=p[17];
  frame->dynamic_stamp=p[18];frame->dynamic_healthy=p[19]==1;frame->ground_safe_distance=p[21];
  frame->ground_ok=p[22]==1;frame->self_ok=p[23]==1;frame->joint_stamp=p[24];frame->dynamic_enabled=p[28]==1;
  std::size_t offset=header_size;frame->body.reserve(nb);
  for(std::size_t i=0;i<nb;++i,offset+=body_stride){
    if(p[offset+3]<=0||p[offset+4]<=0||p[offset+5]<=0||!integer(p[offset+6],0,1))return fail();
    frame->body.push_back({{p[offset],p[offset+1],p[offset+2]},p[offset+3],p[offset+4],p[offset+5],int(p[offset+6])});
  }
  // Same track AND exact same velocity permits inverse translation of the
  // query. The KD-tree retains every measured point; no enclosing ball is used.
  frame->dynamic_wire.assign(p+offset,p+prefix);
  if(!frame->dynamic_enabled&&nd!=0)return fail();
  if(state.snapshot&&state.snapshot->dynamic_wire==frame->dynamic_wire){
    frame->groups=state.snapshot->groups;state.cached_map=map;state.snapshot=frame;return true;
  }
  using Key=std::tuple<double,double,double,double>;std::map<Key,std::vector<Point>> grouped;
  for(std::size_t i=0;i<nd;++i,offset+=dynamic_stride){
    if(!integer(p[offset+6],-1,9007199254740991.))return fail();
    grouped[{p[offset+6],p[offset+3],p[offset+4],p[offset+5]}].push_back({p[offset],p[offset+1],p[offset+2]});
  }
  auto groups=std::make_shared<std::vector<DynamicGroup>>();groups->reserve(grouped.size());
  for(auto&entry:grouped){
    const auto&key=entry.first;
    groups->push_back({std::get<0>(key),{std::get<1>(key),std::get<2>(key),std::get<3>(key)},KDTree(std::move(entry.second))});
  }
  frame->groups=groups;
  state.cached_map=map;state.snapshot=frame;return true;
}
inline bool fresh(const Snapshot&s,double now){
  return std::isfinite(now)&&s.healthy&&s.dynamic_healthy&&s.ground_ok&&s.self_ok&&s.map&&
    now-s.stamp>=-.05&&now-s.stamp<=s.max_age&&
    now-s.dynamic_stamp>=-.05&&now-s.dynamic_stamp<=s.max_age;
}
struct Context {std::shared_ptr<const Snapshot> snapshot;double now=0;bool active=false;};
inline Context& context(){static thread_local Context c;return c;}
struct Scope {
  Context previous;
  explicit Scope(const State&s):previous(context()){context()={s.snapshot,s.now,true};}
  ~Scope(){context()=previous;}
  bool valid()const{return context().snapshot&&fresh(*context().snapshot,context().now);}
  Scope(const Scope&)=delete;Scope&operator=(const Scope&)=delete;
};
inline double now(){return context().now;}
inline bool hasDynamic(){
  const auto&s=context().snapshot;
  return s&&s->dynamic_enabled&&s->groups&&!s->groups->empty();
}
inline double predictionAge(){const auto&s=context().snapshot;return s?std::max(0.,now()-s->dynamic_stamp):0.;}
inline double maxPrediction(){const auto&s=context().snapshot;return s?s->max_prediction:0.;}
struct Evaluation {double gap=-1e6,cost=1e12,static_gap=-1e6,dynamic_gap=-1e6;bool valid=false;};
// Reproduce GridMap::getSurroundPts/getPreciseDistance, including its half-cell
// offset. Crop edges without all eight native neighbours are UNKNOWN, not free.
inline bool preciseDistance(const Map&m,const Point&p,double&distance){
  const double f[]={ (p.x-m.origin.x)/m.resolution-.5,
    (p.y-m.origin.y)/m.resolution-.5,(p.z-m.origin.z)/m.resolution-.5 };
  for(int axis=0;axis<3;++axis)if(!std::isfinite(f[axis])||f[axis]<0)return false;
  if(f[0]>=m.nx-1||f[1]>=m.ny-1||f[2]>=m.nz-1)return false;
  const int ix=int(std::floor(f[0])),iy=int(std::floor(f[1])),iz=int(std::floor(f[2]));
  const double a=f[0]-ix,b=f[1]-iy,c=f[2]-iz;distance=0;
  for(int x=0;x<2;++x)for(int y=0;y<2;++y)for(int z=0;z<2;++z)
    distance+=(x?a:1-a)*(y?b:1-b)*(z?c:1-c)*m.distance[m.address(ix+x,iy+y,iz+z)];
  return std::isfinite(distance);
}
inline Evaluation query(const Snapshot&s,double x,double y,double yaw,double absolute_time,bool dynamic=true,bool early_exit=false){
  Evaluation out;
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(absolute_time)||
    !s.map||!s.ground_ok||!s.self_ok||s.body.empty())return out;
  const double forecast=absolute_time-s.dynamic_stamp;
  if(dynamic&&s.dynamic_enabled&&s.groups&&!s.groups->empty()&&
    (forecast<-.05||forecast>s.max_prediction+1e-9))return out;
  const double dt=std::max(0.,forecast),co=std::cos(yaw),si=std::sin(yaw);
  out.gap=out.static_gap=out.dynamic_gap=std::numeric_limits<double>::infinity();out.cost=0;
  for(const auto&body:s.body){
    const Point p{x+co*body.p.x-si*body.p.y,y+si*body.p.x+co*body.p.y,body.p.z};
    double stat;
    if(!preciseDistance(*s.map,p,stat))return Evaluation{};
    out.static_gap=std::min(out.static_gap,stat-body.hard_radius);
    double dyn=std::numeric_limits<double>::infinity();
    if(dynamic&&s.dynamic_enabled&&s.groups)for(const auto&group:*s.groups){
      const Point local{p.x-dt*group.velocity.x,p.y-dt*group.velocity.y,p.z-dt*group.velocity.z};
      dyn=std::min(dyn,group.tree.distance(local));
    }
    out.dynamic_gap=std::min(out.dynamic_gap,dyn-body.hard_radius);
    // Fixed self/ground penalties are constant across planar rollouts. Native
    // ground collision uses the arm sample centre, not thickness-expanded z.
    if(body.type==1)out.static_gap=std::min(out.static_gap,p.z-s.ground_safe_distance);
    const double error=std::max(0.,body.soft_radius-std::min(stat,dyn));
    out.cost+=body.weight*error*error*error;
    if(early_exit&&std::min(out.static_gap,out.dynamic_gap)<0){
      out.gap=std::min(out.static_gap,out.dynamic_gap);out.valid=true;return out;
    }
  }
  out.gap=std::min(out.static_gap,out.dynamic_gap);out.valid=std::isfinite(out.gap)&&std::isfinite(out.cost);
  return out;
}
inline Evaluation query(double x,double y,double yaw,double absolute_time,bool dynamic=true,bool early_exit=false){
  const auto&c=context();if(!c.active||!c.snapshot||!fresh(*c.snapshot,c.now))return {};
  return query(*c.snapshot,x,y,yaw,absolute_time,dynamic,early_exit);
}
inline double planarExtent(){
  const auto&s=context().snapshot;if(!s)return 0.;double extent=0;
  for(const auto&b:s->body)extent=std::max(extent,std::hypot(b.p.x,b.p.y)+b.hard_radius);return extent;
}
} // namespace native3d
