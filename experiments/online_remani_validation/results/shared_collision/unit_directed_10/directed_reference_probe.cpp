// Standard-C++ cursor probe. Does not initialize ROS or issue any command.
#include <cmath>
#include <cstdint>
#include <vector>
#include "directed_reference.hpp"

using directed_reference::Cursor;
using directed_reference::Sample;
using directed_reference::Measurement;

namespace {
std::vector<Sample> samples(const double*data,int count){
  std::vector<Sample> result;
  for(int i=0;i<count;++i)result.push_back({data[4*i],data[4*i+1],data[4*i+2],data[4*i+3]});
  return result;
}
bool unchanged(Cursor&cursor,int phase,unsigned transitions,std::uint64_t hash){
  return cursor.phase_id()==phase&&cursor.transitions()==transitions&&cursor.reference_hash()==hash;
}
}

extern "C" void* directed_cursor_create(){return new Cursor;}
extern "C" void directed_cursor_destroy(void*ptr){delete static_cast<Cursor*>(ptr);}
extern "C" int directed_cursor_update(void*ptr,const double*data,int count,double x,double y,double v,double*out){
  if(!ptr||!data||count<2||!out)return 1;
  try{
    auto&cursor=*static_cast<Cursor*>(ptr);
    cursor.update(samples(data,count),Measurement{x,y,v});cursor.debug(out);return 0;
  }catch(...){return 2;}
}
extern "C" void directed_cursor_debug(void*ptr,double*out){static_cast<Cursor*>(ptr)->debug(out);}
extern "C" int directed_cursor_candidate_reads(void*ptr,int count){
  auto&cursor=*static_cast<Cursor*>(ptr);int phase=cursor.phase_id();
  unsigned transitions=cursor.transitions();auto hash=cursor.reference_hash();
  // Candidate geometry may be evaluated far beyond a cusp, but reading the
  // association cannot mutate measured execution state. Kernel integration
  // is a separate test; this is deliberately only the standalone interface.
  for(int i=0;i<count;++i){auto active=cursor.active();double candidate=i*.01;
    volatile double score=candidate+active.begin+active.end+active.gear;(void)score;
    if(!unchanged(cursor,phase,transitions,hash))return 1;
  }return 0;
}

extern "C" int directed_cursor_regression(){
  std::vector<Sample> reference{{0,0,0,.05},{.5,0,0,.10},{1,0,0,.05},
    {1,0,0,-.05},{.5,0,0,-.10},{0,0,0,-.05},
    {0,0,0,.05},{.5,0,0,.10},{1,0,0,.05}};
  Cursor c;
  auto u=c.update(reference,{1,0,0});
  if(!u.started||u.switched||!c.ready()||c.phase_id()!=0||c.phase_count()!=3||c.active().gear!=1)return 1;
  // A new reference beginning at a cusp does not skip the initial phase.
  const auto hash=c.reference_hash();
  u=c.update(reference,{.5,0,0});
  if(u.started||u.switched||!unchanged(c,0,0,hash))return 2;
  u=c.update(reference,{1,0,.026});
  if(u.switched||c.phase_id()!=0)return 3;
  u=c.update(reference,{1,0,-.026});
  if(u.switched||c.phase_id()!=0)return 4;
  u=c.update(reference,{1.0501,0,0});
  if(u.switched||c.phase_id()!=0)return 5;
  if(directed_cursor_candidate_reads(&c,5000)!=0)return 6;
  u=c.update(reference,{1.04,0,.024});
  if(!u.switched||u.previous_phase!=0||c.phase_id()!=1||c.transitions()!=1||c.active().gear!=-1)return 7;
  if(c.active().begin<=0||c.active().end>=int(reference.size())-1)return 8;
  // The next cusp is elsewhere: a second observation here cannot leap again.
  u=c.update(reference,{1.04,0,0});
  if(u.switched||c.phase_id()!=1||c.transitions()!=1)return 9;
  u=c.update(reference,{.04,0,-.024});
  if(!u.switched||c.phase_id()!=2||c.transitions()!=2||c.active().gear!=1)return 10;
  u=c.update(reference,{1,0,0});
  if(u.switched||c.phase_id()!=2||c.transitions()!=2)return 11;
  // Adjacent same-gear pieces, including duplicate zero samples, are not cusps.
  std::vector<Sample> same{{0,0,0,0},{.25,0,0,.05},{.5,0,0,0},
    {.5,0,0,.05},{.75,0,0,.05},{1,0,0,0}};
  Cursor d;d.update(same,{0,0,0});u=d.update(same,{.5,0,0});
  if(d.phase_count()!=1||d.phase_id()!=0||u.switched||d.active().gear!=1)return 12;
  std::vector<Sample> leading{{0,0,0,0},{-.5,0,0,-.05},{-1,0,0,0}};
  Cursor e;e.update(leading,{0,0,0});
  if(e.phase_count()!=1||e.active().gear!=-1||e.active().begin!=0)return 13;
  std::vector<Sample> zero{{0,0,0,0},{1,0,0,0}};
  Cursor f;f.update(zero,{0,0,0});
  if(f.phase_count()!=1||f.active().gear!=1)return 14;
  auto changed=reference;changed[1].y=.01;
  u=c.update(changed,{1,0,0});
  if(!u.started||u.switched||c.phase_id()!=0||c.transitions()!=0||c.reference_hash()==hash)return 15;
  c.reset();if(c.ready()||c.transitions()!=0)return 16;
  c.update(reference,{1,0,0});
  if(c.phase_id()!=0||c.transitions()!=0)return 17;
  // Association is metadata, not a destructive truncation of the task path.
  if(reference.size()!=9||reference.back().x!=1||reference.back().signed_speed!=.05)return 18;
  return 0;
}

#ifdef DIRECTED_WITH_CONTROLLER
// Optional integration probe compiles just this small translation unit using
// the completed isolated controller's flags/archives. No ROS runtime is used.
#include "online_api.cpp"

extern "C" int directed_controller_regression(){
  double parameters[]={.3,.8,.35,1.,.37,.2296,.2975,.10,.35,3.,0.,0.,0.,.05,.10,.05};
  double body[]={0.,0.,.3};void*p=online_create(parameters,body,1,1);if(!p)return 1;
  auto&h=*static_cast<Online*>(p);h.actual={.12,.20};h.m.side=-1;
  online_reference_changed(p);
  if(!h.m.warm.empty()||h.m.side!=0||h.actual.x!=.12||h.actual.y!=.20)return 2;
  online_reset(p);
  if(h.actual.x!=.12||h.actual.y!=.20)return 3;
  online_feedback(p,.10,.18);
  if(h.actual.x!=.10||h.actual.y!=.18||!h.m.warm.empty())return 4;
  std::vector<double>payload(16+3+300*200,10.);
  double header[]={1.,1.,1000.,300.,200.,-7.5,-5.,.05,.10,.12,50000.,1.,0.,.35,3.,1.};
  std::copy(header,header+16,payload.begin());std::copy(body,body+3,payload.begin()+16);
  if(online_set_collision_snapshot(p,payload.data(),payload.size())!=0)return 5;
  map_distance=[](double,double){return 10.;};
  Scene native;native.ymax=100.;native.directed=true;
  native.active_begin=0;native.active_end=2;native.active_gear=1;
  native.path={{{0,0},pi/2,.05},{{0,.5},pi/2,.10},{{0,1},pi/2,.05},
    {{0,1},pi/2,-.05},{{0,.5},pi/2,-.10},{{0,0},pi/2,-.05},
    {{0,0},0,.05},{{1.5,0},0,.10},{{3,0},0,.05}};
  auto arc=h.m.arcs(native);std::vector<V>targets;
  for(const auto&r:native.path)targets.push_back(r.p);
  if(rangeBegin(native)!=0||rangeEnd(native)!=2||finalRange(native)||native.path.back().p.x!=3)return 6;
  S at_goal{3,0,0,0,0};
  auto projection=h.m.project(at_goal,native,arc);
  if(projection.index<0||projection.index>=2||h.m.direction(native,projection.index)!=1||h.m.goalCapture(at_goal,native))return 7;
  // A nonfinal exact cusp must not manufacture a heading from atan2(0,0).
  int hint=-1;S cusp{0,1,pi/2,.05,0};auto u=h.m.guide(cusp,native,arc,targets,hint);
  if(std::abs(u.x)>1e-12||std::abs(u.y)>1e-12)return 8;
  hint=-1;S near_cusp{0,.99,pi/2,.05,0};u=h.m.guide(near_cusp,native,arc,targets,hint);
  if(std::abs(u.x)>.009+1e-9||std::abs(u.y)>1e-9)return 9;
  // An off-path pose projects to the endpoint but has NOT reached it. It
  // must retain a feasible nonzero reconnect proposal, not arc-clamp deadlock.
  hint=-1;S off_path{.30,1,pi,.05,0};u=h.m.guide(off_path,native,arc,targets,hint);
  if(u.x<=.001||!std::isfinite(u.y))return 10;
  // Connector indices are in its own path, not copied native indices.
  native.active_begin=3;native.active_end=5;native.active_gear=-1;h.m.gear=-1;
  Scene local=h.m.connector({-.2,.5,pi,0,0},native,4);
  if(local.directed||rangeBegin(local)!=0||local.local_end==native.active_end||finalRange(local)||local.path.back().p.x!=3)return 11;
  if(rangeEnd(local)!=int(local.path.size())-1-3||h.m.direction(local,int(local.path.size())-1)!=-1)return 12;
  auto local_arc=h.m.arcs(local);projection=h.m.project(at_goal,local,local_arc);
  if(projection.index<0||projection.index>rangeEnd(local)||h.m.goalCapture(at_goal,local))return 13;
  bool threw=false;try{h.m.connector({0,0,0,0,0},native,6);}catch(const std::invalid_argument&){threw=true;}
  if(!threw)return 14;
  // True terminal capture still uses the complete bundle's real final pose.
  native.active_begin=6;native.active_end=8;native.active_gear=1;
  if(!finalRange(native)||!h.m.goalCapture(at_goal,native))return 15;
  hint=-1;u=h.m.guide({3,0,pi/2,0,0},native,arc,targets,hint);
  if(std::abs(u.x)>1e-12||u.y>=-.1)return 16;
  // Exercise the actual ingress. The entire forecast cannot update cursor.
  std::vector<double>refs;
  for(const auto&r:native.path){refs.push_back(r.p.x);refs.push_back(r.p.y);refs.push_back(r.yaw);refs.push_back(r.v);}
  std::vector<double>field(300*200,10.);double debug[12],result[8];
  double measured[]={0,.8,pi/2,.05,0};online_reference_changed(p);
  if(online_step(p,measured,refs.data(),native.path.size(),nullptr,0,.05,field.data(),300,200,-7.5,-5.,.05,0,result)!=0)return 17;
  online_directed_cursor_debug(p,debug);
  if(debug[1]!=0||debug[2]!=3||debug[7]!=0)return 18;
  // Explicit hypothetical planning calls receive immutable Scene context.
  native.active_begin=0;native.active_end=2;native.active_gear=1;
  auto cursor_hash=h.cursor.reference_hash();Z z;z.fresh=true;
  for(int k=0;k<3;++k)h.m.plan({3.,.1*k,0,0,0},z,native);
  if(!unchanged(h.cursor,0,0,cursor_hash))return 19;
  measured[1]=1.;measured[3]=.026;
  if(online_step(p,measured,refs.data(),native.path.size(),nullptr,0,.05,field.data(),300,200,-7.5,-5.,.05,0,result)!=0)return 20;
  online_directed_cursor_debug(p,debug);if(debug[1]!=0||debug[7]!=0)return 21;
  measured[3]=.024;
  if(online_step(p,measured,refs.data(),native.path.size(),nullptr,0,.05,field.data(),300,200,-7.5,-5.,.05,0,result)!=0)return 22;
  online_directed_cursor_debug(p,debug);if(debug[1]!=1||debug[7]!=1)return 23;
  if(h.actual.x!=.10||h.actual.y!=.18)return 24;
  online_reference_changed(p);online_directed_cursor_debug(p,debug);
  if(debug[0]!=0||!h.m.warm.empty()||!h.m.route.path.empty()||h.m.side!=0||h.actual.x!=.10||h.actual.y!=.18)return 25;
  online_reset(p);online_directed_cursor_debug(p,debug);
  if(debug[0]!=0||h.actual.x!=.10||h.actual.y!=.18)return 26;
  online_destroy(p);return 0;
}
#endif
