// Compile the real C API implementation with libstdc++ bounds assertions.
// No ROS initialization, publisher or simulator command is used by this test.
#include "online_api.cpp"

extern "C" int local_state_regression(){
  double parameters[]={.3,.8,.35,1.,.37,.2296,.2975,.10,.35,3.,0.,0.,0.,.05,.10,.05};
  double body[]={0.,0.,.3};
  void*p=online_create(parameters,body,1,1);if(!p)return 1;
  auto&h=*static_cast<Online*>(p);
  h.actual={.12,.20};h.m.side=-1;
  online_reference_changed(p);
  if(!h.m.warm.empty()||h.m.side!=0||h.actual.x!=.12||h.actual.y!=.20)return 2;
  // Simulate a rejected shared-body binding after reference clear.
  std::vector<double>bad(16+3+4,10.);
  double header[]={1,1,1000,2,2,-7.5,-5.,.05,.10,.12,50000,1,0,.35,3.,1};
  std::copy(header,header+16,bad.begin());bad[16]=bad[17]=0;bad[18]=.31;
  if(online_set_collision_snapshot(p,bad.data(),bad.size())!=3)return 3;
  online_feedback(p,0.,0.); // Old unguarded code aborts under bounds assertions.
  if(!h.m.warm.empty()||h.actual.x!=0||h.actual.y!=0)return 4;
  online_reset(p);online_feedback(p,.1,.2);
  if(!h.m.warm.empty()||h.actual.x!=.1||h.actual.y!=.2)return 5;
  // Preserve the exact original nonempty feedback interpolation.
  h.m.warm={{.2,.1},{.2,.1}};online_feedback(p,.3,.2);
  if(std::abs(h.m.warm[0].x-.25)>1e-12||std::abs(h.m.warm[0].y-.15)>1e-12)return 6;

  std::vector<double>payload(16+3+300*200,10.);
  std::copy(header,header+16,payload.begin());payload[3]=300;payload[4]=200;
  std::copy(body,body+3,payload.begin()+16);
  if(online_set_collision_snapshot(p,payload.data(),payload.size())!=0)return 7;
  map_distance=[](double,double){return 10.;};
  Scene mixed;mixed.ymax=100;
  // First segment is reverse, but the measured state is on the forward one.
  for(int k=0;k<25;++k)mixed.path.push_back({{-.04*k,0},0,-.18});
  for(int k=0;k<=50;++k)mixed.path.push_back({{-1.+.04*k,0},0,.18});
  S s{.3,0.,0.,.18,0.};Z z;z.fresh=true;z.t=z.stamp=0;
  auto q=h.m.project(s,mixed,h.m.arcs(mixed));
  if(h.m.direction(mixed,q.index)!=1)return 8;
  h.m.envelope=.3;
  if(!h.m.makeRoute(s,z,mixed)||h.m.route.path.empty())return 9;
  if(h.m.route.path.front().v<=0||std::abs(h.m.route.path.front().yaw)>.1)return 10;

  // A static nominal connection outside the COMMON comfort zone must be
  // eligible to rejoin. Physical distance=.15, hard gap=.05, comfort=.12.
  std::fill(payload.begin()+19,payload.end(),.45);payload[1]=2;
  if(online_set_collision_snapshot(p,payload.data(),payload.size())!=0)return 11;
  online_reset(p);h.m.clear_count=3;
  Scene straight;straight.ymax=100;
  for(int k=0;k<=100;++k)straight.path.push_back({{-.5+.025*k,0},0,.18});
  S displaced{0.,.06,0.,.1,0.};auto u=h.m.plan(displaced,z,straight);
  if(!std::isfinite(u.x)||!std::isfinite(u.y)||h.m.warm.empty())return 12;
  if(h.m.decision_debug[0]!=3||h.m.clear_count<4)return 13;
  online_destroy(p);return 0;
}
