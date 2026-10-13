// Diagnostic-only translation unit; never linked into the execution kernel.
#include ONLINE_API_SOURCE
struct ProbeGuide:V5MPPI{
  size_t join_index=0;
  ProbeGuide(const C&c):V5MPPI(c,1){}
  size_t chooseJoin(const S&,const Scene&)const override{return join_index;}
};
extern "C" int online_probe_static_guide(void*p,const double*state,const double*refs,int nr,double*out){
  auto&h=*static_cast<Online*>(p);auto&m=h.m;
  S s{state[0],state[1],state[2],state[3],state[4]};Scene sc;sc.ymax=100.;Z z;z.fresh=true;
  for(int i=0;i<nr;++i)sc.path.push_back({{refs[4*i],refs[4*i+1]},refs[4*i+2],refs[4*i+3]});
  auto arcs=m.arcs(sc);
  size_t j=m.chooseJoin(s,sc);
  out[0]=m.route.path.size();out[1]=m.usable(s);out[2]=j;out[3]=arcs[j];out[4]=m.gear;out[5]=m.failed;
  ProbeGuide probe(h.c);probe.guide_yaw=s.yaw;probe.gear=m.gear;
  int n=0;
  for(size_t k=j;k<sc.path.size();k+=10){
    probe.join_index=k;bool made=probe.makeRoute(s,z,sc);bool usable=made&&probe.usable(s);
    V goal=sc.path[k].p,rounded{ s.x+.1*std::round((goal.x-s.x)/.1),s.y+.1*std::round((goal.y-s.y)/.1)};
    double row[]={double(k),arcs[k],goal.x,goal.y,probe.freeDistance(goal),probe.freeDistance(rounded),
      double(probe.segment(rounded,goal)),double(made),double(usable),double(probe.route.path.size())};
    std::copy(row,row+10,out+6+10*n);++n;
  }
  return n;
}
