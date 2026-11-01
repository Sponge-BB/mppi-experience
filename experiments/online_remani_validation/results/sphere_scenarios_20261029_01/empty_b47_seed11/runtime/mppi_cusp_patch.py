"""Controlled native_v12_07 phase-association / phase-objective ablations.

Generated build sources only. Full native suffix/final goal remain in Scene;
range metadata is independent for native references and generated connectors.
"""
from directed_reference_patch import exact, PROJECT, RANGE_HELPERS


def transform_kernel(text):
    text=exact(text,'  double velocity_error=0;\n};\nScene scene(',
        '  double velocity_error=0;\n'
        '  bool directed=false;int active_begin=0,active_end=-1,active_gear=0;\n'
        '  int local_end=-1,local_gear=0;\n};\n'+RANGE_HELPERS+'\nScene scene(')
    start='  Projection project(const S& s,const Scene& sc,const std::vector<double>& arc,int hint=-1)const {'
    stop='  int direction(const Scene& sc,int i)const {'
    if text.count(start)!=1 or text.count(stop)!=1:raise ValueError('ambiguous frozen projection')
    a,b=text.index(start),text.index(stop)
    text=text[:a]+PROJECT+text[b:]
    text=exact(text,stop,stop+'''
    if(sc.directed)return sc.active_gear;
    if(sc.local_end>=0)return sc.local_gear;''')
    text=exact(text,'    double remaining=arc.back()-p.arc,goal=norm(sc.path.back().p-V{s.x,s.y});',
        '''    const int active_end=rangeEnd(sc);
    double remaining=std::max(0.,arc[active_end]-p.arc),goal=norm(sc.path.back().p-V{s.x,s.y});
    const double cusp_distance=norm(sc.path[active_end].p-V{s.x,s.y});
    // Brake and align at a cusp. Only a later measured API call can switch.
    if(!finalRange(sc)&&remaining<=.05&&cusp_distance<=.04)
      return bound({0,1.8*wrap(sc.path[active_end].yaw-s.yaw)},c);''')
    text=exact(text,'    if(goal<.025)return bound({0,1.8*wrap(sc.path.back().yaw-s.yaw)},c);',
        '    if(finalRange(sc)&&goal<.025)return bound({0,1.8*wrap(sc.path.back().yaw-s.yaw)},c);')
    text=exact(text,'    while(target+1<int(sc.path.size())&&arc[target]<p.arc+lookahead){',
        '    while(target<active_end&&arc[target]<p.arc+lookahead){')
    text=exact(text,'    speed=std::min({speed,.85*c.wheel_v,.9*std::max(goal,remaining)});',
        '    speed=std::min({speed,.85*c.wheel_v,.9*std::max(goal,remaining)});\n'
        '    if(!finalRange(sc))speed=std::min(speed,.9*std::max(remaining,cusp_distance));')
    text=exact(text,'      double yaw_error=wrap((goal<.05?sc.path.back().yaw:p.yaw)-s.yaw);',
        '      double yaw_error=wrap((finalRange(sc)&&goal<.05?sc.path.back().yaw:p.yaw)-s.yaw);')
    text=exact(text,'      V pa=sc.path[p.index].p,pb=sc.path[p.index+1].p;\n'
        '      double f=clip((p.arc-arc[p.index])/std::max(1e-9,arc[p.index+1]-arc[p.index]),0,1);',
        '      const int next=std::min(p.index+1,rangeEnd(sc));\n'
        '      V pa=sc.path[p.index].p,pb=sc.path[next].p;\n'
        '      double f=clip((p.arc-arc[p.index])/std::max(1e-9,arc[next]-arc[p.index]),0,1);')
    text=exact(text,'    if(arc.back()-p.arc<.15){value.total+=30*goal*goal;',
        '    if(finalRange(sc)&&arc.back()-p.arc<.15){value.total+=30*goal*goal;')
    pairs=[
      ('    for(size_t i=0;i<route.path.size();++i){double d=norm(route.path[i].p-V{s.x,s.y});',
       '    for(size_t i=rangeBegin(route);i<=size_t(rangeEnd(route));++i){double d=norm(route.path[i].p-V{s.x,s.y});'),
      ('    for(size_t i=closest;i+1<route.path.size();++i)','    for(size_t i=closest;i<size_t(rangeEnd(route));++i)'),
      ('  virtual size_t chooseJoin(const S&,const Scene& sc)const{return sc.path.size()-1;}',
       '  virtual size_t chooseJoin(const S&,const Scene& sc)const{return rangeEnd(sc);}'),
      ('    V origin{s.x-mid*res,s.y-mid*res};size_t join=sc.path.size()-1;',
       '    V origin{s.x-mid*res,s.y-mid*res};size_t join=rangeEnd(sc);'),
      ('    Scene candidate=sc;candidate.path.clear();int gear=direction(sc,0);',
       '    Scene candidate=sc;clearDirectedRange(candidate);candidate.path.clear();int gear=direction(sc,rangeBegin(sc));'),
      ('    for(size_t k=join;k<sc.path.size();++k)candidate.path.push_back(sc.path[k]);',
       '    const int suffix_begin=int(candidate.path.size());\n'
       '    for(size_t k=join;k<sc.path.size();++k)candidate.path.push_back(sc.path[k]);\n'
       '    bindLocalRange(candidate,sc,suffix_begin,int(join),gear);'),
      ('    double most=0;for(const auto&r:candidate.path){V d=r.p-V{s.x,s.y};',
       '    double most=0;for(int i=rangeBegin(candidate);i<=rangeEnd(candidate);++i){const auto&r=candidate.path[i];V d=r.p-V{s.x,s.y};'),
      ('    for(size_t i=p.index;i<sc.path.size();++i)if(freeDistance(sc.path[i].p)<.04)',
       '    for(size_t i=p.index;i<=size_t(rangeEnd(sc));++i)if(freeDistance(sc.path[i].p)<.04)'),
      ('      for(size_t k=i;k<sc.path.size()&&a[k]<a[i]+.35;++k)',
       '      for(size_t k=i;k<=size_t(rangeEnd(sc))&&a[k]<a[i]+.35;++k)'),
      ('    return sc.path.size()-1;\n  }\n  Scene connector(',
       '    return rangeEnd(sc);\n  }\n  Scene connector('),
      ('    Scene out=sc;out.path.clear();V a{s.x,s.y},b=sc.path[j].p,d=b-a;',
       '    if(j<size_t(rangeBegin(sc))||j>size_t(rangeEnd(sc)))throw std::invalid_argument("connector outside phase");\n'
       '    Scene out=sc;clearDirectedRange(out);out.path.clear();V a{s.x,s.y},b=sc.path[j].p,d=b-a;'),
      ('    out.path.insert(out.path.end(),sc.path.begin()+j,sc.path.end());return out;',
       '    const int suffix_begin=int(out.path.size());\n'
       '    out.path.insert(out.path.end(),sc.path.begin()+j,sc.path.end());\n'
       '    bindLocalRange(out,sc,suffix_begin,int(j),direction(sc,rangeBegin(sc)));return out;'),
      ('    if(sc.path.size()<2)return 1e6;auto a=arcs(sc);return a.back()-project(s,sc,a).arc;',
       '    if(sc.path.size()<2)return 1e6;auto a=arcs(sc);return a[rangeEnd(sc)]-project(s,sc,a).arc;'),
      ('    bool blocked=false;for(size_t i=p.index;i<sc.path.size()&&a[i]<=p.arc+2.5;++i)',
       '    bool blocked=false;for(size_t i=p.index;i<=size_t(rangeEnd(sc))&&a[i]<=p.arc+2.5;++i)'),
    ]
    for old,new in pairs:text=exact(text,old,new)
    text=exact(text,'    for(size_t i=p.index;i<sc.path.size();++i){',
        '    for(size_t i=p.index;i<=size_t(rangeEnd(sc));++i){',2)
    # Existing empty warm-state access is unrelated to merits, but clearing
    # state at an association change must be safe. Guard only the empty case.
    text=exact(text,'  void feedback(V actual){double fraction=',
        '  void feedback(V actual){if(warm.empty())return;double fraction=')
    return text


def transform_v12(text,objective):
    text=exact(text,'        while(look+1<int(sc.path.size())&&a[look]<p.arc+.35)++look;',
        '        while(look<rangeEnd(sc)&&a[look]<p.arc+.35)++look;')
    if objective:
        # Stage merit AND stage proposal share one target. The full bundle's
        # final pose is still in sc.path.back(); task completion is external.
        text=text.replace('sc.path.back()','sc.path[rangeEnd(sc)]')
    else:
        text=exact(text,'    if(sc.path.size()<2)return false;',
            '    if(sc.path.size()<2||!finalRange(sc))return false;')
    return text


CLEAR=r'''
void clearCuspAssociation(Online&h){
  auto&m=h.m;m.warm.clear();m.route=Scene{};m.published_route.clear();
  m.side=m.cycles=m.clear_ticks=m.strategy=m.phase=0;m.failed=false;
  m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;
  m.d_lateral_needed=false;m.d_using_new=false;m.referenceChanged();
  m.search_trace.clear();m.status={};m.debug={};m.avoidance_debug={};
  m.decision_debug={};m.recovery_debug={};m.margin_debug={};
}
'''
CURSOR=r'''
    std::vector<mppi_cusp::Sample>phase_reference;phase_reference.reserve(sc.path.size());
    for(const auto&r:sc.path)phase_reference.push_back({r.p.x,r.p.y,r.yaw,r.v});
    if(h.cursor.update(phase_reference,{s.x,s.y,s.yaw,s.v,s.w}))clearCuspAssociation(h);
    const auto&active=h.cursor.active();sc.directed=true;
    sc.active_begin=active.begin;sc.active_end=active.end;sc.active_gear=active.gear;
'''

GATED_CURSOR=r'''
    std::vector<mppi_cusp::Sample>phase_reference;phase_reference.reserve(sc.path.size());
    for(const auto&r:sc.path)phase_reference.push_back({r.p.x,r.p.y,r.yaw,r.v});
    const bool cursor_changed=h.cursor.update(phase_reference,{s.x,s.y,s.yaw,s.v,s.w});
    double association[14];h.cursor.debug(association);
    const auto&active=h.cursor.active();const auto&cusp=sc.path[active.end];
    // This is an association gate, NOT a route/collision certificate. Do not
    // force a blocked historical cusp to remain an obligatory local target.
    const S cusp_pose{cusp.p.x,cusp.p.y,cusp.yaw,0.,0.};
    h.cusp_gate_gap=clearance(cusp_pose,predict(z,0.,h.c),h.c,sc.ymax,h.c.margin);
    const bool cusp_free=h.cusp_gate_gap>=0.;
    const bool near_cusp=association[6]<=.20&&association[8]<=.20;
    const bool bind=cusp_free&&(h.cusp_bound||association[10]==2.||near_cusp);
    if(bind!=h.cusp_bound||(bind&&cursor_changed))clearCuspAssociation(h);
    h.cusp_bound=bind;
    if(bind){sc.directed=true;sc.active_begin=active.begin;sc.active_end=active.end;sc.active_gear=active.gear;}
'''


def transform_api(text,gated=False):
    text=exact(text,'#include "online_kernel.hpp"','#include "online_kernel.hpp"\n#include "mppi_cusp_cursor.hpp"')
    text=exact(text,'struct Online {V actual{};',
        'struct Online {V actual{};mppi_cusp::Cursor cursor;'+('bool cusp_bound=false;double cusp_gate_gap=0.;' if gated else ''))
    text=exact(text,'}\nextern "C" void* online_create(',CLEAR+'}\nextern "C" void* online_create(')
    if gated:
        text=exact(text,'    auto start=std::chrono::steady_clock::now();auto u=h.m.plan(s,z,sc);',
            GATED_CURSOR+'    auto start=std::chrono::steady_clock::now();auto u=h.m.plan(s,z,sc);')
    else:
        text=exact(text,'    Z z;z.t=std::max(0.,age);z.stamp=0;z.fresh=true;',
            CURSOR+'    Z z;z.t=std::max(0.,age);z.stamp=0;z.fresh=true;')
    text=exact(text,'extern "C" void online_reset(void*p){if(p)static_cast<Online*>(p)->m.reset();}',
        'extern "C" void online_reset(void*p){if(p){auto&h=*static_cast<Online*>(p);h.cursor.reset();h.m.reset();}}')
    old='''extern "C" void online_reference_changed(void*p){
  if(p){auto&m=static_cast<Online*>(p)->m;m.route.path.clear();m.published_route.clear();m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;m.referenceChanged();}
}'''
    text=exact(text,old,'''extern "C" void online_reference_changed(void*p){
  if(p){auto&h=*static_cast<Online*>(p);h.cursor.reset();clearCuspAssociation(h);}
}''')
    text+='\nextern "C" void online_cusp_debug(void*p,double*out){if(p&&out)static_cast<Online*>(p)->cursor.debug(out);}\n'
    if gated:
        text=text.replace('h.cursor.reset();h.m.reset();','h.cursor.reset();h.cusp_bound=false;h.m.reset();')
        text=text.replace('h.cursor.reset();clearCuspAssociation(h);','h.cursor.reset();h.cusp_bound=false;clearCuspAssociation(h);')
        text+='\nextern "C" int online_cusp_bound(void*p){return p&&static_cast<Online*>(p)->cusp_bound;}\n'
        text+='\nextern "C" double online_cusp_gate_gap(void*p){return p?static_cast<Online*>(p)->cusp_gate_gap:0.;}\n'
    return text
