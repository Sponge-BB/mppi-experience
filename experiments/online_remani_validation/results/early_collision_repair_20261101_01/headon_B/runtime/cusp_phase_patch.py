#!/usr/bin/env python3
"""Default-off association-only intervention for current native MPPI sources."""
from pathlib import Path

HERE=Path(__file__).resolve().parent
SWITCH='MPPI_CUSP_PHASE_ASSOCIATION'


def exact(text,old,new):
    if text.count(old)!=1:raise ValueError('Missing/ambiguous cusp patch anchor: '+old[:90])
    return text.replace(old,new,1)


def transform_kernel(text):
    text=exact(text,'namespace experiment {','#include "cusp_phase_cursor.hpp"\n\nnamespace experiment {')
    old='''    Projection out{0,1e9,0,0,0};double best_fraction=0;
    int lo=hint<0?0:std::max(0,hint-8),hi=hint<0?int(sc.path.size())-1:std::min(int(sc.path.size())-1,hint+45);'''
    new='''    const auto* phase=cusp_phase::bound(sc.path.data(),sc.path.size());
    const int begin=phase?phase->begin:0,end=phase?phase->end:int(sc.path.size())-1;
    Projection out{begin,1e9,0,0,0};double best_fraction=0;
    int lo=hint<0?begin:std::max(begin,hint-8),hi=hint<0?end:std::min(end,hint+45);
    if(phase&&lo>=hi){lo=std::clamp(hint,begin,end-1);hi=lo+1;}'''
    text=exact(text,old,new)
    text=exact(text,'  int direction(const Scene& sc,int i)const {',
        '''  int direction(const Scene& sc,int i)const {
    if(const auto* phase=cusp_phase::bound(sc.path.data(),sc.path.size()))return phase->gear;''')
    text=exact(text,'    while(target+1<int(sc.path.size())&&arc[target]<p.arc+lookahead){',
        '''    const auto* phase=cusp_phase::bound(sc.path.data(),sc.path.size());
    const int association_end=phase?phase->end:int(sc.path.size())-1;
    while(target<association_end&&arc[target]<p.arc+lookahead){''')
    return text


def transform_v12(text):
    return exact(text,'        while(look+1<int(sc.path.size())&&a[look]<p.arc+.35)++look;',
        '''        const auto* phase=cusp_phase::bound(sc.path.data(),sc.path.size());
        const int association_end=phase?phase->end:int(sc.path.size())-1;
        while(look<association_end&&a[look]<p.arc+.35)++look;''')


def transform_api(text):
    text=exact(text,'struct Online {double execution_lease=.075;',
        'struct Online {const bool cusp_phase_enabled=cusp_phase::enabled();cusp_phase::Cursor cusp_phase_cursor;double execution_lease=.075;')
    anchor='''    native_realtime::Session native_realtime_session(h.realtime_probe,native3d::enabled());'''
    text=exact(text,anchor,'''    const cusp_phase::Phase* association=nullptr;
    if(h.cusp_phase_enabled){
      std::vector<cusp_phase::Sample> reference;reference.reserve(sc.path.size());
      for(const auto& r:sc.path)reference.push_back({r.p.x,r.p.y,r.yaw,r.v});
      if(h.cusp_phase_cursor.update(reference,{s.x,s.y,s.yaw,s.v,s.w}))h.m.warm.clear();
      association=&h.cusp_phase_cursor.active();
    }
    cusp_phase::Scope measured_association(sc.path.data(),sc.path.size(),association);
'''+anchor)
    text=exact(text,'extern "C" void online_reset(void*p){if(p)static_cast<Online*>(p)->m.reset();}',
        'extern "C" void online_reset(void*p){if(p){auto& h=*static_cast<Online*>(p);h.cusp_phase_cursor.reset();h.m.reset();}}')
    text=exact(text,'  if(p){auto&m=static_cast<Online*>(p)->m;m.route.path.clear();',
        '  if(p){auto& h=*static_cast<Online*>(p);h.cusp_phase_cursor.reset();auto&m=h.m;m.route.path.clear();')
    text+='''
extern "C" int online_cusp_phase_debug(void* p,double* out){
  if(!p||!out)return 0;auto& h=*static_cast<Online*>(p);
  if(h.cusp_phase_enabled)h.cusp_phase_cursor.debug(out);else std::fill(out,out+14,0.);
  return 14;
}
'''
    return text


def transform(source_map):
    result=dict(source_map)
    result['online_kernel.hpp']=transform_kernel(result['online_kernel.hpp'])
    result['mppi_v12.hpp']=transform_v12(result['mppi_v12.hpp'])
    result['online_api.cpp']=transform_api(result['online_api.cpp'])
    result['cusp_phase_cursor.hpp']=(HERE/'cusp_phase_cursor.hpp').read_text()
    return result
