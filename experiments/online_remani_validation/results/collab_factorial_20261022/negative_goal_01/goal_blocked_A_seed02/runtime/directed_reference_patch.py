"""Exact build-09 transforms; never edit archived/runtime/production sources.

The cursor is updated ONCE at the actual-state C API ingress. Native references
remain complete, with their real global goal. Controller candidates receive
only an immutable direction/range context, not the mutable Cursor. Local guides
retain full suffixes but have a separately mapped local range/index space.
"""


def exact(text, old, new, count=1):
    found = text.count(old)
    if found != count:
        raise ValueError("directed patch expected %d matches, got %d: %r" %
                         (count, found, old[:110]))
    return text.replace(old, new)


RANGE_HELPERS = r'''
// Inclusive bounds in THIS Scene's own index space. A local connector never
// inherits the full-native cursor's indices. Neither context changes in plan().
inline int rangeBegin(const Scene&sc){
  if(sc.path.empty())throw std::invalid_argument("empty reference range");
  const int begin=sc.directed?sc.active_begin:0;
  if(begin<0||begin>=int(sc.path.size()))throw std::invalid_argument("invalid reference begin");
  return begin;
}
inline int rangeEnd(const Scene&sc){
  const int end=sc.directed?sc.active_end:sc.local_end>=0?sc.local_end:int(sc.path.size())-1;
  if(end<rangeBegin(sc)||end>=int(sc.path.size()))throw std::invalid_argument("invalid reference end");
  return end;
}
inline bool finalRange(const Scene&sc){return rangeEnd(sc)==int(sc.path.size())-1;}
inline bool limitedRange(const Scene&sc){return sc.directed||sc.local_end>=0;}
inline void clearDirectedRange(Scene&sc){
  sc.directed=false;sc.active_begin=0;sc.active_end=-1;sc.active_gear=0;
  sc.local_end=-1;sc.local_gear=0;
}
inline void bindLocalRange(Scene&out,const Scene&native,int local_suffix_begin,int native_join,int gear){
  const int native_end=rangeEnd(native);const bool limited=limitedRange(native);
  if(native_join<rangeBegin(native)||native_join>native_end)
    throw std::invalid_argument("connector join outside active direction phase");
  clearDirectedRange(out);
  if(limited){
    out.local_end=local_suffix_begin+native_end-native_join;out.local_gear=gear;
    if(gear!=1&&gear!=-1)throw std::invalid_argument("invalid local reference gear");
    (void)rangeEnd(out);
  }
}
'''


PROJECT = r'''  Projection project(const S& s,const Scene& sc,const std::vector<double>& arc,int hint=-1)const {
    const int begin=rangeBegin(sc),end=rangeEnd(sc);
    Projection out{begin,1e100,arc[begin],sc.path[begin].yaw,sc.path[begin].v};
    double best_fraction=0.;bool found=false;
    int lo=hint<0?begin:std::max(begin,std::min(end,hint)-8);
    int hi=hint<0?end:std::min(end,std::max(begin,hint)+45);
    for(int i=lo;i<hi;++i){V a=sc.path[i].p,b=sc.path[i+1].p,d=b-a;double length=arc[i+1]-arc[i];if(length<1e-9)continue;
      double f=clip(dot(V{s.x,s.y}-a,d)/(length*length),0,1);V delta=V{s.x,s.y}-(a+f*d);double dist=dot(delta,delta);
      if(dist<out.distance){out={i,dist,arc[i]+f*length,0,0};best_fraction=f;found=true;}
    }
    if(!found){
      // A stationary/single-vertex phase is still a valid bounded projection.
      for(int i=lo;i<=hi;++i){V delta=V{s.x,s.y}-sc.path[i].p;double dist=dot(delta,delta);
        if(dist<out.distance)out={i,dist,arc[i],sc.path[i].yaw,sc.path[i].v};}
      out.distance=std::sqrt(out.distance);return out;
    }
    out.distance=std::sqrt(out.distance);int i=out.index;double f=best_fraction;
    out.yaw=wrap(sc.path[i].yaw+f*wrap(sc.path[i+1].yaw-sc.path[i].yaw));out.speed=(1-f)*sc.path[i].v+f*sc.path[i+1].v;
    return out;
  }
'''


def transform_kernel(text):
    text = exact(text, "  double velocity_error=0;\n};\nScene scene(",
                 "  double velocity_error=0;\n"
                 "  bool directed=false;int active_begin=0,active_end=-1,active_gear=0;\n"
                 "  int local_end=-1,local_gear=0; // Independent connector indices.\n};\n" +
                 RANGE_HELPERS + "\nScene scene(")
    start = "  Projection project(const S& s,const Scene& sc,const std::vector<double>& arc,int hint=-1)const {"
    stop = "  int direction(const Scene& sc,int i)const {"
    if text.count(start) != 1 or text.count(stop) != 1:
        raise ValueError("ambiguous project/direction boundaries")
    begin = text.index(start)
    end = text.index(stop, begin)
    expected = '''  Projection project(const S& s,const Scene& sc,const std::vector<double>& arc,int hint=-1)const {
    Projection out{0,1e9,0,0,0};double best_fraction=0;
    int lo=hint<0?0:std::max(0,hint-8),hi=hint<0?int(sc.path.size())-1:std::min(int(sc.path.size())-1,hint+45);
    for(int i=lo;i<hi;++i){V a=sc.path[i].p,b=sc.path[i+1].p,d=b-a;double length=arc[i+1]-arc[i];if(length<1e-9)continue;
      double f=clip(dot(V{s.x,s.y}-a,d)/(length*length),0,1);V delta=V{s.x,s.y}-(a+f*d);double dist=dot(delta,delta);
      if(dist<out.distance){out={i,dist,arc[i]+f*length,0,0};best_fraction=f;}
    }
    out.distance=std::sqrt(out.distance);int i=out.index;double f=best_fraction;
    out.yaw=wrap(sc.path[i].yaw+f*wrap(sc.path[i+1].yaw-sc.path[i].yaw));out.speed=(1-f)*sc.path[i].v+f*sc.path[i+1].v;
    return out;
  }
'''
    if text[begin:end] != expected:
        raise ValueError("project body differs from verified build-09 source")
    text = text[:begin] + PROJECT + text[end:]
    text = exact(text, stop, stop + '''
    if(sc.directed){if(sc.active_gear!=1&&sc.active_gear!=-1)throw std::invalid_argument("invalid native gear");return sc.active_gear;}
    if(sc.local_end>=0){if(sc.local_gear!=1&&sc.local_gear!=-1)throw std::invalid_argument("invalid local gear");return sc.local_gear;}''')
    text = exact(text, "    double remaining=arc.back()-p.arc,goal=norm(sc.path.back().p-V{s.x,s.y});",
                 "    const int active_end=rangeEnd(sc);\n"
                 "    double remaining=std::max(0.,arc[active_end]-p.arc),goal=norm(sc.path.back().p-V{s.x,s.y});\n"
                 "    const double cusp_distance=norm(sc.path[active_end].p-V{s.x,s.y});\n"
                 "    if(!finalRange(sc)&&remaining<=.05&&cusp_distance<=.05)return {}; // Brake/hold; only actual API cursor may switch.")
    text = exact(text, "    if(goal<.025)return bound({0,1.8*wrap(sc.path.back().yaw-s.yaw)},c);",
                 "    if(finalRange(sc)&&goal<.025)return bound({0,1.8*wrap(sc.path.back().yaw-s.yaw)},c);")
    text = exact(text, "    while(target+1<int(sc.path.size())&&arc[target]<p.arc+lookahead){",
                 "    while(target<active_end&&arc[target]<p.arc+lookahead){")
    text = exact(text, "    speed=std::min({speed,.85*c.wheel_v,.9*std::max(goal,remaining)});",
                 "    speed=std::min({speed,.85*c.wheel_v,.9*std::max(goal,remaining)});\n"
                 "    if(!finalRange(sc))speed=std::min(speed,.9*std::max(remaining,cusp_distance)); // Off-path poses still reconnect; no predicted gear switch.")
    text = exact(text, "      double yaw_error=wrap((goal<.05?sc.path.back().yaw:p.yaw)-s.yaw);",
                 "      double yaw_error=wrap((finalRange(sc)&&goal<.05?sc.path.back().yaw:p.yaw)-s.yaw);")
    text = exact(text, "      V pa=sc.path[p.index].p,pb=sc.path[p.index+1].p;\n"
                 "      double f=clip((p.arc-arc[p.index])/std::max(1e-9,arc[p.index+1]-arc[p.index]),0,1);",
                 "      const int next=std::min(p.index+1,rangeEnd(sc));\n"
                 "      V pa=sc.path[p.index].p,pb=sc.path[next].p;\n"
                 "      double f=clip((p.arc-arc[p.index])/std::max(1e-9,arc[next]-arc[p.index]),0,1);")
    text = exact(text, "    if(arc.back()-p.arc<.15){value.total+=30*goal*goal;",
                 "    if(finalRange(sc)&&arc.back()-p.arc<.15){value.total+=30*goal*goal;")
    text = exact(text, "    for(size_t i=0;i<route.path.size();++i){double d=norm(route.path[i].p-V{s.x,s.y});",
                 "    for(size_t i=rangeBegin(route);i<=size_t(rangeEnd(route));++i){double d=norm(route.path[i].p-V{s.x,s.y});")
    text = exact(text, "    for(size_t i=closest;i+1<route.path.size();++i)",
                 "    for(size_t i=closest;i<size_t(rangeEnd(route));++i)")
    text = exact(text, "  virtual size_t chooseJoin(const S&,const Scene& sc)const{return sc.path.size()-1;}",
                 "  virtual size_t chooseJoin(const S&,const Scene& sc)const{return rangeEnd(sc);}")
    text = exact(text, "    V origin{s.x-mid*res,s.y-mid*res};size_t join=sc.path.size()-1;",
                 "    V origin{s.x-mid*res,s.y-mid*res};size_t join=rangeEnd(sc);")
    text = exact(text, "    Scene candidate=sc;candidate.path.clear();int gear=direction(sc,project(s,sc,native_arc).index);",
                 "    Scene candidate=sc;clearDirectedRange(candidate);candidate.path.clear();int gear=direction(sc,project(s,sc,native_arc).index);")
    text = exact(text, "    for(size_t k=join;k<sc.path.size();++k)candidate.path.push_back(sc.path[k]);",
                 "    const int suffix_begin=int(candidate.path.size());\n"
                 "    for(size_t k=join;k<sc.path.size();++k)candidate.path.push_back(sc.path[k]);\n"
                 "    bindLocalRange(candidate,sc,suffix_begin,int(join),gear);")
    text = exact(text, "    double most=0;for(const auto&r:candidate.path){V d=r.p-V{s.x,s.y};",
                 "    double most=0;for(int i=rangeBegin(candidate);i<=rangeEnd(candidate);++i){const auto&r=candidate.path[i];V d=r.p-V{s.x,s.y};")
    text = exact(text, "    for(size_t i=p.index;i<sc.path.size();++i)if(freeDistance(sc.path[i].p)<.04)",
                 "    for(size_t i=p.index;i<=size_t(rangeEnd(sc));++i)if(freeDistance(sc.path[i].p)<.04)")
    text = exact(text, "    for(size_t i=p.index;i<sc.path.size();++i){", "    for(size_t i=p.index;i<=size_t(rangeEnd(sc));++i){", count=2)
    text = exact(text, "      for(size_t k=i;k<sc.path.size()&&a[k]<a[i]+.35;++k)",
                 "      for(size_t k=i;k<=size_t(rangeEnd(sc))&&a[k]<a[i]+.35;++k)")
    text = exact(text, "    return sc.path.size()-1;\n  }\n  Scene connector(",
                 "    return rangeEnd(sc);\n  }\n  Scene connector(")
    text = exact(text, "    Scene out=sc;out.path.clear();V a{s.x,s.y},b=sc.path[j].p,d=b-a;",
                 "    if(j<size_t(rangeBegin(sc))||j>size_t(rangeEnd(sc)))throw std::invalid_argument(\"connector outside native phase\");\n"
                 "    Scene out=sc;clearDirectedRange(out);out.path.clear();V a{s.x,s.y},b=sc.path[j].p,d=b-a;")
    text = exact(text, "    out.path.insert(out.path.end(),sc.path.begin()+j,sc.path.end());return out;",
                 "    const int suffix_begin=int(out.path.size());\n"
                 "    out.path.insert(out.path.end(),sc.path.begin()+j,sc.path.end());\n"
                 "    bindLocalRange(out,sc,suffix_begin,int(j),direction(sc,rangeBegin(sc)));return out;")
    text = exact(text, "    if(sc.path.size()<2)return 1e6;auto a=arcs(sc);return a.back()-project(s,sc,a).arc;",
                 "    if(sc.path.size()<2)return 1e6;auto a=arcs(sc);return a[rangeEnd(sc)]-project(s,sc,a).arc;")
    text = exact(text, "    bool blocked=false;for(size_t i=p.index;i<sc.path.size()&&a[i]<=p.arc+2.5;++i)",
                 "    bool blocked=false;for(size_t i=p.index;i<=size_t(rangeEnd(sc))&&a[i]<=p.arc+2.5;++i)")
    return text


def transform_v12(text):
    text = exact(text, "  bool goalCapture(const S&s,const Scene&sc)const{\n    if(sc.path.size()<2)return false;",
                 "  bool goalCapture(const S&s,const Scene&sc)const{\n"
                 "    if(sc.path.size()<2)return false;\n"
                 "    if(!finalRange(sc))return false; // Cusp is NOT the true task goal.")
    text = exact(text, "        while(look+1<int(sc.path.size())&&a[look]<p.arc+.35)++look;",
                 "        while(look<rangeEnd(sc)&&a[look]<p.arc+.35)++look;")
    return text


PHASE_CLEAR = r'''
// A phase switch is a reference association change, not an actuator reset.
// In particular h.actual and the mutable Cursor are deliberately preserved.
void clearDirectedPhaseState(Online&h){
  auto&m=h.m;m.warm.clear();m.route=Scene{};m.published_route.clear();
  m.side=0;m.cycles=0;m.failed=false;m.clear_ticks=0;m.strategy=0;m.phase=0;
  m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;
  m.d_lateral_needed=false;m.d_using_new=false;m.referenceChanged();
  m.search_trace.clear();m.status={};m.debug={};m.avoidance_debug={};
  m.decision_debug={};m.recovery_debug={};m.margin_debug={};
}
'''


API_CURSOR = r'''
    std::vector<directed_reference::Sample>cursor_reference;cursor_reference.reserve(sc.path.size());
    for(const auto&r:sc.path)cursor_reference.push_back({r.p.x,r.p.y,r.yaw,r.v});
    // The ONLY mutable cursor update. Candidates below have only this Scene's
    // value snapshot, and cannot see or advance h.cursor.
    const auto cursor_event=h.cursor.update(cursor_reference,{s.x,s.y,s.v});
    const auto&directed_phase=h.cursor.active();
    sc.directed=true;sc.active_begin=directed_phase.begin;sc.active_end=directed_phase.end;
    sc.active_gear=directed_phase.gear;sc.local_end=-1;sc.local_gear=0;
    if(cursor_event.started||cursor_event.switched){
      clearDirectedPhaseState(h);
      auto collision=shared_collision::snapshot();
      double event_time=collision?collision->stamp+std::max(0.,age)-1000.:std::numeric_limits<double>::quiet_NaN();
      std::ostringstream line;line<<std::setprecision(17)<<"[DirectedReference] t="<<event_time
        <<" event="<<(cursor_event.started?"phase_start":"phase_switch")
        <<" ref_fnv64="<<std::hex<<h.cursor.reference_hash()<<std::dec
        <<" from="<<cursor_event.previous_phase<<" phase="<<h.cursor.phase_id()
        <<" count="<<h.cursor.phase_count()<<" gear="<<sc.active_gear
        <<" begin="<<sc.active_begin<<" end="<<sc.active_end
        <<" measured="<<s.x<<","<<s.y<<","<<s.yaw<<","<<s.v<<","<<s.w
        <<" checked_cusp_distance="<<cursor_event.checked_cusp_distance;
      std::cout<<line.str()<<std::endl;
    }
'''


def transform_api(text):
    text = exact(text, '#include "online_kernel.hpp"',
                 '#include "online_kernel.hpp"\n#include "directed_reference.hpp"\n#include <limits>')
    text = exact(text, "struct Online {V actual{};", "struct Online {V actual{};directed_reference::Cursor cursor;")
    text = exact(text, '}\nextern "C" void* online_create(', PHASE_CLEAR + '}\nextern "C" void* online_create(')
    text = exact(text, "    Z z;z.t=std::max(0.,age);z.stamp=0;z.fresh=true;", API_CURSOR +
                 "    Z z;z.t=std::max(0.,age);z.stamp=0;z.fresh=true;")
    text = exact(text, 'extern "C" void online_reset(void*p){if(p)static_cast<Online*>(p)->m.reset();}',
                 'extern "C" void online_reset(void*p){if(p){auto&h=*static_cast<Online*>(p);h.cursor.reset();h.m.reset();}}')
    text = exact(text, '''extern "C" void online_reference_changed(void*p){
  if(p){auto&m=static_cast<Online*>(p)->m;m.side=0;m.route.path.clear();m.published_route.clear();m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;m.referenceChanged();}
}''', '''extern "C" void online_reference_changed(void*p){
  if(p){auto&h=*static_cast<Online*>(p);h.cursor.reset();clearDirectedPhaseState(h);}
}''')
    text += '''
// Caller owns a buffer of 12 doubles; see directed_reference::Cursor::debug.
extern "C" void online_directed_cursor_debug(void*p,double*out){
  if(!out)return;if(!p){std::fill(out,out+12,0.);return;}
  static_cast<Online*>(p)->cursor.debug(out);
}
'''
    return text


MANIFEST_METADATA = {
    "cursor_update_scope": "online_step actual measurement only; no predicted updates",
    "native_context": "full-reference inclusive active_begin/active_end/active_gear",
    "local_context": "cleared native metadata; independent local_end/local_gear mapping",
    "global_goal_source": "complete received native bundle last pose, not an external task-goal API",
    "phase_switch_position_m": .05,
    "phase_switch_linear_speed_mps": .025,
    "phase_switch_angular_gate": False,
    "phase_switch_per_measured_step_max": 1,
    "nonfinal_endpoint_behavior": "geometric cusp capture brakes/holds; off-path endpoint projection retains attainable reconnect speed",
    "cursor_debug_getter": "online_directed_cursor_debug(void*, double* buffer12)",
    "cursor_debug_fields": ["ready", "phase", "phase_count", "active_begin", "active_end",
                            "active_gear", "current_cusp_distance", "transitions", "event",
                            "previous_phase", "reference_fnv_hi32", "reference_fnv_lo32"],
    "cursor_event_codes": {"none": 0, "phase_start": 1, "phase_switch": 2},
    "reference_identity": "canonical-double FNV64 telemetry; source SHA256 remains separate",
    "hard_safety": "existing footprint, hard margin, CV validity, rollout/braking validation unchanged",
}
