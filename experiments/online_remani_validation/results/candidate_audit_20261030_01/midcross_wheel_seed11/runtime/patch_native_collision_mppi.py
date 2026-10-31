#!/usr/bin/env python3
"""Pure source transforms for the isolated native-3D collision child build.

No writes, production edits, control-mode changes, or ROS point-query RPCs.
Unexpected parent source is rejected instead of silently applying a partial patch.
"""


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('native3d patch anchor count != 1: ' + old[:100])
    return text.replace(old, new, 1)


def region(text, begin, end, replacement):
    if text.count(begin) != 1 or text.count(end) != 1:
        raise ValueError('native3d region anchor mismatch: ' + begin)
    start = text.index(begin)
    finish = text.index(end, start)
    return text[:start] + replacement + text[finish:]


def patch_kernel(text):
    legacy_region = text[text.index('double clearance('):text.index('struct Constraint')]
    legacy_region = legacy_region.replace('clearance(', 'legacyClearance(').replace('barrier(', 'legacyBarrier(')
    text = once(text, '#include <plan_env/dynamic_point_buffer.h>',
                '#include <plan_env/dynamic_point_buffer.h>\n#include "native_collision_3d.hpp"')
    text = once(text, 'struct O{V p,vel;double r=.23;bool moving=true;};',
                'struct O{V p,vel;double r=.23;bool moving=true;double absolute_query_time=0;};')
    text = once(text, 'std::vector<O> predict(const Z&z,double ahead,const C&c){\n  auto os=z.os;', r'''bool dynamicForecastRequired(const Z&z){return native3d::enabled()?native3d::hasDynamic():!z.os.empty();}
double forecastAge(const Z&z){return native3d::enabled()?native3d::predictionAge():std::max(0.,z.t-z.stamp);}
double forecastHorizon(const C&c){return native3d::enabled()?native3d::maxPrediction():c.max_prediction;}
std::vector<O> predict(const Z&z,double ahead,const C&c){
  auto os=z.os;
  if(native3d::enabled()){
    for(auto&o:os){if(o.moving)o.p=o.p+(forecastAge(z)+ahead)*o.vel;
      o.absolute_query_time=native3d::now()+ahead;}
    // An empty XY proposal list cannot suppress actual 3D dynamic safety.
    // This time carrier has NO obstacle geometry, velocity, or radius.
    if(os.empty())os.push_back({{1e9,1e9},{0,0},0.,false,native3d::now()+ahead});
    return os;
  }''')
    text = once(text, '  }return os;\n}\ndouble clearance',
                '  }for(auto&o:os)o.absolute_query_time=native3d::now()+ahead;return os;\n}\ndouble clearance')
    text = region(text, 'double clearance(', 'struct Constraint', legacy_region + r'''// Native geometry is authoritative. XY observations remain proposal/mode hints
// only; their disk radii, map field and the legacy c.margin never grant/reject
// collision authority. An empty list explicitly requests static-only guidance.
double clearance(const S&s,const std::vector<O>&os,const C&c,double ymax,double margin=0){
  if(!native3d::enabled())return legacyClearance(s,os,c,ymax,margin);
  const bool dynamic=!os.empty();const double t=dynamic?os.front().absolute_query_time:native3d::now();
  return native3d::query(s.x,s.y,s.yaw,t,dynamic,true).gap;
}
double nativeCollisionCost(const S&s,const std::vector<O>&os){
  const bool dynamic=!os.empty();const double t=dynamic?os.front().absolute_query_time:native3d::now();
  return native3d::query(s.x,s.y,s.yaw,t,dynamic).cost;
}
std::vector<double> barrier(const S&s,const Z&z,double ahead,const C&c,double ymax){
  if(!native3d::enabled())return legacyBarrier(s,z,ahead,c,ymax);
  return {clearance(s,predict(z,ahead,c),c,ymax)};
}
''')
    text = once(text, 'double collision=d<0?1600+9000*d*d:0,near=std::max(0.,.04-d);',
                'double collision=native3d::enabled()?(d<0?1e12:nativeCollisionCost(s,nominal[k])):(d<0?1600+9000*d*d:0),near=std::max(0.,.04-d);')
    text = once(text, 'double risk_cost=2*std::pow(clip(-risk,0.,.5),2)+15*near*near;',
                'double risk_cost=native3d::enabled()?0.:2*std::pow(clip(-risk,0.,.5),2)+15*near*near;')
    text = once(text, 'virtual double freeDistance(V p)const{return map_distance?map_distance(p.x,p.y)-envelope-c.margin:1e3;}',
                'virtual double freeDistance(V p)const{return native3d::enabled()?native3d::query(p.x,p.y,0.,native3d::now(),false).gap:(map_distance?map_distance(p.x,p.y)-envelope-c.margin:1e3);}')
    envelope = 'envelope=0;for(const auto&b:c.body)envelope=std::max(envelope,norm(b.p)+b.r);'
    if text.count(envelope) != 2:
        raise ValueError('native3d expected two legacy envelope assignments')
    text = text.replace(envelope, 'if(native3d::enabled()){envelope=native3d::planarExtent();}else{'+envelope+'}')
    text = once(text, 'cost+=c.rdt*(2*p.distance*p.distance+10*reverse*reverse+20*std::pow(std::max(0.,.04-d),2));',
                'cost+=c.rdt*(2*p.distance*p.distance+10*reverse*reverse+(native3d::enabled()?nativeCollisionCost(s,predict(z,(k+1)*c.rdt,c)):20*std::pow(std::max(0.,.04-d),2)));')
    text = once(text, 'if(d<0)cost+=c.rdt*(1600+9000*d*d);', 'if(d<0)cost+=native3d::enabled()?1e12:c.rdt*(1600+9000*d*d);')
    text = once(text, 'bool restore=z.os.empty() && previous_gap<0 && previous_gap+c.margin>1e-6;',
                'bool restore=!native3d::enabled() && z.os.empty() && previous_gap<0 && previous_gap+c.margin>1e-6;')
    text = once(text, 'm.cost+=.5*c.rdt*40*std::pow(std::max(0.,.18-d),2);',
                'm.cost+=.5*c.rdt*(native3d::enabled()?nativeCollisionCost(s,predict(z,t,c)):40*std::pow(std::max(0.,.18-d),2));')
    text = once(text, '''    m.cost+=200*std::pow(std::max(0.,.03+closing*response-last_gap),2)
      +30*std::pow(std::max(0.,.18-last_gap),2);''',
                '''    if(native3d::enabled())m.cost+=200*std::pow(std::max(0.,.03+closing*response-last_gap),2);
    else m.cost+=200*std::pow(std::max(0.,.03+closing*response-last_gap),2)
      +30*std::pow(std::max(0.,.18-last_gap),2);''')
    text = once(text, 'bool restoring=z.os.empty() && static_gap<0 && static_gap+c.margin>1e-6;',
                'bool restoring=!native3d::enabled() && z.os.empty() && static_gap<0 && static_gap+c.margin>1e-6;')
    # Forecast validity belongs to the actual 3D points, not XY mode hints.
    text = text.replace('z.os.empty()?30:', '!dynamicForecastRequired(z)?30:')
    text = text.replace('c.max_prediction-std::max(0.,z.t-z.stamp)', 'forecastHorizon(c)-forecastAge(z)')
    text = once(text, 'for(size_t j=0;j<future.size();++j)if(z.os[j].moving)',
                'for(size_t j=0;j<future.size()&&j<z.os.size();++j)if(z.os[j].moving)')
    return text


def patch_mppi(text):
    text = once(text, 'm.cost+=c.rdt/2*70*std::pow(std::max(0.,.25-gap),2);',
                'm.cost+=c.rdt/2*(native3d::enabled()?nativeCollisionCost(s,future[2*k+j+1]):70*std::pow(std::max(0.,.25-gap),2));')
    text = once(text, '''    m.cost+=250*std::pow(std::max(0.,.06+closing*response-last),2)+
      60*std::pow(std::max(0.,.22-last),2);''',
                '''    if(native3d::enabled())m.cost+=250*std::pow(std::max(0.,.06+closing*response-last),2);
    else m.cost+=250*std::pow(std::max(0.,.06+closing*response-last),2)+
      60*std::pow(std::max(0.,.22-last),2);''')
    text = once(text, 'double extent=0;for(const auto&b:c.body)extent=std::max(extent,norm(b.p)+b.r);',
                'double extent=0;if(native3d::enabled())extent=native3d::planarExtent();else for(const auto&b:c.body)extent=std::max(extent,norm(b.p)+b.r);')
    text = once(text, 'double width=0;for(const auto&b:c.body)width=std::max(width,norm(b.p)+b.r);',
                'double width=0;if(native3d::enabled())width=native3d::planarExtent();else for(const auto&b:c.body)width=std::max(width,norm(b.p)+b.r);')
    text = once(text, '''        for(const auto&b:c.body)
          if(norm(bodypoint(continuing,b.p)-o.p)-b.r-o.r-c.margin<.20){d_lateral_needed=true;break;}''',
                '''        if(native3d::enabled()){
          if(native3d::query(continuing.x,continuing.y,continuing.yaw,
               o.absolute_query_time,true).dynamic_gap<.20){d_lateral_needed=true;break;}
        }else{
          for(const auto&b:c.body)
            if(norm(bodypoint(continuing,b.p)-o.p)-b.r-o.r-c.margin<.20){d_lateral_needed=true;break;}
        }''')
    text = once(text, 'double valid=c.max_prediction-std::max(0.,z.t-z.stamp)-.01;',
                'double valid=forecastHorizon(c)-forecastAge(z)-.01;')
    return text


def patch_api(text):
    text = once(text, 'struct Online {V actual{};',
                'struct Online {native3d::State native_collision;V actual{};')
    text = once(text, 'auto&h=*static_cast<Online*>(ptr);grid=(h.classified_static',
                'auto&h=*static_cast<Online*>(ptr);native3d::Scope native_scope(h.native_collision);if(native3d::enabled()&&!native_scope.valid())return 7;grid=(h.classified_static')
    text = once(text, 'auto&h=*static_cast<Online*>(ptr);grid=field;nx=width;',
                'auto&h=*static_cast<Online*>(ptr);native3d::Scope native_scope(h.native_collision);if(native3d::enabled()&&!native_scope.valid())return 7;grid=field;nx=width;')
    text = once(text, 'extern "C" double online_pose_clearance(void*p,double x,double y,double yaw){return static_cast<Online*>(p)->m.poseDistance({x,y},yaw);}',
                'extern "C" double online_pose_clearance(void*p,double x,double y,double yaw){if(!p)return -1e6;auto&h=*static_cast<Online*>(p);native3d::Scope native_scope(h.native_collision);return h.m.poseDistance({x,y},yaw);}')
    text = once(text, 'auto&h=*static_cast<Online*>(p);auto&c=h.c;',
                'auto&h=*static_cast<Online*>(p);native3d::Scope native_scope(h.native_collision);if(native3d::enabled()&&!native_scope.valid())return 7;auto&c=h.c;const bool dynamic_present=native3d::enabled()?native3d::hasDynamic():no>0;const double prediction_horizon=native3d::enabled()?native3d::maxPrediction():c.max_prediction;if(native3d::enabled())age=native3d::predictionAge();')
    text = once(text, 'if(no>0)count=std::min(count,int(std::floor((c.max_prediction-age-.02)/c.rdt)));',
                'if(dynamic_present)count=std::min(count,int(std::floor((prediction_horizon-age-.02)/c.rdt)));')
    text = once(text, 'if(no>0&&age+t>c.max_prediction-1e-6)return 4;',
                'if(dynamic_present&&age+t>prediction_horizon-1e-6)return 4;')
    text = once(text, 'if(no>0&&age+t>c.max_prediction-1e-6)break;',
                'if(dynamic_present&&age+t>prediction_horizon-1e-6)break;')
    text = once(text, 'bool restoration=no==0&&initial_gap<0&&initial_gap+c.margin>1e-6;',
                'bool restoration=!native3d::enabled()&&no==0&&initial_gap<0&&initial_gap+c.margin>1e-6;')
    text += r'''

// Native-3D-only authority: bind exact raw ESDF/robot/dynamic-point snapshot.
// A failed bind invalidates this handle; online_step/validate cannot fall back.
extern "C" int online_set_native_collision_snapshot(void*p,const double*data,int count){
  if(!p)return 1;auto&state=static_cast<Online*>(p)->native_collision;
  try{return count>=0&&native3d::bind(state,data,std::size_t(count))?0:2;}
  catch(...){state.snapshot.reset();return 3;}
}
extern "C" int online_set_native_collision_clock(void*p,double absolute_time){
  if(!p||!std::isfinite(absolute_time)||absolute_time<=0)return 1;
  static_cast<Online*>(p)->native_collision.now=absolute_time;return 0;
}
// Diagnostic numerical parity API. Not execution authorization.
extern "C" int online_native_collision_query(void*p,double x,double y,double yaw,
    double absolute_time,int dynamic,double*out){
  if(!p||!out)return 1;auto&state=static_cast<Online*>(p)->native_collision;
  native3d::Scope scope(state);auto q=native3d::query(x,y,yaw,absolute_time,dynamic!=0);
  out[0]=q.gap;out[1]=q.cost;out[2]=q.static_gap;out[3]=q.dynamic_gap;
  out[4]=q.valid?1.:0.;return q.valid?0:2;
}
extern "C" int online_native3d_query(void*p,double x,double y,double yaw,
    double absolute_time,int static_only,double*out){
  return online_native_collision_query(p,x,y,yaw,absolute_time,static_only?0:1,out);
}
'''
    return text


def patch_sources(sources):
    """Return patched copies; caller owns child-build preparation/writes."""
    result = dict(sources)
    result['online_kernel.hpp'] = patch_kernel(result['online_kernel.hpp'])
    result['mppi_v12.hpp'] = patch_mppi(result['mppi_v12.hpp'])
    result['online_api.cpp'] = patch_api(result['online_api.cpp'])
    return result
