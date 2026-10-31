#!/usr/bin/env python3
"""Exact reversible child transforms of native_collision_3d_20261027_04.

Only skip the native-3D free-path Base sampler when explicitly requested.
Conflict/dynamic/recovery policies and all collision/execution rules remain.
The optional probe is only active when PROFILE and REPLAY are both one.
This module performs no filesystem writes.
"""
import hashlib

PARENT_SHA256 = {
    'online_kernel.hpp': 'c6d6c5335b09c6dc0f425c937933345cd605b792bb107880920593f7dc0529b8',
    'online_api.cpp': '88ce6e287a28eb668b64c6229a5733e1c97adb391b2345cd810c2895b5701aa2',
    'native_collision_3d.hpp': '20055cc1fa6f62c9645b3bdee4ef67883f2dc94aed9b10d86b578a4b66e29411',
}

GETTER = r'''

// Read-only last online_step profile. Timing is only collected for an explicit
// offline PROFILE=1 AND REPLAY=1 session, never an execution approval.
extern "C" int online_native_realtime_profile(void*p,double*out,int capacity){
  if(!p)return -1;
  return native_realtime::read(static_cast<Online*>(p)->realtime_probe,out,capacity);
}
'''

TRANSFORMS = {
    'online_kernel.hpp': (
        ('  V plan(const S& s,const Z& z,const Scene& sc){\n',
         '  V plan(const S& s,const Z& z,const Scene& sc){\n    native_realtime::Timer native_realtime_base_timer(native_realtime::Tag::base);\n'),
        ('    const int samples=std::max(32,c.samples*c.horizon/horizon/proposal_divisor);',
         '    const int samples=std::max(32,c.samples*c.horizon/horizon/proposal_divisor);\n    native_realtime::baseWork(samples,horizon);'),
        ('  Merit evaluate(S s,const Z&z,const Scene&sc,const std::vector<double>&a,const std::vector<V>&us,bool prune=true)const{\n',
         '  Merit evaluate(S s,const Z&z,const Scene&sc,const std::vector<double>&a,const std::vector<V>&us,bool prune=true)const{\n    native_realtime::Timer native_realtime_v7_timer(native_realtime::Tag::v7);\n'),
        ('    if(nm.min_gap>.30&&!displaced){',
         '    if(nm.min_gap>.30&&!displaced&&!native_realtime::skipFreePath(native3d::enabled())){'),
    ),
    'native_collision_3d.hpp': (
        ('#pragma once\n', '#pragma once\n#include "native_realtime_probe.hpp"\n'),
        ('    if(!preciseDistance(*s.map,p,stat))return Evaluation{};',
         '    if(!preciseDistance(*s.map,p,stat)){native_realtime::bodyVisits(std::size_t(&body-s.body.data())+1);return Evaluation{};}'),
        ('      out.gap=std::min(out.static_gap,out.dynamic_gap);out.valid=true;return out;',
         '      out.gap=std::min(out.static_gap,out.dynamic_gap);out.valid=true;native_realtime::bodyVisits(std::size_t(&body-s.body.data())+1);return out;'),
        ('  out.gap=std::min(out.static_gap,out.dynamic_gap);out.valid=std::isfinite(out.gap)&&std::isfinite(out.cost);\n  return out;',
         '  out.gap=std::min(out.static_gap,out.dynamic_gap);out.valid=std::isfinite(out.gap)&&std::isfinite(out.cost);\n  native_realtime::bodyVisits(s.body.size());\n  return out;'),
        ('inline Evaluation query(double x,double y,double yaw,double absolute_time,bool dynamic=true,bool early_exit=false){\n',
         'inline Evaluation query(double x,double y,double yaw,double absolute_time,bool dynamic=true,bool early_exit=false){\n  native_realtime::Timer native_realtime_query_timer(native_realtime::Tag::query);\n'),
        ('      m.dynamic==dynamic&&(early_exit||m.complete))return m.value;',
         '      m.dynamic==dynamic&&(early_exit||m.complete)){native_realtime::cacheHit();return m.value;}'),
    ),
    'online_api.cpp': (
        ('struct Online {native3d::State native_collision;V actual{};',
         'struct Online {native_realtime::Stats realtime_probe;native3d::State native_collision;V actual{};'),
        ('    auto start=std::chrono::steady_clock::now();auto u=h.m.plan(s,z,sc);F f{u};if(cbf)f=filter(s,u,z,h.c,sc.ymax);',
         '''    native_realtime::Session native_realtime_session(h.realtime_probe,native3d::enabled());
    auto start=std::chrono::steady_clock::now();V u;
    {native_realtime::Timer native_realtime_plan_timer(native_realtime::Tag::plan);u=h.m.plan(s,z,sc);}
    native_realtime_session.complete();F f{u};if(cbf)f=filter(s,u,z,h.c,sc.ymax);'''),
    ),
}


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('native realtime anchor count != 1: ' + old[:110])
    return text.replace(old, new, 1)


def patch_one(text, kind):
    digest = hashlib.sha256(text.encode()).hexdigest()
    if digest != PARENT_SHA256[kind]:
        raise ValueError('native realtime parent mismatch: ' + kind)
    for old, new in TRANSFORMS[kind]:
        text = once(text, old, new)
    if kind == 'online_api.cpp':
        text += GETTER
    return text


def project_off(text, kind):
    """Strip every declared experiment edit and recover exact 04 source."""
    if kind == 'online_api.cpp':
        if not text.endswith(GETTER):
            raise ValueError('native realtime getter suffix mismatch')
        text = text[:-len(GETTER)]
    for old, new in reversed(TRANSFORMS[kind]):
        text = once(text, new, old)
    if hashlib.sha256(text.encode()).hexdigest() != PARENT_SHA256[kind]:
        raise ValueError('native realtime inverse projection mismatch: ' + kind)
    return text


def patch_sources(sources):
    if set(sources) != set(PARENT_SHA256):
        raise ValueError('native realtime requires exactly three declared sources')
    result = {kind: patch_one(text, kind) for kind, text in sources.items()}
    for kind, text in result.items():
        if project_off(text, kind) != sources[kind]:
            raise ValueError('native realtime reversible source mismatch: ' + kind)
    return result
