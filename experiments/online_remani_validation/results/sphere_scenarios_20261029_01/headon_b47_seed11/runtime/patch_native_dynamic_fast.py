#!/usr/bin/env python3
"""Exact reversible MPPI-only child of native_realtime_20261027_01.

NATIVE_DYNAMIC_FAST=0 retains the original dynamic execution path. With =1,
native-3D infeasible pruned rollouts omit useless soft cost, and plan-local
nominal half-step geometry is reused by threat detection and re-scoring.
No shape, horizon, threshold, candidate order or execution approval changes.
This module performs no filesystem writes.
"""
import hashlib

PARENT_SHA256 = {
    'mppi_v12.hpp': '68da03d9645e005779637ca72e989cd83f9049ddc9180d9f8efb45087eda1871',
}

TRANSFORMS = {
    'mppi_v12.hpp': (
        ('struct MPPI : LegacyMPPI {',
         '#include "native_dynamic_fast_probe.hpp"\nstruct MPPI : LegacyMPPI {'),
        ('      bool intervention,int desired_side=0,bool prune=true)const {\n'
         '    DynamicMerit out;auto&m=out.merit;auto p=project(s,sc,a);',
         '''      bool intervention,int desired_side=0,bool prune=true,
      native_dynamic_fast::NominalCache*nominal_cache=nullptr,bool record_nominal=false)const {
    const bool native_fast=native_dynamic_fast::enabled()&&native3d::enabled();
    const bool write_nominal=nominal_cache&&record_nominal&&!prune&&nominal_cache->begin(s,us,future);
    const bool read_nominal=nominal_cache&&!record_nominal&&nominal_cache->matches(s,us,future);
    DynamicMerit out;auto&m=out.merit;auto p=project(s,sc,a);'''),
        ('      for(int j=0;j<2;++j){s=step(s,us[k],c.rdt/2,c);\n'
         '        double gap=clearance(s,future[2*k+j+1],c,sc.ymax,c.margin);',
         '''      for(int j=0;j<2;++j){
        const auto*cached=read_nominal?&nominal_cache->samples[2*k+j]:nullptr;
        s=cached?cached->state:step(s,us[k],c.rdt/2,c);
        double gap=cached?cached->clearance_gap:clearance(s,future[2*k+j+1],c,sc.ymax,c.margin);'''),
        ('        m.cost+=c.rdt/2*(native3d::enabled()?nativeCollisionCost(s,future[2*k+j+1]):70*std::pow(std::max(0.,.25-gap),2));\n'
         '        if(prune&&!m.feasible)return out;',
         '''        // Once hard collision is known, this pruned candidate cannot
        // be selected. Its remaining soft cost is diagnostic only.
        if(native_fast&&prune&&!m.feasible)return out;
        if(write_nominal){
          const auto key=native_dynamic_fast::forecastKey(future[2*k+j+1]);
          const auto full=native3d::query(s.x,s.y,s.yaw,key.time,key.dynamic,false);
          nominal_cache->append(s,future[2*k+j+1],gap,full);
          m.cost+=c.rdt/2*full.cost;
        }else if(cached)m.cost+=c.rdt/2*cached->complete.cost;
        else m.cost+=c.rdt/2*(native3d::enabled()?nativeCollisionCost(s,future[2*k+j+1]):70*std::pow(std::max(0.,.25-gap),2));
        if(prune&&!m.feasible)return out;'''),
        ('    auto nm=dynamicScore(s,sc,a,nominal,future,false,0,false);',
         '''    native_dynamic_fast::NominalCache nominal_cache;
    auto nm=dynamicScore(s,sc,a,nominal,future,false,0,false,&nominal_cache,true);'''),
        ('    S continuing=s;V normal_axis{-d_forward.y,d_forward.x};\n'
         '    for(int k=0;k<horizon&&!d_lateral_needed;++k)for(int j=0;j<2;++j){\n'
         '      continuing=step(continuing,nominal[k],c.rdt/2,c);',
         '''    S continuing=s;V normal_axis{-d_forward.y,d_forward.x};
    const bool cached_nominal=nominal_cache.matches(s,nominal,future);
    for(int k=0;k<horizon&&!d_lateral_needed;++k)for(int j=0;j<2;++j){
      continuing=cached_nominal?nominal_cache.samples[2*k+j].state:step(continuing,nominal[k],c.rdt/2,c);'''),
        ('          if(native3d::query(continuing.x,continuing.y,continuing.yaw,\n'
         '               o.absolute_query_time,true).dynamic_gap<.20){d_lateral_needed=true;break;}',
         '''          const auto*cached=nominal_cache.at(2*k+j,continuing,o.absolute_query_time,true);
          const auto evaluation=cached?cached->complete:native3d::query(continuing.x,continuing.y,continuing.yaw,
               o.absolute_query_time,true);
          if(evaluation.dynamic_gap<.20){d_lateral_needed=true;break;}'''),
        ('    auto add=[&](std::vector<V>us,int side=0){\n'
         '      if(us.size()!=size_t(horizon))us.resize(horizon);\n'
         '      auto m=dynamicScore(s,sc,a,us,future,intervention,side);',
         '''    auto add=[&](std::vector<V>us,int side=0,bool reuse_nominal=false){
      if(us.size()!=size_t(horizon))us.resize(horizon);
      auto m=dynamicScore(s,sc,a,us,future,intervention,side,true,reuse_nominal?&nominal_cache:nullptr);'''),
        ('    add(nominal);for(double scale:{.7,.35,0.})add(following(scale));',
         '    add(nominal,0,true);for(double scale:{.7,.35,0.})add(following(scale));'),
    ),
}


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('native dynamic fast anchor count != 1: ' + old[:110])
    return text.replace(old, new, 1)


def patch_one(text, kind):
    if kind not in PARENT_SHA256 or hashlib.sha256(text.encode()).hexdigest() != PARENT_SHA256[kind]:
        raise ValueError('native dynamic fast parent mismatch: ' + kind)
    for old, new in TRANSFORMS[kind]:
        text = once(text, old, new)
    return text


def project_off(text, kind):
    """Remove all declared changes, recovering the byte-exact frozen parent."""
    for old, new in reversed(TRANSFORMS[kind]):
        text = once(text, new, old)
    if hashlib.sha256(text.encode()).hexdigest() != PARENT_SHA256[kind]:
        raise ValueError('native dynamic fast inverse projection mismatch: ' + kind)
    return text


def patch_sources(sources):
    if set(sources) != set(PARENT_SHA256):
        raise ValueError('native dynamic fast requires exactly the declared MPPI source')
    result = {kind: patch_one(text, kind) for kind, text in sources.items()}
    for kind, text in result.items():
        if project_off(text, kind) != sources[kind]:
            raise ValueError('native dynamic fast reversible source mismatch: ' + kind)
    return result
