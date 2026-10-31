#include <native_realtime_probe.hpp>
#include <cassert>
#include <iostream>
#ifdef NATIVE_REALTIME_GEOMETRY_TEST
#include <native_collision_3d.hpp>
#endif

using namespace native_realtime;
static void flags(const char*demand,const char*profile,const char*replay){
  ::setenv("NATIVE_REALTIME_DEMAND",demand,1);
  ::setenv("NATIVE_REALTIME_PROFILE",profile,1);
  ::setenv("NATIVE_REALTIME_REPLAY",replay,1);
}
#ifdef NATIVE_REALTIME_GEOMETRY_TEST
static void geometryCounters(){
  std::vector<double> payload(32,0.);payload[0]=3;payload[1]=1;payload[2]=1;payload[3]=100.;payload[4]=1;
  payload[5]=8;payload[6]=8;payload[7]=8;payload[8]=-.3;payload[9]=-.3;payload[10]=-.1;payload[11]=.1;
  payload[12]=3;payload[14]=1;payload[15]=.35;payload[16]=3.;payload[17]=100.;payload[18]=100.;
  payload[19]=1;payload[20]=1;payload[21]=.01;payload[22]=1;payload[23]=1;payload[24]=100.;payload[28]=1;
  for(int i=0;i<3;++i){const double b[]={i*.05,0.,.2+i*.02,.1,.2,1.,0.};
    payload.insert(payload.end(),std::begin(b),std::end(b));}
  payload.insert(payload.end(),8*8*8,1.);
  native3d::State state;state.now=100.;assert(native3d::bind(state,payload.data(),payload.size()));
  flags("1","1","1");Stats stats;
  {Session session(stats,true);native3d::Scope scope(state);
    assert(native3d::query(0.,0.,0.,100.,true,true).gap>0);
    assert(native3d::query(0.,0.,0.,100.,true,false).gap>0); // Complete safe memo reused.
    assert(native3d::query(.01,0.,0.,100.,true,false).gap>0);
  }
  assert(stats.values[query_calls]==3&&stats.values[cache_hits]==1&&stats.values[body_visits]==6);
  payload[14]=2;std::fill(payload.begin()+32+3*7,payload.end(),0.);
  assert(native3d::bind(state,payload.data(),payload.size()));
  {Session session(stats,true);native3d::Scope scope(state);
    assert(native3d::query(0.,0.,0.,100.,true,true).gap<0); // Exactly one body visited.
    assert(native3d::query(0.,0.,0.,100.,true,false).gap<0); // Partial memo cannot stand in for full cost.
    assert(native3d::query(0.,0.,0.,100.,true,false).gap<0);
    assert(native3d::query(0.,0.,0.,100.,true,true).gap<0);
  }
  assert(stats.values[query_calls]==4&&stats.values[cache_hits]==2&&stats.values[body_visits]==4);
  {Session session(stats,true);native3d::Scope scope(state);
    assert(!native3d::query(100.,0.,0.,100.,true,false).valid);
    assert(!native3d::query(100.,0.,0.,100.,true,false).valid);
  }
  assert(stats.values[query_calls]==2&&stats.values[cache_hits]==0&&stats.values[body_visits]==2);
}
#endif
int main(){
  Stats stats;
  for(const auto&pair:std::array<std::array<const char*,2>,3>{{{{"0","0"}},{{"1","0"}},{{"0","1"}}}}){
    flags("1",pair[0],pair[1]);
    {Session scope(stats,true);Timer plan(Tag::plan);bodyVisits(111);cacheHit();baseWork(96,30);
      assert(active()==nullptr);assert(skipFreePath(true));scope.complete();}
    assert(stats.values[profile_active]==0&&stats.values[plan_calls]==0&&stats.values[body_visits]==0);
    assert(stats.values[base_candidates]==0&&stats.values[cache_hits]==0&&stats.values[freepath_skipped]==0);
    assert(stats.values[effective_demand]==1&&stats.values[completed]==1);
  }
  flags("1","1","1");
  {Session scope(stats,true);
    assert(active()==&stats);assert(!skipFreePath(false));assert(skipFreePath(true));
    {Timer plan(Tag::plan);
      {Timer base(Tag::base);baseWork(96,30);
        {Timer query(Tag::query);bodyVisits(27);}
        {Timer query(Tag::query);cacheHit();}
      }
      {Timer evaluate(Tag::v7);Timer query(Tag::query);bodyVisits(5);}
    }
    scope.complete();
  }
  assert(active()==nullptr);
  assert(stats.values[version]==1&&stats.values[profile_active]==1&&stats.values[completed]==1);
  assert(stats.values[plan_calls]==1&&stats.values[base_calls]==1&&stats.values[v7_calls]==1);
  assert(stats.values[query_calls]==3&&stats.values[cache_hits]==1&&stats.values[body_visits]==32);
  assert(stats.values[base_candidates]==96&&stats.values[base_candidate_steps]==2880);
  assert(stats.values[freepath_skipped]==1&&stats.values[effective_demand]==1);
  assert(stats.values[plan_ns]>=stats.values[base_ns]);
  assert(stats.values[plan_ns]>=stats.values[query_ns]);
  double output[layout_size];assert(read(stats,output,layout_size)==layout_size);
  assert(output[body_visits]==32&&read(stats,output,layout_size-1)==-1);
  assert(read(stats,nullptr,layout_size)==-1);
  // A disabled nested session cannot charge work to an outer session, and
  // its destruction restores the exact previous pointer.
  {Session outer(stats,true);bodyVisits(2);
    flags("1","1","0");Stats inner_stats;
    {Session inner(inner_stats,true);bodyVisits(100);assert(active()==nullptr);}
    assert(active()==&stats);bodyVisits(3);
  }
  assert(stats.values[body_visits]==5&&active()==nullptr);
  // OFF and native-disabled modes retain original free-path selection.
  flags("0","1","1");
  {Session off(stats,true);assert(!skipFreePath(true));assert(stats.values[effective_demand]==0);}
  flags("1","1","1");
  {Session no_native(stats,false);assert(!skipFreePath(false));assert(stats.values[effective_demand]==0);}
  assert(stats.values[body_visits]==0&&stats.values[query_calls]==0); // Per-plan reset.
#ifdef NATIVE_REALTIME_GEOMETRY_TEST
  geometryCounters();
#endif
  std::cout<<"native realtime probe: dual offline gate, native-only demand, exact counters, timers, nesting and reset passed\n";
}
