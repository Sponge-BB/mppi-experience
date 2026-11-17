#pragma once
// Offline-only, opt-in counters. This probe never changes a candidate, RNG,
// geometry, cache entry, observation clock, deadline, or execution authority.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdlib>

namespace native_realtime {
constexpr int layout_size=20;
enum Field {
  version=0,profile_active=1,effective_demand=2,plan_calls=3,plan_ns=4,
  query_calls=5,query_ns=6,cache_hits=7,body_visits=8,base_calls=9,base_ns=10,
  base_candidates=11,base_candidate_steps=12,v7_calls=13,v7_ns=14,
  freepath_skipped=15,completed=16,native_enabled=17,reserved18=18,reserved19=19
};
inline bool flag(const char*name){const char*p=std::getenv(name);return p&&p[0]=='1'&&p[1]=='\0';}
inline bool demandEnabled(){return flag("NATIVE_REALTIME_DEMAND");}
inline bool profileEnabled(){return flag("NATIVE_REALTIME_PROFILE")&&flag("NATIVE_REALTIME_REPLAY");}
struct Stats {std::array<double,layout_size> values{};};
inline Stats*& active(){static thread_local Stats*value=nullptr;return value;}
class Session {
  Stats& stats_;Stats* previous_;
public:
  Session(Stats&stats,bool native):stats_(stats),previous_(active()){
    stats_.values.fill(0.);stats_.values[version]=1.;
    const bool profile=profileEnabled();stats_.values[profile_active]=profile?1.:0.;
    stats_.values[effective_demand]=native&&demandEnabled()?1.:0.;
    stats_.values[native_enabled]=native?1.:0.;active()=profile?&stats_:nullptr;
  }
  ~Session(){active()=previous_;}
  void complete(){stats_.values[completed]=1.;}
  Session(const Session&)=delete;Session&operator=(const Session&)=delete;
};
enum class Tag {plan,query,base,v7};
class Timer {
  Stats*stats_;Tag tag_;std::chrono::steady_clock::time_point start_;
public:
  explicit Timer(Tag tag):stats_(active()),tag_(tag){
    if(!stats_)return;
    const int calls=tag==Tag::plan?plan_calls:tag==Tag::query?query_calls:tag==Tag::base?base_calls:v7_calls;
    stats_->values[calls]+=1.;start_=std::chrono::steady_clock::now();
  }
  ~Timer(){
    if(!stats_)return;
    const int ns=tag_==Tag::plan?plan_ns:tag_==Tag::query?query_ns:tag_==Tag::base?base_ns:v7_ns;
    stats_->values[ns]+=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start_).count();
  }
  Timer(const Timer&)=delete;Timer&operator=(const Timer&)=delete;
};
inline void cacheHit(){if(auto*s=active())s->values[cache_hits]+=1.;}
// Record once at a full/early query return, not once per sphere. This counts
// the exact attempted body visits without adding a branch to the body loop.
inline void bodyVisits(std::size_t count){if(auto*s=active())s->values[body_visits]+=double(count);}
inline void baseWork(int candidates,int horizon){if(auto*s=active()){
  s->values[base_candidates]+=candidates;s->values[base_candidate_steps]+=double(candidates)*horizon;
}}
inline bool skipFreePath(bool native){
  const bool skip=native&&demandEnabled();
  if(skip)if(auto*s=active())s->values[freepath_skipped]+=1.;
  return skip;
}
inline int read(const Stats&stats,double*out,int capacity){
  if(!out||capacity<layout_size)return -1;
  std::copy(stats.values.begin(),stats.values.end(),out);return layout_size;
}
} // namespace native_realtime
