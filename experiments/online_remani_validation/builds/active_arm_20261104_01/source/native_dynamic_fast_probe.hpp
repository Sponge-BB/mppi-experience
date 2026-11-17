#pragma once
// Included by mppi_v12.hpp inside namespace experiment, after S/V/O and
// native3d are defined. This cache is a plan-local value, never a controller
// member. It stores collision work only, NOT intervention-dependent merit.
namespace native_dynamic_fast {
inline bool enabled(){
  const char*p=std::getenv("NATIVE_DYNAMIC_FAST");
  return p&&p[0]=='1'&&p[1]=='\0';
}
inline bool sameState(const S&a,const S&b){
  return a.x==b.x&&a.y==b.y&&a.yaw==b.yaw&&a.v==b.v&&a.w==b.w;
}
struct ForecastKey {double time=0;bool dynamic=false;};
inline ForecastKey forecastKey(const std::vector<O>&os){
  return {!os.empty()?os.front().absolute_query_time:native3d::now(),!os.empty()};
}
struct Sample {
  S state;
  ForecastKey key;
  // The native early collision result is NOT the full-body minimum. Keep
  // both: the first preserves legacy clearance/closing diagnostics, while
  // only the complete evaluation can supply cost or head-on dynamic_gap.
  double clearance_gap=0;
  native3d::Evaluation complete;
};
struct NominalCache {
  bool active=false;
  std::uint64_t epoch=0;
  const native3d::Snapshot*snapshot=nullptr;
  double now=0;
  S initial;
  std::vector<V> controls;
  std::vector<ForecastKey> forecasts;
  std::vector<Sample> samples;
  bool scopeMatches()const{
    const auto&ctx=native3d::context();
    return active&&enabled()&&native3d::enabled()&&ctx.active&&
      ctx.epoch==epoch&&ctx.snapshot.get()==snapshot&&ctx.now==now&&
      ctx.snapshot&&native3d::fresh(*ctx.snapshot,ctx.now);
  }
  bool begin(const S&s,const std::vector<V>&us,
      const std::vector<std::vector<O>>&future){
    active=false;samples.clear();controls.clear();forecasts.clear();
    const auto&ctx=native3d::context();
    if(!enabled()||!native3d::enabled()||!ctx.active||!ctx.snapshot||
       !native3d::fresh(*ctx.snapshot,ctx.now)||future.size()!=2*us.size()+1)return false;
    epoch=ctx.epoch;snapshot=ctx.snapshot.get();now=ctx.now;initial=s;controls=us;
    forecasts.reserve(future.size());for(const auto&os:future)forecasts.push_back(forecastKey(os));
    samples.reserve(2*us.size());active=true;return true;
  }
  bool matches(const S&s,const std::vector<V>&us,
      const std::vector<std::vector<O>>&future)const{
    if(!scopeMatches()||!sameState(s,initial)||us.size()!=controls.size()||
       samples.size()!=2*us.size()||future.size()!=forecasts.size())return false;
    for(std::size_t k=0;k<us.size();++k)
      if(us[k].x!=controls[k].x||us[k].y!=controls[k].y)return false;
    for(std::size_t k=0;k<future.size();++k){const auto key=forecastKey(future[k]);
      if(key.time!=forecasts[k].time||key.dynamic!=forecasts[k].dynamic)return false;}
    return true;
  }
  const Sample* at(std::size_t index,const S&s,double time,bool dynamic)const{
    if(!scopeMatches()||index>=samples.size())return nullptr;
    const auto&v=samples[index];
    return sameState(v.state,s)&&v.key.time==time&&v.key.dynamic==dynamic?&v:nullptr;
  }
  void append(const S&s,const std::vector<O>&os,double gap,const native3d::Evaluation&full){
    // Even an invalid COMPLETE query is retained as the same fail-closed
    // value. An early/partial Evaluation must never be passed here.
    samples.push_back({s,forecastKey(os),gap,full});
  }
};
} // namespace native_dynamic_fast
