#pragma once
// One paused, same-process native-search comparison; never authorizes a path.
#include "state_contract_trace.hpp"
#include "delivery_replay_probe.hpp"

namespace frontend_rejoin {
inline bool& running(){static thread_local bool value=false;return value;}
inline bool& used(){static bool value=false;return value;}
inline std::string mode(){
  const char* value=std::getenv("FRONTEND_REJOIN_MODE");
  return value&&*value?value:"baseline";
}
inline void apply(const std::string& selected,double& steer,double& change,double& inv_yaw){
  if(selected!="baseline"&&selected!="penalty"&&selected!="yaw"&&selected!="both")
    throw std::invalid_argument("invalid FRONTEND_REJOIN_MODE");
  if(selected=="penalty"||selected=="both"){steer*=.25;change*=.25;}
  if(selected=="yaw"||selected=="both")inv_yaw=1./.3;
}
inline bool requested(double x){
  const auto* context=boundary_admission::currentRequest();
  return state_contract_trace::flag("FRONTEND_REJOIN_PROBE")&&!running()&&!used()&&
    coupling_bridge::referenceRevision()>0&&x>1.8&&context&&!context->dynamic_repair;
}
inline void emit(const std::string& body){
  const char* path=std::getenv("FRONTEND_REJOIN_LOG");
  if(!path||!*path)throw std::runtime_error("missing FRONTEND_REJOIN_LOG");
  std::ofstream stream(path,std::ios::app);
  stream<<"{\"schema\":\"frontend_rejoin_probe_v1\",\"request_id\":"<<dual_probe::request()<<','<<body<<"}\n";
  if(!stream)throw std::runtime_error("cannot write frontend probe");
}
template<class Paths,class Yaws,class Times>
inline std::string pathText(const Paths& paths,const Yaws& yaws,const Times& times,
                            const std::vector<int>& signs){
  std::string text="{\"gears\":"+state_contract_trace::signsJson(signs)+",\"parts\":[";
  for(size_t i=0;i<paths.size();++i){
    if(i)text+=',';text+="{\"points\":[";
    for(size_t j=0;j<paths[i].size();++j){if(j)text+=',';text+=delivery_diag::vector(paths[i][j]);}
    text+="],\"yaw\":[";
    if(i<yaws.size())for(size_t j=0;j<yaws[i].size();++j){if(j)text+=',';text+=delivery_diag::number(yaws[i][j]);}
    text+="],\"times\":"+(i<times.size()?delivery_diag::vector(times[i]):"null")+"}";
  }
  return text+"]}";
}
}
