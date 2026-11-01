#pragma once
// Read-only per-request geometry capture. Serialization and file output use the
// existing post-publication trace queue; never sample curves in a cost loop.
#include "state_contract_trace.hpp"
#include <functional>
#include <sstream>

namespace static_width_trace {
inline const char* path() {
  const char* value=std::getenv("STATIC_WIDTH_TRACE_PATH");
  return value&&*value?value:nullptr;
}
inline bool enabled() { return path() && state_contract_trace::enabled(); }
inline void append(const std::string& row) {
  const int fd=::open(path(),O_WRONLY|O_CREAT|O_APPEND,0600);
  if(fd<0)throw std::runtime_error("static width trace open failed");
  size_t offset=0;
  while(offset<row.size()) {
    const auto count=::write(fd,row.data()+offset,row.size()-offset);
    if(count<0&&errno==EINTR)continue;
    if(count<=0){::close(fd);throw std::runtime_error("static width trace write failed");}
    offset+=size_t(count);
  }
  if(::close(fd))throw std::runtime_error("static width trace close failed");
}
inline void enqueue(const std::string& stage,unsigned long long resource_id,
                    std::function<std::string()> serialize) {
  if(!enabled())return;
  auto record=state_contract_trace::record("static_width_record",
    dual_probe::Fields().str("width_stage",stage).add("resource_replan_id",resource_id));
  const auto request=record.request_id;
  const double sim=record.capture_sim,mono=record.capture_mono;
  record.postsolve=[stage,resource_id,request,sim,mono,serialize](dual_probe::Fields& fields) {
    const double begin=dual_probe::mono();
    const std::string row="{\"schema\":\"static_width_v1\",\"stage\":"+delivery_diag::quote(stage)+
      ",\"request_id\":"+std::to_string(request)+",\"resource_replan_id\":"+std::to_string(resource_id)+
      ",\"capture_sim\":"+delivery_diag::number(sim)+",\"capture_mono\":"+delivery_diag::number(mono)+
      ",\"flush_sim\":"+delivery_diag::number(ros::Time::now().toSec())+","+serialize()+"}\n";
    append(row);
    fields.add("width_bytes",row.size()).add("width_flush_ms",(dual_probe::mono()-begin)*1000.);
  };
  state_contract_trace::enqueue(std::move(record));
}
template<class Paths,class Yaws,class Times,class Matrix>
inline void search(const Paths& paths,const Yaws& yaws,const Times& times,
                   const std::vector<int>& signs,int status,const Matrix& head,const Matrix& tail) {
  if(!enabled())return;
  enqueue("native_search_raw",0,[paths,yaws,times,signs,status,head,tail]() {
    std::string text="\"semantics\":\"native search resampled output waypoints before MINCO; not raw A-star expansion nodes\",\"status\":"+std::to_string(status)+",\"head\":"+delivery_diag::matrix(head)+
      ",\"tail\":"+delivery_diag::matrix(tail)+",\"gears\":"+state_contract_trace::signsJson(signs)+",\"parts\":[";
    for(size_t i=0;i<paths.size();++i) {
      if(i)text+=',';text+="{\"points\":[";
      for(size_t j=0;j<paths[i].size();++j){if(j)text+=',';text+=delivery_diag::vector(paths[i][j]);}
      text+="],\"yaw\":[";
      if(i<yaws.size())for(size_t j=0;j<yaws[i].size();++j){if(j)text+=',';text+=delivery_diag::number(yaws[i][j]);}
      text+="],\"times\":"+(i<times.size()?delivery_diag::vector(times[i]):"null")+"}";
    }
    return text+"]";
  });
}
template<class Trajectories>
inline std::string curvesJson(const Trajectories& trajectories,const std::vector<int>& signs) {
  std::string text="[";
  for(size_t i=0;i<trajectories.size();++i) {
    if(i)text+=',';const auto& trajectory=trajectories[i];
    text+="{\"gear\":"+std::to_string(signs[i])+",\"duration\":"+
      delivery_diag::number(trajectory.getTotalDuration())+",\"pieces\":[";
    for(int j=0;j<trajectory.getPieceNum();++j) {
      if(j)text+=',';const auto& piece=trajectory.getPiece(j);
      text+="{\"T\":"+delivery_diag::number(piece.getDuration())+
        ",\"basis\":\"descending\",\"coeffs\":"+delivery_diag::matrix(piece.getCoeffMat())+"}";
    }
    text+="]}";
  }
  return text+"]";
}
template<class MincoContainer>
inline void curves(const std::string& stage,const MincoContainer& minco,
                   const std::vector<int>& signs,bool success,unsigned long long resource_id) {
  if(!enabled())return;
  using Trajectory=decltype(minco[0].getTraj(1));
  std::vector<Trajectory> copied;
  const size_t count=std::min(minco.size(),signs.size());
  copied.reserve(count);
  for(size_t i=0;i<count;++i)copied.push_back(minco[i].getTraj(signs[i]));
  const size_t minco_count=minco.size(),gear_count=signs.size();
  enqueue(stage,resource_id,[copied,signs,success,count,minco_count,gear_count]() {
    return std::string("\"success\":")+(success?"true":"false")+
      ",\"minco_count\":"+std::to_string(minco_count)+",\"gear_count\":"+std::to_string(gear_count)+
      ",\"parts_recorded\":"+std::to_string(count)+",\"parts\":"+curvesJson(copied,signs);
  });
}
} // namespace static_width_trace
