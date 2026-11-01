#pragma once
// Isolated SIL trace. New records stay memory-only through epoch choice, latest
// gates and publication. Request destruction flushes after its final decision.
// No safety decision, budget clock, spinner, or objective is replaced.
#include "state_contract_probe.hpp"
#include "coupling_geometry_ros.hpp"
#include "delivery_diagnostics.hpp"
#include <functional>
#include <stdexcept>

namespace state_contract_trace {
inline bool flag(const char* name) {
  const char* value=std::getenv(name);
  return value && value[0]=='1' && value[1]=='\0';
}
inline bool enabled() { return flag("STATE_CONTRACT_CAPTURE"); }
inline bool signedEnabled() { return flag("STATE_CONTRACT_SIGNED_LOW_SPEED"); }
inline void initialize(ros::NodeHandle&) {
  if (!enabled() && !signedEnabled()) return;
  bool sil=false,sim=false;
  ros::param::param("/online_validation/enabled",sil,false);
  ros::param::param("/use_sim_time",sim,false);
  const char* recovery=std::getenv("COUPLING_FEASIBILITY_RECOVERY");
  if (recovery && std::string(recovery)!="0" && std::string(recovery)!="")
    throw std::runtime_error("state-contract trial requires absent/OFF feasibility recovery");
  if (!sil || !sim || (signedEnabled() && (!enabled() || !coupling_geometry::enabled())))
    throw std::runtime_error("state-contract requires explicit capture + SIL; signed additionally requires engineering");
}

struct Record {
  std::string event;
  dual_probe::Fields fields;
  double capture_mono=0.,capture_sim=0.;
  unsigned long long request_id=0;
  std::function<void(dual_probe::Fields&)> postsolve;
};
struct Context {
  std::vector<Record> records;
  std::uint64_t dropped=0;
};
inline Context*& current() { static thread_local Context* value=nullptr;return value; }
inline Context& orphan() { static thread_local Context value;return value; }
inline Context& context() { return current()?*current():orphan(); }
inline Record record(const std::string& event,const dual_probe::Fields& fields) {
  Record value;value.event=event;value.fields=fields;
  value.capture_mono=dual_probe::mono();value.capture_sim=ros::Time::now().toSec();
  value.request_id=dual_probe::request();return value;
}
inline void enqueue(Record value) {
  if(!enabled())return;
  auto& target=context();
  if(target.records.size()<1024)target.records.push_back(std::move(value));
  else state_contract::incrementBounded(target.dropped);
}
inline void queue(const std::string& event,const dual_probe::Fields& fields=dual_probe::Fields()) {
  if(enabled())enqueue(record(event,fields));
}
inline void flush() noexcept {
  if(!enabled())return;
  auto& target=context();
  const auto saved=dual_probe::request();
  for(auto& value:target.records) {
    try {
      auto fields=value.fields;
      if(value.postsolve)value.postsolve(fields); // All snapshot file I/O here.
      fields.str("state_contract_trace_schema","state_contract_trace_v1")
        .add("capture_mono",value.capture_mono).add("capture_sim",value.capture_sim)
        .add("signed_low_speed",signedEnabled()?"true":"false")
        .str("event_clock","capture_mono/capture_sim; mono/sim are deferred flush clocks");
      dual_probe::request()=value.request_id;
      dual_probe::emit(value.event,fields);
    }catch(const std::exception& error) {
      dual_probe::request()=value.request_id;
      dual_probe::emit("state_contract_trace_error",dual_probe::Fields()
        .str("failed_event",value.event).str("reason",error.what())
        .add("capture_mono",value.capture_mono).add("capture_sim",value.capture_sim));
    }catch(...) {
      dual_probe::request()=value.request_id;
      dual_probe::emit("state_contract_trace_error",dual_probe::Fields()
        .str("failed_event",value.event).str("reason","unknown trace exception")
        .add("capture_mono",value.capture_mono).add("capture_sim",value.capture_sim));
    }
  }
  if(target.dropped)dual_probe::emit("state_contract_trace_dropped",dual_probe::Fields()
    .add("records",target.dropped).add("capture_mono",dual_probe::mono())
    .add("capture_sim",ros::Time::now().toSec()));
  target.records.clear();target.dropped=0;dual_probe::request()=saved;
}

template<class P,class V,class A,class J>
inline void pvaj(const std::string& event,const P& p,const V& v,const A& a,const J& j,
                 double yaw,int gear,double epoch,
                 const dual_probe::Fields& metadata=dual_probe::Fields()) {
  if(!enabled())return;
  auto fields=state_contract::boundaryFields(state_contract::snapshot(),p,v,yaw,gear,epoch);
  fields.add("planner_a",state_contract::vectorJson(a))
    .add("planner_j",state_contract::vectorJson(j));
  fields.text+=metadata.text;
  queue(event,fields);
}
class Request {
  Context value_;Context* previous_=nullptr;bool active_=false;const bool* success_=nullptr;
 public:
  explicit Request(const bool* success=nullptr):success_(success) {
    if(!(active_=enabled()))return;
    previous_=current();current()=&value_;
  }
  ~Request() noexcept {
    if(!active_)return;
    try{queue("state_contract_request_end",dual_probe::Fields()
      .add("success",success_&&*success_?"true":"false"));}catch(...){}
    flush();current()=previous_;
  }
};

template<class Matrix>
inline std::string matrixJson(const Matrix& matrix) { return delivery_diag::matrix(matrix); }
template<class Matrices>
inline std::string matricesJson(const Matrices& matrices) {
  std::string text="[";for(std::size_t i=0;i<matrices.size();++i){
    if(i)text+=',';text+=matrixJson(matrices[i]);}return text+"]";
}
inline std::string signsJson(const std::vector<int>& signs) {
  std::string text="[";for(std::size_t i=0;i<signs.size();++i){
    if(i)text+=',';text+=std::to_string(signs[i]);}return text+"]";
}
inline double inferredYaw(const Eigen::MatrixXd& state,int gear) {
  if(state.rows()<2 || state.cols()<2 || (gear!=1&&gear!=-1) ||
     !state.col(1).head(2).allFinite() || state.col(1).head(2).norm()==0.)
    return state_contract::unavailable();
  return std::atan2(gear*state(1,1),gear*state(0,1));
}

// Created immediately AFTER coupling_geometry::Scope, before its validity gate.
// Captures the exact fixed input/frame even when validation fails immediately.
// Destructor only queues; actual frame serialization happens only after the
// request's failure decision or successful publication, never before epoch
// choice/latest gates. All callback/optimizer objective loops remain untouched.
class BackendRecord {
  bool active_=false;
  std::uint64_t backend_id_=0;
  unsigned long long input_hash_=0;
  static std::uint64_t nextId() {
    static std::atomic<std::uint64_t> serial{0};
    auto old=serial.load();
    do {
      if(old==std::numeric_limits<std::uint64_t>::max())return 0;
    }while(!serial.compare_exchange_weak(old,old+1));
    return old+1;
  }
 public:
  template<class Scope,class States>
  BackendRecord(const Scope& scope,const States& initial,const States& final,
                const std::vector<int>& signs,unsigned long long input_hash) {
    if(!(active_=enabled()))return;
    backend_id_=nextId();input_hash_=input_hash;
    const auto raw=state_contract::snapshot();
    const auto frame=scope.snapshot();const double now=scope.now();
    const Eigen::MatrixXd first=initial.empty()?Eigen::MatrixXd():initial.front();
    const int gear=signs.empty()?0:signs.front();
    const double yaw=inferredYaw(first,gear);
    dual_probe::Fields fields;
    if(first.cols()>=4)fields=state_contract::boundaryFields(raw,first.col(0),first.col(1),yaw,gear,
      state_contract::unavailable()); // Frozen objective epoch is not desired handoff epoch.
    fields.add("initial_states",matricesJson(initial)).add("final_states",matricesJson(final))
      .add("gears",signsJson(signs)).str("input_hash",std::to_string(input_hash))
      .add("state_contract_backend_id",backend_id_)
      .add("state_contract_backend_id_valid",backend_id_?"true":"false")
      .add("backend_frame_valid",scope.valid()?"true":"false");
    state_contract::number(fields,"backend_frame_now",now);
    auto value=record("state_contract_backend_fixed",fields);
    value.postsolve=[raw,frame,now,first,gear,yaw](dual_probe::Fields& output){
      output.add("backend_frame",delivery_diag::directory()?
        delivery_diag::frameJson({frame,now}):
        "{\"now\":"+delivery_diag::number(now)+",\"snapshot\":null}");
      output.add("backend_frame_saved",delivery_diag::directory()&&frame?"true":"false");
      output.str("clearance_scope","frozen projected static field only; not native 3D map replay or safety authorization");
      if(frame && raw.received && raw.raw_finite)output.add("raw_static",
        delivery_diag::evaluationJson(coupling_collision::evaluate(*frame,raw.x,raw.y,raw.yaw,0.,{true,false,0.})));
      else output.add("raw_static","{\"valid\":false,\"reason\":\"missing_raw_or_frame\"}");
      if(frame && first.rows()>=2 && first.cols()>=4 && first.col(0).allFinite() && std::isfinite(yaw))
        output.add("fixed_static",delivery_diag::evaluationJson(coupling_collision::evaluate(
          *frame,first(0,0),first(1,0),yaw,0.,{true,false,0.})));
      else output.add("fixed_static","{\"valid\":false,\"reason\":\"missing_fixed_pose_or_frame\"}");
      state_contract::number(output,"fixed_yaw",yaw);
      output.add("fixed_gear",gear);
    };
    enqueue(std::move(value));
  }
  void link(unsigned long long scope_id) {
    if(!active_)return;
    queue("state_contract_backend_scope_link",dual_probe::Fields()
      .add("state_contract_backend_id",backend_id_).add("scope_id",scope_id)
      .str("input_hash",std::to_string(input_hash_)));
  }
  ~BackendRecord() noexcept {
    if(active_)try{queue("state_contract_backend_scope_end",dual_probe::Fields()
      .add("state_contract_backend_id",backend_id_));}catch(...){}
  }
};

inline void nativeRoot(const Eigen::VectorXd& state,int gear,double threshold) {
  if(!enabled())return;
  auto fields=dual_probe::Fields().add("native_start_state",state_contract::vectorJson(state))
    .add("native_root_gear",gear).str("selector","observed getSingularity strict abs(signed_speed)>threshold");
  state_contract::number(fields,"native_threshold",threshold);
  state_contract::number(fields,"native_signed_speed",state.size()>3?state(3):state_contract::unavailable());
  queue("state_contract_native_astar_root",fields);
}
template<class Matrix>
inline void mincoHead(const Matrix& head,const Matrix& tail,int block,int gear,int start_gear) {
  if(!enabled())return;
  queue("state_contract_native_minco_head",dual_probe::Fields()
    .add("head_state",matrixJson(head)).add("tail_state",matrixJson(tail))
    .add("block_index",block).add("native_first_gear",gear).add("requested_start_gear",start_gear));
}
template<class Data>
inline void candidate(const std::string& event,const Data& data,
                      const std::string& transaction=std::string()) {
  if(!enabled())return;
  dual_probe::Fields fields;
  fields.add("candidate_empty",data.singul_traj.empty()?"true":"false");
  if(!data.singul_traj.empty()) {
    const auto& first=data.singul_traj.front();
    const auto p=first.traj.getJuncPos(0),v=first.traj.getJuncVel(0);
    const auto a=first.traj.getJuncAcc(0),j=first.traj.getJuncJerk(0);
    const double yaw=v.size()>=2&&v.head(2).norm()>0.?
      std::atan2(first.singul*v(1),first.singul*v(0)):state_contract::unavailable();
    fields=state_contract::boundaryFields(state_contract::snapshot(),p,v,yaw,first.singul,data.start_time);
    fields.add("planner_a",state_contract::vectorJson(a)).add("planner_j",state_contract::vectorJson(j))
      .add("candidate_empty","false").add("candidate_segments",data.singul_traj.size());
  }
  state_contract::number(fields,"actual_start_epoch",data.start_time);
  if(!transaction.empty())fields.str("transaction_fnv64",std::to_string(dual_probe::hash(transaction)))
    .str("transaction_standard_fnv64",std::to_string(state_contract::fnv1a64(
      reinterpret_cast<const std::uint8_t*>(transaction.data()),transaction.size())))
    .str("transaction_fnv64_algorithm","historical_dual_probe_fnv1a64_offset1469598103934665603");
  queue(event,fields);
}
} // namespace state_contract_trace
