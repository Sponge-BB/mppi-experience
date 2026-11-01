#pragma once
// Diagnostic-only, isolated SIL instrumentation. All native return values,
// branches and ROS budget clocks are retained. Serialization/file I/O occurs
// in the existing outer Request postsolve flush, not inside a search budget.
#include "state_contract_trace.hpp"
#include <atomic>
#include <cstring>
#include <utility>

namespace delivery_search_probe {
inline bool enabled() {
  return state_contract_trace::enabled() &&
         state_contract_trace::flag("DELIVERY_SEARCH_PROBE");
}
class Scope;
inline Scope*& current() { static thread_local Scope* value=nullptr; return value; }
inline unsigned long long nextId() {
  static std::atomic<unsigned long long> serial{0}; return ++serial;
}
class Scope {
  bool active_=false,finished_=false,native_attempted_=false;
  Scope* previous_=nullptr;
  unsigned long long id_=0,parent_=0;
  const char* solver_="unknown";
  const char* reason_="exception_or_unmarked_exit";
  const char* route_="not_recorded";
  const char* skip_="none";
  double entered_=0.,finished_mono_=0.,native_budget_=0.;
  int code_=0,expanded_=0,used_=0,capacity_=0,start_gear_=0;
  int failures_=-1,try_astar_times_=-1;
  std::size_t open_=0;
  Eigen::VectorXd start_,end_;
 public:
  Scope(const char* solver,const Eigen::VectorXd& start,
        const Eigen::VectorXd& end,int start_gear,double native_budget,
        int capacity,int failures=-1,int try_astar_times=-1) noexcept {
    try {
      if(!(active_=enabled()))return;
      previous_=current(); parent_=previous_?previous_->id_:0;
      id_=nextId(); solver_=solver;start_=start;end_=end;
      native_attempted_=std::strcmp(solver,"kino_astar")==0;
      start_gear_=start_gear;native_budget_=native_budget;capacity_=capacity;
      failures_=failures;try_astar_times_=try_astar_times;
      entered_=dual_probe::mono();current()=this;
    }catch(...) { active_=false; }
  }
  void attempted() noexcept { if(active_)native_attempted_=true; }
  void skipped(const char* why) noexcept { if(active_)skip_=why; }
  void route(const char* value) noexcept { if(active_)route_=value; }
  void finish(const char* reason,int code,int expanded,int used,std::size_t open) noexcept {
    if(!active_)return;
    // Capture before any diagnostic serialization, after the native outcome.
    finished_mono_=dual_probe::mono();finished_=true;reason_=reason;code_=code;
    expanded_=expanded;used_=used;open_=open;
  }
  ~Scope() noexcept {
    if(!active_)return;
    current()=previous_;
    try {
      if(!finished_)finished_mono_=dual_probe::mono();
      const auto id=id_,parent=parent_;
      const auto solver=solver_,reason=reason_,route=route_,skip=skip_;
      const auto start=std::move(start_),end=std::move(end_);
      const auto entered=entered_,ended=finished_mono_,budget=native_budget_;
      const auto code=code_,expanded=expanded_,used=used_,capacity=capacity_;
      const auto gear=start_gear_,failures=failures_,tries=try_astar_times_;
      const auto open=open_;
      const bool marked=finished_,attempted=native_attempted_;
      auto record=state_contract_trace::record("delivery_search_exit",dual_probe::Fields());
      record.postsolve=[id,parent,solver,reason,route,skip,start,end,entered,ended,budget,
                        code,expanded,used,capacity,gear,failures,tries,open,marked,attempted]
                        (dual_probe::Fields& fields) {
        fields.str("delivery_search_schema","delivery_search_exit_v1")
          .add("probe_call_id",id).add("probe_parent_call_id",parent)
          .str("solver",solver).str("exit_reason",reason).add("return_code",code)
          .add("marked_native_return",marked?"true":"false")
          .add("wall_ms",(ended-entered)*1000.)
          .add("entered_mono",entered).add("decision_mono",ended)
          .add("expanded_iterations",expanded).add("used_nodes",used)
          .add("node_capacity",capacity).add("open_nodes_at_exit",open)
          .add("native_astar_budget_ros_seconds",budget)
          .str("native_budget_clock","unchanged ros::Time; wall_ms is monotonic diagnostic only")
          .add("start_state",state_contract::vectorJson(start))
          .add("end_state",state_contract::vectorJson(end)).add("start_gear",gear)
          .add("native_astar_attempted",attempted?"true":"false")
          .str("native_astar_skip_reason",skip).str("sampling_route",route)
          .add("continous_failures_count",failures).add("try_astar_times",tries)
          .str("stats_scope","native A* only, not arm-sampler/RRT internals")
          .str("sampling_failure_scope","wrapper sampling failure; arm/RRT internal exit not observed")
          .str("timing_scope","scope wall until return decision, excludes deferred serialization and file I/O; wrapper includes native trace enqueue overhead")
          .str("scope_start_location","after state construction, before visualization for wrapper; before native start_time for A*");
      };
      state_contract_trace::enqueue(std::move(record));
    }catch(...) { /* Diagnostic failures cannot alter native search results. */ }
  }
};
} // namespace delivery_search_probe
