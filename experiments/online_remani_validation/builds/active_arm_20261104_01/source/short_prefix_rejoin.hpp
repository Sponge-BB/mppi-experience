#pragma once
// Isolated fallback-search ablation: a fresh short MPPI prefix can suggest a
// reverse maneuver without installing a guessed future state as the boundary.
#include <traj_utils/poly_traj_utils.hpp>
#include "collab_factorial.hpp"
#include "state_contract_trace.hpp"
#include <array>
#include <vector>

namespace short_prefix_rejoin {
inline bool requested(){
  const char* value=std::getenv("SHORT_PREFIX_REJOIN");
  return value&&std::string(value)=="1";
}

struct Hint {
  bool valid=false;int gear=0;long row_index=-1;double dt_s=-1.;
};
inline Hint reverseHint(const std::vector<std::array<double,8>>& rows,
                        double seed_age_s,int current_gear){
  Hint out;if(current_gear!=1&&current_gear!=-1)return out;
  for(std::size_t i=0;i<rows.size();++i){
    const double dt=rows[i][0]-seed_age_s;
    if(dt < -1e-6)continue;
    if(dt > 1.+1e-6)break;
    if(std::abs(rows[i][4])>=.015&&rows[i][4]*current_gear<0.){
      out.valid=true;out.gear=-current_gear;out.row_index=static_cast<long>(i);
      out.dt_s=std::max(0.,dt);return out;
    }
  }
  return out;
}
inline bool activate(bool on,bool fallback_prefix_context,bool short_prefix,
                     int current_gear,const Hint& hint){
  return on&&fallback_prefix_context&&short_prefix&&
    (current_gear==1||current_gear==-1)&&hint.valid&&hint.gear==-current_gear;
}

struct Context {
  bool requested=false,fallback_prefix_context=false,hint_valid=false,active=false;
  std::string reason="not_prefix_fallback",prefix_rows_reason="not_checked";
  int current_gear=0,hint_gear=0;long hint_row_index=-1;
  double seed_age_s=0.,suggested_switch_dt_s=-1.,seed_revision=0.,seed_stamp=0.;
  unsigned long long switch_penalty_evaluations=0,zeroed_switch_penalty_evaluations=0;
  double observed_original_switch_penalty=-1.;
};
inline Context*& current(){static thread_local Context* value=nullptr;return value;}
struct ScopedContext {
  Context* previous;
  explicit ScopedContext(Context& value):previous(current()){current()=&value;}
  ~ScopedContext(){current()=previous;}
  ScopedContext(const ScopedContext&)=delete;
  ScopedContext& operator=(const ScopedContext&)=delete;
};

class Scope {
  Context value_;ScopedContext binding_;
 public:
  Scope(const Eigen::VectorXd& start,double yaw,int gear,bool fallback_context)
      :binding_(value_){
    auto& c=value_;c.requested=requested();c.fallback_prefix_context=fallback_context;
    c.current_gear=gear;const double now=ros::Time::now().toSec();
    const auto& seed=collab::seed();c.seed_stamp=seed.stamp;c.seed_revision=seed.revision;
    c.seed_age_s=now-seed.stamp;
    if(!fallback_context)return;
    std::vector<std::array<double,8>> prefix;
    collab::prefixRows(start,yaw,gear,prefix,c.prefix_rows_reason);
    if(c.prefix_rows_reason!="short_prefix"){
      c.reason="prefix_rows_"+c.prefix_rows_reason;return;
    }
    if(gear!=1&&gear!=-1){c.reason="unknown_current_gear";return;}
    const auto hint=reverseHint(seed.rows,c.seed_age_s,gear);
    c.hint_valid=hint.valid;c.hint_gear=hint.gear;c.hint_row_index=hint.row_index;
    c.suggested_switch_dt_s=hint.dt_s;
    c.active=activate(c.requested,fallback_context,true,gear,hint);
    c.reason=hint.valid?"short_prefix_reverse_hint":"short_prefix_no_reverse_hint";
  }
  ~Scope(){
    const auto& c=value_;
    state_contract_trace::queue("short_prefix_rejoin_scope",dual_probe::Fields()
      .add("requested",c.requested?"true":"false")
      .add("fallback_prefix_context",c.fallback_prefix_context?"true":"false")
      .add("hint_valid",c.hint_valid?"true":"false")
      .add("hint_eligible",c.hint_valid?"true":"false").add("active",c.active?"true":"false")
      .str("reason",c.reason).str("prefix_rows_reason",c.prefix_rows_reason)
      .add("current_gear",c.current_gear).add("hint_gear",c.hint_gear)
      .add("seed_age_s",c.seed_age_s).add("suggested_switch_dt_s",c.suggested_switch_dt_s)
      .add("seed_revision",c.seed_revision).add("seed_stamp",c.seed_stamp)
      .add("hint_row_index",c.hint_row_index)
      .add("switch_penalty_evaluations",c.switch_penalty_evaluations)
      .add("zeroed_switch_penalty_evaluations",c.zeroed_switch_penalty_evaluations)
      .add("observed_original_switch_penalty",c.observed_original_switch_penalty)
      .str("policy","uniform per-search gear-switch coefficient; scoped restore")
      .str("counter_semantics","candidate-edge cost evaluations, not executed gear switches"));
  }
  Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
};

inline double switchPenalty(double original){
  auto* c=current();if(!c)return original;
  ++c->switch_penalty_evaluations;c->observed_original_switch_penalty=original;
  if(c->active){++c->zeroed_switch_penalty_evaluations;return 0.;}
  return original;
}

struct SelectedNode {
  std::array<double,3> state;
  std::array<double,2> input;
  int singul=0;double g=0.;
};
template<class Nodes,class Start,class Goal>
inline void selectedNodes(const Nodes& path,int shot_index,bool shot_success,
                          const Start& actual_search_start,const Goal& goal){
  if(!state_contract_trace::enabled())return;
  std::vector<SelectedNode> copied;copied.reserve(path.size());
  for(const auto* node:path){
    SelectedNode out;for(int i=0;i<3;++i)out.state[i]=node->state(i);
    for(int i=0;i<2;++i)out.input[i]=node->input(i);
    out.singul=node->singul;out.g=node->g_score;copied.push_back(out);
  }
  const std::array<double,3> end{goal(0),goal(1),goal(2)};
  const Eigen::VectorXd start=actual_search_start;
  const auto* context=current();
  auto record=state_contract_trace::record("short_prefix_selected_nodes",dual_probe::Fields()
    .add("shot_index",shot_index).add("shot_success",shot_success?"true":"false")
    .add("node_count",copied.size()).add("short_prefix_scope_present",context?"true":"false")
    .add("hint_valid",context&&context->hint_valid?"true":"false")
    .add("active",context&&context->active?"true":"false")
    .str("semantics","selected A-star parent chain before resampling; excludes shot interpolation and nonselected expanded nodes"));
  record.postsolve=[copied,end,start](dual_probe::Fields& fields){
    std::string text="[";
    for(std::size_t i=0;i<copied.size();++i){
      const auto& node=copied[i];if(i)text+=',';
      text+="{\"state\":["+delivery_diag::number(node.state[0])+","+
        delivery_diag::number(node.state[1])+","+delivery_diag::number(node.state[2])+
        "],\"input\":["+delivery_diag::number(node.input[0])+","+delivery_diag::number(node.input[1])+
        "],\"singul\":"+std::to_string(node.singul)+",\"g\":"+delivery_diag::number(node.g)+"}";
    }
    fields.add("actual_search_start_state",delivery_diag::vector(start))
      .add("nodes",text+"]").add("goal",std::string("[")+delivery_diag::number(end[0])+","+
      delivery_diag::number(end[1])+","+delivery_diag::number(end[2])+"]");
  };
  state_contract_trace::enqueue(std::move(record));
}
} // namespace short_prefix_rejoin
