"""Continue after feasibility, preserving the best fully checked accepted x."""
from native_collision_runtime import replace_once


def transform_gate(text):
    text=replace_once(text,'#include "delivery_accepted_candidate.hpp"',
        '#include "delivery_accepted_candidate.hpp"\n#include "feasible_quality_polish.hpp"')
    text=replace_once(text,'  Session* previous;bool enabled=false,selected=false;',
        '  Session* previous;bool enabled=false,selected=false;\n'
        '  const bool polish=feasible_quality_polish::requested();\n'
        '  feasible_quality_polish::Checkpoint quality;')
    text=replace_once(text,'    if(!enabled||selected)return selected;',
        '''    if(!enabled)return false;
    if(selected&&!polish)return true;''')
    text=replace_once(text,'    c.begin_mono_s=monotonic();c.elapsed_begin_s=elapsed();',
        '''    c.begin_mono_s=monotonic();c.elapsed_begin_s=elapsed();
    const auto* actual=delivery_candidate::active();
    const bool accepted_matches=actual&&actual->usable()&&!actual->initial&&
      actual->complete_evaluation&&actual->evaluated_hash==c.candidate_hash&&
      delivery_candidate::hash(actual->x)==c.candidate_hash&&actual->cost==actual->evaluated_cost;
    const double accepted_cost=accepted_matches?actual->cost:std::numeric_limits<double>::infinity();
    const auto objective=feasible_quality_polish::matching(x,accepted_cost);
    if(polish&&(!accepted_matches||!objective.valid||!quality.better(accepted_cost))){
      c.reason=!accepted_matches?"quality_missing_accepted_cost":
        !objective.valid?"quality_missing_objective":"quality_not_better";
      c.elapsed_end_s=elapsed();c.end_mono_s=monotonic();checks.push_back(c);return false;
    }''')
    text=replace_once(text,'        c.native_checked=true;last_native_start=monotonic();began=last_native_start;',
        '        c.native_checked=true;++quality.native_checks;last_native_start=monotonic();began=last_native_start;')
    text=replace_once(text,'        if(c.selected){selected=true;selected_x=x;}',
        '''        if(c.selected){
          selected=true;selected_x=x;
          quality.consider(x,accepted_cost,true,true,objective,iteration,elapsed());
        }''')
    text=replace_once(text,'(c.native_pass?"native_pass_early_exit":"native_gate_reject")',
        '(c.native_pass?(polish?"native_pass_incumbent":"native_pass_early_exit"):"native_gate_reject")')
    text=replace_once(text,'checks.push_back(c);return selected;',
        'checks.push_back(c);return selected&&!polish;')
    return text


REPORT=r'''    // Snapshot copies and candidate reconstruction occur after the optimizer.
    // Serialization is deferred until the request's publication/failure decision.
    const auto quality_initial=quality_cost_scope.initial;
    const auto quality_checkpoint=early_delivery_session.quality;
    const bool quality_enabled=early_delivery_session.polish;
    const double quality_budget_s=early_delivery_session.budget;
    const auto quality_plan=temporal_capture::currentPlan();
    if(temporal_capture::enabled()&&quality_plan&&quality_plan->request==dual_probe::request()){
      const auto quality_first=quality_checkpoint.available?
        early_delivery_checker->trajectory(quality_checkpoint.first_x):SingulTrajData();
      const auto quality_final=quality_checkpoint.available?
        early_delivery_checker->trajectory(quality_checkpoint.best_x):SingulTrajData();
      temporal_capture::enqueue("feasible_quality_polish",[quality_plan,quality_initial,quality_checkpoint,
          quality_enabled,quality_budget_s,quality_first,quality_final,result](){
        return std::string("\"plan_ordinal\":")+std::to_string(quality_plan->ordinal)+
          ",\"polish_enabled\":"+(quality_enabled?"true":"false")+
          ",\"safe_checkpoint_available\":"+(quality_checkpoint.available?"true":"false")+
          ",\"solver_result\":"+std::to_string(result)+
          ",\"original_budget_s\":"+feasible_quality_polish::number(quality_budget_s)+
          ",\"first_feasible_elapsed_s\":"+feasible_quality_polish::number(quality_checkpoint.first_elapsed_s)+
          ",\"selected_elapsed_s\":"+feasible_quality_polish::number(quality_checkpoint.best_elapsed_s)+
          ",\"first_feasible_iteration\":"+std::to_string(quality_checkpoint.first_iteration)+
          ",\"selected_iteration\":"+std::to_string(quality_checkpoint.best_iteration)+
          ",\"improvements\":"+std::to_string(quality_checkpoint.improvements)+
          ",\"native_checks\":"+std::to_string(quality_checkpoint.native_checks)+
          ",\"first_cost\":"+feasible_quality_polish::number(quality_checkpoint.first_cost)+
          ",\"best_cost\":"+feasible_quality_polish::number(quality_checkpoint.best_cost)+
          ",\"initial_objective\":"+feasible_quality_polish::snapshotJson(quality_initial)+
          ",\"first_feasible_objective\":"+feasible_quality_polish::snapshotJson(quality_checkpoint.first_snapshot)+
          ",\"selected_objective\":"+feasible_quality_polish::snapshotJson(quality_checkpoint.best_snapshot)+
          ",\"first_feasible_candidate\":"+(quality_checkpoint.available?temporal_capture::candidateJson(quality_first):"null")+
          ",\"selected_candidate\":"+(quality_checkpoint.available?temporal_capture::candidateJson(quality_final):"null")+
          ",\"scope\":\"original objective and deadline; checked accepted incumbents only; final native/latest gates still decide delivery\"";
      });
    }
'''


def transform_optimizer(text):
    text=replace_once(text,'    delivery_candidate::Scope accepted_candidate;',
        '    delivery_candidate::Scope accepted_candidate;\n    feasible_quality_polish::Scope quality_cost_scope;')
    text=replace_once(text,'    delivery_candidate::completed(x,g,costall);',
        '''    delivery_candidate::completed(x,g,costall);
    feasible_quality_polish::completed(x,g,total_smoo_cost,total_obs_feas_costs,total_time_cost,
      extra_geometry,extra_prediction,costall);''')
    text=replace_once(text,'.add("selected_for_exit",check.selected?1:0)',
        '.add("selected_for_exit",check.selected&&!early_delivery_session.polish?1:0)\n'
        '        .add("selected_as_incumbent",check.selected?1:0)')
    text=replace_once(text,'      .add("enabled",early_delivery_enabled?1:0).add("selected",early_delivery_session.selected?1:0)',
        '      .add("enabled",early_delivery_enabled?1:0).add("selected",early_delivery_session.selected?1:0)\n'
        '      .add("polish",early_delivery_session.polish?1:0)\n'
        '      .add("quality_improvements",early_delivery_session.quality.improvements)')
    text=replace_once(text,'    probe.add("iterations",iter_num_);probe.add("lbfgs_result",result);',
        '''    if(early_delivery_session.polish&&early_delivery_session.selected){
      // Keep the certified incumbent through budget or line-search exits.
      // A later non-finite error still follows the original rejection below.
      if(!early_delivery_session.quality.available)throw std::runtime_error("polish selected without objective checkpoint");
      accepted_candidate.x=early_delivery_session.selected_x;
      accepted_candidate.cost=early_delivery_session.quality.best_cost;
      accepted_candidate.available=true; // preserve invalid/error state verbatim
      final_cost=early_delivery_session.quality.best_cost;
      salvage_attempted=true;
    }
'''+REPORT+'''    probe.add("iterations",iter_num_);probe.add("lbfgs_result",result);''')
    text=replace_once(text,'      if(!accepted_candidate.usable())return false;\n      // No blind acceptance:',
        '''      const bool have_polished_safe=early_delivery_session.polish&&early_delivery_session.selected;
      if(!have_polished_safe&&!accepted_candidate.usable())return false;
      const auto& timeout_candidate=have_polished_safe?early_delivery_session.selected_x:accepted_candidate.x;
      // No blind acceptance:''')
    text=replace_once(text,'      bool rebuilt=delivery_candidate::decode(accepted_candidate.x,piece_num_container_,traj_dim_,',
        '      bool rebuilt=delivery_candidate::decode(timeout_candidate,piece_num_container_,traj_dim_,')
    return text
