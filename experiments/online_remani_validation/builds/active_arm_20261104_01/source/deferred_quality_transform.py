"""Defer post-feasibility checks within the unchanged optimizer deadline."""
from native_collision_runtime import replace_once


SESSION = r'''struct Session {
  using Elapsed=std::function<double()>;
  using Native=std::function<NativeVerdict(const remani_planner::SingulTrajData&)>;
  Session* previous;bool enabled=false,selected=false;
  bool polish=feasible_quality_polish::requested();
  const bool deferred_requested=deferred_quality::requested();
  feasible_quality_polish::Checkpoint quality;
  deferred_quality::Pending pending;
  Checker* checker=nullptr;Elapsed elapsed;Native native;double budget=0.;
  double last_native_start=-std::numeric_limits<double>::infinity();
  Eigen::VectorXd selected_x;std::vector<Check> checks;
  bool reserve_stop_requested=false,finalized=false,promoted=false;
  bool deferred_check_attempted=false;
  std::string deferred_check_reason="not_finalized";
  Check deferred_check;
  int first_accepted_steps=0;
  Session(bool on,Checker* c,Elapsed e,double max_time,Native n):previous(active()),enabled(on),checker(c),elapsed(std::move(e)),native(std::move(n)),budget(max_time){if(enabled)active()=this;}
  ~Session(){if(enabled)active()=previous;}
  Session(const Session&)=delete;Session& operator=(const Session&)=delete;
  bool deferredEnabled()const{return enabled&&polish&&deferred_requested;}
  double cutoff()const{return std::max(0.,budget-deferred_quality::reserve_s);}
  bool reserveStopDue(){
    const bool due=deferredEnabled()&&selected&&elapsed()>=cutoff();
    if(due)reserve_stop_requested=true;
    return due;
  }
  bool accepted(const Eigen::VectorXd& x,int iteration){
    if(!enabled)return false;
    if(selected&&!polish)return true;
    Check c;c.accepted_iteration=iteration;c.candidate_hash=delivery_candidate::hash(x);
    c.begin_mono_s=monotonic();c.elapsed_begin_s=elapsed();
    const auto* actual=delivery_candidate::active();
    const bool accepted_matches=actual&&actual->usable()&&!actual->initial&&
      actual->complete_evaluation&&actual->evaluated_hash==c.candidate_hash&&
      delivery_candidate::hash(actual->x)==c.candidate_hash&&actual->cost==actual->evaluated_cost;
    const double accepted_cost=accepted_matches?actual->cost:std::numeric_limits<double>::infinity();
    const auto objective=feasible_quality_polish::matching(x,accepted_cost);
    // Only actual, fully evaluated accepted points can become pending.
    if(polish&&(!accepted_matches||!objective.valid||!quality.better(accepted_cost))){
      c.reason=!accepted_matches?"quality_missing_accepted_cost":
        !objective.valid?"quality_missing_objective":"quality_not_better";
      c.elapsed_end_s=elapsed();c.end_mono_s=monotonic();checks.push_back(c);
      return reserveStopDue();
    }
    if(deferredEnabled()&&selected){
      const bool stored=pending.consider(x,accepted_cost,objective,iteration);
      c.reason=stored?"deferred_pending_accepted":"deferred_pending_not_better";
      c.elapsed_end_s=elapsed();c.end_mono_s=monotonic();checks.push_back(c);
      return reserveStopDue();
    }
    if(c.elapsed_begin_s>=budget){c.budget_expired=true;c.reason="budget_before_check";}
    else{
      auto began=monotonic();const auto candidate=checker->trajectory(x);c.reconstruction_s=monotonic()-began;
      began=monotonic();c.wheel_pass=checker->wheelPass(candidate);c.wheel_s=monotonic()-began;
      if(!c.wheel_pass)c.reason="wheel_precheck_reject";
      else if(elapsed()>=budget){c.budget_expired=true;c.reason="budget_after_wheel";}
      else if(monotonic()-last_native_start<.1)c.reason="native_check_interval";
      else{
        c.native_checked=true;++quality.native_checks;last_native_start=monotonic();began=last_native_start;
        const auto verdict=native(candidate);c.native_pass=verdict.pass;c.native_reason=verdict.reason;c.collision_type=verdict.collision_type;
        c.rejection_time=verdict.rejection_time;c.native_s=monotonic()-began;
        c.budget_expired=elapsed()>=budget;
        c.selected=c.native_pass&&!c.budget_expired;
        c.reason=c.budget_expired?"budget_after_native":(c.native_pass?(polish?"native_pass_incumbent":"native_pass_early_exit"):"native_gate_reject");
        if(c.selected){
          if(!quality.available)first_accepted_steps=actual?actual->accepted_steps:0;
          selected=true;selected_x=x;
          quality.consider(x,accepted_cost,true,true,objective,iteration,elapsed());
        }
      }
    }
    c.elapsed_end_s=elapsed();c.end_mono_s=monotonic();checks.push_back(c);
    return (selected&&!polish)||reserveStopDue();
  }
  void finalize(bool allow){
    if(finalized)return;finalized=true;
    if(!deferredEnabled()){deferred_check_reason="disabled";return;}
    if(!allow){deferred_check_reason="invalid_or_error";return;}
    if(!selected){deferred_check_reason="no_first_safe";return;}
    if(!pending.available){deferred_check_reason="no_better_pending";return;}
    if(quality.first_elapsed_s>=cutoff()){deferred_check_reason="first_safe_after_cutoff";return;}
    if(elapsed()>=budget){deferred_check_reason="budget_before_deferred_check";return;}
    deferred_check_attempted=true;
    Check c;c.accepted_iteration=pending.iteration;c.candidate_hash=delivery_candidate::hash(pending.x);
    c.begin_mono_s=monotonic();c.elapsed_begin_s=elapsed();
    auto began=monotonic();const auto candidate=checker->trajectory(pending.x);c.reconstruction_s=monotonic()-began;
    began=monotonic();c.wheel_pass=checker->wheelPass(candidate);c.wheel_s=monotonic()-began;
    if(!c.wheel_pass)c.reason="deferred_wheel_reject";
    else if(elapsed()>=budget){c.budget_expired=true;c.reason="deferred_budget_after_wheel";}
    else{
      c.native_checked=true;++quality.native_checks;began=monotonic();
      const auto verdict=native(candidate);c.native_pass=verdict.pass;c.native_reason=verdict.reason;
      c.collision_type=verdict.collision_type;c.rejection_time=verdict.rejection_time;c.native_s=monotonic()-began;
      c.budget_expired=elapsed()>=budget;
      c.selected=c.native_pass&&!c.budget_expired;
      c.reason=c.budget_expired?"deferred_budget_after_native":
        (c.native_pass?"deferred_native_pass_promoted":"deferred_native_reject");
      if(c.selected){
        promoted=quality.consider(pending.x,pending.cost,true,true,pending.snapshot,pending.iteration,elapsed());
        if(promoted)selected_x=pending.x;
      }
    }
    c.elapsed_end_s=elapsed();c.end_mono_s=monotonic();
    deferred_check_reason=c.reason;deferred_check=c;checks.push_back(c);
  }
};
'''


def transform_gate(text):
    text = replace_once(text, '#include "feasible_quality_polish.hpp"',
                        '#include "feasible_quality_polish.hpp"\n#include "deferred_quality.hpp"')
    start = text.index('struct Session {')
    end = text.index('inline bool accepted(const Eigen::VectorXd& x,int iteration)', start)
    text = text[:start]+SESSION+text[end:]
    return replace_once(text, '} // namespace early_delivery',
        'inline bool reserveStopDue(){auto* s=active();return s&&s->reserveStopDue();}\n} // namespace early_delivery')


REPORT = r'''    const auto deferred_state=early_delivery_session.pending;
    const auto deferred_check=early_delivery_session.deferred_check;
    const bool deferred_requested=early_delivery_session.deferred_requested;
    const bool deferred_enabled=early_delivery_session.deferredEnabled();
    const bool deferred_stop=early_delivery_session.reserve_stop_requested;
    const bool deferred_attempted=early_delivery_session.deferred_check_attempted;
    const bool deferred_promoted=early_delivery_session.promoted;
    const auto deferred_reason=early_delivery_session.deferred_check_reason;
    const double deferred_cutoff=early_delivery_session.cutoff();
    const auto deferred_actual_hash=accepted_candidate.available?delivery_candidate::hash(accepted_candidate.x):0ULL;
    const double deferred_actual_cost=accepted_candidate.available?accepted_candidate.cost:std::numeric_limits<double>::infinity();
    const int deferred_actual_steps=accepted_candidate.accepted_steps;
    const int deferred_after_count=quality_checkpoint.available?
      std::max(0,deferred_actual_steps-early_delivery_session.first_accepted_steps):0;
    const auto deferred_request_id=dual_probe::request();
    if(temporal_capture::enabled()&&quality_plan&&quality_plan->request==deferred_request_id){
      const auto deferred_candidate=deferred_state.available?
        early_delivery_checker->trajectory(deferred_state.x):SingulTrajData();
      temporal_capture::enqueue("deferred_quality",[quality_plan,quality_checkpoint,quality_enabled,
          deferred_state,deferred_check,deferred_requested,deferred_enabled,deferred_stop,
          deferred_attempted,deferred_promoted,deferred_reason,deferred_cutoff,deferred_actual_hash,
          deferred_actual_cost,deferred_actual_steps,deferred_after_count,deferred_request_id,deferred_candidate](){
        return std::string("\"plan_ordinal\":")+std::to_string(quality_plan->ordinal)+
          ",\"request_id\":"+std::to_string(deferred_request_id)+
          ",\"deferred_requested\":"+(deferred_requested?"true":"false")+
          ",\"deferred_enabled\":"+(deferred_enabled?"true":"false")+
          ",\"polish_enabled\":"+(quality_enabled?"true":"false")+
          ",\"reserve_s\":"+feasible_quality_polish::number(deferred_quality::reserve_s)+
          ",\"cutoff_s\":"+feasible_quality_polish::number(deferred_cutoff)+
          ",\"cooperative_stop_requested\":"+(deferred_stop?"true":"false")+
          ",\"pending_available\":"+(deferred_state.available?"true":"false")+
          ",\"pending_iteration\":"+std::to_string(deferred_state.iteration)+
          ",\"pending_hash\":\""+(deferred_state.available?std::to_string(delivery_candidate::hash(deferred_state.x)):"")+"\""+
          ",\"pending_cost\":"+feasible_quality_polish::number(deferred_state.cost)+
          ",\"pending_objective\":"+feasible_quality_polish::snapshotJson(deferred_state.snapshot)+
          ",\"pending_candidate\":"+(deferred_state.available?temporal_capture::candidateJson(deferred_candidate):"null")+
          ",\"post_first_accepted_count\":"+std::to_string(deferred_after_count)+
          ",\"deferred_check_attempted\":"+(deferred_attempted?"true":"false")+
          ",\"deferred_check_reason\":\""+deferred_reason+"\""+
          ",\"deferred_check_native_reason\":\""+deferred_check.native_reason+"\""+
          ",\"deferred_check_collision_type\":"+std::to_string(deferred_check.collision_type)+
          ",\"deferred_check_rejection_time\":"+feasible_quality_polish::number(deferred_check.rejection_time)+
          ",\"deferred_check_native_pass\":"+(deferred_check.native_pass?"true":"false")+
          ",\"deferred_check_native_ms\":"+feasible_quality_polish::number(1000.*deferred_check.native_s)+
          ",\"deferred_check_wheel_ms\":"+feasible_quality_polish::number(1000.*deferred_check.wheel_s)+
          ",\"deferred_check_total_ms\":"+feasible_quality_polish::number(1000.*(deferred_check.end_mono_s-deferred_check.begin_mono_s))+
          ",\"deferred_check_elapsed_begin_s\":"+feasible_quality_polish::number(deferred_check.elapsed_begin_s)+
          ",\"deferred_check_elapsed_end_s\":"+feasible_quality_polish::number(deferred_check.elapsed_end_s)+
          ",\"promoted\":"+(deferred_promoted?"true":"false")+
          ",\"fallback_to_first\":"+(deferred_enabled&&quality_checkpoint.available&&!deferred_promoted?"true":"false")+
          ",\"selected_hash\":\""+(quality_checkpoint.available?std::to_string(delivery_candidate::hash(quality_checkpoint.best_x)):"")+"\""+
          ",\"selected_cost\":"+feasible_quality_polish::number(quality_checkpoint.best_cost)+
          ",\"last_actual_accepted_hash\":\""+std::to_string(deferred_actual_hash)+"\""+
          ",\"last_actual_accepted_cost\":"+feasible_quality_polish::number(deferred_actual_cost)+
          ",\"last_actual_accepted_steps\":"+std::to_string(deferred_actual_steps)+
          ",\"scope\":\"pending is unverified; selected is budget-valid native-certified; ordinary final/latest gates still decide delivery\"";
      });
    }
'''


def transform_optimizer(text):
    text = replace_once(text, '    /* ---------- optimize ---------- */',
        '''    // Both arms return the first safe initial plan; treatment begins at replan.
    early_delivery_session.polish=feasible_quality_polish::requested()&&dual_probe::request()>1;
    /* ---------- optimize ---------- */''')
    text = replace_once(text, '    } catch(const domain_step::RejectedTrial& error) {',
        '''    } catch(const deferred_quality::ReserveStop&) {
      // Preserve the actual accepted bookkeeping. The certified selected x is
      // rebuilt below after the one deferred native check.
      result=domain_lbfgs::LBFGS_CANCELED;
      if(accepted_candidate.usable()){x=accepted_candidate.x;final_cost=accepted_candidate.cost;}
      delivery_trial.finished(x,final_cost,result);
    } catch(const domain_step::RejectedTrial& error) {''')
    text = replace_once(text, '    dual_probe::emit("domain_solver_status",dual_probe::Fields()',
        '''    early_delivery_session.finalize(force_stop_type_!=STOP_FOR_ERROR&&!accepted_candidate.invalid);
    dual_probe::emit("domain_solver_status",dual_probe::Fields()''')
    text = replace_once(text, '      accepted_candidate.x=early_delivery_session.selected_x;\n'
        '      accepted_candidate.cost=early_delivery_session.quality.best_cost;\n'
        '      accepted_candidate.available=true; // preserve invalid/error state verbatim\n',
        '      // selected_x is an independent certified incumbent; the actual\n'
        '      // last accepted Scope remains untouched for truthful audit.\n')
    text = replace_once(text, '    probe.add("iterations",iter_num_);probe.add("lbfgs_result",result);',
                        REPORT+'    probe.add("iterations",iter_num_);probe.add("lbfgs_result",result);')
    text = replace_once(text, '    candidate_audit::recordEvaluationBegin();',
        '''    // Cooperative reserve stop precedes beginEvaluation: an unstarted
    // trial must not invalidate the preceding actual accepted record.
    PolyTrajOptimizer* reserve_opt=reinterpret_cast<PolyTrajOptimizer*>(instance);
    if(reserve_opt->force_stop_type_==DONT_STOP&&early_delivery::reserveStopDue())
      throw deferred_quality::ReserveStop();
    candidate_audit::recordEvaluationBegin();''')
    return text
