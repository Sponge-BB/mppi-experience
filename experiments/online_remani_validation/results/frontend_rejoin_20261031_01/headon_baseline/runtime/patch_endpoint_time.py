#!/usr/bin/env python3
"""Small invertible additions to the immutable, ownership-fixed wheel-cause child."""
from build_state_contract import Transformer


def transform_optimizer(text):
    t = Transformer(text)
    t.after('#include "wheel_cause_replay.hpp"', '\n#include "endpoint_time_replay.hpp"')
    t.after('      t_list[piece_num - 1] *= 1.5;', '''
      // Private child only; unset/0 preserves the original numerical output.
      if(endpoint_time::fixRequested()) t_list_container[i]=t_list;''')
    t.change('      return wheel_cause::capture(wheel_cause_trial,candidate_record,accepted_candidate,', '''
      // Keep all solver counters but audit/write only the initial and final actual iterates.
      candidate_audit::Session endpoint_record(false);
      if(!candidate_record.steps.empty()){
        endpoint_record.steps.push_back(candidate_record.steps.front());
        if(candidate_record.steps.size()>1)endpoint_record.steps.push_back(candidate_record.steps.back());
      }
      endpoint_record.solver_entry_key=candidate_record.solver_entry_key;
      endpoint_record.solver_exit_key=candidate_record.solver_exit_key;
      endpoint_record.evaluate_complete_count=candidate_record.evaluate_complete_count;
      endpoint_record.evaluate_overlay_changed_count=candidate_record.evaluate_overlay_changed_count;
      auto endpoint_capture=wheel_cause::capture(wheel_cause_trial,endpoint_record,accepted_candidate,''')
    t.after('''        [&](double squared_speed){double value=0.,gradient=0.;return smoothedLog(.1*.1-squared_speed,.001,value,gradient);});''', '''
      endpoint_capture.body+=",\\\"all_recorded_stage_count\\\":"+std::to_string(candidate_record.steps.size())+
        ",\\\"recording_policy\\\":\\\"initial_and_last_only; full accepted/evaluation counts retained\\\"";
      return endpoint_capture;''')
    return t.finish()


def transform_finish(text):
    t = Transformer(text)
    t.change('#include "wheel_cause_replay.inc"', '#include "endpoint_time_replay.inc"')
    t.after('''      for(std::size_t index=0;index<candidate_record.steps.size();++index){''', '''
        if(index!=0 && index+1!=candidate_record.steps.size())continue; // bounded initial/final audit only.''')
    return t.finish()


transform = transform_optimizer
