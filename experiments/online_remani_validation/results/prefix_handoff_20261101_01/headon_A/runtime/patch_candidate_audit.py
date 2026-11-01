#!/usr/bin/env python3
"""Observe accepted iterates and audit a native rejection in a private child."""
from build_state_contract import Transformer


def transform(text):
    t = Transformer(text)
    t.after('#include "native67_probe.hpp"',
            '\n#include "candidate_audit.hpp"\n#include <sys/stat.h>\n#include <cerrno>')
    t.after('    bool salvage_attempted=false;', '''
    candidate_audit::Session candidate_record(
      std::getenv("CANDIDATE_AUDIT_TARGET") &&
      std::abs(max_wall_time_-.6)<1e-6 && native67_context &&
      !native67_context->dynamic_repair);
    const auto candidate_entry_raw=grid_map_->getDynamicObservationSeq();
    const auto candidate_entry_esdf=grid_map_->getESDFObservationSeq();
''')
    t.before('    delivery_trial.begin();', '    candidate_record.begin();\n')
    t.before('    probe.add("candidate_available",accepted_candidate.usable()?1:0);',
             '    const double candidate_solver_wall=(ros::WallTime::now()-optimization_start_wall_time_).toSec();\n')
    t.after('    delivery_candidate::completed(x,g,costall);', '''
    candidate_audit::recordCostComponents(total_smoo_cost,total_obs_feas_costs,total_time_cost);
    candidate_audit::recordInitial(x,costall);''')
    t.after('    if(opt->force_stop_type_==DONT_STOP)delivery_candidate::accepted(x,fx);', '''
    if(opt->force_stop_type_==DONT_STOP){
      const auto* saved=delivery_candidate::active();
      if(saved && saved->usable() && !saved->initial && saved->complete_evaluation &&
         saved->evaluated_cost==fx && saved->evaluated_hash==delivery_candidate::hash(x))
        candidate_audit::recordAccepted(x,fx,k,ls);
    }''')
    t.before('    // native67: native final collision/feasibility gate only.',
             '#include "candidate_audit_finish.inc"\n')
    return t.finish()
