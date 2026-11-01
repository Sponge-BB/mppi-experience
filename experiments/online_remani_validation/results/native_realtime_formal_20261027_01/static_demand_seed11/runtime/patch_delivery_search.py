#!/usr/bin/env python3
"""Pure fail-closed search instrumentation. No build, ROS or source writes."""
from pathlib import Path

ROOT=Path(__file__).resolve().parent
DEFAULT_SOURCE=ROOT/'builds/boundary_admission_20261024_02/source/kino_astar.cpp'
INCLUDE='#include "search_exit_probe.hpp"\n'
WRAPPER_INIT='''
  delivery_search_probe::Scope delivery_search_scope("frontend_wrapper",start_state,end_state,
    start_singul,max_search_time_,allocate_num_,continous_failures_count,try_astar_times_);
'''
NATIVE_INIT='''
  delivery_search_probe::Scope delivery_search_scope("kino_astar",start_state,end_state,
    getSingularity(start_state(3)),max_search_time_,allocate_num_);
'''

def _finish(reason,code,indent='    '):
    return (indent+'delivery_search_scope.finish("'+reason+'",'+code+
            ',iter_num_,use_node_num_,open_set_.size());\n')

# Exact bounded replacements, each expected once in the attested parent source.
PATCHES=[
 ('#include "state_contract_trace.hpp"\n',INCLUDE+'#include "state_contract_trace.hpp"\n'),
 ('  end_state.tail(manipulator_dof_) = end_pos.tail(manipulator_dof_);\n',
  '  end_state.tail(manipulator_dof_) = end_pos.tail(manipulator_dof_);\n'+WRAPPER_INIT),
 ('    search(start_state, end_state, init_ctrl);',
  '    delivery_search_scope.attempted();\n    search(start_state, end_state, init_ctrl);'),
 ('    // ROS_WARN("FAILED TOO MANY TIMES, use rrt!");',
  '    delivery_search_scope.skipped(continous_failures_count >= try_astar_times_ ?\n'
  '      "continuous_failure_threshold" : "start_goal_close_arm_change");\n'
  '    // ROS_WARN("FAILED TOO MANY TIMES, use rrt!");'),
 ('  sdf_map_->resetAstarBuffer();\n',
  '  delivery_search_scope.route(has_path_ ? "astar_then_arm_sampling" : "rrt_fallback");\n'
  '  sdf_map_->resetAstarBuffer();\n'),
 ('    if(achieve_hori) return KinoAstar::REACH_HORIZON;\n'
  '    else return KinoAstar::REACH_END;',
  '    if(achieve_hori) {\n'+_finish('sampling_success_horizon','KinoAstar::REACH_HORIZON','      ')+
  '      return KinoAstar::REACH_HORIZON;\n    }\n    else {\n'+
  _finish('sampling_success_end','KinoAstar::REACH_END','      ')+
  '      return KinoAstar::REACH_END;\n    }'),
 ('    return KinoAstar::NO_PATH;',
  _finish('arm_sampling_or_rrt_failed','KinoAstar::NO_PATH')+'    return KinoAstar::NO_PATH;'),
 ('int KinoAstar::search(Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state, const Eigen::Vector2d &init_ctrl){\n',
  'int KinoAstar::search(Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state, const Eigen::Vector2d &init_ctrl){\n'+NATIVE_INIT),
 ('    return START_COLLISION;',_finish('start_collision','START_COLLISION')+'    return START_COLLISION;'),
 ('    return GOAL_COLLISION;',_finish('goal_collision','GOAL_COLLISION')+'    return GOAL_COLLISION;'),
 ('        return REACH_END;',_finish('oneshot_reach_end','REACH_END','        ')+'        return REACH_END;'),
 ('      return NO_PATH;',_finish('time_budget_exhausted','NO_PATH','      ')+'      return NO_PATH;'),
 ('          return NO_PATH;',_finish('node_capacity_exhausted','NO_PATH','          ')+'          return NO_PATH;'),
 ('  return NO_PATH;',_finish('open_set_exhausted','NO_PATH','  ')+'  return NO_PATH;'),
]

def transform_search(text):
    """Return instrumented text; reject drift, ambiguous matches and reapplication."""
    if INCLUDE in text or 'delivery_search_scope' in text:
        raise ValueError('search already instrumented')
    result=text
    for old,new in PATCHES:
        # Match entire lines so indented native returns do not alias one another.
        needle='\n'+old if not old.startswith('#') else old
        replacement='\n'+new if not old.startswith('#') else new
        if result.count(needle)!=1:
            raise ValueError('expected exactly one search anchor: '+repr(old))
        result=result.replace(needle,replacement,1)
    return result

def project_off(text):
    """Remove inserted diagnostic text and recover byte-exact original source."""
    result=text
    for old,new in reversed(PATCHES):
        needle='\n'+new if not old.startswith('#') else new
        replacement='\n'+old if not old.startswith('#') else old
        if result.count(needle)!=1:
            raise ValueError('instrumented source drift: '+repr(new))
        result=result.replace(needle,replacement,1)
    return result

def self_test(source=DEFAULT_SOURCE):
    original=Path(source).read_text()
    generated=transform_search(original)
    assert project_off(generated)==original
    assert generated.count('delivery_search_scope.finish(')==9
    return {'native_exits':6,'wrapper_exits':3,'exact_off_projection':True,
            'arm_rrt_internal_exit_observed':False}

if __name__=='__main__':
    print(self_test())
