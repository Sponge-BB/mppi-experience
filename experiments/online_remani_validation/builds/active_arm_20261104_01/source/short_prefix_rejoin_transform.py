"""Retain a short-prefix reverse hint as a uniform fallback search cost choice."""
from native_collision_runtime import replace_once


def transform_manager(text):
    text = replace_once(text, '#include "collab_factorial.hpp"',
        '#include "collab_factorial.hpp"\n#include "short_prefix_rejoin.hpp"')
    return replace_once(text,
        '''      else status=collab::search(*this,headState,tailState,start_yaw,start_singul,start_gripper,
          local_target_yaw,local_target_gripper,continous_failures_count,
          simple_path_container,yaw_list_container,initMJO_container,singul_container);''',
        '''      else {
        short_prefix_rejoin::Scope short_prefix_scope(headState.col(0),start_yaw,start_singul,
          collab::refresh()&&collab::settings().prefix);
        status=collab::search(*this,headState,tailState,start_yaw,start_singul,start_gripper,
          local_target_yaw,local_target_gripper,continous_failures_count,
          simple_path_container,yaw_list_container,initMJO_container,singul_container);
      }''')


def transform_search(text):
    text = replace_once(text, '#include "state_contract_trace.hpp"',
        '#include "state_contract_trace.hpp"\n#include "short_prefix_rejoin.hpp"')
    text = replace_once(text,
        '      if(singul * cur_node->singul < 0) tmp_g_score += traj_gear_switch_penalty_;',
        '      if(singul * cur_node->singul < 0)\n'
        '        tmp_g_score += short_prefix_rejoin::switchPenalty(traj_gear_switch_penalty_);')
    return replace_once(text,
        '  if(has_path_){\n    getKinoNode();',
        '  if(has_path_){\n'
        '    short_prefix_rejoin::selectedNodes(path_nodes_,shotptrindex_,is_shot_succ_,start_state_,end_state_);\n'
        '    getKinoNode();')
