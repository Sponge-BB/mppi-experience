"""Only slow candidate hard admission changes; soft/actual execution stay fixed."""
from native_collision_runtime import replace_once


def transform_collision(text):
    text=replace_once(text,'#include "dual_timescale_probe.hpp"',
        '#include "dual_timescale_probe.hpp"\n#include "slow_reference_contract.hpp"')
    text=replace_once(text,'    .str("execution_scales","1,0.5")',
        '    .str("execution_scales","1,0.5 (soft objective)")\n'
        '    .str("hard_execution_scales",slow_reference_contract::nominalOnly()?"1":"1,0.5")\n'
        '    .add("slow_reference_nominal_only",slow_reference_contract::nominalOnly()?"true":"false")')
    start=text.index('inline double hardDistance(')
    end=text.index('\ntemplate<class Model,class Trajectory>',start)
    body=text[start:end]
    body=replace_once(body,'  for(const auto& b:s->branches){',
        '  for(std::size_t branch_index=0;branch_index<s->branches.size();++branch_index){\n'
        '    if(!slow_reference_contract::includeHardBranch(branch_index))continue;\n'
        '    const auto& b=s->branches[branch_index];')
    return text[:start]+body+text[end:]


def transform_fsm(text):
    text=replace_once(text,'#include "coverage_collision.hpp"',
        '#include "coverage_collision.hpp"\n#include "slow_reference_contract.hpp"')
    start=text.index('  bool REMANIReplanFSM::validatePredictedDynamicTrajectory(')
    end=text.index('  void REMANIReplanFSM::publishDynamicPointFlowStatus(',start)
    body=text[start:end]
    old='''    std::vector<double> execution_scales{
        dynamic_point_flow_active_resume_scale_,
        std::max(0.1, 0.5 * dynamic_point_flow_active_resume_scale_)};'''
    new='''    const bool geometric_reference_only=local_execution_owner_&&phase_aligned&&
        slow_reference_contract::nominalOnly();
    std::vector<double> execution_scales{dynamic_point_flow_active_resume_scale_};
    if(!geometric_reference_only)
      execution_scales.push_back(std::max(0.1,0.5*dynamic_point_flow_active_resume_scale_));
    dual_probe::emit("slow_reference_latest_contract",dual_probe::Fields()
      .add("nominal_only",geometric_reference_only?"true":"false")
      .add("nominal_scale",dynamic_point_flow_active_resume_scale_)
      .add("hard_scale_count",execution_scales.size())
      .str("scope","slow reference admission; actual MPPI rollout and brake gate unchanged"));'''
    body=replace_once(body,old,new)
    return text[:start]+body+text[end:]
