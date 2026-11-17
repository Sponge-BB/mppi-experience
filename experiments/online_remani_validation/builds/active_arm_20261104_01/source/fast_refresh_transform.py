"""Add a fast avoidance notification to the existing cooperative refresh test."""
from native_collision_runtime import replace_once


def transform_fsm(text):
    text=replace_once(text,'#include "coupling_bridge.hpp"',
        '#include "coupling_bridge.hpp"\n#include "fast_refresh_feedback.hpp"')
    text=replace_once(text,'    collab::configure(nh);',
        '    collab::configure(nh);\n    fast_refresh_feedback::configure(nh);')
    start=text.index('void REMANIReplanFSM::checkCooperativeExecution(){')
    end=text.index('\n  REMANIReplanFSM::~REMANIReplanFSM()',start)
    body=text[start:end]
    body=replace_once(body,'  dual_probe::emit("cooperative_check",dual_probe::Fields()',
        '''  const bool feedback_enabled=fast_refresh_feedback::enabled();
  auto& feedback=fast_refresh_feedback::state();
  const fast_refresh_feedback::Identity feedback_identity{
    coupling_bridge::goalGeneration(),coupling_bridge::referenceRevision(),
    dual_probe::hash(execution_goal_key_)&0xffffffffULL};
  const bool fast_feedback_eligible=fast_refresh_feedback::eligible(feedback,now.toSec(),feedback_identity);
  const bool fast_feedback_request=feedback_enabled&&fast_feedback_eligible;
  const bool refresh_ready=last_cooperative_replan_.isZero()||
      (now-last_cooperative_replan_).toSec()>.75;
  const bool fast_feedback_trigger=fast_feedback_request&&refresh_ready;
  dual_probe::emit("cooperative_check",dual_probe::Fields()''')
    body=replace_once(body,'    .add("raw_seq",planner_manager_->grid_map_->getDynamicObservationSeq())',
        '''    .add("feedback_enabled",feedback_enabled?"true":"false")
    .add("fast_feedback_eligible",fast_feedback_eligible?"true":"false")
    .add("fast_feedback_trigger",fast_feedback_trigger?"true":"false")
    .add("episode",feedback.latest.episode).add("nominal_gap",feedback.latest.nominal_gap)
    .add("raw_seq",planner_manager_->grid_map_->getDynamicObservationSeq())''')
    old='''  if((blocked||continue_partial)&&(last_cooperative_replan_.isZero()||
      (now-last_cooperative_replan_).toSec()>.75)){
    dual_probe::emit("refresh_trigger",dual_probe::Fields().add("guide_id",info.traj_id).add("guide_time",current));'''
    new='''  if((blocked||continue_partial||fast_feedback_request)&&refresh_ready){
    const char* source=fast_feedback_trigger?"fast_avoidance_feedback":blocked?"guide_blocked":"partial";
    dual_probe::emit("refresh_trigger",dual_probe::Fields().add("guide_id",info.traj_id).add("guide_time",current)
      .add("blocked",blocked?"true":"false").add("continue_partial",continue_partial?"true":"false")
      .add("feedback_enabled",feedback_enabled?"true":"false")
      .add("fast_feedback_eligible",fast_feedback_eligible?"true":"false")
      .add("fast_feedback_trigger",fast_feedback_trigger?"true":"false")
      .add("episode",feedback.latest.episode).add("nominal_gap",feedback.latest.nominal_gap)
      .str("source",source));
    if(fast_feedback_trigger)fast_refresh_feedback::consume(feedback);'''
    body=replace_once(body,old,new)
    body=replace_once(body,
        '    ROS_INFO("[LocalExecution] Native guide blocked; request refresh while the validated local rollout executes.");',
        '    ROS_INFO("[LocalExecution] Request native refresh (%s) while the validated local rollout executes.",source);')
    return text[:start]+body+text[end:]
