"""Preserve the native nonfinite rejection before optional safe-timeout recovery."""
from native_collision_runtime import replace_once


GUARD='''    if(accepted_candidate.invalid){
      diagnosis.reason="nonfinite";
      ROS_ERROR("[Optimizer] Reject trajectory after a non-finite cost or gradient.");
      return false;
    }
'''


def transform_optimizer(text):
    marker='    if(force_stop_type_ == STOP_FOR_TIMEOUT){'
    text=replace_once(text,marker,GUARD+marker)
    guard=text.index(GUARD)
    timeout=text.index(marker,guard)
    final_gate=text.index('    // test collision',timeout)
    if not guard<timeout<final_gate:
        raise ValueError('Nonfinite rejection must precede timeout recovery and final gate')
    return text
