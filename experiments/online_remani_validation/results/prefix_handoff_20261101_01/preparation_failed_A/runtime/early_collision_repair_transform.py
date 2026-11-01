"""Earlier same-request handoff to the existing bounded environment repair."""
from native_collision_runtime import replace_once


def transform_gate(text):
    text=replace_once(text,'  int accepted_iteration=0;unsigned long long candidate_hash=0;',
        '  int accepted_iteration=0;unsigned long long candidate_hash=0;\n'
        '  int collision_type=-1;double rejection_time=0.;')
    text=replace_once(text,'struct NativeVerdict { bool pass=false;std::string reason="unknown"; };',
        'struct NativeVerdict { bool pass=false;std::string reason="unknown";\n'
        '  int collision_type=-1;double rejection_time=0.; };')
    return replace_once(text,'c.native_reason=verdict.reason;c.native_s=monotonic()-began;',
        'c.native_reason=verdict.reason;c.collision_type=verdict.collision_type;\n'
        '        c.rejection_time=verdict.rejection_time;c.native_s=monotonic()-began;')


def transform_optimizer(text):
    text=replace_once(text,'#include "window_continuity.hpp"',
        '#include "window_continuity.hpp"\n#include "early_collision_repair.hpp"')
    text=replace_once(text,'    const bool early_delivery_enabled=early_delivery::requested();',
        '    early_collision_repair::Session early_collision_repair_session;\n'
        '    const bool early_delivery_enabled=early_delivery::requested();')
    anchor='        const auto* frame=failure_probe::active();verdict.reason=verdict.pass?"ok":(frame?frame->reason:"native_reject");'
    text=replace_once(text,anchor,anchor+'\n'
        '        if(frame&&!verdict.pass){verdict.collision_type=int(frame->rejection_value);\n'
        '          verdict.rejection_time=frame->rejection_time;}')
    anchor='    // Emit buffered observations after solving; recorded monotonic timestamps'
    text=replace_once(text,anchor,
        '    if(early_collision_repair_session.yielded){\n'
        '      x=early_collision_repair_session.x;\n'
        '      if(!rebuild_exit_main(x))throw std::runtime_error("early collision repair accepted-x rebuild failed");\n'
        '      const auto& trigger=early_collision_repair_session.trigger;\n'
        '      dual_probe::emit("early_collision_repair_yield",dual_probe::Fields()\n'
        '        .str("accepted_x_hash",std::to_string(delivery_candidate::hash(x)))\n'
        '        .add("accepted_iteration",trigger.accepted_iteration).add("collision_type",trigger.collision_type)\n'
        '        .add("rejection_time",trigger.rejection_time).add("trigger_elapsed_s",trigger.elapsed_end_s)\n'
        '        .add("shared_budget_s",max_wall_time_).add("native_safe",false)\n'
        '        .str("scope","handoff to one existing repair; not a successful delivery"));\n'
        '    }\n\n'+anchor)
    anchor='    const bool early_native_done=opt->force_stop_type_==DONT_STOP && early_delivery::accepted(x,k);'
    text=replace_once(text,anchor,anchor+'\n'
        '    const bool early_collision_yield=opt->force_stop_type_==DONT_STOP && !early_native_done &&\n'
        '      early_collision_repair::shouldYield(x,k);')
    text=replace_once(text,'    return early_native_done || opt->force_stop_type_ != DONT_STOP;',
        '    return early_native_done || early_collision_yield || opt->force_stop_type_ != DONT_STOP;')
    # Independent native checks and reconstruction remain mandatory after an
    # intentional positive L-BFGS cancel. Do not report it as an optimizer error.
    text=replace_once(text,'    if((result != domain_lbfgs::LBFGS_CONVERGENCE)&&(result != domain_lbfgs::LBFGS_STOP)&&(result != domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH)){',
        '    if((result != domain_lbfgs::LBFGS_CONVERGENCE)&&(result != domain_lbfgs::LBFGS_STOP)&&(result != domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH) &&\n'
        '       !(early_collision_repair_session.yielded&&result==domain_lbfgs::LBFGS_CANCELED)){')
    return text


def transform_window(text):
    text=replace_once(text,'#include "coverage_collision.hpp"',
        '#include "coverage_collision.hpp"\n#include "candidate_collision_repair.hpp"')
    anchor='''      evaluate(p,v,gp,gv,car,arm,self);
      const double F=car+arm,geometry=gp.dot(v)+gv.dot(a);'''
    return replace_once(text,anchor,'''      evaluate(p,v,gp,gv,car,arm,self);
      candidate_collision_repair::sample(trajectory,piece,w,car,arm,0.,gp,gv,
        time_indexed_collision::sample()->temporal_cost_gradient);
      const double F=car+arm,geometry=gp.dot(v)+gv.dot(a);''')
