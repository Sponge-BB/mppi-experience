#!/usr/bin/env python3
"""Small diagnostic child; strip old experimental acceptance changes entirely."""
from build_state_contract import Transformer


def transform_optimizer(text):
    t=Transformer(text)
    t.before('#include "budget_scaling.hpp"', '#include "native67_probe.hpp"\n')
    t.change('    if(delivery_replay::requested(max_wall_time_)){', '''    const auto* native67_context=boundary_admission::currentRequest();
    const auto native67_snapshot=grid_map_->getDynamicPointSnapshot(ros::Time::now());
    const bool native67_eligible=native_collision_export::enabled() && native67_context &&
      !native67_context->dynamic_repair && native67_snapshot.valid && !native67_snapshot.points.empty();
    if(native67_eligible && delivery_replay::requested(max_wall_time_)){
''')
    t.change('const double budgets[]={.60,.60,3.00,3.00,.60};','const double budgets[]={.60,3.00};')
    t.change('const char* labels[]={"budget_original_060","budget_reference_060","budget_extended_300","budget_scaled_300","budget_repeat_060"};',
             'const char* labels[]={"native67_original_060","native67_extended_300"};')
    t.change('for(size_t i=0;i<5;++i){','for(size_t i=0;i<2;++i){')
    t.after('      failure_probe::PauseOwned delivery_paused;', '''
      if(!mm_config_->hasCustomBaseSpheres())throw std::runtime_error("native67 requires candidate B geometry");
      std::vector<Eigen::Vector3d> native67_base_points;mm_config_->getCarPts(Eigen::Vector3d::Zero(),native67_base_points);
      if(native67_base_points.size()!=47)throw std::runtime_error("native67 fixed sphere count is not 47");
''')
    t.after('      const auto native_dynamic=grid_map_->getDynamicPointSnapshot(native_query_time);', '''
      if(!native_dynamic.valid||native_dynamic.points.empty())
        throw std::runtime_error("eligible native dynamic snapshot became invalid during pause");
''')
    t.after('      delivery_replay::emit(captured.str());', '''
      delivery_replay::emit("\\\"kind\\\":\\\"capture_predicate\\\",\\\"ordinary_request\\\":true,\\\"nonempty_valid_native_dynamic\\\":true,\\\"fixed_spheres\\\":47,\\\"selection\\\":\\\"first eligible refresh before observing its solver outcome\\\",\\\"map_callback_owner\\\":\\\"blocked single ros::spin queue; immutable native overlay\\\"");
''')
    # Original parent when diagnostics are off has NONE of these extra gates.
    start=text.index('    budget_scaling::FinalState budget_final;')
    end=text.index('    /* ---------- optimize ---------- */',start)
    t.change(text[start:end], '    delivery_trial.begin();\n\n')
    start=text.index('    const bool scaled_coordinates=delivery_replay::running()')
    end=text.index('    result = domain_lbfgs::lbfgs_optimize(',start)
    t.change(text[start:end],'    // native67: original coordinates, without diagnostic scaling.\n')
    t.change('''    }
    budget_final.solver_result_known=true;budget_final.solver_result=result;
    } catch(const domain_step::RejectedTrial& error) {''','''    delivery_trial.finished(x,final_cost,result);
    } catch(const domain_step::RejectedTrial& error) {''')
    start=text.index('    if(delivery_replay::running()){\n      budget_final.native_known=true;')
    end=text.index('    failure_probe::trajectoryStats',start)
    t.change(text[start:end],'    // native67: native final collision/feasibility gate only.\n')
    start=text.index('    if(delivery_replay::running()){\n      budget_final.output_constructed=true;')
    end=text.index('    diagnosis.complete=good_traj;',start)
    t.change(text[start:end],'    // native67: native return, no added deadline or cache gate.\n')
    t.change('    budget_scaling::recordCanonical(x);\n    feasibility_stop_gate::recordEvaluated(x);\n',
             '    // native67: no diagnostic stop-veto cache.\n')
    t.change('    if(opt->force_stop_type_==DONT_STOP)budget_scaling::recordAccepted(x);\n',
             '    // native67: progress observation only.\n')
    t.after('    failure_probe::Timer callback_timer(failure_probe::COST);','\n    native67_probe::Callback native67_callback;')
    t.before('    failure_probe::firstCost(costall,g);',
        '    native67_probe::completed(total_smoo_cost,total_obs_feas_costs,total_time_cost);\n')
    t.before('      auto* diagnosis_now=failure_probe::active();', '''      s<<"],"<<native67_probe::residuals(rebuilt,*mm_config_,*grid_map_,mobile_base_wheel_base_,
        mobile_base_wheel_radius_,max_wheel_omega_,max_wheel_alpha_,manipulator_thickness_,ground_safe_dis_);
''')
    t.change('s<<"],\\\"native_IsTrajSafe_pass\\\":"','s<<",\\\"native_IsTrajSafe_pass\\\":"')
    t.change('delivery_replay_complete_v1','native67_probe_complete_v1')
    return t.finish()


def transform_replay(text):
    t=Transformer(text)
    t.after('#include "boundary_admission.hpp"','\n#include "native67_probe.hpp"\n')
    t.change('if(s.size()!=5)return false;','if(s.size()!=2)return false;')
    t.after('  Trial* previous=nullptr;bool enabled=false,first_seen=false;', '''
  native67_probe::Counters counters;native67_probe::Counters* previous_counters=nullptr;
  double solver_cpu_start=0.;int solver_result=0;bool finished_seen=false;
''')
    t.change('if(!enabled)return;active()=this;accepted.reserve(512);','''if(!enabled)return;active()=this;accepted.reserve(3);
    previous_counters=native67_probe::active();native67_probe::active()=&counters;
    if(auto* f=failure_probe::active())f->profile_enabled=true; // Paused diagnostic arms only.
''')
    t.change('  void begin(){if(enabled)solver_start=failure_probe::wall();}', '''  void begin(){if(enabled){solver_start=failure_probe::wall();solver_cpu_start=native67_probe::cpu();}}
  void finished(const Eigen::VectorXd& x,double fx,int result){
    if(!enabled)return;finished_seen=true;solver_result=result;
    counters.solver_wall=failure_probe::wall()-solver_start;counters.solver_cpu=native67_probe::cpu()-solver_cpu_start;
    accepted.push_back({x,Eigen::VectorXd(),fx,counters.solver_wall,0.,0,0,"solver_return_x"});
  }''')
    start=text.index('    if(!enabled)return;++accepted_total;')
    end=text.index('\n  }\n  ~Trial()',start)
    t.change(text[start:end],'''    if(!enabled)return;++accepted_total;++counters.accepted;counters.accepted_ls_sum+=ls;
    // Keep initial and the last accepted only. No per-iteration output/audit.
    Accepted a{x,g,fx,failure_probe::wall()-solver_start,step,k,ls,"last_accepted"};
    if(accepted.size()==1)accepted.push_back(std::move(a));else accepted[1]=std::move(a);''')
    t.after('      signatures().push_back(signature);', '''
      emit("\\\"kind\\\":\\\"native67_timing\\\",\\\"backend_decision_wall_ms\\\":"+num((failure_probe::wall()-solver_start)*1000.)+
        ",\\\"backend_decision_thread_cpu_ms\\\":"+num((native67_probe::cpu()-solver_cpu_start)*1000.)+
        ",\\\"solver_result\\\":"+std::to_string(solver_result)+
        ",\\\"solver_return_seen\\\":"+(finished_seen?"true":"false")+","+native67_probe::countersJson(counters));
      native67_probe::active()=nullptr; // Audits are explicitly outside solver accounting.
''')
    t.before('    active()=previous;', '    native67_probe::active()=previous_counters;\n')
    return t.finish()


def transform_lbfgs(text):
    t=Transformer(text)
    t.before('                f = cd.proc_evaluate(cd.instance, x, g);',
             '                native67_probe::trial();\n')
    return '#include "native67_probe.hpp"\n'+t.finish()
