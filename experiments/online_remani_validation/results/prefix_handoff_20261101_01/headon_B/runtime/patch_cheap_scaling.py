#!/usr/bin/env python3
"""Optimizer-only, default-off coordinate change in a new isolated child."""
from build_state_contract import Transformer


CALLBACKS = r'''
namespace cheap_online_callbacks {
struct Context {
  using Vector=Eigen::VectorXd;
  using Evaluate=double(*)(void*,const Vector&,Vector&);
  using Progress=int(*)(void*,const Vector&,const Vector&,double,double,int,int);
  void* native;Evaluate evaluate_native;Progress progress_native;
  Vector origin,scale,last_gradient;
  Vector physical(const Vector& z)const{
    Vector x=origin;
    for(int i=0;i<z.size();++i)if(z[i]!=0.)x[i]+=scale[i]*z[i];
    return x;
  }
  static double evaluate(void* raw,const Vector& z,Vector& gradient){
    auto& c=*static_cast<Context*>(raw);
    c.last_gradient=Vector::Zero(z.size()); // native callback maps into pre-sized storage
    const double f=c.evaluate_native(c.native,c.physical(z),c.last_gradient);
    gradient=c.scale.cwiseProduct(c.last_gradient);return f;
  }
  static int progress(void* raw,const Vector& z,const Vector&,double f,double step,int k,int ls){
    auto& c=*static_cast<Context*>(raw);
    return c.progress_native(c.native,c.physical(z),c.last_gradient,f,step,k,ls);
  }
};
}
'''


def transform_header(text):
    t=Transformer(text)
    t.before('namespace remani_planner\n{', 'namespace cheap_variable_scaling { class Context; }\n\n')
    t.after('  class PolyTrajOptimizer\n  {', '\n    friend class cheap_variable_scaling::Context;')
    return t.finish()


def transform_optimizer(text):
    t=Transformer(text)
    t.after('#include "failure_probe.hpp"', '\n#include "cheap_variable_scaling.hpp"\n'+CALLBACKS)
    # Stable objective state is common to both groups, without expensive audit.
    t.change('const bool candidate_solver_freeze_requested=candidate_record.enabled && candidate_solver_freeze_env &&',
             'const bool candidate_solver_freeze_requested=candidate_solver_freeze_env &&')
    t.change('if(candidate_record.enabled)candidate_record.context=[&](){',
             'candidate_record.context=[&](){')
    t.before('    candidate_record.begin();',
        '    candidate_record.solver_entry_key=candidate_record.context(); // one coarse entry query only\n')
    t.change('if(candidate_record.enabled && candidate_record.context)\n      candidate_record.solver_exit_key=candidate_record.context();',
             'if(candidate_record.context)\n      candidate_record.solver_exit_key=candidate_record.context();')
    anchor='''    // native67: original coordinates, without diagnostic scaling.
    result = domain_lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        lbfgs_params);'''
    t.change(anchor,'''    const char* cheap_switch=std::getenv("CHEAP_VARIABLE_SCALING");
    const bool cheap_enabled=cheap_switch && std::strcmp(cheap_switch,"1")==0;
    double cheap_prepare_s=0.;int cheap_proxy_evaluations=0;
    if(cheap_enabled){
      // optimization_start_wall_time_ is intentionally NOT reset after setup.
      const auto prepare_started=ros::WallTime::now();
      cheap_variable_scaling::Context proxy(this);
      const auto prepared=proxy.estimate(x);
      cheap_prepare_s=(ros::WallTime::now()-prepare_started).toSec();
      cheap_proxy_evaluations=prepared.evaluations;
      cheap_online_callbacks::Context callback{this,PolyTrajOptimizer::costFunctionCallback,
        PolyTrajOptimizer::earlyExitCallback,x,prepared.scale,{}};
      Eigen::VectorXd z=Eigen::VectorXd::Zero(x.size());
      result=domain_lbfgs::lbfgs_optimize(z,final_cost,
        cheap_online_callbacks::Context::evaluate,NULL,
        cheap_online_callbacks::Context::progress,&callback,lbfgs_params);
      x=callback.physical(z);
    }else{
      result=domain_lbfgs::lbfgs_optimize(x,final_cost,
        PolyTrajOptimizer::costFunctionCallback,NULL,
        PolyTrajOptimizer::earlyExitCallback,this,lbfgs_params);
    }
    probe.add("cheap_scaling_enabled",cheap_enabled?1:0);
    probe.add("cheap_scaling_prepare_ms",cheap_prepare_s*1000.);
    probe.add("cheap_scaling_proxy_evaluations",cheap_proxy_evaluations);
    dual_probe::emit("cheap_scaling_summary",dual_probe::Fields()
      .add("enabled",cheap_enabled?"true":"false")
      .add("prepare_ms",cheap_prepare_s*1000.)
      .add("proxy_evaluations",cheap_proxy_evaluations)
      .add("prepare_plus_solver_ms",(ros::WallTime::now()-optimization_start_wall_time_).toSec()*1000.)
      .add("shared_budget_s",max_wall_time_));''')
    return t.finish()
