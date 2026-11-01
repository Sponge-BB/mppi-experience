// Same complete native input and original unspent request budget. No plant,
// sensing stream, hardware, or trajectory/control publisher is instantiated.
#include <feasibility_replay_fixture.hpp>
#include <early_collision_repair.hpp>

namespace feasibility_replay {
struct Gate {
  bool pass=false;
  std::string reason="not_checked";
  double time=0.,value=0.,wall=0.,duration=0.;
};
Gate gate(Fixture& f,const Eigen::VectorXd& x){
  const auto started=ros::WallTime::now();
  early_delivery::Checker checker(f.pieces,f.gears,f.opt->iniState_container_,f.opt->finState_container_,
    f.opt->non_singul_v_,f.opt->mobile_base_wheel_base_,f.opt->mobile_base_wheel_radius_,
    f.opt->max_wheel_omega_,f.opt->max_wheel_alpha_);
  const auto trajectory=checker.trajectory(x);
  failure_probe::Frame verdict(1000.,f.opt->max_wheel_omega_,0,
    f.map->getDynamicObservationSeq(),f.map->getESDFObservationSeq());
  verdict.reason="ok";
  handoff_alignment::FinalGate final_gate;
  Gate out;out.pass=f.opt->IsTrajSafe(trajectory);out.reason=verdict.reason;
  out.time=verdict.rejection_time;out.value=verdict.rejection_value;
  out.duration=trajectory.duration;out.wall=(ros::WallTime::now()-started).toSec();return out;
}

struct Stage {
  std::string name;
  int result=domain_lbfgs::LBFGSERR_UNKNOWNERROR,force=0,evaluations=0,accepted=0;
  bool domain_rejected=false,early_selected=false,candidate_usable=false;
  double began=0.,solver_wall=0.,end=0.,initial_cost=0.,final_cost=0.;
  unsigned long long initial_x_hash=0,initial_gradient_hash=0,return_hash=0;
  Eigen::VectorXd x;
  Gate final_gate;
  std::vector<candidate_audit::Step> steps;
  std::vector<early_delivery::Check> checks;
};

Stage stage(Fixture& f,const Eigen::VectorXd& origin,const Eigen::VectorXd& scales,const std::string& name){
  Stage out;out.name=name;
  const auto elapsed=[&](){return (ros::WallTime::now()-f.opt->optimization_start_wall_time_).toSec();};
  out.began=elapsed();const auto started=ros::WallTime::now();const int old_evaluations=f.opt->iter_num_;
  candidate_audit::Session record(true);delivery_candidate::Scope accepted(true);
  early_delivery::Checker checker(f.pieces,f.gears,f.opt->iniState_container_,f.opt->finState_container_,
    f.opt->non_singul_v_,f.opt->mobile_base_wheel_base_,f.opt->mobile_base_wheel_radius_,
    f.opt->max_wheel_omega_,f.opt->max_wheel_alpha_);
  early_delivery::Session early(true,&checker,elapsed,f.opt->max_wall_time_,
    [&](const remani_planner::SingulTrajData& t){
      delivery_replay::RestoreFailure restore;early_delivery::NativeVerdict v;
      v.pass=f.opt->IsTrajSafe(t);const auto* frame=failure_probe::active();
      v.reason=v.pass?"ok":(frame?frame->reason:"unknown");
      if(frame){v.collision_type=int(frame->rejection_value);v.rejection_time=frame->rejection_time;}
      return v;
    });
  Coordinates c{&f,origin,scales,{}};Eigen::VectorXd z=Eigen::VectorXd::Zero(origin.size());
  domain_lbfgs::lbfgs_parameter_t p;p.mem_size=256;p.g_epsilon=0.;p.delta=1e-3;
  p.max_iterations=f.opt->max_iterations_;record.begin();domain_step::reset();
  try{out.result=domain_lbfgs::lbfgs_optimize(z,out.final_cost,Coordinates::evaluate,nullptr,
      Coordinates::progress,&c,p);}
  catch(const domain_step::RejectedTrial&){out.domain_rejected=true;}
  out.solver_wall=(ros::WallTime::now()-started).toSec();
  out.x=c.physical(z);out.return_hash=delivery_candidate::hash(out.x);
  out.early_selected=early.selected;out.candidate_usable=accepted.usable();
  if(early.selected)out.x=early.selected_x;else if(accepted.usable())out.x=accepted.x;
  out.force=int(f.opt->force_stop_type_);out.evaluations=f.opt->iter_num_-old_evaluations;
  out.accepted=accepted.accepted_steps;out.initial_cost=c.first_cost;
  out.initial_x_hash=c.first_x;out.initial_gradient_hash=c.first_g;
  out.final_gate=gate(f,out.x);out.end=elapsed();out.steps=record.steps;out.checks=early.checks;return out;
}

void gateJson(std::ostream& out,const Gate& g){
  out<<"{\"native_pass\":"<<(g.pass?"true":"false")<<",\"native_reason\":"<<candidate_audit::quote(g.reason)
    <<",\"native_rejection_time\":"<<candidate_audit::number(g.time)
    <<",\"native_rejection_value\":"<<candidate_audit::number(g.value)
    <<",\"native_gate_and_reconstruction_s\":"<<g.wall<<",\"duration_s\":"<<g.duration<<'}';
}
void stageJson(std::ostream& out,const Stage& s){
  out<<"{\"name\":"<<candidate_audit::quote(s.name)<<",\"result\":"<<s.result
    <<",\"force_stop\":"<<s.force<<",\"domain_rejected\":"<<(s.domain_rejected?"true":"false")
    <<",\"start_elapsed_s\":"<<s.began<<",\"solver_wall_s\":"<<s.solver_wall
    <<",\"end_through_final_gate_s\":"<<s.end<<",\"evaluations\":"<<s.evaluations
    <<",\"accepted_steps\":"<<s.accepted<<",\"early_selected\":"<<(s.early_selected?"true":"false")
    <<",\"candidate_usable\":"<<(s.candidate_usable?"true":"false")<<",\"initial_cost\":"<<s.initial_cost
    <<",\"initial_x_hash\":"<<candidate_audit::quote(std::to_string(s.initial_x_hash))
    <<",\"initial_gradient_hash\":"<<candidate_audit::quote(std::to_string(s.initial_gradient_hash))
    <<",\"return_x_hash\":"<<candidate_audit::quote(std::to_string(s.return_hash))
    <<",\"final_x_hash\":"<<candidate_audit::quote(std::to_string(delivery_candidate::hash(s.x)))
    <<",\"final_x\":";candidate_audit::vectorJson(out,s.x);out<<",\"final_gate\":";gateJson(out,s.final_gate);
  out<<",\"native_progress_checks\":[";bool first=true;
  for(const auto& c:s.checks){if(!first)out<<',';first=false;
    out<<"{\"iteration\":"<<c.accepted_iteration<<",\"candidate_hash\":"<<candidate_audit::quote(std::to_string(c.candidate_hash))
      <<",\"elapsed_begin_s\":"<<c.elapsed_begin_s<<",\"elapsed_end_s\":"<<c.elapsed_end_s
      <<",\"wheel_pass\":"<<(c.wheel_pass?"true":"false")<<",\"native_checked\":"<<(c.native_checked?"true":"false")
      <<",\"native_pass\":"<<(c.native_pass?"true":"false")<<",\"selected\":"<<(c.selected?"true":"false")
      <<",\"budget_expired\":"<<(c.budget_expired?"true":"false")<<",\"reason\":"<<candidate_audit::quote(c.reason)
      <<",\"native_reason\":"<<candidate_audit::quote(c.native_reason)
      <<",\"collision_type\":"<<c.collision_type<<",\"rejection_time\":"<<c.rejection_time<<'}';}
  out<<"],\"accepted_trace\":[";first=true;
  for(const auto& a:s.steps){if(!first)out<<',';first=false;
    out<<"{\"iteration\":"<<a.iteration<<",\"elapsed_s\":"<<a.solver_elapsed_s
      <<",\"cost\":"<<a.cost<<",\"x\":";candidate_audit::vectorJson(out,a.x);out<<'}';}
  out<<"]}";
}

void solve(std::ostream& out,ros::NodeHandle& nh,const Tree& root,const std::string& map_path,
    const Eigen::VectorXd& scales,bool repair){
  setenv("DYNAMIC_WINDOW_CONTINUITY","1",1);
  setenv("CANDIDATE_COLLISION_REPAIR","1",1);
  setenv("EARLY_COLLISION_REPAIR",repair?"1":"0",1);
  Fixture f(nh,root,map_path);const double budget=root.get<double>("capture.original_remaining_budget_s");
  failure_probe::Frame verdict(budget,f.opt->max_wheel_omega_,0,
    f.map->getDynamicObservationSeq(),f.map->getESDFObservationSeq());
  candidate_collision_repair::Scope objective;early_collision_repair::Session policy;
  f.reset(budget);const auto original_start=f.opt->optimization_start_wall_time_;
  std::vector<Stage> stages;stages.push_back(stage(f,f.x,scales,"normal_objective"));
  const auto& first_gate=stages.front().final_gate;
  bool repair_attempted=false;int target_trajectory=-1,target_piece=-1;
  // Reconstruct from the accepted x, never from the final rejected trial's
  // callback-owned polynomial cache. Every check and continuation uses the same
  // optimization_start_wall_time_; neither stage receives a fresh budget.
  if(!first_gate.pass&&first_gate.reason=="planning_collision"&&
      first_gate.value>=0.&&first_gate.value<=1.&&
      stages.front().result>=0&&
      f.opt->force_stop_type_==Opt::DONT_STOP&&
      (ros::WallTime::now()-original_start).toSec()<budget){
    std::vector<delivery_candidate::Part> parts;
    if(!delivery_candidate::decode(stages.front().x,f.pieces,f.opt->traj_dim_,f.opt->iniState_container_,
        f.opt->finState_container_,f.opt->non_singul_v_,parts))throw std::runtime_error("repair decode");
    std::vector<Eigen::VectorXd> times;for(const auto& part:parts)times.push_back(part.times);
    objective.target(times,first_gate.time);target_trajectory=objective.target_trajectory;target_piece=objective.target_piece;
    if(target_piece>=0){
      objective.repairing=true;objective.environment_factor=candidate_collision_repair::factor;
      policy.repairing=true;repair_attempted=true;
      const Eigen::VectorXd repaired_start=stages.front().x;
      stages.push_back(stage(f,repaired_start,scales,"failed_piece_environment_weight10"));
      objective.repairing=false;policy.repairing=false;
    }
  }
  const auto& final=stages.back();const bool within=final.end<=budget;
  const bool terminal_eligible=!final.domain_rejected&&final.force!=Opt::STOP_FOR_ERROR&&
    (final.result>=0 || (final.force==Opt::STOP_FOR_TIMEOUT&&final.candidate_usable));
  out<<"{\"kind\":\"feasibility_continuation\",\"policy\":"
    <<candidate_audit::quote(repair?"early_collision_yield_then_failed_piece_weight10":"late_failed_piece_weight10")
    <<",\"continuous\":true,\"repair_enabled\":true,\"early_yield_enabled\":"<<(repair?"true":"false")
    <<",\"early_yield_triggered\":"<<(policy.yielded?"true":"false")
    <<",\"yield_iteration\":"<<(policy.yielded?policy.trigger.accepted_iteration:0)
    <<",\"yield_elapsed_s\":"<<(policy.yielded?policy.trigger.elapsed_end_s:0.)
    <<",\"repair_attempted\":"<<(repair_attempted?"true":"false")
    <<",\"target_trajectory\":"<<target_trajectory<<",\"target_piece\":"<<target_piece
    <<",\"environment_factor\":"<<candidate_collision_repair::factor
    <<",\"original_request_budget_s\":"<<root.get<double>("capture.shared_budget_s")
    <<",\"original_spent_s\":"<<root.get<double>("capture.original_elapsed_s")
    <<",\"budget_s\":"<<budget<<",\"shared_start_unchanged\":"
    <<(f.opt->optimization_start_wall_time_==original_start?"true":"false")
    <<",\"total_through_final_gate_s\":"<<final.end<<",\"within_original_remaining_budget\":"<<(within?"true":"false")
    <<",\"terminal_eligible\":"<<(terminal_eligible?"true":"false")
    <<",\"native_pass_by_original_deadline\":"<<(within&&terminal_eligible&&final.final_gate.pass?"true":"false")
    <<",\"initial_x_hash\":"<<candidate_audit::quote(std::to_string(delivery_candidate::hash(f.x)))
    <<",\"final_x\":";candidate_audit::vectorJson(out,final.x);out<<",\"final_gate\":";gateJson(out,final.final_gate);
  out<<",\"published\":false,\"original_coordinate_scale_available\":false,\"same_common_scale\":true,\"stages\":[";
  for(std::size_t i=0;i<stages.size();++i){if(i)out<<',';stageJson(out,stages[i]);}out<<"]}\n";out.flush();
}
} // namespace feasibility_replay

int main(int argc,char** argv){
  try{
    ros::init(argc,argv,"feasibility_continuation_replay",ros::init_options::NoSigintHandler);
    if(argc!=5)throw std::runtime_error("usage input.json map.json output.jsonl baseline|repair|both");
    const std::string mode=argv[4];
    if(mode!="baseline"&&mode!="repair"&&mode!="both")throw std::runtime_error("unknown mode");
    Tree root;boost::property_tree::read_json(argv[1],root);
    ros::Time now;now.fromSec(root.get<double>("map.sim"));ros::Time::setNow(now);
    ros::NodeHandle nh("/remani_planner_node");std::ofstream out(argv[3]);
    if(!out)throw std::runtime_error("output");out<<std::setprecision(17);
    setenv("DYNAMIC_WINDOW_CONTINUITY","0",1);setenv("EARLY_COLLISION_REPAIR","0",1);
    Eigen::VectorXd scales;
    {
      Fixture f(nh,root,argv[2]);Eigen::VectorXd g;
      failure_probe::Frame verdict(1000.,f.opt->max_wheel_omega_,0,
        f.map->getDynamicObservationSeq(),f.map->getESDFObservationSeq());
      const double cost=f.evaluate(f.x,g);
      const bool cost_matches=cost==root.get<double>("capture.accepted_evaluation.cost");
      const bool x_matches=std::to_string(delivery_candidate::hash(f.x))==root.get<std::string>("capture.accepted_evaluation.x_hash");
      const bool g_matches=std::to_string(delivery_candidate::hash(g))==root.get<std::string>("capture.accepted_evaluation.gradient_hash");
      out<<"{\"kind\":\"original_exact_check\",\"cost\":"<<cost<<",\"cost_exact\":"<<(cost_matches?"true":"false")
        <<",\"x_exact\":"<<(x_matches?"true":"false")<<",\"gradient_exact\":"<<(g_matches?"true":"false")<<"}\n";out.flush();
      if(!cost_matches||!x_matches||!g_matches)throw std::runtime_error("saved native objective or gradient differs; comparison forbidden");
      cheap_variable_scaling::Context proxy(f.opt.get());const auto prepared=proxy.estimate(f.x);scales=prepared.scale;
      out<<"{\"kind\":\"common_scale\",\"original_scale_available\":false,\"setup_s\":"<<prepared.elapsed_s
        <<",\"preparation_outside_equal_continuation_budgets\":true,\"scale\":";
      candidate_audit::vectorJson(out,scales);out<<"}\n";out.flush();
    }
    if(mode=="baseline"||mode=="both")feasibility_replay::solve(out,nh,root,argv[2],scales,false);
    if(mode=="repair"||mode=="both")feasibility_replay::solve(out,nh,root,argv[2],scales,true);
    ros::shutdown();return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
