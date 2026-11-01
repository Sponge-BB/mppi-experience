// Complete saved native input; no simulator, hardware or control publication.
// A private ROS master supplies only the original parameter snapshot.
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <candidate_audit.hpp>
#include <delivery_accepted_candidate.hpp>
#include <failure_probe.hpp>
#include <native67_probe.hpp>
#include <domain_lbfgs.hpp>
#include <visualization_msgs/MarkerArray.h>
#include "traj_utils/plan_container.hpp"
#include "traj_utils/poly_traj_utils.hpp"
#include "path_searching/kino_astar.h"
#include "path_searching/rrt.h"
#include "mm_config/mm_config.hpp"
#define private public
#include "optimizer/poly_traj_optimizer.hpp"
#undef private
#include <delivery_replay_probe.hpp>
#include "coverage_collision.hpp"
#include "window_continuity.hpp"
#include "cheap_variable_scaling.hpp"
#include "early_delivery_gate.hpp"
#include "handoff_alignment_probe.hpp"
#include <iostream>
#include <memory>
#include <numeric>

using Tree=boost::property_tree::ptree;
using Opt=remani_planner::PolyTrajOptimizer;

Eigen::VectorXd vectorOf(const Tree& p){
  Eigen::VectorXd v(p.size());int i=0;for(const auto& c:p)v[i++]=c.second.get_value<double>();return v;
}
std::vector<Eigen::MatrixXd> matricesOf(const Tree& p){
  std::vector<Eigen::MatrixXd> out;
  for(const auto& block:p){
    const auto& rows=block.second;const int nr=rows.size();
    if(!nr)throw std::runtime_error("empty boundary matrix");
    const int nc=rows.begin()->second.size();Eigen::MatrixXd m(nr,nc);int i=0;
    for(const auto& row:rows){if(row.second.size()!=std::size_t(nc))throw std::runtime_error("ragged boundary");
      int j=0;for(const auto& x:row.second)m(i,j++)=x.second.get_value<double>();++i;}
    out.push_back(m);
  }return out;
}
std::vector<int> integersOf(const Tree& p){std::vector<int> out;for(const auto& c:p)out.push_back(c.second.get_value<int>());return out;}
coupling_objective::Target targetOf(const Tree& p){
  const auto v=vectorOf(p);if(v.size()!=6)throw std::runtime_error("target shape");
  coupling_objective::Target t;t.pos={v[0],v[1]};t.yaw=v[2];t.speed=v[3];t.time=v[4];t.gear=int(v[5]);return t;
}
coupling_bridge::Prediction predictionOf(const Tree& p){
  coupling_bridge::Prediction out;out.valid=p.get<bool>("valid");out.reason=p.get<std::string>("reason");
  out.request_time=p.get<double>("request_time");out.delay=p.get<double>("delay");
  out.seed.stamp=p.get<double>("seed_stamp");out.seed.revision=p.get<double>("seed_revision");
  out.seed.generation=p.get<double>("seed_generation");
  for(const auto& row:p.get_child("rows")){
    std::array<double,8> r{};const auto v=vectorOf(row.second);
    if(v.size()!=8)throw std::runtime_error("seed row shape");std::copy(v.data(),v.data()+8,r.begin());out.seed.rows.push_back(r);
  }
  for(const auto& t:p.get_child("execution_targets"))out.targets.execution.push_back(targetOf(t.second));
  out.targets.has_delivery=p.get<bool>("has_delivery");
  if(out.targets.has_delivery)out.targets.delivery=targetOf(p.get_child("delivery"));
  out.targets.checked_begin=p.get<double>("checked_begin");out.targets.checked_end=p.get<double>("checked_end");return out;
}

struct Fixture {
  std::shared_ptr<GridMap> map;std::shared_ptr<remani_planner::MMConfig> model;
  std::unique_ptr<Opt> opt;
  std::vector<int> pieces,gears;Eigen::VectorXd x;
  time_indexed_collision::Context context;time_indexed_collision::Context* previous_context;
  coupling_bridge::Prediction prediction;coupling_bridge::Prediction* previous_prediction;
  std::unique_ptr<coupling_bridge::Optimizer> coupling;
  Fixture(ros::NodeHandle& nh,const Tree& root,const std::string& map_path):
      previous_context(time_indexed_collision::current()),previous_prediction(coupling_bridge::activePrediction()){
    map=std::make_shared<GridMap>();GridMapParallelTestAccessor::restoreSnapshot(*map,map_path);
    model=std::make_shared<remani_planner::MMConfig>();model->setParam(nh,map);
    opt.reset(new Opt);opt->setParam(nh,map,model);
    const auto& captured=root.get_child("capture");
    opt->iniState_container_=matricesOf(captured.get_child("effective_initial_states"));
    opt->finState_container_=matricesOf(captured.get_child("effective_final_states"));
    pieces=integersOf(root.get_child("pieces"));gears=integersOf(root.get_child("gears"));
    if(pieces.size()!=gears.size()||pieces.size()!=opt->iniState_container_.size())throw std::runtime_error("part metadata shape");
    opt->traj_num_=pieces.size();opt->piece_num_container_=pieces;opt->singul_container_=gears;
    opt->SnapOpt_container_.resize(pieces.size());opt->cps_container_.resize(pieces.size());
    for(std::size_t i=0;i<pieces.size();++i){opt->SnapOpt_container_[i].reset(pieces[i]);
      opt->cps_container_[i].resize_cp(pieces[i]*opt->cps_num_prePiece_+1);}
    x=vectorOf(captured.get_child("accepted_evaluation.x"));opt->variable_num_=x.size();
    opt->mani_hold_reference_=opt->iniState_container_.front().col(0).tail(opt->manipulator_dof_);
    opt->mani_hold_enabled_=opt->wei_mani_hold_>0. &&
      (opt->finState_container_.back().col(0).tail(opt->manipulator_dof_)-opt->mani_hold_reference_).norm()<1e-3;
    const auto& s=captured.get_child("source_snapshot");auto& snapshot=context.source.snapshot;
    snapshot.valid=s.get<bool>("valid");snapshot.sequence=s.get<std::uint64_t>("sequence");
    snapshot.stamp.fromNSec(s.get<std::uint64_t>("stamp_nsec"));snapshot.receive_time.fromNSec(s.get<std::uint64_t>("receive_time_nsec"));
    snapshot.frame_id=s.get<std::string>("frame_id");snapshot.invalid_reason=s.get<std::string>("invalid_reason");
    for(const auto& row:s.get_child("points")){
      const auto v=vectorOf(row.second);if(v.size()!=8)throw std::runtime_error("dynamic point shape");
      plan_env::DynamicPoint q;q.position=v.head(3);q.velocity=v.segment(3,3);q.track_id=std::uint32_t(v[6]);q.confidence=v[7];snapshot.points.push_back(q);
    }
    const auto& p=captured.get_child("time_indexed_parameters");auto& parameters=context.source.parameters;
    parameters.max_prediction=p.get<double>("max_prediction");parameters.uncertainty0=p.get<double>("uncertainty0");
    parameters.uncertainty_v=p.get<double>("uncertainty_v");parameters.uncertainty_a=p.get<double>("uncertainty_a");
    parameters.influence=p.get<double>("influence");parameters.cap=p.get<double>("cap");
    const auto& m=root.get_child("map");parameters.horizon=m.get<double>("overlay_horizon");parameters.step=m.get<double>("overlay_time_step");
    parameters.extra=m.get<double>("overlay_extra_inflation");context.edge_radius=p.get<double>("edge_radius");
    context.source.refresh_epoch=m.get<double>("overlay_refresh_nsec")*1e-9;
    context.source.exact=true;context.source.overlay_enabled=m.get<bool>("overlay_enabled");
    context.map=map.get();context.requested_epoch=captured.get<double>("requested_epoch");context.entry_epoch=m.get<double>("sim");
    context.begin_epoch=captured.get<double>("cv_begin_epoch");context.end_epoch=captured.get<double>("cv_end_epoch");
    context.frozen=true;context.active=true;context.full_coverage=p.get<bool>("full_coverage");
    time_indexed_collision::current()=&context;
    prediction=predictionOf(captured.get_child("frozen_prediction"));coupling_bridge::activePrediction()=&prediction;
    coupling.reset(new coupling_bridge::Optimizer(0,pieces.size(),std::accumulate(pieces.begin(),pieces.end(),0),gears));
    if(!coupling->valid)throw std::runtime_error("frozen coupling prediction rejected: "+coupling->reason);
    reset(1000.);
  }
  ~Fixture(){coupling.reset();coupling_bridge::activePrediction()=previous_prediction;time_indexed_collision::current()=previous_context;}
  void reset(double budget){opt->force_stop_type_=Opt::DONT_STOP;opt->iter_num_=0;opt->max_wall_time_=budget;
    opt->optimization_start_wall_time_=ros::WallTime::now();domain_step::reset();}
  double evaluate(const Eigen::VectorXd& value,Eigen::VectorXd& g){g=Eigen::VectorXd::Zero(value.size());return Opt::costFunctionCallback(opt.get(),value,g);}
  void audit(std::ostream& out,const Eigen::VectorXd& value){
    early_delivery::Checker checker(pieces,gears,opt->iniState_container_,opt->finState_container_,opt->non_singul_v_,
      opt->mobile_base_wheel_base_,opt->mobile_base_wheel_radius_,opt->max_wheel_omega_,opt->max_wheel_alpha_);
    const auto started=ros::WallTime::now();const auto trajectory=checker.trajectory(value);
    failure_probe::Frame verdict(1000.,opt->max_wheel_omega_,0,map->getDynamicObservationSeq(),map->getESDFObservationSeq());
    verdict.reason="ok";handoff_alignment::FinalGate final_gate;
    const bool pass=opt->IsTrajSafe(trajectory);
    out<<"\"native_pass\":"<<(pass?"true":"false")<<",\"native_reason\":"<<candidate_audit::quote(verdict.reason)
       <<",\"native_rejection_value\":"<<candidate_audit::number(verdict.rejection_value)
       <<",\"native_rejection_time\":"<<candidate_audit::number(verdict.rejection_time)
       <<",\"native_gate_and_reconstruction_s\":"<<(ros::WallTime::now()-started).toSec()<<",\"duration_s\":"<<trajectory.duration;
  }
};

struct Coordinates {
  Fixture* fixture;Eigen::VectorXd origin,scale,last_gradient;
  bool first_seen=false;double first_cost=0.;unsigned long long first_x=0,first_g=0;
  Eigen::VectorXd physical(const Eigen::VectorXd& z)const{
    Eigen::VectorXd out=origin;for(int i=0;i<z.size();++i)if(z[i]!=0.)out[i]+=scale[i]*z[i];return out;
  }
  static double evaluate(void* raw,const Eigen::VectorXd& z,Eigen::VectorXd& g){
    auto& c=*static_cast<Coordinates*>(raw);const auto x=c.physical(z);const double cost=c.fixture->evaluate(x,c.last_gradient);
    if(!c.first_seen){c.first_seen=true;c.first_cost=cost;c.first_x=delivery_candidate::hash(x);c.first_g=delivery_candidate::hash(c.last_gradient);}
    g=c.scale.cwiseProduct(c.last_gradient);return cost;
  }
  static int progress(void* raw,const Eigen::VectorXd& z,const Eigen::VectorXd&,double f,double step,int k,int ls){
    auto& c=*static_cast<Coordinates*>(raw);return Opt::earlyExitCallback(c.fixture->opt.get(),c.physical(z),c.last_gradient,f,step,k,ls);
  }
};

void mechanismChecks(std::ostream& out,ros::NodeHandle& nh,const Tree& root,const std::string& map_path){
  const auto last_trial=vectorOf(root.get_child("capture.last_evaluation.x"));
  for(bool continuous:{false,true}){
    setenv("DYNAMIC_WINDOW_CONTINUITY",continuous?"1":"0",1);
    Fixture f(nh,root,map_path);f.reset(1000.);
    if(last_trial.size()!=f.x.size())throw std::runtime_error("last trial x shape");
    const auto direction=Eigen::VectorXd(last_trial-f.x);
    Eigen::VectorXd g;const double initial=f.evaluate(f.x,g);
    std::vector<std::pair<int,std::string>> indices;int at=0;
    for(std::size_t n=0;n<f.pieces.size();++n)for(int knot=0;knot<f.pieces[n]-1;++knot){
      indices.push_back({at,"base_waypoint_x"});indices.push_back({at+1,"base_waypoint_y"});at+=f.opt->traj_dim_;
    }
    for(std::size_t n=0;n<f.pieces.size();++n)for(int piece=0;piece<f.pieces[n];++piece)
      indices.push_back({at++,"virtual_duration"});
    const double epsilon=1e-6;double largest_absolute=0.,largest_scaled=0.;
    out<<"{\"kind\":\"native_central_difference\",\"continuous\":"<<(continuous?"true":"false")
      <<",\"cost_at_saved_accepted_x\":"<<initial<<",\"epsilon\":"<<epsilon
      <<",\"trajectory\":\"full native MINCO reconstruction and objective at each perturbed x\""
      <<",\"coordinates\":\"base waypoint x/y and all virtual durations; fixed arm excluded\",\"samples\":[";
    bool first=true;
    for(const auto& entry:indices){
      Eigen::VectorXd plus=f.x,minus=f.x,gplus,gminus;plus[entry.first]+=epsilon;minus[entry.first]-=epsilon;
      const double cost_plus=f.evaluate(plus,gplus),cost_minus=f.evaluate(minus,gminus);
      const double finite=(cost_plus-cost_minus)/(2.*epsilon),analytic=g[entry.first];
      const double absolute=std::abs(finite-analytic),scaled=absolute/std::max({1.,std::abs(analytic),std::abs(finite)});
      largest_absolute=std::max(largest_absolute,absolute);largest_scaled=std::max(largest_scaled,scaled);
      if(!first)out<<',';first=false;
      out<<"{\"index\":"<<entry.first<<",\"type\":"<<candidate_audit::quote(entry.second)
         <<",\"analytic\":"<<analytic<<",\"central_difference\":"<<finite
         <<",\"cost_plus\":"<<cost_plus<<",\"cost_minus\":"<<cost_minus
         <<",\"absolute_error\":"<<absolute<<",\"relative_error_max1\":"<<scaled<<'}';
    }
    out<<"],\"max_absolute_error\":"<<largest_absolute<<",\"max_relative_error_max1\":"<<largest_scaled
      <<",\"threshold_predeclared\":false}\n";out.flush();
    out<<"{\"kind\":\"saved_line_search_boundary_sweep\",\"continuous\":"<<(continuous?"true":"false")
      <<",\"line\":\"accepted_x + alpha * (saved_last_evaluation_x - accepted_x)\""
      <<",\"direction_norm\":"<<direction.norm()<<",\"samples\":[";
    first=true;
    // The saved rejected trial lies at the 3 s validity tolerance boundary.
    // Evaluate both sides through the entire native MINCO callback, without
    // substituting the accepted curve geometry or an estimated penalty.
    for(double alpha:{.9999,.999999,1.,1.000001,1.0001}){
      const Eigen::VectorXd value=alpha==1.?last_trial:Eigen::VectorXd(f.x+alpha*direction);Eigen::VectorXd gradient;
      const double cost=f.evaluate(value,gradient);std::vector<delivery_candidate::Part> parts;
      if(!delivery_candidate::decode(value,f.pieces,f.opt->traj_dim_,f.opt->iniState_container_,
          f.opt->finState_container_,f.opt->non_singul_v_,parts))throw std::runtime_error("boundary sweep decode");
      double elapsed=0.;for(int j=0;j<10;++j)elapsed+=parts.front().times[0]/f.opt->cps_num_prePiece_;
      const double raw_dt=f.context.requested_epoch+2.*elapsed-f.context.source.snapshot.stamp.toSec();
      const bool original_branch_valid=raw_dt>=-1e-6&&raw_dt<=f.context.source.parameters.max_prediction+1e-6;
      if(!first)out<<',';first=false;
      out<<"{\"alpha\":"<<alpha<<",\"cost\":"<<cost<<",\"gradient_dot_direction\":"<<gradient.dot(direction)
        <<",\"first_piece_half_speed_sample_j10_dt\":"<<raw_dt
        <<",\"original_branch_valid\":"<<(original_branch_valid?"true":"false")
        <<",\"x_hash\":"<<candidate_audit::quote(std::to_string(delivery_candidate::hash(value)))<<'}';
    }
    out<<"]}\n";out.flush();
  }
}

void solve(std::ostream& out,ros::NodeHandle& nh,const Tree& root,const std::string& map_path,
           const Eigen::VectorXd& scales,bool continuous){
  setenv("DYNAMIC_WINDOW_CONTINUITY",continuous?"1":"0",1);
  Fixture f(nh,root,map_path);const double budget=root.get<double>("capture.original_remaining_budget_s");
  failure_probe::Frame verdict(budget,f.opt->max_wheel_omega_,0,f.map->getDynamicObservationSeq(),f.map->getESDFObservationSeq());
  candidate_audit::Session record(true);delivery_candidate::Scope accepted(true);
  early_delivery::Checker checker(f.pieces,f.gears,f.opt->iniState_container_,f.opt->finState_container_,f.opt->non_singul_v_,
    f.opt->mobile_base_wheel_base_,f.opt->mobile_base_wheel_radius_,f.opt->max_wheel_omega_,f.opt->max_wheel_alpha_);
  f.reset(budget);const auto started=f.opt->optimization_start_wall_time_;
  early_delivery::Session early(true,&checker,[&](){return (ros::WallTime::now()-started).toSec();},budget,
    [&](const remani_planner::SingulTrajData& t){delivery_replay::RestoreFailure restore;early_delivery::NativeVerdict v;
      v.pass=f.opt->IsTrajSafe(t);v.reason=v.pass?"ok":verdict.reason;return v;});
  Coordinates c{&f,f.x,scales,{}};Eigen::VectorXd z=Eigen::VectorXd::Zero(f.x.size());
  domain_lbfgs::lbfgs_parameter_t p;p.mem_size=256;p.g_epsilon=0.;p.delta=1e-3;p.max_iterations=f.opt->max_iterations_;
  double cost=0.;int result=domain_lbfgs::LBFGSERR_UNKNOWNERROR;bool domain_rejected=false;record.begin();
  try{result=domain_lbfgs::lbfgs_optimize(z,cost,Coordinates::evaluate,nullptr,Coordinates::progress,&c,p);}
  catch(const domain_step::RejectedTrial&){domain_rejected=true;}
  const double solver_wall=(ros::WallTime::now()-started).toSec();
  const auto return_x=c.physical(z);Eigen::VectorXd final_x=return_x;
  if(early.selected)final_x=early.selected_x;else if(accepted.usable())final_x=accepted.x;
  out<<"{\"kind\":\"continuation\",\"continuous\":"<<(continuous?"true":"false")
    <<",\"result\":"<<result<<",\"force_stop\":"<<int(f.opt->force_stop_type_)<<",\"domain_rejected\":"<<(domain_rejected?"true":"false")
    <<",\"budget_s\":"<<budget<<",\"solver_wall_s\":"<<solver_wall<<",\"evaluations\":"<<f.opt->iter_num_
    <<",\"accepted_steps\":"<<accepted.accepted_steps<<",\"early_selected\":"<<(early.selected?"true":"false")
    <<",\"initial_cost\":"<<c.first_cost<<",\"initial_x_hash\":"<<candidate_audit::quote(std::to_string(c.first_x))
    <<",\"initial_gradient_hash\":"<<candidate_audit::quote(std::to_string(c.first_g))
    <<",\"final_x_hash\":"<<candidate_audit::quote(std::to_string(delivery_candidate::hash(final_x)))
    <<",\"return_x_hash\":"<<candidate_audit::quote(std::to_string(delivery_candidate::hash(return_x)))<<",\"final_x\":";
  candidate_audit::vectorJson(out,final_x);out<<",\"final_gate\":{";f.audit(out,final_x);
  out<<"},\"total_through_final_gate_s\":"<<(ros::WallTime::now()-started).toSec()
    <<",\"original_coordinate_scale_available\":false,\"same_common_scale\":true,\"published\":false,\"stages\":[";
  bool first=true;for(const auto& s:record.steps){if(!first)out<<',';first=false;
    out<<"{\"iteration\":"<<s.iteration<<",\"elapsed_s\":"<<s.solver_elapsed_s<<",\"cost\":"<<s.cost<<",\"x\":";
    candidate_audit::vectorJson(out,s.x);out<<'}';}
  out<<"]}\n";out.flush();
}

int main(int argc,char** argv){
  try{
    ros::init(argc,argv,"window_continuity_replay",ros::init_options::NoSigintHandler);
    if(argc!=5)throw std::runtime_error("usage input.json map.json output.jsonl check|old|continuous|both");
    const std::string mode=argv[4];if(mode!="check"&&mode!="old"&&mode!="continuous"&&mode!="both")throw std::runtime_error("unknown mode");
    Tree root;boost::property_tree::read_json(argv[1],root);
    ros::Time now;now.fromSec(root.get<double>("map.sim"));ros::Time::setNow(now);
    ros::NodeHandle nh("/remani_planner_node");std::ofstream out(argv[3]);if(!out)throw std::runtime_error("output");out<<std::setprecision(17);
    setenv("DYNAMIC_WINDOW_CONTINUITY","0",1);Eigen::VectorXd scales;
    {
      Fixture f(nh,root,argv[2]);Eigen::VectorXd g;
      failure_probe::Frame verdict(1000.,f.opt->max_wheel_omega_,0,f.map->getDynamicObservationSeq(),f.map->getESDFObservationSeq());
      const double cost=f.evaluate(f.x,g);const auto xh=delivery_candidate::hash(f.x),gh=delivery_candidate::hash(g);
      const bool cost_matches=cost==root.get<double>("capture.accepted_evaluation.cost");
      const bool x_matches=std::to_string(xh)==root.get<std::string>("capture.accepted_evaluation.x_hash");
      const bool g_matches=std::to_string(gh)==root.get<std::string>("capture.accepted_evaluation.gradient_hash");
      out<<"{\"kind\":\"original_exact_check\",\"cost\":"<<cost<<",\"expected_cost\":"<<root.get<double>("capture.accepted_evaluation.cost")
        <<",\"x_hash\":"<<candidate_audit::quote(std::to_string(xh))<<",\"gradient_hash\":"<<candidate_audit::quote(std::to_string(gh))
        <<",\"cost_exact\":"<<(cost_matches?"true":"false")<<",\"x_exact\":"<<(x_matches?"true":"false")
        <<",\"gradient_exact\":"<<(g_matches?"true":"false")<<",\"gradient\":";candidate_audit::vectorJson(out,g);
      out<<",\"gate\":{";f.audit(out,f.x);out<<"}}\n";out.flush();
      if(!cost_matches||!x_matches||!g_matches)throw std::runtime_error("saved original native objective or gradient does not match exactly; continuations forbidden");
      cheap_variable_scaling::Context proxy(f.opt.get());const auto prepared=proxy.estimate(f.x);scales=prepared.scale;
      out<<"{\"kind\":\"common_scale\",\"original_scale_available\":false,\"source\":\"cheap native motion proxy estimated once at captured accepted_x\",\"setup_s\":"<<prepared.elapsed_s
        <<",\"proxy_evaluations\":"<<prepared.evaluations<<",\"preparation_outside_equal_continuation_budgets\":true,\"scale\":";
      candidate_audit::vectorJson(out,scales);out<<"}\n";out.flush();
    }
    mechanismChecks(out,nh,root,argv[2]);
    if(mode=="old"||mode=="both")solve(out,nh,root,argv[2],scales,false);
    if(mode=="continuous"||mode=="both")solve(out,nh,root,argv[2],scales,true);
    ros::shutdown();return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
