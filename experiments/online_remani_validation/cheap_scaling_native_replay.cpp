// One frozen-request comparison. The cheap proxy sets coordinates only;
// every solver objective and every final gate remains the native one.
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
#include "cheap_variable_scaling.hpp"
#include <iostream>

using Tree=boost::property_tree::ptree;
using Opt=remani_planner::PolyTrajOptimizer;
Eigen::VectorXd vec(const Tree& p){Eigen::VectorXd v(p.size());int i=0;for(auto& c:p)v[i++]=c.second.get_value<double>();return v;}
Eigen::MatrixXd mat(const Tree& p){
  Eigen::MatrixXd v(p.get<int>("rows"),p.get<int>("cols"));const auto d=vec(p.get_child("data_column_major"));
  if(d.size()!=v.size())throw std::runtime_error("matrix size");std::copy(d.data(),d.data()+d.size(),v.data());return v;
}
std::vector<Eigen::MatrixXd> mats(const Tree& p){std::vector<Eigen::MatrixXd> r;for(auto& c:p)r.push_back(mat(c.second));return r;}

struct AuditResult {
  bool pass=false;double reconstruction_s=0.,gate_s=0.,motion_diagnostic_s=0.;
  std::string reason;
};
struct Fixture {
  std::shared_ptr<GridMap> map;std::shared_ptr<remani_planner::MMConfig> model;
  std::unique_ptr<Opt> opt;std::vector<int> pieces,gears;Eigen::VectorXd x;
  candidate_audit::Params limits;
  explicit Fixture(ros::NodeHandle& nh,const Tree& input,const Tree& stage,const std::string& map_path){
    map=std::make_shared<GridMap>();GridMapParallelTestAccessor::restoreSnapshot(*map,map_path);
    model=std::make_shared<remani_planner::MMConfig>();model->setParam(nh,map);
    opt.reset(new Opt);opt->setParam(nh,map,model);
    opt->iniState_container_=mats(input.get_child("effective_initial_states"));
    opt->finState_container_=mats(input.get_child("effective_final_states"));
    for(auto& c:input.get_child("directions"))gears.push_back(c.second.get_value<int>());
    for(auto& c:stage.get_child("parts"))pieces.push_back(c.second.get_child("times").size());
    opt->traj_num_=pieces.size();opt->piece_num_container_=pieces;opt->singul_container_=gears;
    opt->SnapOpt_container_.resize(pieces.size());opt->cps_container_.resize(pieces.size());
    for(std::size_t n=0;n<pieces.size();++n){opt->SnapOpt_container_[n].reset(pieces[n]);opt->cps_container_[n].resize_cp(pieces[n]*opt->cps_num_prePiece_+1);}
    x=vec(stage.get_child("x"));opt->variable_num_=x.size();
    opt->mani_hold_enabled_=opt->wei_mani_hold_>0.;
    opt->mani_hold_reference_=opt->iniState_container_.front().col(0).tail(opt->manipulator_dof_);
    opt->max_wheel_omega_=input.get<double>("limits.wheel_limit");opt->max_wheel_alpha_=input.get<double>("limits.alpha_limit");
    opt->max_vel_=opt->max_wheel_omega_*opt->mobile_base_wheel_radius_;
    opt->max_acc_=opt->max_wheel_alpha_*opt->mobile_base_wheel_radius_;
    limits.wheel_base=opt->mobile_base_wheel_base_;limits.wheel_radius=opt->mobile_base_wheel_radius_;
    limits.wheel_limit=opt->max_wheel_omega_;limits.alpha_limit=opt->max_wheel_alpha_;
    limits.arm_radius=opt->manipulator_thickness_;limits.ground=opt->ground_safe_dis_;
    limits.base_safe_margin=opt->safe_margin_;limits.arm_safe_margin=opt->safe_margin_mani_;
    limits.joint_vel_limit=opt->max_joint_vel_;limits.joint_acc_limit=opt->max_joint_acc_;reset(1000.);
  }
  void reset(double budget){opt->force_stop_type_=Opt::DONT_STOP;opt->iter_num_=0;opt->max_wall_time_=budget;opt->optimization_start_wall_time_=ros::WallTime::now();}
  double evaluate(const Eigen::VectorXd& value,Eigen::VectorXd& g){g.resize(value.size());g.setZero();return Opt::costFunctionCallback(opt.get(),value,g);}
  remani_planner::SingulTrajData trajectory(const Eigen::VectorXd& value){
    std::vector<delivery_candidate::Part> p;
    if(!delivery_candidate::decode(value,pieces,opt->traj_dim_,opt->iniState_container_,opt->finState_container_,opt->non_singul_v_,p))throw std::runtime_error("decode");
    remani_planner::SingulTrajData out;out.clearSingulTraj();double t=0;
    for(std::size_t n=0;n<p.size();++n){opt->SnapOpt_container_[n].generate(p[n].points,p[n].times,p[n].initial,p[n].final);out.addSingulTraj(opt->SnapOpt_container_[n].getTraj(gears[n]),t);t=out.duration;}return out;
  }
  AuditResult audit(std::ostream& out,const Eigen::VectorXd& value){
    AuditResult a;auto began=ros::WallTime::now();auto t=trajectory(value);a.reconstruction_s=(ros::WallTime::now()-began).toSec();
    failure_probe::Frame verdict(1000.,limits.wheel_limit,0,0,0);verdict.reason="ok";
    began=ros::WallTime::now();a.pass=opt->IsTrajSafe(t);a.gate_s=(ros::WallTime::now()-began).toSec();a.reason=verdict.reason;
    began=ros::WallTime::now();candidate_audit::Summary s;
    for(double at=.05;at<std::floor(t.duration/.01)*.01;at+=.01)s.add(candidate_audit::motionSample(t,limits,at),limits);
    a.motion_diagnostic_s=(ros::WallTime::now()-began).toSec();
    out<<"\"native_pass\":"<<(a.pass?"true":"false")<<",\"native_reason\":"<<candidate_audit::quote(a.reason)
       <<",\"native_rejection_value\":"<<candidate_audit::number(verdict.rejection_value)<<",\"native_rejection_time\":"<<candidate_audit::number(verdict.rejection_time)
       <<",\"duration_s\":"<<candidate_audit::number(t.duration)<<",\"trajectory_reconstruction_s\":"<<a.reconstruction_s
       <<",\"native_gate_s\":"<<a.gate_s<<",\"motion_diagnostic_s\":"<<a.motion_diagnostic_s<<",\"wheel_max\":";
    candidate_audit::vectorJson(out,s.max_wheels);return a;
  }
};

struct Coordinates {
  Fixture* fixture;bool scaled=false;Eigen::VectorXd origin,scale,last_gradient;
  bool first_seen=false;double first_cost=0.;unsigned long long first_x_hash=0,first_gradient_hash=0;
  Eigen::VectorXd physical(const Eigen::VectorXd& z)const{return scaled?origin+scale.cwiseProduct(z):z;}
  static double evaluate(void* ptr,const Eigen::VectorXd& z,Eigen::VectorXd& g){
    auto& c=*static_cast<Coordinates*>(ptr);const auto x=c.physical(z);const double cost=c.fixture->evaluate(x,c.last_gradient);
    if(!c.first_seen){c.first_seen=true;c.first_cost=cost;c.first_x_hash=delivery_candidate::hash(x);c.first_gradient_hash=delivery_candidate::hash(c.last_gradient);}
    g=c.scaled?c.scale.cwiseProduct(c.last_gradient):c.last_gradient;return cost;
  }
  static int progress(void* ptr,const Eigen::VectorXd& z,const Eigen::VectorXd&,double cost,double step,int k,int ls){
    auto& c=*static_cast<Coordinates*>(ptr);return Opt::earlyExitCallback(c.fixture->opt.get(),c.physical(z),c.last_gradient,cost,step,k,ls);
  }
};

int main(int argc,char** argv){
  try{
    ros::init(argc,argv,"cheap_scaling_native_replay",ros::init_options::NoSigintHandler);
    if(argc!=4)throw std::runtime_error("usage input.json map.json output.jsonl");
    Tree root;boost::property_tree::read_json(argv[1],root);ros::Time now;now.fromSec(root.get<double>("sim"));ros::Time::setNow(now);
    ros::NodeHandle nh("/remani_planner_node");std::ofstream out(argv[3]);if(!out)throw std::runtime_error("output");out<<std::setprecision(17);
    for(auto& item:root.get_child("checks")){
      const auto& r=item.second;Fixture f(nh,r.get_child("input"),r.get_child("stage"),argv[2]);Eigen::VectorXd g;
      candidate_audit::Session record(true);const double cost=f.evaluate(f.x,g);
      cheap_variable_scaling::Context proxy(f.opt.get());const auto cheap=proxy.evaluate(f.x);
      const auto gh=delivery_candidate::hash(g),xh=delivery_candidate::hash(f.x);
      out<<"{\"kind\":\"check\",\"label\":"<<candidate_audit::quote(r.get<std::string>("label"))<<",\"cost\":"<<cost
         <<",\"expected_cost\":"<<r.get<double>("expected_cost")<<",\"gradient_hash\":"<<candidate_audit::quote(std::to_string(gh))
         <<",\"x_hash\":"<<candidate_audit::quote(std::to_string(xh))<<",\"gradient\":";candidate_audit::vectorJson(out,g);
      out<<",\"components\":"<<native67_probe::costJson(record.costs)<<",\"cheap_proxy_cost\":"<<cheap.cost<<",\"cheap_proxy_gradient\":";
      candidate_audit::vectorJson(out,cheap.gradient);out<<',';f.audit(out,f.x);out<<"}\n";out.flush();
      const double expected=r.get<double>("expected_cost");
      if(std::abs(cost-expected)>1e-9*std::max(1.,std::abs(expected)))throw std::runtime_error("native cost replay differs from capture");
      const auto expected_g=r.get<std::string>("expected_gradient_hash","");
      if(!expected_g.empty()&&expected_g!=std::to_string(gh))throw std::runtime_error("native first-gradient hash differs from capture");
    }
    for(auto& item:root.get_child("solves")){
      const auto& r=item.second;Fixture f(nh,r.get_child("input"),r.get_child("stage"),argv[2]);
      const bool scaled=r.get<bool>("cheap_scaling");const double total_budget=r.get<double>("total_budget_s");
      Coordinates c{&f,scaled,f.x,Eigen::VectorXd::Ones(f.x.size()),{}};
      int setup_evaluations=0;f.reset(total_budget);const auto request_begin=f.opt->optimization_start_wall_time_;
      if(scaled){cheap_variable_scaling::Context proxy(f.opt.get());const auto estimated=proxy.estimate(f.x);c.scale=estimated.scale;setup_evaluations=estimated.evaluations;}
      const double setup_s=(ros::WallTime::now()-request_begin).toSec();
      candidate_audit::Session record(true);delivery_candidate::Scope accepted;
      domain_lbfgs::lbfgs_parameter_t p;p.mem_size=256;p.g_epsilon=0.;p.delta=1e-3;p.max_iterations=300;
      int result=0;double cost=0.;const bool setup_exhausted=setup_s>=total_budget;
      const auto solve_begin=ros::WallTime::now();record.begin();
      if(!setup_exhausted){
        Eigen::VectorXd coordinates=scaled?Eigen::VectorXd::Zero(f.x.size()).eval():f.x;
        result=domain_lbfgs::lbfgs_optimize(coordinates,cost,Coordinates::evaluate,nullptr,Coordinates::progress,&c,p);f.x=c.physical(coordinates);
      }
      const double solve_s=(ros::WallTime::now()-solve_begin).toSec(),budget_elapsed_s=(ros::WallTime::now()-request_begin).toSec();
      // No native gates occur inside the budget. Final returned x is checked
      // separately from the last accepted step, before historical step audits.
      std::ostringstream final_audit;const auto final=f.audit(final_audit,f.x);
      std::ostringstream latest_audit;AuditResult latest;
      if(!record.steps.empty())latest=f.audit(latest_audit,record.steps.back().x);
      const bool final_same_as_latest=!record.steps.empty()&&f.x.size()==record.steps.back().x.size()&&(f.x.array()==record.steps.back().x.array()).all();
      const bool timeout=f.opt->force_stop_type_==Opt::STOP_FOR_TIMEOUT;
      const bool timeout_policy_selected=timeout&&accepted.usable();
      std::ostringstream policy_audit;AuditResult policy;
      if(timeout_policy_selected)policy=f.audit(policy_audit,accepted.x);
      out<<"{\"kind\":\"solve\",\"label\":"<<candidate_audit::quote(r.get<std::string>("label"))
         <<",\"request_name\":"<<candidate_audit::quote(root.get<std::string>("request_name"))
         <<",\"repeat\":"<<r.get<int>("repeat")<<",\"order_index\":"<<r.get<int>("order_index")<<",\"cheap_scaling\":"<<(scaled?"true":"false")
         <<",\"result\":"<<result<<",\"force_stop\":"<<int(f.opt->force_stop_type_)<<",\"setup_exhausted\":"<<(setup_exhausted?"true":"false")
         <<",\"setup_s\":"<<setup_s<<",\"solver_s\":"<<solve_s<<",\"budget_elapsed_s\":"<<budget_elapsed_s<<",\"total_budget_s\":"<<total_budget
         <<",\"setup_evaluations\":"<<setup_evaluations<<",\"native_solver_evaluations\":"<<f.opt->iter_num_<<",\"accepted_steps\":"<<accepted.accepted_steps
         <<",\"initial_native_cost\":"<<c.first_cost<<",\"initial_native_x_hash\":"<<candidate_audit::quote(std::to_string(c.first_x_hash))
         <<",\"initial_native_gradient_hash\":"<<candidate_audit::quote(std::to_string(c.first_gradient_hash))<<",\"scales\":";candidate_audit::vectorJson(out,c.scale);
      out<<",\"returned_x\":";candidate_audit::vectorJson(out,f.x);
      out<<",\"returned_audit\":{"<<final_audit.str()<<"},\"latest_accepted_audit\":";
      if(record.steps.empty())out<<"null";else out<<'{'<<latest_audit.str()<<'}';
      out<<",\"returned_equals_latest_accepted\":"<<(final_same_as_latest?"true":"false")
         <<",\"prepare_solve_reconstruct_gate_s\":"<<budget_elapsed_s+final.reconstruction_s+final.gate_s
         <<",\"existing_timeout_policy_selected\":"<<(timeout_policy_selected?"true":"false")
         <<",\"existing_timeout_policy_usable\":"<<(accepted.usable()?"true":"false")
         <<",\"existing_timeout_policy_audit\":";
      if(timeout_policy_selected)out<<'{'<<policy_audit.str()<<'}';else out<<"null";
      out<<",\"prepare_solve_timeout_candidate_reconstruct_gate_s\":";
      if(timeout_policy_selected)out<<budget_elapsed_s+policy.reconstruction_s+policy.gate_s;else out<<"null";
      out<<",\"normal_return_audit_is_offline_reconstruction\":true,\"actual_online_publication_measured\":false"
         <<",\"posthoc_native_audits_inside_solver_budget\":false,\"historical_feasible_candidates_published\":false,\"stages\":[";
      bool first=true;for(auto& s:record.steps){if(!first)out<<',';first=false;
        out<<"{\"iteration\":"<<s.iteration<<",\"line_search\":"<<s.line_search<<",\"solver_elapsed_s\":"<<s.solver_elapsed_s
           <<",\"prepare_plus_solver_elapsed_s\":"<<setup_s+s.solver_elapsed_s<<",\"cost\":"<<s.cost
           <<",\"components\":"<<native67_probe::costJson(s.costs)<<",\"x\":";candidate_audit::vectorJson(out,s.x);out<<',';f.audit(out,s.x);out<<'}';}
      out<<"]}\n";out.flush();
      std::cout<<r.get<std::string>("label")<<" setup="<<setup_s<<" solver="<<solve_s<<" native_pass="<<final.pass<<std::endl;
    }
    ros::shutdown();return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
