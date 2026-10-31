// Same frozen input and native libraries; no simulator, controller or publisher.
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
#include <iostream>

using Tree=boost::property_tree::ptree;
using Opt=remani_planner::PolyTrajOptimizer;
Eigen::VectorXd vec(const Tree& p){Eigen::VectorXd v(p.size());int i=0;for(auto& c:p)v[i++]=c.second.get_value<double>();return v;}
Eigen::MatrixXd mat(const Tree& p){
  Eigen::MatrixXd v(p.get<int>("rows"),p.get<int>("cols"));const auto d=vec(p.get_child("data_column_major"));
  if(d.size()!=v.size())throw std::runtime_error("matrix size");std::copy(d.data(),d.data()+d.size(),v.data());return v;
}
std::vector<Eigen::MatrixXd> mats(const Tree& p){std::vector<Eigen::MatrixXd> r;for(auto& c:p)r.push_back(mat(c.second));return r;}
struct Fixture {
  std::shared_ptr<GridMap> map;
  std::shared_ptr<remani_planner::MMConfig> model;
  std::unique_ptr<Opt> opt;
  std::vector<int> pieces,gears;
  Eigen::VectorXd x;
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
    // Exact live runtime limits, rather than the pre-launch parameter dump.
    opt->max_wheel_omega_=input.get<double>("limits.wheel_limit");opt->max_wheel_alpha_=input.get<double>("limits.alpha_limit");
    opt->max_vel_=opt->max_wheel_omega_*opt->mobile_base_wheel_radius_;
    opt->max_acc_=opt->max_wheel_alpha_*opt->mobile_base_wheel_radius_;
    limits.wheel_base=opt->mobile_base_wheel_base_;limits.wheel_radius=opt->mobile_base_wheel_radius_;
    limits.wheel_limit=opt->max_wheel_omega_;limits.alpha_limit=opt->max_wheel_alpha_;
    limits.arm_radius=opt->manipulator_thickness_;limits.ground=opt->ground_safe_dis_;
    limits.base_safe_margin=opt->safe_margin_;limits.arm_safe_margin=opt->safe_margin_mani_;
    limits.joint_vel_limit=opt->max_joint_vel_;limits.joint_acc_limit=opt->max_joint_acc_;
    reset(1000.);
  }
  void reset(double budget){opt->force_stop_type_=Opt::DONT_STOP;opt->iter_num_=0;opt->max_wall_time_=budget;opt->optimization_start_wall_time_=ros::WallTime::now();}
  double evaluate(const Eigen::VectorXd& value,Eigen::VectorXd& g){g.resize(value.size());g.setZero();return Opt::costFunctionCallback(opt.get(),value,g);}
  remani_planner::SingulTrajData trajectory(const Eigen::VectorXd& value){
    std::vector<delivery_candidate::Part> p;
    if(!delivery_candidate::decode(value,pieces,opt->traj_dim_,opt->iniState_container_,opt->finState_container_,opt->non_singul_v_,p))throw std::runtime_error("decode");
    remani_planner::SingulTrajData out;out.clearSingulTraj();double t=0;
    for(std::size_t n=0;n<p.size();++n){opt->SnapOpt_container_[n].generate(p[n].points,p[n].times,p[n].initial,p[n].final);out.addSingulTraj(opt->SnapOpt_container_[n].getTraj(gears[n]),t);t=out.duration;}
    return out;
  }
  void audit(std::ostream& out,const Eigen::VectorXd& value){
    auto t=trajectory(value);candidate_audit::Summary s;for(double at=.05;at<std::floor(t.duration/.01)*.01;at+=.01)s.add(candidate_audit::motionSample(t,limits,at),limits);
    failure_probe::Frame f(1000.,limits.wheel_limit,0,0,0);f.reason="ok";
    const bool good=opt->IsTrajSafe(t);
    out<<"\"native_pass\":"<<(good?"true":"false")<<",\"native_reason\":"<<candidate_audit::quote(f.reason)
       <<",\"native_rejection_value\":"<<candidate_audit::number(f.rejection_value)<<",\"native_rejection_time\":"<<candidate_audit::number(f.rejection_time)
       <<",\"duration_s\":"<<candidate_audit::number(t.duration)<<",\"wheel_max\":";candidate_audit::vectorJson(out,s.max_wheels);
  }
};

int main(int argc,char** argv){
  try{
    ros::init(argc,argv,"rootcause_native_replay",ros::init_options::NoSigintHandler);
    if(argc!=4)throw std::runtime_error("usage input.json map.json output.jsonl");
    Tree root;boost::property_tree::read_json(argv[1],root);
    ros::Time now;now.fromSec(root.get<double>("sim"));ros::Time::setNow(now);
    ros::NodeHandle nh("/remani_planner_node");std::ofstream out(argv[3]);if(!out)throw std::runtime_error("output");out<<std::setprecision(17);
    for(auto& item:root.get_child("checks")){
      const auto& r=item.second;const auto& input=r.get_child("input");const auto& stage=r.get_child("stage");
      Fixture f(nh,input,stage,argv[2]);Eigen::VectorXd g;
      candidate_audit::Session record(true);const double cost=f.evaluate(f.x,g);
      out<<"{\"kind\":\"check\",\"label\":"<<candidate_audit::quote(r.get<std::string>("label"))<<",\"cost\":"<<cost
         <<",\"expected_cost\":"<<r.get<double>("expected_cost")<<",\"gradient_hash\":"<<candidate_audit::quote(std::to_string(delivery_candidate::hash(g)))
         <<",\"gradient\":";candidate_audit::vectorJson(out,g);out<<",\"components\":"<<native67_probe::costJson(record.costs)<<',';f.audit(out,f.x);out<<"}\n";out.flush();
      const double expected=r.get<double>("expected_cost");
      if(expected>=0. && std::abs(cost-expected)>1e-9*std::max(1.,std::abs(expected)))throw std::runtime_error("native objective replay differs from capture");
    }
    for(auto& item:root.get_child("solves")){
      const auto& r=item.second;Fixture f(nh,r.get_child("input"),r.get_child("stage"),argv[2]);
      if(r.get<bool>("remove_fixed_self",false))f.opt->wei_mani_self_=0.;
      if(r.get<double>("feasibility_multiplier",1.)!=1.)f.opt->wei_feas_*=r.get<double>("feasibility_multiplier");
      candidate_audit::Session record(true);delivery_candidate::Scope accepted;
      domain_lbfgs::lbfgs_parameter_t p;p.mem_size=256;p.g_epsilon=0.;p.delta=r.get<double>("delta",1e-3);p.max_iterations=r.get<int>("max_iterations",300);
      const double budget=r.get<double>("budget_s");f.reset(budget);record.begin();double cost=0;
      const int result=domain_lbfgs::lbfgs_optimize(f.x,cost,Opt::costFunctionCallback,nullptr,Opt::earlyExitCallback,f.opt.get(),p);
      const double wall=(ros::WallTime::now()-f.opt->optimization_start_wall_time_).toSec();
      out<<"{\"kind\":\"solve\",\"label\":"<<candidate_audit::quote(r.get<std::string>("label"))<<",\"result\":"<<result<<",\"force_stop\":"<<int(f.opt->force_stop_type_)
         <<",\"elapsed_s\":"<<wall<<",\"evaluations\":"<<f.opt->iter_num_<<",\"accepted_steps\":"<<accepted.accepted_steps<<",\"budget_s\":"<<budget<<",\"stages\":[";
      bool first=true;for(auto& s:record.steps){if(!first)out<<',';first=false;out<<"{\"iteration\":"<<s.iteration<<",\"line_search\":"<<s.line_search<<",\"elapsed_s\":"<<s.solver_elapsed_s<<",\"cost\":"<<s.cost
        <<",\"components\":"<<native67_probe::costJson(s.costs)<<",\"x\":";candidate_audit::vectorJson(out,s.x);out<<',';f.audit(out,s.x);out<<'}';}
      out<<"]}\n";out.flush();std::cout<<r.get<std::string>("label")<<" result="<<result<<" eval="<<f.opt->iter_num_<<" accepted="<<accepted.accepted_steps<<std::endl;
    }
    ros::shutdown();return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
