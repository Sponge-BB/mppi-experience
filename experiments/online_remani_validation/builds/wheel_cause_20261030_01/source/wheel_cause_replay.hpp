#pragma once
// Isolated, paused SIL diagnosis. No publisher, constraint change, or acceptance override.
#include "candidate_audit.hpp"
#include "delivery_accepted_candidate.hpp"
#include "delivery_replay_probe.hpp"
#include "native67_probe.hpp"
#include "traj_utils/plan_container.hpp"
#include <functional>

namespace wheel_cause {
inline bool requested(){const char* s=std::getenv("WHEEL_CAUSE_REPLAY");return s&&std::strcmp(s,"1")==0;}
inline bool& running(){static thread_local bool value=false;return value;}
struct Result {
  std::string body,error;
  Eigen::VectorXd first_x;
  double first_cost=0;
  unsigned long long first_gradient_hash=0;
  std::vector<int> pieces;
  int dim=0;
};
inline Result& lastResult(){static thread_local Result value;return value;}
struct Trial;
inline Trial*& active(){static thread_local Trial* value=nullptr;return value;}
struct Trial {
  bool enabled=false,solver_seen=false,native_gate_seen=false,native_gate_pass=false;
  Trial* previous=nullptr;
  native67_probe::Counters own_counters;
  native67_probe::Counters* previous_counters=nullptr;
  int solver_result=0,force_stop=0;
  double solver_elapsed_s=0,native_rejection_value=0,native_rejection_time=0;
  std::string native_reason;
  std::function<Result()> capture;
  explicit Trial(bool on):enabled(on),previous(active()){
    if(enabled){active()=this;previous_counters=native67_probe::active();native67_probe::active()=&own_counters;}
  }
  ~Trial() noexcept {
    if(!enabled)return;
    try{lastResult()=capture?capture():Result{};if(!capture)lastResult().error="capture callback missing";}
    catch(const std::exception& e){lastResult()=Result{};lastResult().error=e.what();}
    catch(...){lastResult()=Result{};lastResult().error="unknown capture failure";}
    native67_probe::active()=previous_counters;active()=previous;
  }
};
inline void solverFinished(int result,int force_stop,double elapsed){
  auto* t=active();if(!t)return;t->solver_seen=true;t->solver_result=result;
  t->force_stop=force_stop;t->solver_elapsed_s=elapsed;
}
inline void nativeGate(bool pass,const std::string& reason,double value,double time){
  auto* t=active();if(!t)return;t->native_gate_seen=true;t->native_gate_pass=pass;
  t->native_reason=reason;t->native_rejection_value=value;t->native_rejection_time=time;
}
inline void emit(const std::string& path,const std::string& body){
  std::ofstream out(path,std::ios::app);if(!out)throw std::runtime_error("cannot open wheel cause log");
  out<<"{\"schema\":\"wheel_cause_replay_v1\","<<body<<"}\n";
  if(!out)throw std::runtime_error("cannot write wheel cause log");
}
template<class Trajectory> Eigen::MatrixXd pvaj(const Trajectory& traj,double t){
  Eigen::MatrixXd out(traj.getPos(t).size(),4);
  out.col(0)=traj.getPos(t);out.col(1)=traj.getVel(t);out.col(2)=traj.getAcc(t);out.col(3)=traj.getJer(t);return out;
}
inline double boundaryError(const Eigen::MatrixXd& actual,const Eigen::MatrixXd& expected){
  if(actual.rows()!=expected.rows()||actual.cols()!=expected.cols()||!actual.allFinite())return INFINITY;
  return (actual-expected).cwiseAbs().maxCoeff();
}
inline std::string location(const candidate_audit::Sample& s,int wheel,
    const std::vector<delivery_candidate::Part>& parts){
  int segment=-1,piece=-1;double local=s.t,prefix=0.;
  for(std::size_t n=0;n<parts.size();++n){for(int k=0;k<parts[n].times.size();++k){
    const double end=prefix+parts[n].times[k];
    if(segment<0 && (s.t<end || (n+1==parts.size()&&k+1==parts[n].times.size()))){
      segment=int(n);piece=k;local=s.t-prefix;
    }prefix=end;
  }}
  std::ostringstream o;o<<"{\"t\":"<<candidate_audit::number(s.t)
    <<",\"x\":"<<candidate_audit::number(s.x)<<",\"y\":"<<candidate_audit::number(s.y)
    <<",\"yaw\":"<<candidate_audit::number(s.yaw)<<",\"signed_speed\":"<<candidate_audit::number(s.v)
    <<",\"abs_speed\":"<<candidate_audit::number(std::abs(s.v))
    <<",\"omega\":"<<candidate_audit::number(s.omega)<<",\"alpha\":"<<candidate_audit::number(s.alpha)
    <<",\"wheel_value\":"<<candidate_audit::number(s.wheels[wheel])
    <<",\"segment_index\":"<<segment<<",\"piece_index\":"<<piece
    <<",\"piece_local_t\":"<<candidate_audit::number(local)<<'}';return o.str();
}
template<class Trajectory> std::string peaks(const Trajectory& traj,const candidate_audit::Params& p,
    const std::vector<double>& grid,const std::vector<delivery_candidate::Part>& parts){
  candidate_audit::Summary summary;
  for(double t:grid)if(t>=0&&t<=traj.duration)summary.add(candidate_audit::motionSample(traj,p,t),p);
  std::ostringstream o;o<<"{\"sample_count\":"<<summary.count;
  const char* names[]={"left_omega","right_omega","left_alpha","right_alpha"};
  for(int k=0;k<4;++k)o<<",\""<<names[k]<<"\":"
    <<location(candidate_audit::motionSample(traj,p,summary.max_wheel_times[k]),k,parts);
  o<<'}';return o.str();
}
inline std::string motionSummary(const candidate_audit::Summary& summary,const candidate_audit::Params& p,
    const std::string& grid){
  auto json=summary.json(p,grid);
  const std::string before="\"collision_queried\":true";
  const auto at=json.find(before);if(at!=std::string::npos)json.replace(at,before.size(),"\"collision_queried\":false");
  return json;
}
template<class Trajectory> std::string cheapAudit(const Trajectory& traj,const candidate_audit::Params& p,
    const std::vector<double>& native_grid,const std::vector<double>& cost_grid,const std::vector<double>& dense_grid){
  candidate_audit::Summary native_summary,cost_summary,dense_summary;
  for(double t:native_grid)if(t>=0&&t<=traj.duration)native_summary.add(candidate_audit::motionSample(traj,p,t),p);
  for(double t:cost_grid)if(t>=0&&t<=traj.duration)cost_summary.add(candidate_audit::motionSample(traj,p,t),p);
  for(double t:dense_grid)if(t>=0&&t<=traj.duration)dense_summary.add(candidate_audit::motionSample(traj,p,t),p);
  std::ostringstream out;out<<"{\"schema\":\"candidate_audit_v1\",\"duration_s\":"<<candidate_audit::number(traj.duration)
    <<",\"detail\":false,\"motion_only\":true,\"sample_start_s\":0.05,\"sample_dt_s\":0.01"
    <<",\"summary\":"<<motionSummary(native_summary,p,"native_acceptance")
    <<",\"optimizer_summary\":"<<motionSummary(cost_summary,p,"optimizer_cost_grid")
    <<",\"feasibility_optimizer_summary\":"<<motionSummary(dense_summary,p,"optimizer_feasibility_grid")
    <<",\"samples\":[],\"optimizer_samples\":[],\"feasibility_optimizer_samples\":[],\"start_samples\":[]"
    <<",\"budget_excluded\":true,\"scope\":\"cheap intermediate motion summaries only; collision predicates and native gate not rerun\"}";
  return out.str();
}

// Same canonical x decoder and sample grids as the existing candidate audit.
// All reconstruction and IsTrajSafe calls occur after the solver budget ends.
template<class SnapVector,class Model,class Map,class NativeGate,class DenseRule>
Result capture(Trial& trial,const candidate_audit::Session& session,
    const delivery_candidate::Scope& accepted,SnapVector& snaps,
    const std::vector<int>& piece_counts,int dim,
    const std::vector<Eigen::MatrixXd>& effective_ini,
    const std::vector<Eigen::MatrixXd>& effective_fin,const std::vector<int>& directions,
    double non_singular_speed,Model& model,Map& map,const candidate_audit::Params& params,
    int quadrature_points,int dense_resolution,unsigned long long input_hash,
    unsigned long long entry_raw,unsigned long long entry_esdf,
    NativeGate native_gate,DenseRule dense_rule){
  delivery_replay::RestoreFailure restore_verdict;
  auto* verdict=failure_probe::active();
  const std::string actual_reason=verdict?verdict->reason:"missing";
  const double actual_value=verdict?verdict->rejection_value:0.,actual_time=verdict?verdict->rejection_time:0.;
  const std::string overlay_begin=GridMapParallelTestAccessor::readOverlayKey(map);
  const double sim_begin=ros::Time::now().toSec();
  Result result;result.dim=dim;result.pieces=piece_counts;
  if(!session.steps.empty()){result.first_x=session.steps.front().x;result.first_cost=session.steps.front().cost;}
  result.first_gradient_hash=verdict?verdict->first_gradient_hash:0;
  std::ostringstream o;o<<std::setprecision(17)<<"\"input_hash\":"<<candidate_audit::quote(std::to_string(input_hash))
    <<",\"effective_initial_states\":";failure_probe::matrices(o,effective_ini);
  o<<",\"effective_final_states\":";failure_probe::matrices(o,effective_fin);
  o<<",\"directions\":[";for(std::size_t n=0;n<directions.size();++n){if(n)o<<',';o<<directions[n];}
  o<<"],\"limits\":{\"wheel_limit\":"<<params.wheel_limit<<",\"alpha_limit\":"<<params.alpha_limit
    <<",\"feas_tol\":"<<params.feas_tol<<",\"wheel_base\":"<<params.wheel_base<<",\"wheel_radius\":"<<params.wheel_radius
    <<",\"quadrature_points\":"<<quadrature_points<<",\"dense_resolution\":"<<dense_resolution
    <<",\"base_sphere_count\":";
  std::vector<Eigen::Vector3d> base_points;model.getCarPts(Eigen::Vector3d::Zero(),base_points);
  o<<base_points.size()<<",\"arm_radius\":"<<params.arm_radius<<",\"ground\":"<<params.ground<<'}'
    <<",\"solver\":{\"finished_hook_seen\":"<<(trial.solver_seen?"true":"false")
    <<",\"lbfgs_result\":"<<(trial.solver_seen?std::to_string(trial.solver_result):"null")
    <<",\"force_stop\":"<<(trial.solver_seen?std::to_string(trial.force_stop):"null")
    <<",\"elapsed_s\":"<<trial.solver_elapsed_s<<",\"complete_evaluations\":"<<session.evaluate_complete_count
    <<",\"evaluate_entries\":"<<trial.own_counters.evaluations
    <<",\"accepted_count\":"<<accepted.accepted_steps<<",\"recorded_stage_count\":"<<session.steps.size()
    <<",\"first_cost\":"<<candidate_audit::number(result.first_cost)
    <<",\"first_x_hash\":"<<candidate_audit::quote(std::to_string(delivery_candidate::hash(result.first_x)))
    <<",\"first_gradient_hash\":"<<candidate_audit::quote(std::to_string(result.first_gradient_hash))<<'}'
    <<",\"actual_native_gate\":{\"seen\":"<<(trial.native_gate_seen?"true":"false")
    <<",\"pass\":"<<(trial.native_gate_seen?(trial.native_gate_pass?"true":"false"):"null")
    <<",\"reason\":"<<candidate_audit::quote(trial.native_reason)
    <<",\"rejection_value\":"<<candidate_audit::number(trial.native_rejection_value)
    <<",\"rejection_time\":"<<candidate_audit::number(trial.native_rejection_time)<<'}'
    <<",\"actual_return_diagnosis\":{\"reason\":"<<candidate_audit::quote(actual_reason)
    <<",\"rejection_value\":"<<candidate_audit::number(actual_value)
    <<",\"rejection_time\":"<<candidate_audit::number(actual_time)<<'}'
    <<",\"stages\":[";
  bool last_matches_saved=false;
  for(std::size_t index=0;index<session.steps.size();++index){
    const auto& step=session.steps[index];
    std::vector<delivery_candidate::Part> parts;
    if(!delivery_candidate::decode(step.x,piece_counts,dim,effective_ini,effective_fin,non_singular_speed,parts))
      throw std::runtime_error("cannot decode wheel diagnostic candidate");
    remani_planner::SingulTrajData rebuilt;rebuilt.clearSingulTraj();double prefix=0.;
    std::vector<double> collision_grid,feasibility_grid,native_grid;
    std::ostringstream part_json;part_json<<std::setprecision(17)<<'[';
    double max_head_error=0.,max_tail_error=0.;
    for(std::size_t n=0;n<parts.size();++n){
      snaps[n].generate(parts[n].points,parts[n].times,parts[n].initial,parts[n].final);
      const auto part_traj=snaps[n].getTraj(directions[n]);
      const auto head=pvaj(part_traj,0.),tail=pvaj(part_traj,parts[n].times.sum());
      const double head_error=boundaryError(head,parts[n].initial),tail_error=boundaryError(tail,parts[n].final);
      max_head_error=std::max(max_head_error,head_error);max_tail_error=std::max(max_tail_error,tail_error);
      rebuilt.addSingulTraj(part_traj,prefix);
      if(n)part_json<<',';
      part_json<<"{\"gear\":"<<directions[n]<<",\"times\":"<<delivery_diag::vector(parts[n].times)
        <<",\"initial_state\":"<<delivery_diag::matrix(parts[n].initial)
        <<",\"final_state\":"<<delivery_diag::matrix(parts[n].final)
        <<",\"actual_initial_pvaj\":"<<delivery_diag::matrix(head)<<",\"actual_final_pvaj\":"<<delivery_diag::matrix(tail)
        <<",\"head_max_abs_error\":"<<candidate_audit::number(head_error)
        <<",\"tail_max_abs_error\":"<<candidate_audit::number(tail_error)
        <<",\"b_rows_ascending_power\":"<<delivery_diag::matrix(snaps[n].get_b())<<'}';
      for(int piece=0;piece<parts[n].times.size();++piece){
        const double duration=parts[n].times[piece],dt=duration/quadrature_points;
        const auto coefficients=snaps[n].get_b().block(piece*8,0,8,dim).eval();
        double local=0.;
        for(int j=0;j<=quadrature_points;++j,local+=dt){
          collision_grid.push_back(prefix+local);feasibility_grid.push_back(prefix+local);
          const double s1=local,s2=s1*s1,s3=s2*s1,s4=s2*s2,s5=s4*s1,s6=s3*s3;
          Eigen::Matrix<double,8,1> beta;beta<<0.,1.,2.*s1,3.*s2,4.*s3,5.*s4,6.*s5,7.*s6;
          const Eigen::VectorXd velocity=coefficients.transpose()*beta;
          if(dense_rule(velocity.head(2).squaredNorm())){
            double dense_local=local;
            const int quantity=(j==0||j==quadrature_points)?dense_resolution/2:dense_resolution;
            if(j!=0)dense_local-=dt*.5;
            for(int l=0;l<=quantity;++l,dense_local+=dt/dense_resolution)feasibility_grid.push_back(prefix+dense_local);
          }
        }prefix+=duration;
      }
    }part_json<<']';
    const int end=int(std::floor(rebuilt.duration/.01));double native_t=.05;
    for(int k=5;k<end;++k,native_t+=.01)native_grid.push_back(native_t);
    const bool detail=index==0||index+1==session.steps.size();
    if(detail&&verdict){verdict->reason="ok";verdict->rejection_value=0.;verdict->rejection_time=0.;}
    const bool native_ok=detail&&native_gate(rebuilt);
    const std::string reason=!detail?"not_rechecked_intermediate":(verdict?verdict->reason:"missing");
    const double value=detail&&verdict?verdict->rejection_value:0.,time=detail&&verdict?verdict->rejection_time:0.;
    const auto audit=detail?candidate_audit::audit(rebuilt,model,map,params,collision_grid,true,feasibility_grid):
      cheapAudit(rebuilt,params,native_grid,collision_grid,feasibility_grid);
    if(index)o<<',';
    o<<"{\"label\":"<<candidate_audit::quote(step.label)<<",\"index\":"<<index
      <<",\"iteration\":"<<step.iteration<<",\"line_search\":"<<step.line_search
      <<",\"cost\":"<<candidate_audit::number(step.cost)<<",\"elapsed_s\":"<<step.solver_elapsed_s
      <<",\"x\":"<<delivery_diag::vector(step.x)<<",\"cost_components\":"<<native67_probe::costJson(step.costs)
      <<",\"cost_components_recorded\":"<<(step.cost_seen?"true":"false")
      <<",\"evaluation_begin_key\":"<<step.evaluation_begin_key<<",\"evaluation_end_key\":"<<step.evaluation_end_key
      <<",\"parts\":"<<part_json.str()<<",\"native_IsTrajSafe_pass\":"<<(detail?(native_ok?"true":"false"):"null")
      <<",\"native_reason\":"<<candidate_audit::quote(reason)<<",\"native_rejection_value\":"<<candidate_audit::number(value)
      <<",\"native_reject_time\":"<<candidate_audit::number(time)
      <<",\"native_verdict_scope\":\"post-solver canonical recheck, not the actual return gate\""
      <<",\"boundary_checks\":{\"max_head_pvaj_error\":"<<candidate_audit::number(max_head_error)
      <<",\"max_tail_pvaj_error\":"<<candidate_audit::number(max_tail_error)
      <<",\"all_within_1e_7\":"<<(max_head_error<=1e-7&&max_tail_error<=1e-7?"true":"false")<<'}'
      <<",\"audit\":"<<audit<<",\"wheel_peak_locations\":{\"native\":"<<peaks(rebuilt,params,native_grid,parts)
      <<",\"K16\":"<<peaks(rebuilt,params,collision_grid,parts)
      <<",\"dense\":"<<peaks(rebuilt,params,feasibility_grid,parts)<<"}}";
    if(index+1==session.steps.size())last_matches_saved=accepted.usable()&&step.x.size()==accepted.x.size()&&
      (step.x.array()==accepted.x.array()).all();
  }
  const std::string overlay_end=GridMapParallelTestAccessor::readOverlayKey(map);
  const bool stable=overlay_begin==overlay_end&&sim_begin==ros::Time::now().toSec()&&
    entry_raw==map.getDynamicObservationSeq()&&entry_esdf==map.getESDFObservationSeq();
  o<<"],\"frozen_checks\":{\"stable\":"<<(stable?"true":"false")
    <<",\"overlay_begin\":"<<overlay_begin<<",\"overlay_end\":"<<overlay_end
    <<",\"solver_entry\":"<<session.solver_entry_key<<",\"solver_exit\":"<<session.solver_exit_key
    <<",\"solver_entry_exit_key_equal\":"<<(session.solver_entry_key==session.solver_exit_key?"true":"false")
    <<",\"evaluate_overlay_changed_count\":"<<session.evaluate_overlay_changed_count
    <<",\"last_matches_saved_candidate\":"<<(last_matches_saved?"true":"false")
    <<"},\"post_solver_audit_budget_excluded\":true,\"diagnostic_candidates_published\":false";
  result.body=o.str();return result;
}

inline bool sameFirst(const Result& a,const Result& b){
  return a.first_x.size()>0&&a.first_x.size()==b.first_x.size()&&
    (a.first_x.array()==b.first_x.array()).all()&&a.first_cost==b.first_cost&&
    a.first_gradient_hash!=0&&a.first_gradient_hash==b.first_gradient_hash;
}
inline bool retimeCoordinates(const Result& original,const Result& retimed,double scale){
  if(original.dim!=retimed.dim||original.pieces!=retimed.pieces||original.first_x.size()==0||
    original.first_x.size()!=retimed.first_x.size())return false;
  int begin=0,count=0;for(int n:original.pieces){begin+=original.dim*(n-1);count+=n;}
  for(int k=0;k<original.first_x.size();++k){
    const double a=original.first_x[k],b=retimed.first_x[k];
    if(k<begin||k>=begin+count){if(a!=b)return false;}
    else{
      const auto real=[](double v){return v>0.?((.5*v+1.)*v+1.):1./((.5*v-1.)*v+1.);};
      if(std::abs(real(b)-scale*real(a))>1e-12*std::max(1.,scale*real(a)))return false;
    }
  }return true;
}
} // namespace wheel_cause
