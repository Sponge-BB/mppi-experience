#pragma once
// Isolated research experiment: frozen fast predictions are SOFT information,
// never a committed prefix, a future-state oracle, or a safety certificate.
#include <optimizer/poly_traj_optimizer.hpp>
#include "collab_factorial.hpp"
#include "coupling_objective.hpp"
#include "coupling_geometry_ros.hpp"
#include "delivery_diagnostics.hpp"
#include <atomic>

namespace coupling_bridge {
inline double setting(const char* key,double fallback){
  const char* value=std::getenv(key);double x=value?std::stod(value):fallback;
  if(!std::isfinite(x)||x<0)throw std::invalid_argument(key);return x;
}
inline coupling_objective::Settings settings(){
  coupling_objective::Settings s;
  s.execution_weight=setting("COUPLING_EXEC_WEIGHT",0);
  s.delivery_weight=setting("COUPLING_HAND_WEIGHT",0);
  s.execution_enabled=s.execution_weight>0;s.delivery_enabled=s.delivery_weight>0;
  // Position_scale=.20m; radius=.5 means a .10m POSITION soft tube, not a
  // change to the physical robot or to the hard collision safety margin.
  s.tube_radius=.5;return s;
}
struct Prediction {
  collab::Seed seed;double request_time=0,delay=.65;
  bool valid=false;std::string reason="missing";
  coupling_objective::FrozenTargets targets;
};
inline Prediction*& activePrediction(){static thread_local Prediction* p=nullptr;return p;}
inline unsigned long long& referenceRevision(){static unsigned long long v=0;return v;}
inline unsigned long long& goalGeneration(){static unsigned long long v=0;return v;}
inline coupling_objective::Target interpolate(const collab::Seed& s,double t,int gear){
  if(s.rows.empty()||t<s.rows.front()[0]||t>s.rows.back()[0])throw std::invalid_argument("prediction interval");
  size_t i=1;while(i<s.rows.size()&&s.rows[i][0]<t)++i;
  if(i==s.rows.size())i=s.rows.size()-1;
  const auto&a=s.rows[i-1];const auto&b=s.rows[i];double u=(t-a[0])/(b[0]-a[0]);
  coupling_objective::Target v;v.pos={a[1]+u*(b[1]-a[1]),a[2]+u*(b[2]-a[2])};
  v.yaw=a[3]+u*collab::angle(b[3]-a[3]);v.speed=a[4]+u*(b[4]-a[4]);
  v.time=t;v.gear=gear;return v;
}
struct Request {
  Prediction value;Prediction* previous;
  Request(const std::string& goal,const Eigen::VectorXd& start,double yaw,int direction):previous(activePrediction()){
    activePrediction()=&value;value.seed=collab::seed();value.request_time=ros::Time::now().toSec();
    value.delay=setting("COUPLING_DELIVERY_DELAY",.65);const auto&s=value.seed;
    const double age=value.request_time-s.stamp;
    if(s.rows.size()<3){value.reason="missing_or_short";return;}
    if(s.goal!=double(dual_probe::hash(goal)&0xffffffffULL)){value.reason="other_goal";return;}
    if(age<-.05||age>.25||value.request_time-s.odom>.25||value.request_time-s.cloud>.35||
       value.request_time-s.dynamic>.35){value.reason="stale";return;}
    if(s.revision<1||s.generation<1||s.revision!=double(referenceRevision())||
       s.generation!=double(goalGeneration())){value.reason="reference_epoch_mismatch";return;}
    const double begin=std::max(s.rows.front()[0],std::max(0.,age));
    if(begin>s.rows.back()[0]){value.reason="expired_checked_interval";return;}
    int gear=direction?direction:(s.rows.front()[4]<0?-1:1);
    auto origin=interpolate(s,begin,gear);
    if((origin.pos-start.head(2)).norm()>.15||std::abs(collab::angle(origin.yaw-yaw))>.4){
      value.reason="measurement_mismatch";return;
    }
    value.targets.checked_begin=s.rows.front()[0];value.targets.checked_end=s.rows.back()[0];
    // Freeze one same-gear maneuver. A future reversal is not silently matched
    // to a spatially adjacent forward segment at a self crossing.
    for(const auto&r:s.rows)if(r[0]>=begin&&r[4]*gear<-.01){
      value.targets.checked_end=r[0];break;
    }
    for(double dt:{.20,.40,.60,.80,1.0}){
      const double t=begin+dt;
      if(t<value.targets.checked_end-1e-6){auto q=interpolate(s,t,gear);
        if(q.speed*gear>=-1e-12)value.targets.execution.push_back(q);}
    }
    const double delivery=begin+value.delay;
    if(delivery<value.targets.checked_end-1e-6){
      auto q=interpolate(s,delivery,gear);
      if(q.speed*gear>=-1e-12){value.targets.has_delivery=true;value.targets.delivery=q;}
    }
    value.valid=!value.targets.execution.empty();value.reason=value.valid?"eligible":"no_checked_future";
  }
  ~Request(){activePrediction()=previous;}
};
struct Optimizer;
inline Optimizer*& current(){static thread_local Optimizer*p=nullptr;return p;}
struct Optimizer {
  Optimizer* previous;Prediction prediction;coupling_objective::Settings config;
  unsigned long long id;bool valid=false;std::string reason;
  size_t callbacks=0,active_callbacks=0;double last_exec=0,last_hand=0,last_norm=0;
  size_t exec_targets=0,hand_targets=0;
  Optimizer(unsigned long long input,int segments,int pieces,const std::vector<int>& signs):previous(current()),config(settings()){
    static std::atomic<unsigned long long> serial{0};id=++serial;current()=this;
    delivery_diag::beginSolve(dual_probe::request(),id);
    if(activePrediction())prediction=*activePrediction();
    reason=prediction.reason;valid=prediction.valid;
    if(signs.empty()||(valid&&prediction.targets.execution.front().gear!=signs.front())){
      valid=false;reason="native_gear_mismatch";
    }
    if(config.delivery_enabled&&!prediction.targets.has_delivery){valid=false;reason="delivery_outside_checked_interval";}
    if(!config.execution_enabled&&!config.delivery_enabled){valid=false;reason="zero_weights";}
    dual_probe::emit("coupling_scope",dual_probe::Fields().add("scope_id",id).add("valid",valid?"true":"false")
      .str("reason",reason).add("exec_weight",config.execution_weight).add("hand_weight",config.delivery_weight)
      .add("seed_stamp",prediction.seed.stamp).add("seed_age_s",prediction.request_time-prediction.seed.stamp)
      .add("delivery_delay_s",prediction.delay).add("reference_revision",prediction.seed.revision)
      .add("reference_generation",prediction.seed.generation).add("rows",prediction.seed.rows.size())
      .add("goal_token",prediction.seed.goal).add("segments",segments).add("pieces",pieces).add("input_hash",input));
  }
  void log(const char* phase){dual_probe::emit("coupling_cost",dual_probe::Fields().add("scope_id",id).str("phase",phase)
    .add("execution_cost",last_exec).add("delivery_cost",last_hand).add("total_coupling_cost",last_exec+last_hand)
    .add("gradient_norm",last_norm).add("callback_count",callbacks).add("active_callback_count",active_callbacks)
    .add("execution_targets",exec_targets).add("delivery_targets",hand_targets).add("active",active_callbacks?"true":"false"));}
  ~Optimizer(){log("final");current()=previous;}
};
inline void basis(double t,Eigen::Matrix<double,8,1>&b0,Eigen::Matrix<double,8,1>&b1,Eigen::Matrix<double,8,1>&b2){
  b0.setZero();b1.setZero();b2.setZero();b0(0)=1;
  for(int k=1;k<8;++k){b0(k)=std::pow(t,k);b1(k)=k*std::pow(t,k-1);if(k>=2)b2(k)=k*(k-1)*std::pow(t,k-2);}
}
template<class Opts>
double addPrediction(Opts& opts,const std::vector<Eigen::VectorXd>& times,const std::vector<int>& signs,
                     std::vector<Eigen::VectorXd>& gradT){
  auto*f=current();if(!f||!f->valid)return 0;
  if(opts.empty()||times.size()!=opts.size()||signs.size()!=opts.size()||gradT.size()!=opts.size())
    throw std::invalid_argument("coupling native container mismatch");
  for(size_t k=0;k<times.size();++k)if(times[k].size()<1||times[k].size()!=gradT[k].size()||
      !times[k].allFinite()||times[k].minCoeff()<=0||(signs[k]!=1&&signs[k]!=-1))
    throw std::invalid_argument("coupling native time/gear mismatch");
  for(size_t k=0;k<times.size();++k)if(opts[k].get_T1().size()!=times[k].size()||
      opts[k].get_b().rows()!=8*times[k].size()||opts[k].get_b().cols()<2||
      opts[k].get_gdC().rows()!=8*times[k].size()||opts[k].get_gdC().cols()<2||
      !(opts[k].get_T1().array()==times[k].array()).all())
    throw std::invalid_argument("coupling coefficient/time metadata mismatch");
  std::vector<coupling_objective::Sample> samples;
  // Fixed indices, independent of optimized durations: the first three pieces
  // of the first same-gear block. No extra polynomial pieces or initial knots.
  const int count=std::min(3,int(times.front().size()));
  for(int i=0;i<count;++i)for(int j=0;j<=8;++j){
    coupling_objective::Sample s;s.traj=0;s.piece=i;s.gear=signs.front();s.u=j/8.;s.T=times.front()(i);
    Eigen::Matrix<double,8,1>b0,b1,b2;basis(s.u*s.T,b0,b1,b2);
    auto c=opts[0].get_b().block(i*8,0,8,2);
    s.pos=c.transpose()*b0;s.vel=c.transpose()*b1;s.acc=c.transpose()*b2;samples.push_back(s);
  }
  const auto r=coupling_objective::evaluate(samples,f->prediction.targets,f->config);
  double norm=0;
  for(size_t k=0;k<samples.size();++k){const auto&s=samples[k];Eigen::Matrix<double,8,1>b0,b1,b2;basis(s.u*s.T,b0,b1,b2);
    opts[s.traj].get_gdC().block(s.piece*8,0,8,2)+=b0*r.grad_p[k].transpose()+b1*r.grad_v[k].transpose();
    gradT[s.traj](s.piece)+=s.u*(r.grad_p[k].dot(s.vel)+r.grad_v[k].dot(s.acc));
    norm+=r.grad_p[k].squaredNorm()+r.grad_v[k].squaredNorm();
  }
  f->last_exec=r.execution_cost;f->last_hand=r.delivery_cost;f->last_norm=std::sqrt(norm);
  f->exec_targets=r.execution_targets;f->hand_targets=r.delivery_targets;
  ++f->callbacks;if(r.cost>1e-12)++f->active_callbacks;if(f->callbacks==1)f->log("initial");return r.cost;
}
// Extra common projected checks supplement, NEVER replace, the original
// 3D/self/wheel checks. Unknown long-horizon dynamic safety is not certified.
template<class Data>
bool projectedCandidate(const Data& data){
  if(!coupling_geometry::enabled())return true;
  delivery_diag::CandidateRecord<Data> diagnosis(data,dual_probe::request());
  coupling_geometry::Scope latest(true);
  diagnosis.acceptance(latest.snapshot(),latest.now());
  if(!latest.valid()){diagnosis.reject("snapshot_invalid");return false;}
  double elapsed=0;size_t dynamic_unknown=0,part_index=0;
  for(const auto&part:data.singul_traj){
    const int steps=std::max(1,int(std::ceil(part.duration/.02)));
    for(int k=0;k<=steps;++k){const double q=part.duration*k/steps;
      auto p=part.traj.getPos(q);auto v=part.traj.getVel(q);
      const double yaw=std::atan2(part.singul*v(1),part.singul*v(0));Eigen::Vector3d pose(p(0),p(1),yaw);
      auto s=coupling_geometry::staticProjectedPose(pose);
      if(!s.valid||s.gap<-1e-8){
        diagnosis.rejectSample(s.valid?"static_gap":"static_invalid",part_index,k,q,
          data.start_time+elapsed+q,p,v,yaw,s,false);return false;
      }
      // Preserve the published candidate's clock. Do not silently restart an
      // old/future candidate at receipt time. MPPI geometric consumption is
      // independently checked each cycle; this is a nominal-reference gate.
      const double query_time=data.start_time+elapsed+q;
      if(query_time<latest.now())continue;
      auto d=coupling_geometry::shortDynamicPose(pose,query_time);
      if(d.valid){if(d.gap<-1e-8){
        diagnosis.rejectSample("dynamic_gap",part_index,k,q,query_time,p,v,yaw,d,true);return false;
      }}else ++dynamic_unknown;
    }elapsed+=part.duration;++part_index;
  }
  diagnosis.accept();
  dual_probe::emit("coupling_projected_accept",dual_probe::Fields().add("dynamic_unknown_samples",dynamic_unknown)
    .str("long_horizon_guard","original_native_checks_retained"));return true;
}
} // namespace coupling_bridge
