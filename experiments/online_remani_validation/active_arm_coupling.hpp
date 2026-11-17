#pragma once
// Isolated SIL extension. A checked whole-body rollout is frozen information,
// never a hard execution commitment or a collision certificate for a new curve.
#include "coupling_objective.hpp"
#include <array>
#include <cstdlib>
#include <string>

namespace active_arm_coupling {
constexpr int dof=6, wire_header=11, wire_stride=20;
using Joints=Eigen::Matrix<double,dof,1>;
inline bool armEnabled(){const char* p=std::getenv("ACTIVE_ARM_ENABLED");return p&&std::string(p)=="1";}
inline bool enabled(){const char* p=std::getenv("ACTIVE_ARM_COUPLING");return armEnabled()&&p&&std::string(p)=="1";}
inline double parameter(const char* name,double fallback){
  const char* p=std::getenv(name);const double v=p?std::stod(p):fallback;
  if(!std::isfinite(v)||v<0.)throw std::invalid_argument(name);return v;
}
struct JointState {
  Joints q=Joints::Zero(),dq=Joints::Zero(),ddq=Joints::Zero(),dddq=Joints::Zero();
};
struct Row {std::array<double,8> base{};JointState arm;};
struct Seed {
  double stamp=0,generation=0,revision=0,goal=0,odom=0,cloud=0,dynamic=0,joint_stamp=0;
  std::vector<Row> rows;
};
inline bool integer(double x,double low,double high){return std::isfinite(x)&&x>=low&&x<=high&&x==std::floor(x);}
// [2,stamp,generation,revision,goal,odom,cloud,dynamic,20,N,joint_stamp],
// then N rows [relative_t,x,y,yaw,signed_v,yaw_rate,base6,base7,q1..6,dq1..6].
inline bool parse(const std::vector<double>& v,Seed& out){
  out=Seed();
  if(v.size()<wire_header||v[0]!=2.||v[8]!=wire_stride||!integer(v[9],2,1000)||
     v.size()!=wire_header+wire_stride*std::size_t(v[9]))return false;
  for(double x:v)if(!std::isfinite(x))return false;
  if(v[1]<=0||v[5]<=0||v[6]<=0||v[7]<=0||v[10]<=0||
     !integer(v[2],1,9007199254740991.)||!integer(v[3],1,9007199254740991.)||
     !integer(v[4],0,4294967295.))return false;
  Seed value;value.stamp=v[1];value.generation=v[2];value.revision=v[3];value.goal=v[4];
  value.odom=v[5];value.cloud=v[6];value.dynamic=v[7];value.joint_stamp=v[10];
  double previous=-1.;
  for(std::size_t i=wire_header;i<v.size();i+=wire_stride){
    if(v[i]<0||v[i]<=previous)return false;
    Row row;std::copy_n(v.begin()+i,8,row.base.begin());previous=v[i];
    for(int j=0;j<dof;++j){row.arm.q[j]=v[i+8+j];row.arm.dq[j]=v[i+14+j];}
    value.rows.push_back(row);
  }
  out=std::move(value);return true;
}
// Hermite interpolation keeps q and dq mutually consistent. No clamping,
// extrapolation or angle wrapping: bounded revolute-joint coordinates are used.
inline JointState interpolate(const Seed& s,double t){
  if(!std::isfinite(t)||s.rows.size()<2||t<s.rows.front().base[0]||t>s.rows.back().base[0])
    throw std::invalid_argument("active arm target outside checked interval");
  std::size_t k=1;while(k<s.rows.size()&&s.rows[k].base[0]<t)++k;
  if(k==s.rows.size())k=s.rows.size()-1;
  const auto& a=s.rows[k-1];const auto& b=s.rows[k];
  const double h=b.base[0]-a.base[0],u=(t-a.base[0])/h;
  if(!(h>0.)||!a.arm.q.allFinite()||!b.arm.q.allFinite()||!a.arm.dq.allFinite()||!b.arm.dq.allFinite())
    throw std::invalid_argument("invalid active arm Hermite segment");
  JointState r;
  r.q=(2*u*u*u-3*u*u+1)*a.arm.q+(u*u*u-2*u*u+u)*h*a.arm.dq+
      (-2*u*u*u+3*u*u)*b.arm.q+(u*u*u-u*u)*h*b.arm.dq;
  r.dq=((6*u*u-6*u)*a.arm.q+(-6*u*u+6*u)*b.arm.q)/h+
       (3*u*u-4*u+1)*a.arm.dq+(3*u*u-2*u)*b.arm.dq;
  r.ddq=((12*u-6)*a.arm.q+(-12*u+6)*b.arm.q)/(h*h)+
        ((6*u-4)*a.arm.dq+(6*u-2)*b.arm.dq)/h;
  r.dddq=(12*a.arm.q-12*b.arm.q)/(h*h*h)+6*(a.arm.dq+b.arm.dq)/(h*h);
  if(!r.q.allFinite()||!r.dq.allFinite()||!r.ddq.allFinite()||!r.dddq.allFinite())
    throw std::overflow_error("active arm interpolation nonfinite");
  return r;
}
inline bool fresh(const Seed& s,double now){
  return s.rows.size()>=2&&std::isfinite(now)&&now-s.stamp>=-.05&&now-s.stamp<=.25&&
    now-s.joint_stamp>=-.05&&now-s.joint_stamp<=.25;
}
struct Settings {double q_weight=1.,dq_weight=.2,q_scale=.2,dq_scale=.2;};
inline Settings settings(){
  Settings s;s.q_weight=parameter("ACTIVE_ARM_HAND_Q_WEIGHT",1.);
  s.dq_weight=parameter("ACTIVE_ARM_HAND_DQ_WEIGHT",.2);
  s.q_scale=parameter("ACTIVE_ARM_HAND_Q_SCALE",.2);s.dq_scale=parameter("ACTIVE_ARM_HAND_DQ_SCALE",.2);
  return s;
}
struct Result {
  coupling_objective::Result base;
  std::vector<Joints> grad_q,grad_dq;
};
// Base and joint errors use ONE soft-min distribution: independent minima
// would falsely match the base at one phase and the arm at another.
inline Result evaluate(const std::vector<coupling_objective::Sample>& samples,
                       const std::vector<JointState>& joints,
                       const coupling_objective::FrozenTargets& targets,
                       const JointState& arm_target,
                       const coupling_objective::Settings& c,
                       const Settings& arm=Settings()){
  Result out;out.grad_q.assign(samples.size(),Joints::Zero());out.grad_dq=out.grad_q;
  if(arm.q_weight==0.&&arm.dq_weight==0.){out.base=coupling_objective::evaluate(samples,targets,c);return out;}
  for(double x:{arm.q_weight,arm.dq_weight})if(!std::isfinite(x)||x<0.)throw std::invalid_argument("invalid active arm weight");
  for(double x:{arm.q_scale,arm.dq_scale})if(!std::isfinite(x)||x<=0.)throw std::invalid_argument("invalid active arm scale");
  for(double x:{arm.q_scale,arm.dq_scale})if(!std::isfinite(x*x)||x*x<=0.)throw std::invalid_argument("active arm scale square out of range");
  auto execution=c;execution.delivery_enabled=false;
  out.base=coupling_objective::evaluate(samples,targets,execution);
  if(!c.delivery_enabled||c.delivery_weight<=0.)return out;
  if(samples.size()!=joints.size()||!targets.has_delivery||!arm_target.q.allFinite()||!arm_target.dq.allFinite())
    throw std::invalid_argument("missing active arm delivery samples");
  if(!std::isfinite(targets.checked_begin)||!std::isfinite(targets.checked_end)||
     targets.checked_begin<0.||targets.checked_end<targets.checked_begin)
    throw std::invalid_argument("invalid active arm checked interval");
  coupling_objective::detail::checkTarget(targets.delivery,targets);
  const double wq=arm.q_weight/(dof*arm.q_scale*arm.q_scale),wv=arm.dq_weight/(dof*arm.dq_scale*arm.dq_scale);
  std::vector<std::size_t> indices;std::vector<double> d;
  std::vector<coupling_objective::detail::Match> base;
  double minimum=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<samples.size();++i){
    if(samples[i].gear!=targets.delivery.gear)continue;
    if(!joints[i].q.allFinite()||!joints[i].dq.allFinite())throw std::invalid_argument("nonfinite active arm sample");
    auto m=coupling_objective::detail::match({samples[i]},targets.delivery,c,true);
    const double value=m.cost+wq*(joints[i].q-arm_target.q).squaredNorm()+wv*(joints[i].dq-arm_target.dq).squaredNorm();
    if(!std::isfinite(value))throw std::overflow_error("active arm match nonfinite");
    indices.push_back(i);d.push_back(value);base.push_back(std::move(m));minimum=std::min(minimum,value);
  }
  if(indices.empty())throw std::invalid_argument("active arm match has no same-gear sample");
  double partition=0.;for(double& value:d){value=std::exp(-(value-minimum)/c.temperature);partition+=value;}
  const double weight=c.delivery_weight*targets.delivery.weight;
  out.base.delivery_cost=weight*(minimum-c.temperature*std::log(partition/d.size()));
  for(std::size_t k=0;k<indices.size();++k){const auto i=indices[k];const double w=weight*d[k]/partition;
    out.base.grad_p[i]+=w*base[k].grad_p[0];out.base.grad_v[i]+=w*base[k].grad_v[0];
    out.grad_q[i]+=2*w*wq*(joints[i].q-arm_target.q);out.grad_dq[i]+=2*w*wv*(joints[i].dq-arm_target.dq);
  }
  out.base.delivery_targets=1;out.base.cost=out.base.execution_cost+out.base.delivery_cost;
  if(!std::isfinite(out.base.cost))throw std::overflow_error("active arm objective nonfinite");
  for(std::size_t i=0;i<samples.size();++i)
    if(!out.base.grad_p[i].allFinite()||!out.base.grad_v[i].allFinite()||
       !out.grad_q[i].allFinite()||!out.grad_dq[i].allFinite())
      throw std::overflow_error("active arm gradient nonfinite");
  return out;
}
} // namespace active_arm_coupling
