// Offline algebra/protocol tests. No ROS graph, simulator or commands.
#include "active_arm_coupling.hpp"
#include <iostream>
#include <functional>
namespace aa=active_arm_coupling;
namespace co=coupling_objective;
namespace {
int checks=0;double maximum_error=0.;
void require(bool v,const char* why){++checks;if(!v)throw std::runtime_error(why);}
void near(double a,double b,const char* why){
  const double e=std::abs(a-b)/(1.+std::abs(a)+std::abs(b));
  maximum_error=std::max(maximum_error,e);require(std::isfinite(e)&&e<3e-6,why);
}
template<class F>void throws(F f,const char* why){bool yes=false;try{f();}catch(const std::exception&){yes=true;}require(yes,why);}
std::vector<double> wire(){
  std::vector<double> v={2.,100.,2.,7.,123.,100.,100.,100.,20.,4.,100.};
  for(int i=0;i<4;++i){
    v.insert(v.end(),{i*.2,i*.04,0.,0.,.2,0.,.2,0.});
    for(int j=0;j<6;++j)v.push_back(.03*j+.02*i*i);
    for(int j=0;j<6;++j)v.push_back(.2*i);
  }return v;
}
void protocol(){
  aa::Seed s;const auto v=wire();require(aa::parse(v,s),"valid v2 rejected");
  require(s.rows.size()==4&&s.rows[1].arm.q[3]==v[11+20+11],"wrong q columns");
  require(s.rows[2].arm.dq[4]==v[11+40+18],"wrong dq columns");
  require(aa::fresh(s,100.1)&&!aa::fresh(s,100.3)&&!aa::fresh(s,99.9),"source freshness wrong");
  auto stale=s;stale.joint_stamp=99.;require(!aa::fresh(stale,100.1),"stale joints accepted");
  for(const auto index:{0,8,9,10,11,31,50}){
    auto bad=v;bad[index]=std::numeric_limits<double>::quiet_NaN();
    require(!aa::parse(bad,s)&&s.rows.empty(),"nonfinite parsed or stale output survived");
  }
  auto bad=v;bad.pop_back();require(!aa::parse(bad,s),"truncated parsed");
  bad=v;bad[31]=bad[11];require(!aa::parse(bad,s),"duplicate time parsed");
  bad=v;bad[51]=bad[31]-.01;require(!aa::parse(bad,s),"backwards time parsed");
  bad=v;bad[2]=1.5;require(!aa::parse(bad,s),"fractional revision parsed");
  require(aa::parse(v,s),"parse reset failed");
  for(const auto& row:s.rows){auto a=aa::interpolate(s,row.base[0]);
    near((a.q-row.arm.q).norm(),0.,"Hermite knot q");near((a.dq-row.arm.dq).norm(),0.,"Hermite knot dq");}
  const double t=.29,e=1e-6;const auto mid=aa::interpolate(s,t),plus=aa::interpolate(s,t+e),minus=aa::interpolate(s,t-e);
  for(int j=0;j<6;++j){near(mid.dq[j],(plus.q[j]-minus.q[j])/(2*e),"Hermite dq");
    near(mid.ddq[j],(plus.dq[j]-minus.dq[j])/(2*e),"Hermite ddq");
    near(mid.dddq[j],(plus.ddq[j]-minus.ddq[j])/(2*e),"Hermite dddq");}
  throws([&]{aa::interpolate(s,-.01);},"negative extrapolation accepted");
  throws([&]{aa::interpolate(s,.61);},"tail extrapolation accepted");
}
struct Fixture {
  std::vector<co::Sample> s;std::vector<aa::JointState> q;
  co::FrozenTargets f;aa::JointState target;co::Settings c;aa::Settings a;
  Fixture(){
    c.execution_enabled=false;c.delivery_weight=2.3;c.temperature=.3;
    f.checked_begin=0.;f.checked_end=1.5;f.has_delivery=true;
    f.delivery.pos={.2,.1};f.delivery.yaw=.16;f.delivery.speed=.2;f.delivery.time=.6;
    target.q.setConstant(.07);target.dq.setConstant(.02);
    for(int i=0;i<4;++i){co::Sample p;p.pos={.14+.03*i,.06+.02*i};p.vel={.19-.01*i,.025+.007*i};
      p.gear=i==3?-1:1;p.u=.2*i;p.T=.6;s.push_back(p);
      aa::JointState j;for(int d=0;d<6;++d){j.q[d]=.03+.013*i+.004*d;j.dq[d]=.012+.006*i-.003*d;}
      q.push_back(j);}
  }
  aa::Result eval()const{return aa::evaluate(s,q,f,target,c,a);}
};
void gradients(){
  Fixture f;const auto r=f.eval();require(r.base.cost>0.,"zero test cost");constexpr double e=1e-6;
  for(std::size_t i=0;i<f.s.size();++i){
    for(int d=0;d<2;++d){auto plus=f,minus=f;plus.s[i].pos[d]+=e;minus.s[i].pos[d]-=e;
      near(r.base.grad_p[i][d],(plus.eval().base.cost-minus.eval().base.cost)/(2*e),"p gradient");
      plus=f;minus=f;plus.s[i].vel[d]+=e;minus.s[i].vel[d]-=e;
      near(r.base.grad_v[i][d],(plus.eval().base.cost-minus.eval().base.cost)/(2*e),"v gradient");}
    for(int d=0;d<6;++d){auto plus=f,minus=f;plus.q[i].q[d]+=e;minus.q[i].q[d]-=e;
      near(r.grad_q[i][d],(plus.eval().base.cost-minus.eval().base.cost)/(2*e),"q gradient");
      plus=f;minus=f;plus.q[i].dq[d]+=e;minus.q[i].dq[d]-=e;
      near(r.grad_dq[i][d],(plus.eval().base.cost-minus.eval().base.cost)/(2*e),"dq gradient");}
  }
  require(r.grad_q.back().isZero(0.)&&r.base.grad_p.back().isZero(0.),"other gear contributed");
  f.a.q_weight=f.a.dq_weight=0.;const auto z=f.eval();const auto old=co::evaluate(f.s,f.f,f.c);
  require(z.base.cost==old.cost,"zero arm weight changes old cost");
  for(std::size_t i=0;i<f.s.size();++i)require(z.base.grad_p[i]==old.grad_p[i]&&z.base.grad_v[i]==old.grad_v[i]&&z.grad_q[i].isZero(0.),"zero arm weight changes old gradient");
  f=Fixture();f.c.delivery_weight=0.;f.q.clear();require(f.eval().base.cost==0.,"disabled handoff requires joints");
  f=Fixture();f.q.clear();throws([&]{f.eval();},"missing joint samples accepted");
  f=Fixture();f.f.checked_begin=std::numeric_limits<double>::quiet_NaN();throws([&]{f.eval();},"invalid checked interval accepted");
  f=Fixture();f.a.q_scale=1e-200;throws([&]{f.eval();},"underflowing scale accepted");
  // If minima were selected separately, base could match sample 0 and arm 1.
  f=Fixture();f.s.resize(2);f.q.resize(2);f.c.heading_weight=f.c.velocity_weight=0.;
  f.target.q.setZero();f.s[0].pos=f.f.delivery.pos;f.s[1].pos=f.f.delivery.pos+Eigen::Vector2d(.5,0.);
  f.q[0].q.setConstant(.5);f.q[1].q.setZero();f.a.dq_weight=0.;
  require(f.eval().base.cost>10.,"independent base/arm phase minima used");
}
// Verify direct coefficient and duration chain at fixed normalized u. Native
// MinSnap's P/T adjoint is verified separately against the transformed bridge.
void polynomialChain(){
  Eigen::Matrix<double,8,8> C;for(int i=0;i<8;++i)for(int d=0;d<8;++d)C(i,d)=.015*(i+1)/(d+1);
  double T=.7,u=.37;
  auto evaluate=[&](const Eigen::Matrix<double,8,8>& c,double time){
    const double t=u*time;Eigen::Matrix<double,8,1>b0,b1,b2;
    for(int k=0;k<8;++k){b0[k]=std::pow(t,k);b1[k]=k? k*std::pow(t,k-1):0.;b2[k]=k>1?k*(k-1)*std::pow(t,k-2):0.;}
    Fixture f;f.s.resize(1);f.q.resize(1);f.s[0].pos=c.leftCols(2).transpose()*b0;
    f.s[0].vel=c.leftCols(2).transpose()*b1;f.s[0].acc=c.leftCols(2).transpose()*b2;
    f.q[0].q=c.rightCols(6).transpose()*b0;f.q[0].dq=c.rightCols(6).transpose()*b1;f.q[0].ddq=c.rightCols(6).transpose()*b2;
    return f;
  };
  auto f=evaluate(C,T);auto r=f.eval();constexpr double e=1e-6;
  const double t=u*T;Eigen::Matrix<double,8,1>b0,b1;
  for(int k=0;k<8;++k){b0[k]=std::pow(t,k);b1[k]=k?k*std::pow(t,k-1):0.;}
  Eigen::Matrix<double,8,8> g;g.leftCols(2)=b0*r.base.grad_p[0].transpose()+b1*r.base.grad_v[0].transpose();
  g.rightCols(6)=b0*r.grad_q[0].transpose()+b1*r.grad_dq[0].transpose();
  for(int k=0;k<8;++k)for(int d=0;d<8;++d){auto plus=C,minus=C;plus(k,d)+=e;minus(k,d)-=e;
    near(g(k,d),(evaluate(plus,T).eval().base.cost-evaluate(minus,T).eval().base.cost)/(2*e),"coefficient chain");}
  const double gt=u*(r.base.grad_p[0].dot(f.s[0].vel)+r.base.grad_v[0].dot(f.s[0].acc)+
                    r.grad_q[0].dot(f.q[0].dq)+r.grad_dq[0].dot(f.q[0].ddq));
  near(gt,(evaluate(C,T+e).eval().base.cost-evaluate(C,T-e).eval().base.cost)/(2*e),"direct duration chain");
}
}
int main(){protocol();gradients();polynomialChain();std::cout<<"ACTIVE_ARM_COUPLING_PASSED checks="<<checks<<" max_relative_error="<<maximum_error<<'\n';}
