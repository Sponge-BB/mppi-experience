#pragma once
// Dynamic soft cost on the intersection of a polynomial piece and fixed CV
// validity intervals. Hard collision queries still use coverage_collision.hpp.
#include "coverage_collision.hpp"
#include "candidate_collision_repair.hpp"
#include <algorithm>

namespace window_continuity {
inline bool enabled(){
  const char* p=std::getenv("DYNAMIC_WINDOW_CONTINUITY");
  return p&&std::string(p)=="1"&&time_indexed_collision::active()&&
    time_indexed_collision::current()->full_coverage;
}
enum class Mode { Static, Increment };
inline Mode& mode(){static thread_local Mode m=Mode::Static;return m;}
struct Scope {
  Mode before;
  explicit Scope(Mode value):before(mode()){mode()=value;}
  ~Scope(){mode()=before;}
};
inline bool increment(){return enabled()&&mode()==Mode::Increment;}

struct Penalty {
  double cost=0.,time_gradient=0.;
  Eigen::Vector3d gradient=Eigen::Vector3d::Zero();
};
inline Penalty cubic(double distance,const Eigen::Vector3d& gradient,
                     double threshold,double weight){
  Penalty out;const double error=threshold-distance;
  if(error>0.){
    out.cost=weight*error*error*error;
    out.gradient=-3.*weight*error*error*gradient;
  }
  return out;
}
inline Penalty environment(GridMap& map,const Eigen::Vector3d& point,
                           double threshold,double weight){
  double distance;Eigen::Vector3d gradient;
  map.evaluateEDTWithGrad(point,distance,gradient);
  const auto baseline=cubic(distance,gradient,threshold,weight);
  if(mode()==Mode::Static)return baseline;
  auto* context=time_indexed_collision::current();
  auto* sample=time_indexed_collision::sample();
  time_indexed_collision::indexedDistance(*context,*sample,point,distance,gradient);
  auto result=cubic(distance,gradient,threshold,weight);
  const double error=threshold-distance;
  if(error>0.)result.time_gradient=-3.*weight*error*error*sample->last_distance_gradient;
  result.cost-=baseline.cost;result.gradient-=baseline.gradient;
  sample->temporal_cost_gradient+=result.time_gradient;
  return result;
}

// a,b are local times; the four derivatives are with respect to the sum of
// preceding durations P and this piece's duration T, with coefficients fixed.
struct Interval {
  double a=0.,b=0.,aP=0.,bP=0.,bT=0.;
};
inline Interval clip(double P,double T,double lower,double upper){
  Interval out;out.a=std::max(0.,lower-P);out.b=std::min(T,upper-P);
  out.aP=lower>P?-1.:0.;out.bP=upper<P+T?-1.:0.;out.bT=T<upper-P?1.:0.;
  return out;
}
inline std::vector<double> boundaries(const time_indexed_collision::Context& c){
  std::vector<double> out{0.};
  const double begin=c.source.snapshot.stamp.toSec()-c.requested_epoch;
  for(double multiplier:{1.,2.}){
    out.push_back(std::max(0.,begin/multiplier));
    out.push_back(std::max(0.,(begin+c.source.parameters.max_prediction)/multiplier));
  }
  std::sort(out.begin(),out.end());
  out.erase(std::unique(out.begin(),out.end()),out.end());return out;
}
inline Eigen::Matrix<double,8,1> basis(double s,int derivative){
  Eigen::Matrix<double,8,1> b=Eigen::Matrix<double,8,1>::Zero();
  for(int k=derivative;k<8;++k){
    double value=1.;for(int j=0;j<derivative;++j)value*=k-j;
    for(int j=0;j<k-derivative;++j)value*=s;b[k]=value;
  }return b;
}

template<class Evaluate>
inline void integratePiece(int trajectory,int piece,const Eigen::MatrixXd& coefficients,
                           int K,Eigen::MatrixXd& coefficient_gradient,
                           Eigen::VectorXd& costs,Evaluate evaluate){
  if(!enabled())return;
  auto* context=time_indexed_collision::current();
  const auto& durations=*context->durations;auto& gradients=*context->gradients;
  const double P=time_indexed_collision::elapsedAt(trajectory,piece,0.);
  const double T=durations[trajectory](piece);const auto edges=boundaries(*context);
  Scope dynamic(Mode::Increment);
  for(std::size_t interval=1;interval<edges.size();++interval){
    const auto range=clip(P,T,edges[interval-1],edges[interval]);
    const double length=range.b-range.a;if(length<=0.)continue;
    // Interior nodes avoid assigning a finite trapezoid weight to a branch
    // which exists only at an endpoint. No CV extrapolation or taper is used.
    for(int j=0;j<K;++j){
      const double alpha=(j+.5)/K,s=range.a+alpha*length,w=length/K;
      const double sP=range.aP+alpha*(range.bP-range.aP),sT=alpha*range.bT;
      const auto b0=basis(s,0),b1=basis(s,1),b2=basis(s,2);
      const Eigen::VectorXd p=coefficients.transpose()*b0;
      const Eigen::VectorXd v=coefficients.transpose()*b1;
      const Eigen::VectorXd a=coefficients.transpose()*b2;
      Eigen::VectorXd gp=Eigen::VectorXd::Zero(p.size()),gv=gp;
      double car=0.,arm=0.,self=0.;
      time_indexed_collision::Sample sample(P+s);
      evaluate(p,v,gp,gv,car,arm,self);
      candidate_collision_repair::sample(trajectory,piece,w,car,arm,0.,gp,gv,
        time_indexed_collision::sample()->temporal_cost_gradient);
      const double F=car+arm,geometry=gp.dot(v)+gv.dot(a);
      const double explicit_time=time_indexed_collision::sample()->temporal_cost_gradient;
      coefficient_gradient.block(piece*8,0,8,p.size())+=w*(b0*gp.transpose()+b1*gv.transpose());
      const double previous=w*(geometry*sP+explicit_time*(1.+sP))+
        F*(range.bP-range.aP)/K;
      for(int n=0;n<trajectory;++n)gradients[n].array()+=previous;
      if(piece)gradients[trajectory].head(piece).array()+=previous;
      gradients[trajectory](piece)+=w*(geometry+explicit_time)*sT+F*range.bT/K;
      costs(0)+=w*car;costs(1)+=w*arm;
    }
  }
}
} // namespace window_continuity
