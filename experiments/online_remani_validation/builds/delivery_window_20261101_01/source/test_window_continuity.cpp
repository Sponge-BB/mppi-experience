#include "window_continuity.hpp"
#include <iostream>
#include <iomanip>

struct Result {double cost=0.;Eigen::VectorXd gradient;};
Result objective(const Eigen::VectorXd& x){
  namespace tc=time_indexed_collision;
  tc::Context context;context.active=true;context.full_coverage=true;
  context.requested_epoch=100.4;context.source.snapshot.stamp.fromSec(100.);
  context.source.parameters.max_prediction=3.;context.source.parameters.influence=2.;
  context.source.parameters.uncertainty0=.04;
  context.source.parameters.uncertainty_v=.006;context.source.parameters.uncertainty_a=.002;
  plan_env::DynamicPoint q;q.position={.65,.07,.3};q.velocity={-.06,.015,0.};
  context.source.snapshot.points.push_back(q);
  auto* previous=tc::current();tc::current()=&context;
  std::vector<Eigen::VectorXd> times(2),gradients(2);
  times[0]=x.segment(0,1);times[1]=x.segment(1,2);
  gradients[0]=Eigen::VectorXd::Zero(1);gradients[1]=Eigen::VectorXd::Zero(2);
  Result out;out.gradient=Eigen::VectorXd::Zero(x.size());
  tc::Durations duration_scope(times,gradients);
  for(int traj=0;traj<2;++traj){
    Eigen::MatrixXd coefficients=Eigen::MatrixXd::Zero(8*times[traj].size(),3);
    Eigen::MatrixXd gc=coefficients;Eigen::VectorXd costs=Eigen::VectorXd::Zero(6);
    for(int piece=0;piece<times[traj].size();++piece){
      coefficients.row(8*piece)<<x[3],.05,.3;
      coefficients.row(8*piece+1)<<x[4],0.,0.;
      coefficients.row(8*piece+2)<<x[5],0.,0.;
      const Eigen::MatrixXd c=coefficients.block(8*piece,0,8,3);
      window_continuity::integratePiece(traj,piece,c,16,gc,costs,
        [&](const Eigen::VectorXd& p,const Eigen::VectorXd&,Eigen::VectorXd& gp,Eigen::VectorXd&,
            double& car,double&,double&){
          double distance=2.;Eigen::Vector3d g=Eigen::Vector3d::Zero();
          tc::indexedDistance(context,*tc::sample(),p.head(3),distance,g);
          const auto penalty=window_continuity::cubic(distance,g,.55,1.2);
          car=penalty.cost;gp=penalty.gradient;
          if(distance<.55)tc::penaltyTimeDerivative(-3.*1.2*std::pow(.55-distance,2));
        });
    }
    out.cost+=costs.sum();
    for(int piece=0;piece<times[traj].size();++piece)
      for(int k=0;k<3;++k)out.gradient[3+k]+=gc(8*piece+k,0);
  }
  out.gradient.segment(0,1)=gradients[0];out.gradient.segment(1,2)=gradients[1];
  tc::current()=previous;return out;
}
int main(){
  setenv("DYNAMIC_WINDOW_CONTINUITY","1",1);
  Eigen::VectorXd x(6);x<<.37,.61,2.2,.2,.09,.01;
  const auto analytic=objective(x);double maximum=0.;
  std::cout<<std::setprecision(17)<<"{\"finite_difference\":[";
  for(int i=0;i<x.size();++i){
    auto plus=x,minus=x;plus[i]+=1e-6;minus[i]-=1e-6;
    const double numeric=(objective(plus).cost-objective(minus).cost)/2e-6;
    const double error=std::abs(numeric-analytic.gradient[i]);maximum=std::max(maximum,error);
    if(i)std::cout<<',';
    std::cout<<"{\"coordinate\":"<<i<<",\"analytic\":"<<analytic.gradient[i]
      <<",\"numeric\":"<<numeric<<",\"error\":"<<error<<'}';
  }
  // A shrinking intersection must have vanishing cost, rather than a whole
  // sample's penalty appearing when a node crosses either prediction edge.
  const auto a=window_continuity::clip(1.3-1e-8,1.,0.,1.3);
  const auto b=window_continuity::clip(1.3+1e-8,1.,0.,1.3);
  const bool boundary=a.b-a.a<2e-8&&b.b-b.a<0.;
  const bool pass=maximum<1e-7&&boundary;
  std::cout<<"],\"maximum_error\":"<<maximum<<",\"shrinking_boundary\":"<<(boundary?"true":"false")
    <<",\"pass\":"<<(pass?"true":"false")<<"}\n";
  return pass?0:1;
}
