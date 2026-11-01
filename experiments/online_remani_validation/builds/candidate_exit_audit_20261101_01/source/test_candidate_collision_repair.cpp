// Finite differences for fixed piece weighting of the actual CV distance/time
// helpers, including base orientation's velocity derivative and duration chain.
#include "coverage_collision.hpp"
#include "candidate_collision_repair.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct Result{double value=0.;Eigen::VectorXd gradient=Eigen::VectorXd::Zero(7);};
Result objective(const Eigen::VectorXd& x,double multiplier){
  namespace tc=time_indexed_collision;
  tc::Context context;context.active=true;context.full_coverage=true;
  context.requested_epoch=100.;context.begin_epoch=100.;context.end_epoch=106.;
  context.source.snapshot.stamp.fromSec(100.);context.source.parameters.max_prediction=6.;
  context.source.parameters.uncertainty0=.01;context.source.parameters.uncertainty_v=.006;
  context.source.parameters.uncertainty_a=.002;context.source.parameters.influence=2.;
  plan_env::DynamicPoint point;point.position={.9,.2,.3};point.velocity={-.12,.03,0.};
  context.source.snapshot.points.push_back(point);
  auto* previous=tc::current();tc::current()=&context;
  std::vector<Eigen::VectorXd> times(2),gradients(2);
  times[0]=x.segment(3,2);times[1]=x.segment(5,2);
  gradients[0]=Eigen::VectorXd::Zero(2);gradients[1]=Eigen::VectorXd::Zero(2);
  Result out;
  {
    tc::Durations duration_scope(times,gradients);
    const int K=8;const double T=times[1](1),step=T/K,weight=1.2*multiplier;
    for(int j=0;j<=K;++j){
      const double alpha=double(j)/K,s=alpha*T,omg=j==0||j==K?.5:1.;
      tc::Sample time(tc::elapsedAt(1,1,s));
      const Eigen::Vector2d base(x[0]+x[1]*s+x[2]*s*s,.05+.04*s),vel(x[1]+2*x[2]*s,.04),acc(2*x[2],0.);
      const double yaw=std::atan2(vel.y(),vel.x()),c=std::cos(yaw),sn=std::sin(yaw);
      const Eigen::Vector2d offset(.18,.08),rotated(c*offset.x()-sn*offset.y(),sn*offset.x()+c*offset.y());
      const Eigen::Vector3d pos(base.x()+rotated.x(),base.y()+rotated.y(),.3);
      double distance=2.;Eigen::Vector3d normal=Eigen::Vector3d::Zero();
      if(tc::indexedDistance(context,*tc::sample(),pos,distance,normal))throw std::runtime_error("test outside CV window");
      const double error=.5-distance;if(error<=0.)throw std::runtime_error("test inactive");
      const double cost=weight*error*error*error;const Eigen::Vector2d gp=-3.*weight*error*error*normal.head(2);
      const Eigen::Vector2d dtheta(-vel.y()/vel.squaredNorm(),vel.x()/vel.squaredNorm());
      const Eigen::Vector2d drot(-rotated.y(),rotated.x());const Eigen::Vector2d gv=gp.dot(drot)*dtheta;
      out.value+=omg*step*cost;
      out.gradient[0]+=omg*step*gp.x();
      out.gradient[1]+=omg*step*(s*gp.x()+gv.x());
      out.gradient[2]+=omg*step*(s*s*gp.x()+2*s*gv.x());
      gradients[1](1)+=omg*(cost/K+step*alpha*(gp.dot(vel)+gv.dot(acc)));
      tc::penaltyTimeDerivative(-3.*weight*error*error);
      tc::accumulateTimeGradient(1,1,alpha,omg*step);
    }
  }
  out.gradient.segment(3,2)=gradients[0];out.gradient.segment(5,2)=gradients[1];tc::current()=previous;return out;
}
int main(){
  candidate_collision_repair::Scope repair;repair.enabled=true;
  std::vector<Eigen::VectorXd> times(2);times[0]=Eigen::Vector2d(.6,.7);times[1]=Eigen::Vector2d(.4,.5);
  repair.target(times,1.95);repair.repairing=true;
  const bool piece_ok=repair.target_trajectory==1&&repair.target_piece==1&&
    candidate_collision_repair::weight(1,1)==10.&&candidate_collision_repair::weight(1,0)==1.;
  Eigen::VectorXd x(7);x<<.25,.1,.015,.6,.7,.4,.5;
  const auto base=objective(x,1.),scaled=objective(x,10.);
  const double scale_error=(scaled.gradient-10.*base.gradient).cwiseAbs().maxCoeff();
  double maximum=0.;std::cout<<std::setprecision(17)<<"{\"test\":\"fixed piece factor10; position, orientation-velocity and explicit CV-time duration derivatives\",\"rows\":[";
  bool first=true;for(double m:{1.,10.}){const auto analytic=objective(x,m);for(int i=0;i<x.size();++i){
    auto plus=x,minus=x;plus[i]+=1e-6;minus[i]-=1e-6;
    const double numerical=(objective(plus,m).value-objective(minus,m).value)/2e-6;
    const double error=std::abs(numerical-analytic.gradient[i]);maximum=std::max(maximum,error);
    if(!first)std::cout<<',';first=false;
    std::cout<<"{\"factor\":"<<m<<",\"parameter\":"<<i<<",\"analytic\":"<<analytic.gradient[i]<<",\"numerical\":"<<numerical<<",\"absolute_error\":"<<error<<'}';
  }}
  const bool pass=piece_ok&&maximum<1e-7&&scale_error<1e-10;
  std::cout<<"],\"maximum_absolute_error\":"<<maximum<<",\"factor10_gradient_scale_error\":"<<scale_error<<",\"piece_selection_pass\":"<<(piece_ok?"true":"false")<<",\"pass\":"<<(pass?"true":"false")<<"}\n";
  return pass?0:1;
}
