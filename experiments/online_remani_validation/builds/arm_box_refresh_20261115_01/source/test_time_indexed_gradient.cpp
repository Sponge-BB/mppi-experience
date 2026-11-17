// Finite-difference check of the actual frozen-CV distance helper, cubic
// obstacle integral and duration-gradient accumulator used by the experiment.
#include "time_indexed_collision.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct Result {double value=0.;Eigen::VectorXd gradient=Eigen::VectorXd::Zero(7);};
Result objective(const Eigen::VectorXd& x,double radius_cap){
  namespace tc=time_indexed_collision;
  tc::Context context;context.active=true;context.requested_epoch=100.;context.begin_epoch=100.;context.end_epoch=106.;
  context.source.snapshot.stamp.fromSec(100.);context.source.parameters.max_prediction=6.;
  context.source.parameters.uncertainty0=.01;context.source.parameters.uncertainty_v=.006;
  context.source.parameters.uncertainty_a=.002;context.source.parameters.cap=radius_cap;
  context.source.parameters.influence=2.;context.edge_radius=.01;
  plan_env::DynamicPoint point;point.position={.9,.2,.3};point.velocity={-.12,.03,0.};
  context.source.snapshot.points.push_back(point);point.position={2.,1.,1.};context.source.snapshot.points.push_back(point);
  auto* previous=tc::current();tc::current()=&context;
  std::vector<Eigen::VectorXd> times(2),gradients(2);
  times[0]=x.segment(3,2);times[1]=x.segment(5,2);
  gradients[0]=Eigen::VectorXd::Zero(2);gradients[1]=Eigen::VectorXd::Zero(2);
  Result result;
  {
    tc::Durations duration_scope(times,gradients);
    const int K=8;const double T=times[1](1),step=T/K,weight=1.2;
    for(int j=0;j<=K;++j){
      const double alpha=double(j)/K,s=alpha*T,omg=(j==0||j==K)?.5:1.;
      tc::Sample sample(tc::elapsedAt(1,1,s));
      const Eigen::Vector3d p(x[0]+x[1]*s+x[2]*s*s,.05,.3),v(x[1]+2*x[2]*s,0.,0.);
      double distance=2.;Eigen::Vector3d grad=Eigen::Vector3d::Zero();
      if(tc::indexedDistance(context,*tc::sample(),p,distance,grad))throw std::runtime_error("test left exact horizon");
      const double error=.5-distance;
      if(error<=0.)throw std::runtime_error("test obstacle cost inactive");
      const double cost=weight*error*error*error;
      const Eigen::Vector3d gradp=-3.*weight*error*error*grad;
      result.value+=omg*step*cost;
      result.gradient[0]+=omg*step*gradp.x();
      result.gradient[1]+=omg*step*s*gradp.x();
      result.gradient[2]+=omg*step*s*s*gradp.x();
      gradients[1](1)+=omg*(cost/K+step*alpha*gradp.dot(v));
      tc::penaltyTimeDerivative(-3.*weight*error*error);
      tc::accumulateTimeGradient(1,1,alpha,omg*step);
    }
  }
  result.gradient.segment(3,2)=gradients[0];result.gradient.segment(5,2)=gradients[1];
  tc::current()=previous;return result;
}
int main(){
  Eigen::VectorXd x(7);x<<.25,.10,.015,.60,.70,.40,.50;
  const char* names[]={"polynomial_c0","polynomial_c1","polynomial_c2","prior_segment_T0","prior_segment_T1","prior_piece_T0","current_piece_T1"};
  std::cout<<std::setprecision(17)<<"{\"test\":\"actual CV helper plus native cubic integral; nominal+half minimum; previous segment and previous piece propagation\",\"rows\":[";
  double maximum=0.;bool first=true;
  for(double cap:{.2,.02}){
    const auto analytic=objective(x,cap);
    for(int k=0;k<x.size();++k){const double h=1e-6;auto plus=x,minus=x;plus[k]+=h;minus[k]-=h;
      const double numerical=(objective(plus,cap).value-objective(minus,cap).value)/(2*h);
      const double error=std::abs(numerical-analytic.gradient[k]);maximum=std::max(maximum,error);
      if(!first)std::cout<<',';first=false;
      std::cout<<"{\"radius_cap\":"<<cap<<",\"parameter\":\""<<names[k]<<"\",\"analytic\":"<<analytic.gradient[k]
        <<",\"numerical\":"<<numerical<<",\"absolute_error\":"<<error<<'}';
    }
  }
  std::cout<<"],\"maximum_absolute_error\":"<<maximum<<",\"pass\":"<<(maximum<1e-7?"true":"false")<<"}\n";
  return maximum<1e-7?0:1;
}
