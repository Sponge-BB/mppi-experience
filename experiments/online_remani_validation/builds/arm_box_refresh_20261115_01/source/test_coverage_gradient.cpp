// Finite-difference check of the actual frozen-CV distance helper, cubic
// obstacle integral and duration-gradient accumulator used by the experiment.
#include "coverage_collision.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct Result {double value=0.;Eigen::VectorXd gradient=Eigen::VectorXd::Zero(7);};
Result objective(const Eigen::VectorXd& x,double radius_cap,bool full_coverage){
  namespace tc=time_indexed_collision;
  tc::Context context;context.active=true;context.requested_epoch=100.;context.begin_epoch=100.;context.end_epoch=106.;
  context.full_coverage=full_coverage;
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
bool coverageChecks(){
  namespace tc=time_indexed_collision;
  tc::Context c;c.active=true;c.requested_epoch=100.4;c.begin_epoch=100.;c.end_epoch=101.2;
  c.source.snapshot.stamp.fromSec(100.);c.source.parameters.max_prediction=3.;
  c.source.parameters.cap=.01;c.source.parameters.influence=2.;
  c.source.parameters.uncertainty0=.04;c.edge_radius=.03;
  plan_env::DynamicPoint q;q.position={.7,0.,0.};q.velocity={.1,0.,0.};c.source.snapshot.points.push_back(q);
  tc::SampleState legacy;legacy.branches[0]=tc::branch(c,.5,1.);legacy.branches[1]=tc::branch(c,.5,2.);
  c.full_coverage=false;double d=2.;Eigen::Vector3d g=Eigen::Vector3d::Zero();
  const bool legacy_fallback=tc::indexedDistance(c,legacy,Eigen::Vector3d::Zero(),d,g);
  c.full_coverage=true;c.end_epoch=103.;
  tc::SampleState covered;covered.branches[0]=tc::branch(c,.5,1.);covered.branches[1]=tc::branch(c,.5,2.);
  d=2.;const bool covered_fallback=tc::indexedDistance(c,covered,Eigen::Vector3d::Zero(),d,g);
  tc::SampleState edge;edge.branches[0]=tc::branch(c,1.4,1.);edge.branches[1]=tc::branch(c,1.4,2.);
  d=2.;const bool unknown_fallback=tc::indexedDistance(c,edge,Eigen::Vector3d::Zero(),d,g);
  return legacy.branches[0].valid&&!legacy.branches[1].valid&&legacy_fallback&&
    covered.branches[0].valid&&covered.branches[1].valid&&!covered_fallback&&
    edge.branches[0].valid&&!edge.branches[1].valid&&!unknown_fallback&&
    tc::branch(c,1.3,2.).valid&&!tc::branch(c,1.3001,2.).valid&&
    tc::branch(c,1.3000004,2.).valid&&!tc::branch(c,1.3000006,2.).valid&&
    std::abs(legacy.branches[0].radius-.01)<1e-12&&
    std::abs(covered.branches[0].radius-.04)<1e-12;
}
int main(){
  const bool coverage_ok=coverageChecks();
  Eigen::VectorXd x(7);x<<.25,.10,.015,.60,.70,.40,.50;
  const char* names[]={"polynomial_c0","polynomial_c1","polynomial_c2","prior_segment_T0","prior_segment_T1","prior_piece_T0","current_piece_T1"};
  std::cout<<std::setprecision(17)<<"{\"test\":\"actual CV helper plus native cubic integral; nominal+half minimum; previous segment and previous piece propagation\",\"rows\":[";
  double maximum=0.;bool first=true;
  for(int mode=0;mode<3;++mode){
    const double cap=mode==0?.2:.02;const bool full=mode==2;
    const auto analytic=objective(x,cap,full);
    for(int k=0;k<x.size();++k){const double h=1e-6;auto plus=x,minus=x;plus[k]+=h;minus[k]-=h;
      const double numerical=(objective(plus,cap,full).value-objective(minus,cap,full).value)/(2*h);
      const double error=std::abs(numerical-analytic.gradient[k]);maximum=std::max(maximum,error);
      if(!first)std::cout<<',';first=false;
      std::cout<<"{\"full_coverage\":"<<(full?"true":"false")<<",\"radius_cap\":"<<cap<<",\"parameter\":\""<<names[k]<<"\",\"analytic\":"<<analytic.gradient[k]
        <<",\"numerical\":"<<numerical<<",\"absolute_error\":"<<error<<'}';
    }
  }
  std::cout<<"],\"maximum_absolute_error\":"<<maximum<<",\"coverage_window_pass\":"<<(coverage_ok?"true":"false")<<",\"pass\":"<<(maximum<1e-7&&coverage_ok?"true":"false")<<"}\n";
  return maximum<1e-7&&coverage_ok?0:1;
}
