// Native MinSnapOpt<8> gradient test, no ROS graph/robot command publication.
#include "coupling_native_integration.hpp"
#include "traj_utils/poly_traj_utils.hpp"
#include <cassert>
#include <iostream>

using Times=std::vector<Eigen::VectorXd>;
using Matrices=std::vector<Eigen::MatrixXd>;
struct Input { Times times;Matrices points,head,tail; };
struct Gradient { Times times;Matrices points,head,tail; };
static Input input() {
  Input x;
  x.times={Eigen::Vector2d(.43,.48),Eigen::Vector2d(.51,.56)};
  x.head.resize(2);x.tail.resize(2);x.points.resize(2);
  for(int b=0;b<2;++b) {
    x.head[b]=Eigen::MatrixXd::Zero(8,4);x.tail[b]=Eigen::MatrixXd::Zero(8,4);
    const double origin=.15*b;
    x.head[b](0,0)=origin+.017;x.head[b](1,0)=-.033;
    x.tail[b](0,0)=origin+.15;x.tail[b](1,0)=.014;
    x.head[b](0,1)=.13;x.tail[b](0,1)=.14;
    x.head[b](1,1)=.02;x.tail[b](1,1)=.03;
    x.points[b]=Eigen::MatrixXd::Zero(8,1);
    x.points[b](0,0)=origin+.083;x.points[b](1,0)=-.009;
  }
  return x;
}
static double evaluate(const Input& x,Gradient* gradient=nullptr) {
  std::vector<poly_traj::MinSnapOpt<8>> opts(2);Times gradT(2);
  for(int b=0;b<2;++b) {
    opts[b].reset(x.times[b].size());
    opts[b].generate(x.points[b],x.times[b],x.head[b],x.tail[b]);
    opts[b].get_gdC().setZero();gradT[b]=Eigen::VectorXd::Zero(x.times[b].size());
  }
  const double cost=coupling_native::addGeometry(opts,x.times,std::vector<int>{1,-1},gradT,8);
  if(gradient) {
    gradient->points.resize(2);gradient->head.resize(2);gradient->tail.resize(2);
    for(int b=0;b<2;++b) {
      gradient->points[b]=Eigen::MatrixXd::Zero(8,1);
      opts[b].getGrad2TP(gradT[b],gradient->points[b],gradient->head[b],gradient->tail[b]);
    }
    gradient->times=gradT;
  }
  return cost;
}
static void near(double actual,double expected,const char* label) {
  const double error=std::abs(actual-expected)/(1.+std::abs(actual)+std::abs(expected));
  if(!std::isfinite(error)||error>5e-6) {
    std::cerr<<label<<" analytic="<<actual<<" numeric="<<expected<<" rel="<<error<<'\n';
    std::abort();
  }
}
int main() {
  setenv("COUPLING_ENGINEERING","1",1);
  ros::Time::init();ros::Time::setNow(ros::Time(1000.));
  auto m=std::make_shared<shared_collision::Snapshot>();
  m->stamp=999.95;m->healthy=true;m->gx=-3;m->gy=-3;m->res=.1;m->nx=61;m->ny=61;
  m->comfort=.32;m->weight=17.;m->max_prediction=1.67;
  m->body={{.31,.12,.18},{-.27,-.08,.21}};
  for(int iy=0;iy<m->ny;++iy)for(int ix=0;ix<m->nx;++ix) {
    double x=m->gx+ix*m->res,y=m->gy+iy*m->res;
    m->field.push_back(.36+.02*x+.025*y+.005*x*y);
  }
  m->obstacles={{.52,.12,-.035,.023,.13,1.},{-.56,-.13,.02,-.01,.12,1.}};
  std::shared_ptr<const shared_collision::Snapshot> frame=m;
  std::atomic_store(&shared_collision::store(),frame);
  Input x=input();Gradient gradient;const double cost=evaluate(x,&gradient);
  assert(cost>0.);
  constexpr double epsilon=2e-6;
  // Durations include explicit later-sample dynamic time shifts across both
  // pieces AND blocks, plus MINCO coefficient dependence through getGrad2TP.
  for(int b=0;b<2;++b)for(int p=0;p<2;++p) {
    auto plus=x,minus=x;plus.times[b][p]+=epsilon;minus.times[b][p]-=epsilon;
    near(gradient.times[b][p],(evaluate(plus)-evaluate(minus))/(2*epsilon),"duration");
  }
  for(int b=0;b<2;++b)for(int d=0;d<8;++d) {
    auto plus=x,minus=x;plus.points[b](d,0)+=epsilon;minus.points[b](d,0)-=epsilon;
    near(gradient.points[b](d,0),(evaluate(plus)-evaluate(minus))/(2*epsilon),"waypoint");
    for(int derivative=0;derivative<4;++derivative) {
      plus=x;minus=x;plus.head[b](d,derivative)+=epsilon;minus.head[b](d,derivative)-=epsilon;
      near(gradient.head[b](d,derivative),(evaluate(plus)-evaluate(minus))/(2*epsilon),"head");
      plus=x;minus=x;plus.tail[b](d,derivative)+=epsilon;minus.tail[b](d,derivative)-=epsilon;
      near(gradient.tail[b](d,derivative),(evaluate(plus)-evaluate(minus))/(2*epsilon),"tail");
    }
  }
  // No engineering switch means no added objective/gradients.
  setenv("COUPLING_ENGINEERING","0",1);Gradient disabled;
  assert(evaluate(x,&disabled)==0.);
  for(const auto& t:disabled.times)assert(t.isZero(0.));
  for(const auto& p:disabled.points)assert(p.isZero(0.));
  setenv("COUPLING_ENGINEERING","1",1);ros::Time::setNow(ros::Time(1001.));
  bool rejected=false;try{evaluate(x);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
  std::cout<<"NATIVE_GEOMETRY_GRADIENTS_PASSED two blocks/four pieces; "
              "time prefixes, waypoint, head/tail; taper and unknown tail; no ROS graph\n";
}
