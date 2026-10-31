// Native MinSnapOpt<8> -> actual coupling_bridge::addPrediction -> getGrad2TP.
// No ros::init, NodeHandle, ROS graph, simulator or robot control publication.
#include <plan_manage/planner_manager.h>
using remani_planner::KinoAstar;  // Legacy collab template's unqualified name.
#include "coupling_bridge.hpp"
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

using Times = std::vector<Eigen::VectorXd>;
using Matrices = std::vector<Eigen::MatrixXd>;
struct Input { Times times; Matrices points, head, tail; std::vector<int> signs; };
struct Gradient { Times times; Matrices points, head, tail; };

namespace {
std::size_t checked_components = 0;
double maximum_error = 0.;
void require(bool ok, const char* why) {
  if (!ok) throw std::runtime_error(why);
}
void near(double analytic, double numeric, const char* label) {
  const double error=std::abs(analytic-numeric)/(1.+std::abs(analytic)+std::abs(numeric));
  maximum_error=std::max(maximum_error,error);++checked_components;
  if (!std::isfinite(error) || error>7e-6) {
    std::cerr<<label<<" analytic="<<analytic<<" numeric="<<numeric<<" error="<<error<<'\n';
    std::abort();
  }
}
Input input() {
  Input x;x.signs={1,-1};
  x.times={Eigen::Vector3d(.53,.61,.57),Eigen::Vector2d(.62,.58)};
  x.head.resize(2);x.tail.resize(2);x.points.resize(2);
  for(int b=0;b<2;++b){
    x.head[b]=Eigen::MatrixXd::Zero(8,4);x.tail[b]=Eigen::MatrixXd::Zero(8,4);
    x.points[b]=Eigen::MatrixXd::Zero(8,x.times[b].size()-1);
    for(int d=2;d<8;++d){
      x.head[b](d,0)=.01*d;x.tail[b](d,0)=.01*d;
      x.points[b].row(d).setConstant(.01*d);
    }
  }
  x.head[0].block<2,1>(0,0)=Eigen::Vector2d(.017,-.023);
  x.head[0].block<2,1>(0,1)=Eigen::Vector2d(.14,.015);
  x.head[0].block<2,1>(0,2)=Eigen::Vector2d(.013,-.011);
  x.head[0].block<2,1>(0,3)=Eigen::Vector2d(.007,.008);
  x.points[0].block<2,1>(0,0)=Eigen::Vector2d(.16,.013);
  x.points[0].block<2,1>(0,1)=Eigen::Vector2d(.29,.026);
  x.tail[0].block<2,1>(0,0)=Eigen::Vector2d(.46,.031);
  x.tail[0].block<2,1>(0,1)=Eigen::Vector2d(.13,.014);
  x.tail[0].block<2,1>(0,2)=Eigen::Vector2d(-.008,.009);
  x.tail[0].block<2,1>(0,3)=Eigen::Vector2d(.006,-.004);
  x.head[1]=x.tail[0];x.head[1].col(1).head(2)*=-1.;
  x.points[1].block<2,1>(0,0)=Eigen::Vector2d(.31,.037);
  x.tail[1].block<2,1>(0,0)=Eigen::Vector2d(.19,.042);
  x.tail[1].block<2,1>(0,1)=Eigen::Vector2d(-.10,.009);
  return x;
}
coupling_objective::Target target(double x,double y,double yaw,double speed,double t,int gear=1){
  coupling_objective::Target q;q.pos={x,y};q.yaw=yaw;q.speed=speed;q.time=t;q.gear=gear;return q;
}
coupling_bridge::Prediction prediction() {
  coupling_bridge::Prediction p;p.valid=true;p.reason="synthetic_frozen_unit_fixture";
  p.request_time=ros::Time::now().toSec();p.seed.stamp=p.request_time-.05;
  p.seed.odom=p.seed.cloud=p.seed.dynamic=p.seed.stamp;
  p.seed.revision=7;p.seed.generation=2;p.seed.goal=123;
  p.targets.checked_begin=0.;p.targets.checked_end=2.;
  p.targets.execution={target(.11,.21,.10,.17,.3),target(.24,.24,.12,.18,.6),target(.39,.20,.08,.16,.9)};
  p.targets.has_delivery=true;p.targets.delivery=target(.22,.19,.11,.17,.7);
  for(int i=0;i<=20;++i)p.seed.rows.push_back({i*.1,.017+i*.02,-.023+i*.006,.10,.17,.01,.2,0.});
  return p;
}
struct PredictionScope {
  coupling_bridge::Prediction* previous;
  explicit PredictionScope(coupling_bridge::Prediction& p):previous(coupling_bridge::activePrediction()){
    coupling_bridge::activePrediction()=&p;
  }
  ~PredictionScope(){coupling_bridge::activePrediction()=previous;}
};
void weights(double execution,double delivery){
  const std::string e=std::to_string(execution),h=std::to_string(delivery);
  setenv("COUPLING_EXEC_WEIGHT",e.c_str(),1);setenv("COUPLING_HAND_WEIGHT",h.c_str(),1);
}
double evaluate(const Input& x,coupling_bridge::Prediction& p,Gradient* gradient=nullptr,
                double* execution=nullptr,double* hand=nullptr){
  const Input unchanged=x;
  std::vector<poly_traj::MinSnapOpt<8>> opts(x.times.size());Times gradT(x.times.size());
  Matrices initial_coefficients(x.times.size());
  int pieces=0;
  for(std::size_t b=0;b<opts.size();++b){
    opts[b].reset(x.times[b].size());opts[b].generate(x.points[b],x.times[b],x.head[b],x.tail[b]);
    opts[b].get_gdC().setZero();gradT[b]=Eigen::VectorXd::Zero(x.times[b].size());pieces+=x.times[b].size();
    initial_coefficients[b]=opts[b].get_b();
  }
  PredictionScope active(p);
  coupling_bridge::Optimizer scope(12345,x.times.size(),pieces,x.signs);
  const double cost=coupling_bridge::addPrediction(opts,x.times,x.signs,gradT);
  for(std::size_t b=0;b<opts.size();++b){
    require((opts[b].get_b()-initial_coefficients[b]).isZero(0.),"coupling changed native initial polynomial");
    require((opts[b].get_T1()-x.times[b]).isZero(0.),"coupling changed native piece count/durations");
  }
  if(execution)*execution=scope.last_exec;
  if(hand)*hand=scope.last_hand;
  if(gradient){
    gradient->points.resize(opts.size());gradient->head.resize(opts.size());gradient->tail.resize(opts.size());
    for(std::size_t b=0;b<opts.size();++b){
      gradient->points[b]=Eigen::MatrixXd::Zero(8,x.points[b].cols());
      opts[b].getGrad2TP(gradT[b],gradient->points[b],gradient->head[b],gradient->tail[b]);
    }
    gradient->times=gradT;
  }
  require(x.times.size()==unchanged.times.size()&&x.signs==unchanged.signs,"native block count/gear changed");
  for(std::size_t b=0;b<x.times.size();++b){
    require((x.times[b]-unchanged.times[b]).isZero(0.),"native initial times mutated");
    require((x.points[b]-unchanged.points[b]).isZero(0.),"native initial waypoints mutated");
    require((x.head[b]-unchanged.head[b]).isZero(0.)&&(x.tail[b]-unchanged.tail[b]).isZero(0.),"native initial boundaries mutated");
  }
  return cost;
}
void gradients(const Input& x,coupling_bridge::Prediction& p){
  Gradient g;const double cost=evaluate(x,p,&g);require(cost>0.,"fixture must exercise objective");
  constexpr double eps=2e-6;
  for(std::size_t b=0;b<x.times.size();++b){
    for(int piece=0;piece<x.times[b].size();++piece){
      auto plus=x,minus=x;plus.times[b][piece]+=eps;minus.times[b][piece]-=eps;
      near(g.times[b][piece],(evaluate(plus,p)-evaluate(minus,p))/(2*eps),"native duration");
    }
    for(int d=0;d<8;++d){
      for(int k=0;k<x.points[b].cols();++k){
        auto plus=x,minus=x;plus.points[b](d,k)+=eps;minus.points[b](d,k)-=eps;
        near(g.points[b](d,k),(evaluate(plus,p)-evaluate(minus,p))/(2*eps),"native waypoint");
      }
      for(int derivative=0;derivative<4;++derivative){
        auto plus=x,minus=x;plus.head[b](d,derivative)+=eps;minus.head[b](d,derivative)-=eps;
        near(g.head[b](d,derivative),(evaluate(plus,p)-evaluate(minus,p))/(2*eps),"native head PVAJ");
        plus=x;minus=x;plus.tail[b](d,derivative)+=eps;minus.tail[b](d,derivative)-=eps;
        near(g.tail[b](d,derivative),(evaluate(plus,p)-evaluate(minus,p))/(2*eps),"native tail PVAJ");
      }
    }
  }
  // A shared gear waypoint receives tail(block 0)+head(block 1) derivatives.
  for(int d=0;d<8;++d){
    auto plus=x,minus=x;plus.tail[0](d,0)+=eps;plus.head[1](d,0)+=eps;
    minus.tail[0](d,0)-=eps;minus.head[1](d,0)-=eps;
    near(g.tail[0](d,0)+g.head[1](d,0),(evaluate(plus,p)-evaluate(minus,p))/(2*eps),"shared gear position");
  }
  // Native angle variable rotates both signed boundary velocities.
  const double theta=.12,speed=.13;
  auto at=[&](double a){auto v=x;v.tail[0].col(1).head(2)=speed*Eigen::Vector2d(std::cos(a),std::sin(a));
    v.head[1].col(1).head(2)=-v.tail[0].col(1).head(2);return v;};
  auto angle_input=at(theta);Gradient angle_gradient;evaluate(angle_input,p,&angle_gradient);
  const Eigen::Vector2d derivative=speed*Eigen::Vector2d(-std::sin(theta),std::cos(theta));
  near(angle_gradient.tail[0].col(1).head(2).dot(derivative)-angle_gradient.head[1].col(1).head(2).dot(derivative),
       (evaluate(at(theta+eps),p)-evaluate(at(theta-eps),p))/(2*eps),"native gear angle");
}
void zero(const Gradient& g){
  for(const auto& v:g.times)require(v.isZero(0.),"nonzero disabled duration gradient");
  for(const auto& v:g.points)require(v.isZero(0.),"nonzero disabled waypoint gradient");
  for(const auto& v:g.head)require(v.isZero(0.),"nonzero disabled head gradient");
  for(const auto& v:g.tail)require(v.isZero(0.),"nonzero disabled tail gradient");
}
void requestBoundaries(){
  const double now=ros::Time::now().toSec();const std::string goal="native_prediction_test";
  collab::Seed s;s.stamp=now-.05;s.odom=s.cloud=s.dynamic=s.stamp;
  s.revision=2;s.generation=1;s.goal=double(dual_probe::hash(goal)&0xffffffffULL);
  coupling_bridge::referenceRevision()=2;coupling_bridge::goalGeneration()=1;
  for(int i=0;i<=20;++i)s.rows.push_back({i*.1,i*.02,0.,0.,.2,0.,.2,0.});
  Eigen::VectorXd start=Eigen::VectorXd::Zero(8);start(0)=.01;
  collab::seed()=s;
  {coupling_bridge::Request r(goal,start,0.,1);require(r.value.valid,"fresh request rejected");
    require(r.value.targets.has_delivery,"fresh delivery missing");
    near(r.value.targets.delivery.time,now-s.stamp+r.value.delay,"seed/request delivery clock");
    const auto frozen=r.value.seed.rows;collab::seed().rows[1][1]+=1.;
    require(r.value.seed.rows==frozen,"request prediction was not frozen");}
  collab::seed()=s;collab::seed().stamp=now-.5;
  {coupling_bridge::Request r(goal,start,0.,1);require(!r.value.valid,"stale request accepted");}
  collab::seed()=s;
  {coupling_bridge::Request r("other",start,0.,1);require(!r.value.valid,"other-goal request accepted");}
  collab::seed()=s;collab::seed().revision=1;
  {coupling_bridge::Request r(goal,start,0.,1);require(!r.value.valid,"old-reference epoch accepted");}
  collab::seed()=s;collab::seed().generation=2;
  {coupling_bridge::Request r(goal,start,0.,1);require(!r.value.valid,"other-generation epoch accepted");}
  collab::seed()=s;collab::seed().rows.resize(4);
  {coupling_bridge::Request r(goal,start,0.,1);require(!r.value.targets.has_delivery,"delivery forecast extrapolated");}
  collab::seed()=s;
  for(std::size_t i=9;i<collab::seed().rows.size();++i)collab::seed().rows[i][4]=-.2;
  {coupling_bridge::Request r(goal,start,0.,1);
    for(const auto&t:r.value.targets.execution)require(t.speed>=0.&&t.time<.9,"execution matched future reverse gear");
    if(r.value.targets.has_delivery)require(r.value.targets.delivery.speed>=0.,"delivery matched reverse gear");}
}
}  // namespace

int main(){
  ros::Time::init();ros::Time::setNow(ros::Time(1000.));
  auto x=input();auto p=prediction();
  // Scope logging has one explicit small temporary artifact, never a run log.
  char directory[]="/tmp/coupling_prediction_native_XXXXXX";
  require(mkdtemp(directory)!=nullptr,"cannot create temporary log directory");
  const std::string logfile=std::string(directory)+"/scope.jsonl";
  setenv("DUAL_TIMELINE_PATH",logfile.c_str(),1);weights(1.7,2.3);
  double e=0.,h=0.;const double joint=evaluate(x,p,nullptr,&e,&h);
  require(e>0.&&h>0.,"both costs must be active");unsetenv("DUAL_TIMELINE_PATH");
  std::ifstream log(logfile);std::stringstream lines;lines<<log.rdbuf();
  require(lines.str().find("coupling_scope")!=std::string::npos&&lines.str().find("coupling_cost")!=std::string::npos,"scope logging missing");
  near(joint,e+h,"cost decomposition");
  weights(1.7,0.);near(evaluate(x,p),e,"execution-only ablation");gradients(x,p);
  weights(0.,2.3);near(evaluate(x,p),h,"delivery-only ablation");gradients(x,p);
  weights(1.7,2.3);gradients(x,p);

  // The native reverse block is deliberately included, but same-gear snapshot
  // matching must not influence this later spatially nearby opposite gear.
  Gradient g;evaluate(x,p,&g);require(g.times[1].isZero(0.)&&g.points[1].isZero(0.)&&
      g.head[1].isZero(0.)&&g.tail[1].isZero(0.),"prediction coupled opposite gear block");
  weights(0.,0.);require(evaluate(x,p,&g)==0.,"zero-weight cost is not exact identity");zero(g);
  // Cold start without a local reference is an inactive, logged scope, not an
  // invented prediction. It must not prevent native-only optimization.
  auto cold=p;cold.valid=false;cold.reason="missing";weights(1.7,2.3);
  require(evaluate(x,cold,&g)==0.,"missing prediction altered native objective");zero(g);
  auto mismatch=x;mismatch.signs[0]=-1;
  require(evaluate(mismatch,p,&g)==0.,"opposite-gear scope was not disabled");zero(g);
  requestBoundaries();
  require(coupling_bridge::activePrediction()==nullptr&&coupling_bridge::current()==nullptr,"scope restoration failed");
  std::cout<<"NATIVE_PREDICTION_GRADIENTS_PASSED "<<checked_components
           <<" components; max_relative_error="<<maximum_error
           <<"; 2 blocks/5 pieces, zero-weight identity, request clocks, no ROS graph; log="<<logfile<<'\n';
}
