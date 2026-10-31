#include "candidate_audit.hpp"
#include <cassert>
#include <iostream>

#ifdef NDEBUG
#error "candidate audit tests require active assertions"
#endif

struct Straight {
  double duration=.2;
  Eigen::VectorXd getPos(double t) const {Eigen::VectorXd v(2);v<<t,0;return v;}
  Eigen::VectorXd getVel(double) const {Eigen::VectorXd v(2);v<<1,0;return v;}
  Eigen::VectorXd getAcc(double) const {return Eigen::VectorXd::Zero(2);}
  Eigen::VectorXd getJer(double) const {return Eigen::VectorXd::Zero(2);}
  double getCarAngle(double) const{return 0;}
  int getSingul(double) const{return 1;}
};
struct Map {
  double getPreciseDistance(const Eigen::Vector3d&) const{return .5;}
  bool getDynamicPointPlanningDistance(const Eigen::Vector3d&,double maximum,double& value,Eigen::Vector3d*) const{
    assert(maximum==.5);value=.12;return true;
  }
};
struct Model {
  int getManiDof() const{return 0;}
  void getCarPts(const Eigen::Vector3d& p,std::vector<Eigen::Vector3d>& out) const{out={p};}
  double getBaseSphereRadius(std::size_t) const{return .1;}
  double getMapResolution() const{return .05;}
  std::vector<Eigen::Matrix4Xd> getLinkPoint() const{return {};}
  void getJointTrans(const Eigen::VectorXd&,std::vector<Eigen::Matrix4d>&,std::vector<Eigen::Matrix4d>&) const{}
  Eigen::Matrix4d getTq0() const{return Eigen::Matrix4d::Identity();}
  bool checkCarPlanningObsCollision(Eigen::Vector3d,bool,bool,double&)const{return true;}
  bool checkManiPlanningObsCollision(Eigen::Vector3d,Eigen::VectorXd,bool,double&)const{return false;}
  bool checkCarManiCollision(Eigen::VectorXd,bool,double&)const{return false;}
  bool checkManiManiCollision(Eigen::VectorXd,bool,double&)const{return false;}
};
int main(){
  Eigen::VectorXd initial(2),accepted(2);initial<<1,2;accepted<<2,3;
  {
    candidate_audit::Session disabled(false);
    candidate_audit::recordInitial(initial,100);assert(disabled.steps.empty());
  }
  {
    candidate_audit::Session scope(true);scope.begin();
    Eigen::VectorXd components(6);components<<1,2,3,4,5,6;
    candidate_audit::recordCostComponents(7,components,8);
    candidate_audit::recordInitial(initial,100);
    candidate_audit::recordInitial(accepted,50);assert(scope.steps.size()==1&&scope.steps[0].x==initial);
    candidate_audit::recordAccepted(accepted,50,2,3);
    assert(scope.steps.size()==2&&scope.steps[1].iteration==2&&scope.steps[1].line_search==3);
    assert(scope.steps[1].cost_seen&&scope.steps[1].costs[0]==7&&scope.steps[1].costs[6]==6&&scope.steps[1].costs[7]==8);
    assert(scope.steps[1].solver_elapsed_s>=scope.steps[0].solver_elapsed_s);
    Eigen::VectorXd bad=initial;bad[0]=std::numeric_limits<double>::quiet_NaN();
    candidate_audit::recordAccepted(bad,1,3,1);assert(scope.steps.size()==2);
  }
  assert(candidate_audit::active()==nullptr);
  Straight traj;Model model;Map map;candidate_audit::Params p;
  p.wheel_radius=.1;p.wheel_base=.5;p.wheel_limit=4;p.alpha_limit=4;p.base_safe_margin=.1;
  const auto sample=candidate_audit::sample(traj,model,map,p,.05);
  assert(std::abs(sample.wheels[0]-10)<1e-12&&sample.wheels[2]==0);
  assert(std::abs(sample.base_gap+.03)<1e-12);
  assert(std::abs(sample.base_static_gap-.35)<1e-12);
  assert(std::abs(sample.base_dynamic_gap+.03)<1e-12);
  assert(std::abs(sample.base_soft_gap+.08)<1e-12);
  candidate_audit::Summary summary;summary.add(sample,p);
  assert(summary.first_failure=="planning_collision"&&summary.first_collision_class==0);
  assert(summary.max_wheels[0]==10);
  const auto detail=candidate_audit::audit(traj,model,map,p,{0,.1,.2},true);
  assert(detail.find("\"sample_count\":15")!=std::string::npos);
  assert(detail.find("\"optimizer_sample_times_count\":3")!=std::string::npos);
  assert(detail.find("\"dynamic_query_available\":true")!=std::string::npos);
  const auto compact=candidate_audit::audit(traj,model,map,p,{0,.1,.2},false,{0,.01,.1,.2});
  assert(compact.find("\"samples\":[]")!=std::string::npos);
  assert(compact.find("\"optimizer_samples\":[]")!=std::string::npos);
  assert(compact.find("\"optimizer_summary\":{\"sample_count\":3")!=std::string::npos);
  assert(compact.find("\"feasibility_optimizer_summary\":{\"sample_count\":4,\"grid\":\"optimizer_feasibility_grid\",\"collision_queried\":false")!=std::string::npos);
  std::cout<<"candidate audit: capture order, finite guards, native wheel algebra, gap decomposition, and sample grids passed\n";
}
