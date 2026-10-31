// Exercise the native quadrature-to-typed-signal integration, without a ROS graph.
#include "coupling_native_integration.hpp"
#include <cassert>
#include <iostream>

struct Mock {
  Eigen::VectorXd times=Eigen::VectorXd::Ones(1);
  Eigen::MatrixXd b=Eigen::MatrixXd::Zero(8,2),gd=Eigen::MatrixXd::Zero(8,2);
  const Eigen::VectorXd& get_T1() const { return times; }
  const Eigen::MatrixXd& get_b() const { return b; }
  Eigen::MatrixXd& get_gdC() { return gd; }
};
static std::shared_ptr<shared_collision::Snapshot> fixture() {
  auto m=std::make_shared<shared_collision::Snapshot>();
  m->healthy=true;m->stamp=999.95;m->gx=-7.5;m->gy=-5.;m->res=.05;m->nx=300;m->ny=200;
  m->body={{0.,0.,.1},{.125,0.,.1}};
  m->field.assign(size_t(m->nx)*m->ny,1.);
  return m;
}
static void bind(std::shared_ptr<shared_collision::Snapshot> m) {
  std::shared_ptr<const shared_collision::Snapshot> c=m;
  std::atomic_store(&shared_collision::store(),c);
}
static void evaluate(Eigen::Vector2d p={.423698995724897,4.9096377779231},int gear=1) {
  std::vector<Mock> opts(1);
  opts[0].b.row(0)=p.transpose();
  opts[0].b.row(1)=Eigen::Vector2d(-.1277832969887734,.5422823571572659).transpose();
  std::vector<Eigen::VectorXd> times{Eigen::VectorXd::Ones(1)},grad{Eigen::VectorXd::Zero(1)};
  coupling_native::addGeometry(opts,times,std::vector<int>{gear},grad,2);
}
static void fatalNotDomain() {
  bool rejected=false;
  try { evaluate(); }
  catch(const domain_step::RejectedTrial&) { assert(false && "fatal input became recoverable"); }
  catch(const std::exception&) { rejected=true; }
  assert(rejected);
}
int main() {
  unsetenv("COUPLING_DIAGNOSTIC_DIR");
  setenv("COUPLING_ENGINEERING","1",1);
  setenv("COUPLING_DOMAIN_RECOVERY","1",1);
  ros::Time::init();ros::Time::setNow(ros::Time(1000.));
  auto m=fixture();bind(m);
  bool typed=false;
  try { evaluate(); }
  catch(const domain_step::RejectedTrial& error) {
    typed=true;assert(error.body_index==1);
    assert(std::abs(error.x-.3950292304431109)<1e-12);
    assert(std::abs(error.y-5.03130554094473)<1e-12);
    assert(std::abs(error.upper_y-4.95)<1e-12);
  }
  assert(typed);
  assert(domain_step::rejectedTrials()==0); // construction is not a solver rejection
  setenv("COUPLING_DOMAIN_RECOVERY","0",1);fatalNotDomain();
  setenv("COUPLING_DOMAIN_RECOVERY","1",1);
  for(int gear:{1,-1}) {
    typed=false;
    try { evaluate({0.,5.1},gear); }
    catch(const domain_step::RejectedTrial&) { typed=true; }
    assert(typed);
  }
  m=fixture();m->stamp=998.;bind(m);fatalNotDomain(); // stale snapshot
  m=fixture();m->field.pop_back();bind(m);fatalNotDomain(); // malformed geometry
  m=fixture();m->body[1].r=NAN;bind(m);fatalNotDomain();
  m=fixture();m->stamp=1000.01;bind(m);fatalNotDomain(); // negative forecast, not geometric recovery
  m=fixture();bind(m);
  bool fatal=false;
  try { evaluate({NAN,0.}); }
  catch(const domain_step::RejectedTrial&) { assert(false); }
  catch(const std::exception&) { fatal=true; }
  assert(fatal);
  const auto hard=coupling_geometry::staticProjectedPose(*m,{0.,5.1,0.});
  assert(!hard.valid && hard.gap<0.);
  std::cout<<"DOMAIN_NATIVE_PASSED recorded query through native quadrature; opt-in only; "
             "stale/malformed/nonfinite/time errors fatal; hard outside query unsafe\n";
}
