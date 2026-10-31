// Pure numerical/policy tests: no ROS dependency, graph, model or hardware.
#define BOUNDARY_ADMISSION_NUMERICAL_ONLY
#include "boundary_admission.hpp"
#include <cassert>
#include <iostream>

namespace {
shared_collision::Snapshot frame(double distance) {
  shared_collision::Snapshot value;value.sequence=17;value.stamp=100.;value.healthy=true;
  value.nx=value.ny=8;value.res=1.;value.gx=value.gy=-3.;value.margin=.1;
  value.comfort=.2;value.weight=1.;value.max_age=.35;
  value.body={{0.,0.,.2}};value.field.assign(64,distance);return value;
}
boundary_admission::Context context(bool treatment) {
  boundary_admission::Context value;value.treatment=treatment;value.static_scene_declared=true;
  value.local_execution_owner=value.have_local_traj=value.exact_processed_measurement=value.request_scope_active=true;
  value.frozen_scope_valid=true;
  value.actual_fixed_heading_known=value.heading_matches_measurement=true;
  return value;
}
void testStrictCategories() {
  auto value=frame(.25);
  auto result=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(result.valid&&result.category==boundary_admission::Category::margin_only);
  assert(result.physical>0.&&result.gap<0.);
  assert(boundary_admission::decide(result,context(true)).defer);
  const auto off=boundary_admission::decide(result,context(false));
  assert(!off.defer&&off.would_defer&&off.eligible);
  value=frame(.2);
  result=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(result.valid&&result.physical==0.&&result.category==boundary_admission::Category::physical_collision);
  assert(!boundary_admission::decide(result,context(true)).defer);
  value=frame(.1);
  result=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(result.category==boundary_admission::Category::physical_collision);
  value=frame(.5);
  result=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(result.category==boundary_admission::Category::full_margin);
  assert(!boundary_admission::decide(result,context(true)).defer);
  // Use exactly representable values for the exact gap==0 boundary.
  value=frame(.5);value.margin=.25;value.comfort=.3;value.body.front().r=.25;
  result=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(result.valid&&result.gap==0.&&result.category==boundary_admission::Category::full_margin);
}
void testSameNativeGapTolerance() {
  using boundary_admission::Category;
  assert(boundary_admission::same_native_gap_tolerance==1e-8);
  // End-to-end geometry fixtures stay well away from representational noise
  // at exactly -eps. The original native validator accepts the -5e-9 case.
  auto value=frame(.3-5e-9);
  auto result=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(result.valid&&result.physical>0.&&result.gap<0.&&result.gap>=-1e-8);
  assert(result.category==Category::native_tolerance);
  auto decision=boundary_admission::decide(result,context(true));
  assert(!decision.defer&&!decision.eligible&&!decision.would_defer);
  assert(decision.reason=="native_gap_tolerance_accepted");
  value=frame(.3-2e-8);
  result=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(result.valid&&result.gap<-1e-8&&result.category==Category::margin_only);
  assert(boundary_admission::decide(result,context(true)).defer);
  // Test the exact literal and adjacent representable values directly, so a
  // subtraction in a synthetic map cannot blur the STRICT native inequality.
  const double eps=boundary_admission::same_native_gap_tolerance;
  assert(boundary_admission::classifyStaticClearance(.1,-eps)==Category::native_tolerance);
  assert(boundary_admission::classifyStaticClearance(.1,-std::nextafter(eps,0.))==Category::native_tolerance);
  assert(boundary_admission::classifyStaticClearance(.1,-std::nextafter(eps,1.))==Category::margin_only);
  result.physical=.1;result.gap=-eps;
  result.category=boundary_admission::classifyStaticClearance(result.physical,result.gap);
  assert(!boundary_admission::decide(result,context(true)).defer);
  // Even a mislabeled margin_only category cannot bypass the explicit gap test.
  result.category=Category::margin_only;
  assert(!boundary_admission::decide(result,context(true)).defer);
}
void testEligibility() {
  auto value=frame(.25);
  auto good=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  auto c=context(true);
  c.local_execution_owner=false;assert(!boundary_admission::decide(good,c).defer);c=context(true);
  c.have_local_traj=false;assert(!boundary_admission::decide(good,c).defer);c=context(true);
  c.dynamic_repair=true;assert(!boundary_admission::decide(good,c).defer);c=context(true);
  c.exact_processed_measurement=false;assert(!boundary_admission::decide(good,c).defer);c=context(true);
  c.static_scene_declared=false;assert(!boundary_admission::decide(good,c).defer);c=context(true);
  c.request_scope_active=false;assert(!boundary_admission::decide(good,c).defer);
  c=context(true);c.frozen_scope_valid=false;assert(!boundary_admission::decide(good,c).defer);
  c=context(true);c.actual_fixed_heading_known=false;assert(!boundary_admission::decide(good,c).defer);
  c=context(true);c.heading_matches_measurement=false;assert(!boundary_admission::decide(good,c).defer);
  value.obstacles={{1.,1.,0.,0.,.1,0.}};
  good=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1);
  assert(good.valid&&!good.static_obstacles_empty&&!boundary_admission::decide(good,context(true)).defer);
  value.obstacles.front().moving=1.;
  assert(!boundary_admission::decide(boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1),context(true)).defer);
}
void testObservedActualFixedHead() {
  Eigen::MatrixXd head=Eigen::MatrixXd::Zero(8,4);
  head(0,0)=.3;head(1,0)=-.4;head(0,1)=-.02;
  const auto desired_reverse=boundary_admission::fixedHead(head,-1);
  assert(desired_reverse.valid&&desired_reverse.heading_known&&desired_reverse.yaw==0.);
  assert(boundary_admission::exactBaseState(desired_reverse,true,.3,-.4,-.02,0.,-1));
  // RRT fallback may push first gear +1 even with a requested reverse gear.
  // Observe that actual gear/body yaw and exclude it from measured-state defer.
  const auto rrt_flip=boundary_admission::fixedHead(head,1);
  assert(rrt_flip.valid&&rrt_flip.heading_known);
  assert(!boundary_admission::sameHeading(rrt_flip.yaw,desired_reverse.yaw));
  assert(!boundary_admission::exactBaseState(rrt_flip,true,.3,-.4,-.02,0.,-1));
  // Warm initialization or polynomial roundoff must not acquire an approximate
  // identity match: every p/v component and actual gear remains exact.
  auto warm_head=head;warm_head(0,1)=-.03;
  assert(!boundary_admission::exactBaseState(boundary_admission::fixedHead(warm_head,-1),true,.3,-.4,-.02,0.,-1));
  auto roundoff=head;roundoff(0,0)=std::nextafter(.3,1.);
  assert(!boundary_admission::exactBaseState(boundary_admission::fixedHead(roundoff,-1),true,.3,-.4,-.02,0.,-1));
  roundoff=head;roundoff(0,1)=std::nextafter(-.02,0.);
  assert(!boundary_admission::exactBaseState(boundary_admission::fixedHead(roundoff,-1),true,.3,-.4,-.02,0.,-1));
  assert(!boundary_admission::exactBaseState(desired_reverse,false,.3,-.4,-.02,0.,-1));
  const auto unknown_gear=boundary_admission::fixedHead(head,0);
  assert(unknown_gear.valid&&!unknown_gear.heading_known&&std::isnan(unknown_gear.yaw));
  auto zero=head;zero.col(1).setZero();
  assert(!boundary_admission::fixedHead(zero,1).heading_known);
  auto bad=head;bad(7,3)=std::numeric_limits<double>::quiet_NaN();
  assert(!boundary_admission::fixedHead(bad,1).valid);
  assert(!boundary_admission::fixedHead(Eigen::MatrixXd::Zero(8,3),1).valid);
}
void testHeading() {
  auto value=frame(.25);const double pi=std::acos(-1.);
  for(int index=-720;index<=720;++index)for(int gear:{-1,1}) {
    const double yaw=index*pi/720.;
    const auto result=boundary_admission::evaluate(&value,100.1,0.,0.,yaw,
      gear*.02*std::cos(yaw),gear*.02*std::sin(yaw),gear);
    assert(result.nonzero_gear&&result.heading_consistent);
    assert(boundary_admission::decide(result,context(true)).defer);
  }
  auto zero=boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,0);
  assert(zero.valid&&!zero.nonzero_gear&&!boundary_admission::decide(zero,context(true)).defer);
  assert(boundary_admission::decide(zero,context(true)).reason=="unknown_fixed_heading");
  auto mismatch=boundary_admission::evaluate(&value,100.1,0.,0.,0.,-.01,0.,1);
  assert(mismatch.valid&&!mismatch.heading_consistent&&!boundary_admission::decide(mismatch,context(true)).defer);
  mismatch=boundary_admission::evaluate(&value,100.1,0.,0.,0.,0.,.01,1);
  assert(!mismatch.heading_consistent&&!boundary_admission::decide(mismatch,context(true)).defer);
  mismatch=boundary_admission::evaluate(&value,100.1,0.,0.,0.,0.,0.,1);
  assert(!mismatch.heading_consistent&&!boundary_admission::decide(mismatch,context(true)).defer);
}
void testInvalid() {
  auto value=frame(.25);const double nan=std::numeric_limits<double>::quiet_NaN();
  for(const auto& result:{boundary_admission::evaluate(nullptr,100.1,0.,0.,0.,.01,0.,1),
      boundary_admission::evaluate(&value,101.,0.,0.,0.,.01,0.,1),
      boundary_admission::evaluate(&value,99.,0.,0.,0.,.01,0.,1),
      boundary_admission::evaluate(&value,100.1,nan,0.,0.,.01,0.,1),
      boundary_admission::evaluate(&value,100.1,0.,0.,nan,.01,0.,1),
      boundary_admission::evaluate(&value,100.1,0.,0.,0.,nan,0.,1),
      boundary_admission::evaluate(&value,100.1,1000.,0.,0.,.01,0.,1),
      boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,2)}) {
    assert(!result.valid&&result.category==boundary_admission::Category::invalid);
    assert(!boundary_admission::decide(result,context(true)).defer);
  }
  value.healthy=false;
  assert(!boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1).valid);
  value=frame(.25);value.field[3*8+3]=nan;
  assert(!boundary_admission::evaluate(&value,100.1,0.,0.,0.,.01,0.,1).valid);
}
}
int main() {
  testStrictCategories();testSameNativeGapTolerance();testEligibility();testHeading();testInvalid();testObservedActualFixedHead();
  std::cout<<"BOUNDARY_ADMISSION_NUMERICAL_PASSED (no ROS)\n";
}
