#include "coverage_collision.hpp"
#include <iostream>

// Policy defaults plus an independent check that the real soft-distance
// operation still sees both branches; no simulator or ROS map worker runs.
int main(){
  using namespace slow_reference_contract;
  setenv("SLOW_REFERENCE_NOMINAL_ONLY","0",1);
  const bool old=includeHardBranch(0)&&includeHardBranch(1);
  setenv("SLOW_REFERENCE_NOMINAL_ONLY","1",1);
  const bool changed=includeHardBranch(0)&&!includeHardBranch(1);
  // Actual soft-distance helper must still consider the half branch.
  time_indexed_collision::Context context;context.full_coverage=true;
  context.source.parameters.influence=2.;
  plan_env::DynamicPoint point;point.velocity=Eigen::Vector3d::Zero();
  context.source.snapshot.points.push_back(point);
  time_indexed_collision::SampleState sample;
  sample.branches[0].valid=true;sample.branches[0].points.push_back({1.,0.,0.});
  sample.branches[1].valid=true;sample.branches[1].points.push_back({.01,0.,0.});
  double distance=2.;Eigen::Vector3d gradient;
  time_indexed_collision::indexedDistance(context,sample,Eigen::Vector3d::Zero(),distance,gradient);
  const bool soft=std::abs(distance-.01)<1e-12;
  unsetenv("SLOW_REFERENCE_NOMINAL_ONLY");
  const bool default_old=includeHardBranch(0)&&includeHardBranch(1);
  const bool pass=old&&changed&&soft&&default_old;
  std::cout<<"{\"pass\":"<<(pass?"true":"false")
    <<",\"both_hard_default\":"<<(default_old?"true":"false")
    <<",\"half_still_in_soft_distance\":"<<(soft?"true":"false")<<"}\n";
  return pass?0:1;
}
