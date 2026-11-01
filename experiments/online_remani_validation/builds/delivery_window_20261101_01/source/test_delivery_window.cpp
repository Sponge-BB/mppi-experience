#include "delivery_window.hpp"
#include "window_continuity.hpp"
#include <cmath>
#include <iostream>
#include <map>

// Exercise the actual prediction branch and the soft integration boundaries.
// No map or robot safety predicate is replaced or asserted by this unit test.
int main(){
  namespace dw=slow_delivery_window;
  namespace tc=time_indexed_collision;
  std::map<std::string,bool> tests;
  const auto off=dw::choose(false,2,true,3.);
  const auto initial=dw::choose(true,1,true,3.);
  const auto legacy=dw::choose(true,2,false,3.);
  const auto plan=dw::choose(true,2,true,3.);
  tests["disabled_unchanged"]=!off.enabled&&off.planning_max_s==3.&&off.allowance_s==0.;
  tests["initial_request_unchanged"]=!initial.enabled&&initial.planning_max_s==3.;
  tests["legacy_window_unchanged"]=!legacy.enabled&&legacy.planning_max_s==3.;
  tests["replan_frozen_copy_extended"]=plan.enabled&&plan.original_max_s==3.&&
    plan.planning_max_s==4.5&&plan.allowance_s==1.5;

  tc::Context source;source.active=true;source.full_coverage=true;
  source.requested_epoch=100.9;source.begin_epoch=100.;source.end_epoch=103.;
  source.source.snapshot.stamp.fromSec(100.);source.source.snapshot.sequence=42;
  source.source.refresh_epoch=100.8;source.source.parameters.max_prediction=3.;
  source.source.parameters.uncertainty0=.02;
  source.source.parameters.uncertainty_v=.003;
  source.source.parameters.uncertainty_a=.001;
  plan_env::DynamicPoint point;point.position={1.4,-.2,.5};point.velocity={-.18,.03,0.};
  source.source.snapshot.points.push_back(point);
  auto extended=source;extended.source.parameters.max_prediction=plan.planning_max_s;
  extended.end_epoch=extended.source.snapshot.stamp.toSec()+plan.planning_max_s;
  const double elapsed=2.385;
  const auto old_branch=tc::branch(source,elapsed,1.);
  const auto new_branch=tc::branch(extended,elapsed,1.);
  const auto half_branch=tc::branch(extended,elapsed/2.,2.);
  const Eigen::Vector3d expected=point.position+3.285*point.velocity;
  tests["previously_unknown_3_285_now_covered"]=!old_branch.valid&&new_branch.valid&&
    std::abs(new_branch.dt-3.285)<1e-12;
  tests["cv_state_and_formula_unchanged"]=new_branch.valid&&
    (new_branch.points[0]-expected).norm()<1e-12&&
    std::abs(new_branch.radius-(.02+.003*new_branch.dt+.5*.001*new_branch.dt*new_branch.dt))<1e-12;
  tests["nominal_half_share_planning_source"]=half_branch.valid&&
    (half_branch.points[0]-new_branch.points[0]).norm()<1e-12&&
    half_branch.dt==new_branch.dt;
  tests["source_and_requested_epochs_unchanged"]=source.source.parameters.max_prediction==3.&&
    extended.source.snapshot.stamp.toSec()==source.source.snapshot.stamp.toSec()&&
    extended.source.snapshot.sequence==42&&extended.source.refresh_epoch==100.8&&
    extended.requested_epoch==source.requested_epoch&&
    (extended.source.snapshot.points[0].position-point.position).norm()==0.&&
    (extended.source.snapshot.points[0].velocity-point.velocity).norm()==0.;
  const double end_elapsed=104.5-extended.requested_epoch;
  const auto endpoint=tc::branch(extended,end_elapsed,1.);
  const auto tolerance=tc::branch(extended,end_elapsed+.5e-6,1.);
  const auto beyond=tc::branch(extended,end_elapsed+2e-6,1.);
  tests["closed_edge_and_original_tolerance"]=endpoint.valid&&tolerance.valid&&
    std::abs(endpoint.dt-4.5)<1e-12&&tolerance.dt==4.5&&!beyond.valid;
  const double before_elapsed=100.-extended.requested_epoch-2e-6;
  tests["history_limit_unchanged"]=!tc::branch(extended,before_elapsed,1.).valid;
  const auto edges=window_continuity::boundaries(extended);
  tests["soft_boundary_matches_hard_branch"]=edges.size()==3&&
    std::abs(edges[1]-1.8)<1e-12&&std::abs(edges[2]-3.6)<1e-12&&
    tc::branch(extended,edges[1],2.).valid&&tc::branch(extended,edges[2],1.).valid;

  bool pass=true;for(const auto& entry:tests)pass=pass&&entry.second;
  std::cout<<"{\"pass\":"<<(pass?"true":"false");
  for(const auto& entry:tests)
    std::cout<<",\""<<entry.first<<"\":"<<(entry.second?"true":"false");
  std::cout<<",\"scope\":\"actual CV branch and soft interval boundaries; no map-safety claim\"}\n";
  return pass?0:1;
}
