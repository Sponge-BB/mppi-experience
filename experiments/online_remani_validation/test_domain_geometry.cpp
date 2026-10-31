// Pure projection-domain reason tests, no ROS graph or robot commands.
#include "coupling_collision.hpp"
#include "domain_step_rejection.hpp"
#include <cassert>
#include <iostream>
#include <limits>

using coupling_collision::Snapshot;
using Failure=coupling_collision::Evaluation::Failure;

static Snapshot fixture() {
  Snapshot m;
  m.gx=-1.;m.gy=-1.;m.res=.1;m.nx=21;m.ny=21;
  m.body={{0.,0.,.1},{.25,.08,.12}};
  m.field.assign(21*21,1.);
  return m;
}
int main() {
  auto m=fixture();
  coupling_collision::Options stat;stat.include_dynamic=false;
  auto at=[&](double x,double y,double yaw=0.) {
    return coupling_collision::evaluate(m,x,y,yaw,0.,stat);
  };
  assert(at(0.,0.).valid);
  for (double yaw:{0.,.31,1.5707963267948966,3.141592653589793}) {
    for (auto p:std::vector<std::pair<double,double>>{{-1.1,0.},{1.1,0.},{0.,-1.1},
        {0.,1.1},{-1.1,-1.1},{1.1,1.1}}) {
      const auto r=at(p.first,p.second,yaw);
      assert(!r.valid && r.failure==Failure::static_map_domain);
      assert(r.gap<0.); // remains UNKNOWN/unsafe to hard qualification
    }
  }
  // Center inside but one noncircular outer body disk leaves the domain.
  assert(!at(.8,0.).valid && at(.8,0.).failure==Failure::static_map_domain);
  assert(!at(1.,0.).valid); // upper boundary has no bilinear cell
  m.body.resize(1);
  assert(at(-1.,-1.).valid); // lower node retains a complete cell
  assert(at(1.-1e-9,0.).valid);
  assert(at(NAN,0.).failure==Failure::malformed);
  m.body.push_back({0.,0.,NAN});
  assert(at(20.,0.).failure==Failure::malformed);
  m=fixture();m.field.pop_back();
  assert(at(20.,0.).failure==Failure::malformed);
  m=fixture();m.res=std::numeric_limits<double>::min();
  assert(at(1e200,0.).failure==Failure::malformed); // arithmetic overflow, not recoverable

  // Exact recorded request-2 finite query; use the recorded offending disk.
  const double px=.3950292304431109,py=5.03130554094473;
  domain_step::RejectedTrial recorded(px,py,-7.5,-5.,7.45,4.95,1);
  m=fixture();m.gx=-7.5;m.gy=-5.;m.res=.05;m.nx=300;m.ny=200;
  m.body={{0.,0.,.1}};m.field.assign(size_t(m.nx)*m.ny,1.);
  const auto outside=coupling_collision::evaluate(m,px,py,0.,0.,stat);
  assert(!outside.valid && outside.failure==Failure::static_map_domain);
  std::cout<<"DOMAIN_GEOMETRY_PASSED finite recorded point; edges/corners/outer disk; "
             "NaN/malformed/overflow remain fatal; outside hard query stays unsafe\n";
}
