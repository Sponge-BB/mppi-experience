#include "coupling_collision.hpp"
#include <array>
#include <cassert>
#include <iostream>
#include <random>

using coupling_collision::Evaluation;
using coupling_collision::Options;
using coupling_collision::Snapshot;

static Snapshot fixture() {
  Snapshot m;
  m.gx = -3.; m.gy = -3.; m.res = .1; m.nx = 61; m.ny = 61;
  m.margin = .1; m.comfort = .32; m.weight = 17.; m.max_prediction = 3.;
  m.body = {{.31, .12, .18}, {-.27, -.08, .21}, {.04, .25, .13}};
  for (int iy = 0; iy < m.ny; ++iy) for (int ix = 0; ix < m.nx; ++ix) {
    const double x = m.gx+ix*m.res, y = m.gy+iy*m.res;
    m.field.push_back(.36+.02*x+.025*y+.005*x*y);
  }
  m.obstacles = {{.52, .12, -.035, .023, .13, 1.},
                 {-.56, -.13, .02, -.01, .12, 1.}};
  return m;
}
static void near(double a, double b, double tolerance = 1e-6) {
  assert(std::isfinite(a) && std::isfinite(b));
  if (std::abs(a-b) > tolerance*(1.+std::abs(a)+std::abs(b))) {
    std::cerr << "gradient/value mismatch: " << a << " vs " << b << '\n';
    std::abort();
  }
}
static Evaluation at(const Snapshot& m, std::array<double,4> q, Options o) {
  auto result = coupling_collision::evaluate(m,q[0],q[1],q[2],q[3],o);
  assert(result.valid);
  return result;
}
static void gradients(const Snapshot& m, std::array<double,4> q, Options o) {
  const auto value = at(m,q,o);
  const double analytic[] = {value.gx,value.gy,value.gyaw,value.gt};
  for (int i = 0; i < 4; ++i) {
    auto plus=q, minus=q;
    plus[i]+=1e-6; minus[i]-=1e-6;
    const double numeric=(at(m,plus,o).cost-at(m,minus,o).cost)/2e-6;
    near(analytic[i],numeric,2e-6);
  }
}
int main() {
  auto m=fixture();
  Options stat;stat.include_dynamic=false;
  Options dyn;dyn.include_static=false;dyn.dynamic_soft_taper_seconds=0.;
  Options both;both.dynamic_soft_taper_seconds=0.;
  const std::array<double,4> pose={.017,-.033,.23,.87};
  gradients(m,pose,stat);gradients(m,pose,dyn);gradients(m,pose,both);
  const auto a=at(m,pose,stat),b=at(m,pose,dyn),c=at(m,pose,both);
  near(c.cost,a.cost+b.cost);near(c.gx,a.gx+b.gx);near(c.gt,b.gt);
  near(a.gt,0.);
  std::mt19937 generator(7919);
  std::uniform_real_distribution<double> xy(-.12,.12),angle(-.8,.8),time(.1,2.4);
  for (int sample=0;sample<80;++sample) {
    const std::array<double,4> q={xy(generator),xy(generator),angle(generator),time(generator)};
    gradients(m,q,stat);gradients(m,q,dyn);gradients(m,q,both);
  }
  // All body disks contribute, not only a base point/circle. Cost and every
  // derivative add, while physical/gap are minima over the full projection.
  double sum=0.,sum_x=0.,sum_y=0.,sum_yaw=0.,sum_t=0.,physical=1e9;
  for (const auto& body:m.body) {
    auto one=m;one.body={body};const auto e=at(one,pose,both);
    sum+=e.cost;sum_x+=e.gx;sum_y+=e.gy;sum_yaw+=e.gyaw;sum_t+=e.gt;
    physical=std::min(physical,e.physical);
  }
  near(c.cost,sum);near(c.gx,sum_x);near(c.gy,sum_y);
  near(c.gyaw,sum_yaw);near(c.gt,sum_t);near(c.physical,physical);
  near(c.gap,c.physical-m.margin);
  // A distant base-origin disk would miss this unsafe projected arm disk.
  auto outer=m;outer.body={{0.,0.,.05},{1.2,0.,.2}};
  outer.obstacles={{1.2,.25,0.,0.,.1,0.}};
  auto full=at(outer,{0.,0.,0.,0.},dyn);assert(full.gap<0.);
  outer.body.resize(1);auto point=at(outer,{0.,0.,0.,0.},dyn);assert(point.gap>0.);
  // Explicit forecast derivative agrees with opposite obstacle translation.
  auto translated=m;const double dt=1e-5;
  for (auto& obstacle:translated.obstacles) {
    obstacle.x+=dt*obstacle.vx*obstacle.moving;
    obstacle.y+=dt*obstacle.vy*obstacle.moving;
  }
  auto later=pose;later[3]+=dt;
  near(at(translated,pose,dyn).cost,at(m,later,dyn).cost,1e-11);
  // Dynamic flag zero prevents false motion even if stored velocity is nonzero.
  auto stopped=m;for(auto& obstacle:stopped.obstacles)obstacle.moving=0.;
  near(at(stopped,pose,dyn).gt,0.);gradients(stopped,pose,dyn);
  // Smooth temporal turn-off includes derivative of the envelope, but never
  // scales the geometry/safety gap or turns an unknown future into free space.
  Options tapered=dyn;tapered.dynamic_soft_taper_seconds=.5;
  gradients(m,{pose[0],pose[1],pose[2],2.77},tapered);
  const auto raw=at(m,{pose[0],pose[1],pose[2],2.77},dyn);
  const auto soft=at(m,{pose[0],pose[1],pose[2],2.77},tapered);
  assert(soft.cost<raw.cost);near(soft.gap,raw.gap);near(soft.physical,raw.physical);
  auto end=at(m,{pose[0],pose[1],pose[2],3.},tapered);
  near(end.cost,0.);near(end.gt,0.);near(end.gx,0.);near(end.gyaw,0.);
  auto near_end=at(m,{pose[0],pose[1],pose[2],3.-1e-5},tapered);
  assert(std::abs(near_end.gt)<1e-6 && std::abs(near_end.cost)<1e-9);
  auto left=at(m,{pose[0],pose[1],pose[2],2.5-1e-7},tapered);
  auto right=at(m,{pose[0],pose[1],pose[2],2.5+1e-7},tapered);
  near(left.gt,right.gt,1e-5);
  assert(!coupling_collision::evaluate(m,0.,0.,0.,3.000001,tapered).valid);
  assert(!coupling_collision::evaluate(m,0.,0.,0.,-.000001,tapered).valid);
  assert(!coupling_collision::evaluate(m,NAN,0.,0.,0.,both).valid);
  assert(!coupling_collision::evaluate(m,1e100,0.,0.,0.,stat).valid);
  assert(!coupling_collision::evaluate(m,3.,0.,0.,0.,stat).valid);
  auto malformed=m;malformed.field.pop_back();
  assert(!coupling_collision::evaluate(malformed,0.,0.,0.,0.,stat).valid);
  malformed=m;malformed.body[0].r=-.1;
  assert(!coupling_collision::evaluate(malformed,0.,0.,0.,0.,stat).valid);
  auto static_only=m;static_only.obstacles.clear();
  assert(coupling_collision::evaluate(static_only,0.,0.,0.,30.,both).valid);
  static_assert(coupling_collision::soft_motion_reserve==0.,"no hidden radius");
  std::cout << "COUPLING_COLLISION_CHECKS_PASSED static/dynamic pose,yaw,time; "
               "C2 taper; full projection; invalid boundaries\n";
}
