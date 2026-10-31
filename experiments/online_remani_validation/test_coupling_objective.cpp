#include "coupling_objective.hpp"
#include <cassert>
#include <functional>
#include <iostream>
#include <limits>

using namespace coupling_objective;

namespace {
Sample sample(double x, double y, double vx, double vy, int gear = 1) {
  Sample s;
  s.pos = Vec2(x, y); s.vel = Vec2(vx, vy); s.gear = gear;
  return s;
}
Target target(double x, double y, double yaw, double speed, double time, int gear = 1) {
  Target t;
  t.pos = Vec2(x, y); t.yaw = yaw; t.speed = speed; t.time = time; t.gear = gear;
  return t;
}
FrozenTargets fixture() {
  FrozenTargets t;
  t.checked_begin = 0.; t.checked_end = 2.;
  t.execution = {target(.15, .27, .2, .18, .3), target(.55, .32, .15, .18, .8)};
  t.has_delivery = true; t.delivery = target(.31, .22, .1, .16, .6);
  return t;
}
void near(double a, double b, double tolerance = 2e-5) {
  assert(std::abs(a-b) <= tolerance * (1. + std::abs(a) + std::abs(b)));
}
void rejects(const std::function<void()>& f) {
  bool bad = false;
  try { f(); } catch (const std::invalid_argument&) { bad = true; }
  assert(bad);
}
void gradient(const std::vector<Sample>& input, const FrozenTargets& t, const Settings& c) {
  const auto analytic = evaluate(input, t, c);
  const double epsilon = 1e-6;
  for (std::size_t i=0; i<input.size(); ++i) for (int kind=0; kind<2; ++kind)
    for (int d=0; d<2; ++d) {
      auto a=input,b=input;
      (kind ? a[i].vel : a[i].pos)(d) += epsilon;
      (kind ? b[i].vel : b[i].pos)(d) -= epsilon;
      const double finite = (evaluate(a,t,c).cost-evaluate(b,t,c).cost)/(2*epsilon);
      near(finite, (kind ? analytic.grad_v : analytic.grad_p)[i](d));
    }
}
}  // namespace

int main() {
  std::vector<Sample> s={sample(0.,0.,.1,.02),sample(.35,.02,.15,-.01),sample(.7,.03,.12,.01)};
  FrozenTargets t=fixture(); Settings c; c.tube_radius=.1;
  const auto both=evaluate(s,t,c);
  assert(both.cost>0. && both.execution_cost>0. && both.delivery_cost>0.);
  assert(both.execution_targets==2 && both.delivery_targets==1);
  Settings exec=c;exec.delivery_enabled=false;
  Settings hand=c;hand.execution_enabled=false;
  const auto a=evaluate(s,t,exec),b=evaluate(s,t,hand);
  near(both.cost,a.cost+b.cost,1e-12);
  for(std::size_t i=0;i<s.size();++i){
    assert((both.grad_p[i]-a.grad_p[i]-b.grad_p[i]).norm()<1e-10);
    assert((both.grad_v[i]-a.grad_v[i]-b.grad_v[i]).norm()<1e-10);
  }
  gradient(s,t,c);gradient(s,t,exec);gradient(s,t,hand);

  // Exact identity when both weights are zero, even without prediction input.
  Settings zero=c;zero.execution_weight=zero.delivery_weight=0.;
  auto z=evaluate(s,FrozenTargets(),zero);
  assert(z.cost==0. && z.execution_targets==0 && z.delivery_targets==0);
  for(std::size_t i=0;i<s.size();++i)assert(z.grad_p[i].isZero()&&z.grad_v[i].isZero());
  zero=c;zero.execution_enabled=zero.delivery_enabled=false;
  assert(evaluate({},FrozenTargets(),zero).cost==0.);

  // Execution tube dead zone is an actual zero objective with zero gradient.
  Settings dead=exec;dead.tube_radius=100.;
  z=evaluate(s,t,dead);assert(z.cost==0.);
  for(std::size_t i=0;i<s.size();++i)assert(z.grad_p[i].isZero()&&z.grad_v[i].isZero());

  // Smooth bounded low-speed heading derivative, including precisely zero.
  auto slow=s;slow[0].vel.setZero();slow[1].vel=Vec2(1e-10,-1e-10);
  gradient(slow,t,c);
  auto stopped=t;
  for(auto& v:stopped.execution)v.speed=0.;
  stopped.delivery.speed=0.;
  gradient(slow,stopped,c);

  // Reverse motion: body yaw zero corresponds to world velocity (-v,0).
  auto reverse=s;
  for(auto& v:reverse){v.pos.x()=-v.pos.x();v.vel.x()=-v.vel.x();v.gear=-1;}
  auto rt=t;
  for(auto& v:rt.execution){v.pos.x()=-v.pos.x();v.speed=-v.speed;v.gear=-1;}
  rt.delivery.pos.x()=-rt.delivery.pos.x();rt.delivery.speed=-rt.delivery.speed;rt.delivery.gear=-1;
  gradient(reverse,rt,c);
  // Opposite-gear near points must never attract this objective.
  auto mixed=s;mixed.push_back(sample(t.delivery.pos.x(),t.delivery.pos.y(),-.16,0.,-1));
  auto same=evaluate(mixed,t,c);
  near(same.cost,both.cost,1e-12);assert(same.grad_p.back().isZero()&&same.grad_v.back().isZero());

  // Duplicate same-gear samples preserve normalized soft-min cost.
  auto repeated=s;repeated.insert(repeated.end(),s.begin(),s.end());
  near(evaluate(repeated,t,c).cost,both.cost,1e-12);
  gradient(repeated,t,c);
  // Widely separated costs and tiny temperature cannot underflow all terms.
  Settings tiny=c;tiny.temperature=1e-10;
  auto extreme=s;extreme[2].pos=Vec2(1e6,-1e6);extreme[2].vel=Vec2(1e4,-1e4);
  assert(std::isfinite(evaluate(extreme,t,tiny).cost));
  auto far=t;
  for(auto& v:far.execution)v.pos=Vec2(-1e6,1e6);
  far.delivery.pos=Vec2(-1e6,1e6);
  assert(std::isfinite(evaluate(s,far,tiny).cost));

  // Strict validation includes unavailable targets, metadata, finite values,
  // signed gear, sorted source time and the checked prediction boundary.
  rejects([&]{evaluate({},t,c);});
  rejects([&]{evaluate(s,FrozenTargets(),c);});
  auto invalid=t;invalid.delivery.time=2.000001;rejects([&]{evaluate(s,invalid,c);});
  invalid=t;invalid.execution[0].time=-1e-9;rejects([&]{evaluate(s,invalid,c);});
  invalid=t;invalid.execution[1].time=invalid.execution[0].time;
  rejects([&]{evaluate(s,invalid,c);});
  invalid=t;invalid.delivery.gear=-1;rejects([&]{evaluate(s,invalid,c);});
  invalid=t;invalid.delivery.speed=-.1;rejects([&]{evaluate(s,invalid,c);});
  invalid=t;invalid.checked_end=-1.;rejects([&]{evaluate(s,invalid,c);});
  invalid=t;invalid.delivery.weight=0.;rejects([&]{evaluate(s,invalid,c);});
  invalid=t;invalid.delivery.pos.x()=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{evaluate(s,invalid,c);});
  auto bad=s;bad[0].u=1.001;rejects([&]{evaluate(bad,t,c);});
  bad=s;bad[0].T=0.;rejects([&]{evaluate(bad,t,c);});
  bad=s;bad[0].gear=0;rejects([&]{evaluate(bad,t,c);});
  bad=s;bad[0].acc.x()=std::numeric_limits<double>::infinity();
  rejects([&]{evaluate(bad,t,c);});
  Settings invalid_setting=c;invalid_setting.temperature=0.;
  rejects([&]{evaluate(s,t,invalid_setting);});
  invalid_setting=c;invalid_setting.execution_weight=-1.;
  rejects([&]{evaluate(s,t,invalid_setting);});
  invalid_setting=c;invalid_setting.position_scale=1e200;
  rejects([&]{evaluate(s,t,invalid_setting);});
  invalid_setting=c;invalid_setting.velocity_epsilon=1e-200;
  rejects([&]{evaluate(s,t,invalid_setting);});
  invalid_setting=c;invalid_setting.tube_radius=1e200;
  rejects([&]{evaluate(s,t,invalid_setting);});
  rejects([&]{evaluate(reverse,t,c);});
  // A disabled objective cannot force the other objective's target to exist.
  invalid=t;invalid.execution.clear();assert(evaluate(s,invalid,hand).cost>0.);
  invalid=t;invalid.has_delivery=false;assert(evaluate(s,invalid,exec).cost>0.);
  std::cout<<"COUPLING_OBJECTIVE_CHECKS_PASSED\n";
}
