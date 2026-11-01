#pragma once
// Yield an actual accepted iterate to the existing one-shot native repair.
// This event is not a feasible-candidate certificate or a control permission.
#include "early_delivery_gate.hpp"
#include "candidate_collision_repair.hpp"
#include "window_continuity.hpp"

namespace early_collision_repair {
inline bool requested(){const char* p=std::getenv("EARLY_COLLISION_REPAIR");return p&&std::string(p)=="1";}
struct Session;
inline Session*& current(){static thread_local Session* p=nullptr;return p;}
struct Session {
  Session* previous;bool enabled=requested(),yielded=false,repairing=false;
  Eigen::VectorXd x;early_delivery::Check trigger;
  Session():previous(current()){current()=this;}
  ~Session(){current()=previous;}
  Session(const Session&)=delete;Session& operator=(const Session&)=delete;
};
inline bool shouldYield(const Eigen::VectorXd& x,int iteration){
  auto* s=current();const auto* native=early_delivery::active();
  const auto* repair=candidate_collision_repair::current();
  if(!s||!s->enabled||s->yielded||s->repairing||!repair||!repair->enabled||
     repair->repairing||!window_continuity::enabled()||!native||native->selected||native->checks.empty())return false;
  const auto& check=native->checks.back();
  if(check.accepted_iteration!=iteration||check.candidate_hash!=delivery_candidate::hash(x)||
     !check.wheel_pass||!check.native_checked||check.native_pass||check.selected||check.budget_expired||
     check.native_reason!="planning_collision"||check.collision_type<0||check.collision_type>1)return false;
  s->yielded=true;s->x=x;s->trigger=check;return true;
}
} // namespace early_collision_repair
