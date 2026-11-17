#pragma once
// Isolated slow-planner experiment. Extend only the frozen planning copy;
// the source buffer and the latest execution checker retain their own limits.
#include <cstdlib>
#include <string>

namespace slow_delivery_window {
constexpr double configured_allowance_s=1.5;
constexpr double optimizer_budget_reference_s=1.2;

inline bool requested(){
  const char* value=std::getenv("SLOW_DELIVERY_COVERAGE");
  return value&&std::string(value)=="1";
}

struct Choice {
  bool enabled=false;
  double original_max_s=0.,planning_max_s=0.,allowance_s=0.;
};

inline Choice choose(bool on,unsigned long long request_id,bool full_coverage,
                     double original_max_s){
  Choice out;out.enabled=on&&request_id>1&&full_coverage;
  out.original_max_s=original_max_s;
  out.allowance_s=out.enabled?configured_allowance_s:0.;
  out.planning_max_s=original_max_s+out.allowance_s;
  return out;
}
} // namespace slow_delivery_window
