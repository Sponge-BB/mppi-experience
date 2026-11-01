#pragma once
// Experiment-only routing state. A pending point is not a certified trajectory.
#include "feasible_quality_polish.hpp"
#include <exception>

namespace deferred_quality {
constexpr double reserve_s=.15;
inline bool requested(){const char* p=std::getenv("DEFERRED_QUALITY_CHECK");return p&&std::string(p)=="1";}
struct ReserveStop:std::exception {
  const char* what()const noexcept override{return "reserved final feasibility check";}
};
struct Pending {
  bool available=false;
  Eigen::VectorXd x;
  double cost=std::numeric_limits<double>::infinity();
  feasible_quality_polish::Snapshot snapshot;
  int iteration=0;
  bool consider(const Eigen::VectorXd& point,double value,
                const feasible_quality_polish::Snapshot& objective,int at){
    if(!point.allFinite()||!std::isfinite(value)||!objective.valid||
       objective.x_hash!=delivery_candidate::hash(point)||objective.total!=value||
       (available&&value>=cost))return false;
    x=point;cost=value;snapshot=objective;iteration=at;available=true;return true;
  }
};
} // namespace deferred_quality
