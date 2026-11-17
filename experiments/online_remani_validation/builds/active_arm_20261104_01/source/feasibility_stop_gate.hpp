#pragma once
// Diagnostic-only veto of the L-BFGS past/delta stop. This header changes no
// objective, direction, history, deadline, boundary, or physical safety gate.
#include <Eigen/Dense>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace feasibility_stop_gate {
struct Check {
  bool native_known=false,native_pass=false;
  bool projected_known=false,projected_pass=false;
  bool budget_expired=false;
  double native_wall_s=0.,projected_wall_s=0.,total_wall_s=0.;
  std::string reason="not_checked";
  bool allowed() const {
    return native_known && native_pass && projected_known && projected_pass && !budget_expired;
  }
};
struct Statistics {
  bool enabled=false,fatal=false;
  std::size_t checks=0,blocked=0,allowed=0,native_pass=0,projected_pass=0,budget_expired=0;
  double native_wall_s=0.,projected_wall_s=0.,total_wall_s=0.;
  std::string fatal_reason;
  Check last;
};
inline std::uint64_t vectorHash(const Eigen::VectorXd& x) {
  std::uint64_t h=1469598103934665603ULL;
  const auto* bytes=reinterpret_cast<const unsigned char*>(x.data());
  for(std::size_t i=0;i<std::size_t(x.size())*sizeof(double);++i){h^=bytes[i];h*=1099511628211ULL;}
  return h;
}
struct Session;
inline Session*& current(){static thread_local Session* value=nullptr;return value;}
struct Session {
  using Predicate=std::function<Check(const Eigen::VectorXd&)>;
  using Reporter=std::function<void(const Statistics&)>;
  Session* previous;
  Predicate predicate;
  Reporter reporter;
  Statistics stats;
  Eigen::VectorXd last_evaluated;
  std::uint64_t last_hash=0;
  bool evaluated=false;
  Session(bool enabled,Predicate check,Reporter report):previous(current()),
      predicate(std::move(check)),reporter(std::move(report)) {
    stats.enabled=enabled;
    if(enabled){if(!predicate)throw std::invalid_argument("empty feasibility stop predicate");current()=this;}
  }
  Session(const Session&)=delete;Session& operator=(const Session&)=delete;
  ~Session() noexcept {
    if(stats.enabled)current()=previous;
    try{if(reporter)reporter(stats);}catch(...){/* Missing summaries fail the outer diagnostic audit. */}
  }
};
inline void recordEvaluated(const Eigen::VectorXd& x) {
  auto* s=current();if(!s)return;
  if(!x.allFinite())throw std::runtime_error("nonfinite successful callback cache");
  s->last_evaluated=x;s->last_hash=vectorHash(x);s->evaluated=true;
}
inline bool allowCostStop(const Eigen::VectorXd& x) {
  auto* s=current();if(!s)return true;
  ++s->stats.checks;
  try {
    // Hash AND exact bytes: a historical successful trial, even with the same
    // vector length, cannot certify the current accepted x's mutable caches.
    if(!s->evaluated || x.size()!=s->last_evaluated.size() || !x.allFinite() ||
       vectorHash(x)!=s->last_hash ||
       std::memcmp(x.data(),s->last_evaluated.data(),std::size_t(x.size())*sizeof(double))!=0)
      throw std::runtime_error("feasibility stop accepted-x/cache mismatch");
    s->stats.last=s->predicate(x);
    const auto& c=s->stats.last;
    s->stats.native_pass+=c.native_known&&c.native_pass;
    s->stats.projected_pass+=c.projected_known&&c.projected_pass;
    s->stats.budget_expired+=c.budget_expired;
    s->stats.native_wall_s+=c.native_wall_s;
    s->stats.projected_wall_s+=c.projected_wall_s;
    s->stats.total_wall_s+=c.total_wall_s;
    if(c.allowed()){++s->stats.allowed;return true;}
    ++s->stats.blocked;return false;
  }catch(const std::exception& error){
    s->stats.fatal=true;s->stats.fatal_reason=error.what();throw;
  }catch(...){
    s->stats.fatal=true;s->stats.fatal_reason="unknown feasibility gate exception";throw;
  }
}
inline bool abortRequested() {
  const auto* s=current();return s && s->stats.last.budget_expired;
}
} // namespace feasibility_stop_gate
