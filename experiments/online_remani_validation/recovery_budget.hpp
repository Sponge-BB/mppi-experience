#pragma once
// Pure budget policy. All phases share the ORIGINAL optimization start/deadline.
#include <cmath>
#include <stdexcept>

namespace recovery_budget {
class Window {
  double start_,duration_,reserve_,minimum_;
  bool claimed_=false;
public:
  Window(double start,double duration,double reserve=.04,double minimum=.02)
      :start_(start),duration_(duration),reserve_(reserve),minimum_(minimum) {
    if(!std::isfinite(start)||!std::isfinite(duration)||duration<=0. ||
       !std::isfinite(reserve)||reserve<0. ||!std::isfinite(minimum)||minimum<=0. ||
       !std::isfinite(start+duration))throw std::invalid_argument("invalid recovery deadline");
  }
  double elapsed(double now) const {return now-start_;}
  double remaining(double now) const {return duration_-elapsed(now);}
  bool expired(double now) const {return !std::isfinite(now)||now<start_||remaining(now)<=0.;}
  bool recoveryExpired(double now) const {return expired(now)||remaining(now)<=reserve_;}
  bool canStart(double now) const {return !claimed_&&!expired(now)&&remaining(now)>=reserve_+minimum_;}
  bool claim(double now) {if(!canStart(now))return false;claimed_=true;return true;}
  bool claimed() const {return claimed_;}
  double duration() const {return duration_;}
  double reserve() const {return reserve_;}
};
}
