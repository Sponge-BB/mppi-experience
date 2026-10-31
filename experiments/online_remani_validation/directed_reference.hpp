#pragma once
// Measured-only native direction cursor. No controller, ROS or collision model
// dependencies: predicted states must never be passed to update().
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace directed_reference {
struct Sample { double x,y,yaw,signed_speed; };
struct Measurement { double x,y,v; };
struct Phase { int begin,end,gear; }; // Inclusive vertex indices in FULL native ref.
struct Update {
  bool started=false,switched=false;
  int previous_phase=-1;
  double checked_cusp_distance=0.;
};

class Cursor {
 public:
  static constexpr double cusp_position_tolerance=.05;
  static constexpr double cusp_speed_tolerance=.025;

  void reset() {
    phases_.clear();phase_=0;hash_=0;transitions_=0;last_event_=0;
    previous_phase_=-1;cusp_distance_=0.;
  }
  bool ready() const { return !phases_.empty(); }
  int phase_id() const { return ready()?phase_:-1; }
  std::size_t phase_count() const { return phases_.size(); }
  const Phase& active() const {
    if(!ready())throw std::logic_error("unbound directed reference cursor");
    return phases_[phase_];
  }
  double cusp_distance() const { return cusp_distance_; }
  unsigned transitions() const { return transitions_; }
  std::uint64_t reference_hash() const { return hash_; }

  Update update(const std::vector<Sample>&reference,const Measurement&measured) {
    if(reference.size()<2||reference.size()>1000000||
       !std::isfinite(measured.x)||!std::isfinite(measured.y)||!std::isfinite(measured.v))
      throw std::invalid_argument("invalid directed reference measurement/size");
    const std::uint64_t incoming=fingerprint(reference);
    Update event;last_event_=0;previous_phase_=-1;
    if(!ready()||incoming!=hash_) {
      phases_=partition(reference);phase_=0;hash_=incoming;transitions_=0;
      event.started=true;last_event_=1;
      cusp_distance_=distance(reference[active().end],measured);
      event.checked_cusp_distance=cusp_distance_;
      // Bind first phase before permitting a transition on a later real step.
      return event;
    }
    cusp_distance_=distance(reference[active().end],measured);
    event.checked_cusp_distance=cusp_distance_;
    if(phase_+1<int(phases_.size())&&cusp_distance_<=cusp_position_tolerance&&
       std::abs(measured.v)<=cusp_speed_tolerance) {
      event.previous_phase=phase_;previous_phase_=phase_;++phase_;++transitions_;
      event.switched=true;last_event_=2;
      // Deliberately one transition per actual update; never inspect forecasts.
      cusp_distance_=distance(reference[active().end],measured);
    }
    return event;
  }

  // 12 doubles: ready, phase, count, begin, end, gear, current cusp distance,
  // transition count, event(1=start,2=switch), previous phase, FNV hi32, lo32.
  // The FNV identity is telemetry, not a cryptographic publication digest.
  void debug(double*out) const {
    if(!out)return;
    out[0]=ready()?1.:0.;out[1]=phase_id();out[2]=phases_.size();
    out[3]=ready()?active().begin:-1.;out[4]=ready()?active().end:-1.;
    out[5]=ready()?active().gear:0.;out[6]=cusp_distance_;out[7]=transitions_;
    out[8]=last_event_;out[9]=previous_phase_;
    out[10]=double(std::uint32_t(hash_>>32));out[11]=double(std::uint32_t(hash_));
  }

 private:
  static double distance(const Sample&point,const Measurement&measured) {
    return std::hypot(point.x-measured.x,point.y-measured.y);
  }
  static int sign(double speed) { return speed>0?1:speed<0?-1:0; }
  static std::vector<Phase> partition(const std::vector<Sample>&reference) {
    int gear=1;
    // Leading stationary samples belong to the first declared nonzero gear;
    // stationary samples BETWEEN gears remain in the preceding phase.
    for(const auto&point:reference)if(sign(point.signed_speed)) {
      gear=sign(point.signed_speed);break;
    }
    std::vector<Phase> phases;int begin=0;
    for(int i=1;i<int(reference.size());++i) {
      const int next=sign(reference[i].signed_speed);
      if(next&&next!=gear) {phases.push_back({begin,i-1,gear});begin=i;gear=next;}
    }
    phases.push_back({begin,int(reference.size())-1,gear});return phases;
  }
  static std::uint64_t fingerprint(const std::vector<Sample>&reference) {
    std::uint64_t hash=14695981039346656037ull;
    auto charge=[&](double value) {
      if(!std::isfinite(value))throw std::invalid_argument("nonfinite directed native sample");
      // Canonical byte order, not struct padding or host-endian byte order.
      if(value==0.)value=0.; // Normalize signed zero.
      std::uint64_t bits=0;static_assert(sizeof(bits)==sizeof(value),"64-bit doubles required");
      std::memcpy(&bits,&value,sizeof(bits));
      for(int i=0;i<8;++i) {hash^=(bits>>(8*i))&255ull;hash*=1099511628211ull;}
    };
    for(const auto&point:reference) {
      charge(point.x);charge(point.y);charge(point.yaw);charge(point.signed_speed);
    }
    return hash;
  }
  std::vector<Phase> phases_;
  int phase_=0,previous_phase_=-1,last_event_=0;
  std::uint64_t hash_=0;
  unsigned transitions_=0;
  double cusp_distance_=0.;
};
} // namespace directed_reference
